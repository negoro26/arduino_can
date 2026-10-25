// Virtual ECUs, a virtual car and the simulated wire. MCU-portable.
//
// No heap, no STL, no exceptions: the same code runs in the PTY simulator, in
// the native tests and in the loopback demo firmware. An ECU is const data
// (flash on the SAMD21) plus ~100 bytes of runtime state.
//
// Time is passed in, in microseconds (consecutive frames are ~230 us apart at
// 500 kbit). Comparisons are wrap-safe int32 differences, valid for schedules
// shorter than 35 minutes.

#ifndef VECU_H
#define VECU_H

#include "can_types.h"

// --------------------------------------------------------------- one ECU

// One canned exchange. Pointers refer to const data.
struct Exchange {
  const uint8_t *request;   // matched as a PREFIX of the reassembled request
  uint8_t requestLen;
  const uint8_t *response;  // service payload, no PCI; segmented when sent
  uint16_t responseLen;     // 0: the ECU stays silent
  uint16_t pendingMs;       // > 0: first "7F <sid> 78", the answer this much later
};

struct EcuSpec {
  const char *name;           // "ECM", "UCH", ...
  CanId physical;             // request id it listens on, e.g. 7E0
  CanId reply;                // id it answers on, e.g. 7E8
  bool obdFunctional;         // also answers OBD-II services on 11-bit 7DF
  const Exchange *exchanges;  // first prefix match wins
  uint8_t exchangeCount;
  uint16_t latencyUs;         // request complete -> first response frame
};

// ISO-TP server for one EcuSpec. Reassembles requests (FC 30 00 00 after a
// client first frame), answers single frames directly, segments long answers
// as FF + CFs honouring the client's FC (BS, STmin; STmin 0 -> back-to-back
// at the bus frame time). A client FC that never comes (N_Bs, 1 s) abandons
// the answer, as a real ECU does. Unknown physical requests -> 7F <sid> 11
// (7F 22 31 for an unknown DID); unknown functional requests -> silence.
class VirtualEcu {
 public:
  void bind(const EcuSpec *spec, Bitrate busRate);
  // A frame some other node put on the wire.
  void deliver(const CanFrame &f, uint32_t nowUs);
  // The frame this ECU transmits now, if one is due at or before nowUs. May
  // return false after a due timeout that produces no frame.
  bool poll(uint32_t nowUs, CanFrame *out);
  // When the next frame or timeout is due; false if nothing is scheduled.
  bool nextDue(uint32_t *atUs) const;

 private:
  enum class Phase : uint8_t {
    Idle, OweFc, Receiving, OwePending, OweAnswer, AwaitFc, SendingCf,
  };
  void answer(uint32_t nowUs);
  void singleFrame(CanFrame *out, const uint8_t *payload, uint8_t len) const;
  uint32_t gapUs() const;

  const EcuSpec *spec_ = nullptr;
  uint16_t frameUs_ = 230;
  Phase phase_ = Phase::Idle;
  uint32_t dueUs_ = 0;
  bool functional_ = false;
  uint8_t req_[64] = {};  // request being reassembled
  uint16_t reqLen_ = 0, reqExpected_ = 0;
  uint8_t reqSeq_ = 0;
  const uint8_t *tx_ = nullptr;  // answer being sent (flash, or nrc_ below)
  uint16_t txLen_ = 0, txSent_ = 0;
  uint8_t txSeq_ = 0, blockSize_ = 0, blockLeft_ = 0;
  uint32_t stMinUs_ = 0;
  uint16_t pendingMs_ = 0;
  uint8_t nrc_[3] = {};  // 7F sid nn
};

// ------------------------------------------------------------ a whole car

// Background traffic, so filters, listen-before-talk and ATMA see a live bus.
struct Periodic {
  CanFrame frame;
  uint16_t periodMs;
};

struct CarSpec {
  const EcuSpec *ecus;
  uint8_t ecuCount;       // <= VirtualCar::kMaxEcus
  const Periodic *traffic;
  uint8_t trafficCount;   // <= VirtualCar::kMaxTraffic
  Bitrate bitrate;        // the rate the car's bus runs at
};

class VirtualCar {
 public:
  static constexpr uint8_t kMaxEcus = 6;
  static constexpr uint8_t kMaxTraffic = 4;

  explicit VirtualCar(const CarSpec &spec);
  Bitrate bitrate() const { return spec_.bitrate; }
  // A frame the adapter put on the wire: offered to every ECU.
  void deliver(const CanFrame &f, uint32_t nowUs);
  // Earliest-due frame from any ECU or periodic source, if due <= nowUs.
  bool poll(uint32_t nowUs, CanFrame *out);
  bool nextDue(uint32_t *atUs) const;
  // True for ids the car transmits (ECU reply ids, periodic ids). The loopback
  // demo uses it to tell the car's frames from echoes of the adapter's own.
  bool transmits(CanId id) const;

 private:
  CarSpec spec_;
  VirtualEcu ecus_[kMaxEcus];
  uint32_t nextPeriodicUs_[kMaxTraffic] = {};
};

// ------------------------------------------------------- the wire, hosted

// The bus as seen through a controller: bitrate, mode, acceptance, RX slots.
//   rate != car rate : nothing is received; every car frame on the wire adds
//                      an error event and REC (the probe's mismatch
//                      signature); send() -> Failed
//   ListenOnly       : receives; send() -> Failed
//   Normal           : send() -> car.deliver(); Sent (the car ACKs)
//   Loopback         : send() -> straight into the RX slots; car not involved
//   RX slots         : 2 = the MCP2515's RXB0/RXB1 with BUKT rollover (tests);
//                      more = a FIFO, so host scheduling jitter in the PTY
//                      simulator cannot fake BUFFER FULL. A frame arriving to
//                      full slots sets rxOverflow.
class SimBus {
 public:
  static constexpr uint8_t kMaxSlots = 32;

  SimBus(VirtualCar &car, uint8_t rxSlots);
  // Returns true when the RX slots were cleared (rate/acceptance change or a
  // closed controller), as a reset MCP2515 would.
  bool open(Bitrate rate, CanMode mode, const Acceptance &acc, uint32_t nowUs);
  void close(uint32_t nowUs);
  TxStatus send(const CanFrame &f, uint32_t nowUs);
  // Moves every car frame due by nowUs onto the wire.
  void advance(uint32_t nowUs);
  // kRxb0Full | kRxb1Full after advancing to nowUs (FIFO: kRxb0Full = any).
  uint8_t rxStatus(uint32_t nowUs);
  bool readRxb(uint8_t n, CanFrame &out);
  CanHealth health() const { return health_; }
  bool nextDue(uint32_t *atUs) const { return car_.nextDue(atUs); }

 private:
  void arrive(const CanFrame &f);
  void clearSlots();

  VirtualCar &car_;
  uint8_t rxSlots_;
  CanFrame slots_[kMaxSlots];
  bool rxbFull_[2] = {false, false};  // two-buffer model: slots_[0], slots_[1]
  uint8_t head_ = 0, used_ = 0;       // FIFO model
  bool open_ = false;
  Bitrate rate_ = Bitrate::k500;
  CanMode mode_ = CanMode::ListenOnly;
  Acceptance acc_ = Acceptance::all();
  CanHealth health_{};
};

#endif  // VECU_H

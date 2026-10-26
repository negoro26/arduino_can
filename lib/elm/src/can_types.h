// Pure C++17: no Arduino, no heap, no STL, no exceptions, so it compiles
// unchanged for the SAMD21 firmware, the native Unity tests and the PTY
// simulator.

#ifndef ELM_CAN_TYPES_H
#define ELM_CAN_TYPES_H

#include <stdint.h>

class CanId {
 public:
  constexpr CanId() : raw_(0) {}
  static constexpr CanId std11(uint32_t v) { return CanId(v & 0x7FFu); }
  static constexpr CanId ext29(uint32_t v) {
    return CanId((v & 0x1FFFFFFFu) | kExt);
  }
  static constexpr CanId of(bool ext, uint32_t v) {
    return ext ? ext29(v) : std11(v);
  }
  constexpr bool isExt() const { return (raw_ & kExt) != 0; }
  constexpr uint32_t value() const { return raw_ & ~kExt; }
  constexpr bool operator==(CanId o) const { return raw_ == o.raw_; }
  constexpr bool operator!=(CanId o) const { return raw_ != o.raw_; }

 private:
  static constexpr uint32_t kExt = 0x80000000u;
  constexpr explicit CanId(uint32_t raw) : raw_(raw) {}
  uint32_t raw_;
};

struct CanFrame {
  CanId id;
  uint8_t dlc;
  uint8_t data[8];  // bytes at index >= dlc are unspecified
};

bool sameFrame(const CanFrame &a, const CanFrame &b);  // id, dlc, dlc bytes

template <uint8_t Capacity>
class SeenIds {
 public:
  bool contains(CanId id) const {
    for (uint8_t i = 0; i < count_; ++i) {
      if (ids_[i] == id) return true;
    }
    return false;
  }
  bool add(CanId id) {
    if (contains(id) || count_ >= Capacity) return false;
    ids_[count_++] = id;
    return true;
  }
  uint8_t count() const { return count_; }
  bool isFull() const { return count_ >= Capacity; }
  CanId at(uint8_t i) const { return ids_[i]; }

 private:
  CanId ids_[Capacity];
  uint8_t count_ = 0;
};

// ISO 15765-2 reading of data[0].
enum class Pci : uint8_t { Single, First, Consecutive, FlowControl, Other };
Pci pciOf(const CanFrame &f);
// Payload length of a single frame, 0 if its PCI is not a valid single frame.
uint8_t singleFrameLength(const CanFrame &f);
uint16_t firstFrameLength(const CanFrame &f);  // Pci::First only: 12-bit length
bool isResponsePending(const CanFrame &f);     // single frame "03 7F xx 78"

enum class Bitrate : uint8_t { k500, k250, k125, k100 };
uint16_t bitrateKbit(Bitrate r);

// Normal ALWAYS means one-shot transmit (no automatic retransmission): there
// is no mode in which the controller may retry a frame that hit a bus error.
enum class CanMode : uint8_t { ListenOnly, Normal, Loopback };

// Receive acceptance: what the controller is programmed to pass AND what the
// core re-checks in software (hardware filters are a load reduction only).
struct Acceptance {
  bool any;       // every frame of both widths
  bool ext;       // width the id/mask apply to
  uint32_t id;
  uint32_t mask;

  static constexpr Acceptance all() { return {true, false, 0, 0}; }
  bool accepts(CanId id) const;
  bool operator==(const Acceptance &o) const;
};

struct CanHealth {
  uint8_t tec;
  uint8_t rec;
  uint16_t errorEvents;  // ERRIF/MERRF occurrences
  bool rxOverflow;       // a frame was lost because both RX buffers were full
};

enum class TxStatus : uint8_t { Sent, Failed };  // Failed: bus error / no ACK

// ELM protocol numbers. 1-5 are K-line/J1850: selectable, never connectable.
enum class Protocol : uint8_t {
  None = 0,
  J1850Pwm = 1, J1850Vpw = 2, Iso9141 = 3, Kwp5Baud = 4, KwpFast = 5,
  Can11_500 = 6, Can29_500 = 7, Can11_250 = 8, Can29_250 = 9,
};

// What AT SP / AT TP selected. {None, true} = "auto, not searched yet";
// {Can11_500, true} = "auto, using 6" (ATDPN prints A6); {Can11_500, false} = 6.
struct ProtocolSel {
  Protocol current;
  bool automatic;
};

constexpr bool isCan(Protocol p) {
  return p >= Protocol::Can11_500 && p <= Protocol::Can29_250;
}
constexpr bool is29Bit(Protocol p) {
  return p == Protocol::Can29_500 || p == Protocol::Can29_250;
}
// Precondition: isCan(p).
constexpr Bitrate bitrateOf(Protocol p) {
  return (p == Protocol::Can11_500 || p == Protocol::Can29_500) ? Bitrate::k500
                                                                : Bitrate::k250;
}

class Deadline {
 public:
  static Deadline after(uint32_t nowMs, uint32_t ms) {
    return Deadline(nowMs + ms, false);
  }
  static Deadline never() { return Deadline(0, true); }
  bool isNever() const { return never_; }
  bool passed(uint32_t nowMs) const {
    return !never_ && static_cast<int32_t>(nowMs - atMs_) >= 0;
  }
  uint32_t atMs() const { return atMs_; }

 private:
  Deadline(uint32_t at, bool never) : atMs_(at), never_(never) {}
  uint32_t atMs_;
  bool never_;
};

#endif  // ELM_CAN_TYPES_H

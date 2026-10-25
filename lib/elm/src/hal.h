// Everything the core needs from the outside world, split along the one rule
// with physical consequences:
//
//   BusSide  CAN controller + clock + serial INPUT (to notice STOPPED).
//            Everything a receive window may touch. No serial output.
//   Hal      BusSide + serial OUTPUT.
//
// The bus procedures in bus.h take BusSide&, so they cannot write to USB while
// an ISO-TP burst is arriving (a blocking USB write is ~1 ms; the MCP2515 has
// two RX buffers and consecutive frames arrive every ~230 us at 500 kbit).
// Only Elm holds a Hal& and prints, and only between windows.

#ifndef ELM_HAL_H
#define ELM_HAL_H

#include "can_types.h"
#include "guard.h"

// readByte() result once the session's link has gone (the TCP client closed or
// left the access point). Whatever is running stops: an ATMA started from a
// phone would otherwise stream into nothing, and hold the session, forever.
constexpr int kLinkClosed = -2;

class BusSide {
 public:
  // Next received byte, -1 if none is buffered, or kLinkClosed. Never blocks.
  virtual int readByte() = 0;

  virtual uint32_t nowMs() = 0;
  // "Nothing to do until d unless a byte or a frame arrives." May return
  // early, never late.
  virtual void idleUntil(Deadline d) = 0;

  // (Re)configure. Resets the health counters. A call that changes only the
  // mode is cheap (MCP2515: one CANCTRL write, no reset). false = the
  // controller did not accept the configuration (hardware fault -> CAN ERROR).
  virtual bool canOpen(Bitrate rate, CanMode mode, const Acceptance &acc) = 0;
  // Leave the bus entirely (config mode: no RX, no ACK).
  virtual void canClose() = 0;
  // One TX buffer, one-shot. Blocks until the frame is on the wire or failed
  // (a few ms at most). Arbitration loss may be retried inside the adapter; a
  // bus error never is. An invalid (empty) ClearedFrame returns Failed.
  virtual TxStatus canSend(const ClearedFrame &frame) = 0;
  // Oldest received frame (controller order preserved), false if none.
  virtual bool canReceive(CanFrame &out) = 0;
  virtual CanHealth canHealth() = 0;
  // AT#SELFTEST stage 1: is the controller there at all? Writes a short
  // human-readable detail ("CANSTAT=80 CANCTRL=87") into `detail`.
  virtual bool canProbe(char *detail, uint8_t capacity) = 0;

 protected:
  ~BusSide() = default;
};

class Hal : public BusSide {
 public:
  // May block (USB backpressure). Never called inside a receive window.
  virtual void write(const char *bytes, uint16_t length) = 0;

 protected:
  ~Hal() = default;
};

#endif  // ELM_HAL_H

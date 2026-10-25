// Bench demo without a car: the virtual Renault/Dacia answers THROUGH the
// MCP2515 in loopback mode, so the SPI, TX and RX paths are the real ones and
// nothing ever reaches the CAN pins.
//   canOpen    : always loopback, whatever mode the core asked for
//   canSend    : the frame goes through TXB0 (and loops back, where it is
//                dropped as an echo), then to the virtual car. At a bitrate
//                other than the car's it fails, as an un-ACKed frame would.
//   canReceive : injects at most one due car frame through TXB0, then reads
//                the chip and drops echoes of the adapter's own frames. One
//                per call keeps the two RX buffers from overflowing.

#ifndef LOOPBACK_DEMO_HAL_H
#define LOOPBACK_DEMO_HAL_H

#include "mcp2515_hal.h"
#include "vecu.h"

class LoopbackDemoHal : public Mcp2515Hal {
 public:
  explicit LoopbackDemoHal(const CarSpec &car) : car_(car) {}

  bool canOpen(Bitrate rate, CanMode mode, const Acceptance &acc) override;
  TxStatus canSend(const ClearedFrame &frame) override;
  bool canReceive(CanFrame &out) override;

  // Called from loop() between commands: lets the car's clock follow
  // micros() (its schedule compares times as 32-bit differences), dropping
  // the frames nobody is listening to at the prompt.
  void keepCarClock();

 private:
  VirtualCar car_;
  Bitrate rate_ = Bitrate::k500;
};

#endif  // LOOPBACK_DEMO_HAL_H

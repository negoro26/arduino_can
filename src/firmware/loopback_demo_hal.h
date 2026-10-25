// Bench demo without a car: the virtual Renault/Dacia answers THROUGH the
// MCP2515 in loopback mode, so the SPI, TX and RX paths are the real ones and
// nothing ever reaches the CAN pins. canReceive injects at most one due car
// frame per call, which keeps the two RX buffers from overflowing.

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

  void keepCarClock();

 private:
  VirtualCar car_;
  Bitrate rate_ = Bitrate::k500;
};

#endif  // LOOPBACK_DEMO_HAL_H

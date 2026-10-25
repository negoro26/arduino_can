// The CAN half of a Hal over SimBus + VirtualCar.
//
// TestHal (virtual time, test/support/test_hal.h) and PtyHal (real time,
// src/sim/pty_hal.h) differ only in clock and serial; the CAN behaviour is
// this class's, so the tests and the simulator exercise the same bus model.

#ifndef SIM_CAN_HAL_H
#define SIM_CAN_HAL_H

#include "hal.h"
#include "rx_order.h"
#include "vecu.h"

class SimCanHal : public Hal {
 public:
  SimCanHal(const CarSpec &car, uint8_t rxSlots);

  bool canOpen(Bitrate rate, CanMode mode, const Acceptance &acc) override;
  void canClose() override;
  TxStatus canSend(const ClearedFrame &frame) override;
  bool canReceive(CanFrame &out) override;
  CanHealth canHealth() override;
  bool canProbe(char *detail, uint8_t capacity) override;

 protected:
  ~SimCanHal() = default;
  virtual uint32_t nowUs() = 0;
  // Puts everything due by now on the wire, then reports when the car next
  // does something; false if it never will.
  bool nextBusEvent(uint32_t *atUs);

 private:
  struct Controller {
    SimBus &bus;
    uint32_t nowUs;
    uint8_t rxStatus() { return bus.rxStatus(nowUs); }
    bool readRxb(uint8_t n, CanFrame &out) { return bus.readRxb(n, out); }
  };

  VirtualCar car_;
  SimBus bus_;
  RolloverOrder order_;
};

#endif  // SIM_CAN_HAL_H

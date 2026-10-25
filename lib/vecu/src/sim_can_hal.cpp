#include "sim_can_hal.h"

#include <string.h>

SimCanHal::SimCanHal(const CarSpec &car, uint8_t rxSlots)
    : car_(car), bus_(car_, rxSlots) {}

bool SimCanHal::canOpen(Bitrate rate, CanMode mode, const Acceptance &acc) {
  if (bus_.open(rate, mode, acc, nowUs())) order_.reset();
  return true;
}

void SimCanHal::canClose() {
  bus_.close(nowUs());
  order_.reset();
}

TxStatus SimCanHal::canSend(const ClearedFrame &frame) {
  if (!frame.valid()) return TxStatus::Failed;
  return bus_.send(frame.frame(), nowUs());
}

bool SimCanHal::canReceive(CanFrame &out) {
  Controller c{bus_, nowUs()};
  return order_.read(c, out);
}

CanHealth SimCanHal::canHealth() {
  bus_.advance(nowUs());
  return bus_.health();
}

bool SimCanHal::canProbe(char *detail, uint8_t capacity) {
  if (capacity > 0) {
    strncpy(detail, "SIMULATED", capacity - 1);
    detail[capacity - 1] = '\0';
  }
  return true;
}

bool SimCanHal::nextBusEvent(uint32_t *atUs) {
  bus_.advance(nowUs());
  return bus_.nextDue(atUs);
}

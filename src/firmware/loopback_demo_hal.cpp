#include "loopback_demo_hal.h"

bool LoopbackDemoHal::canOpen(Bitrate rate, CanMode, const Acceptance &acc) {
  rate_ = rate;
  return Mcp2515Hal::canOpen(rate, CanMode::Loopback, acc);
}

TxStatus LoopbackDemoHal::canSend(const ClearedFrame &frame) {
  if (!frame.valid()) return TxStatus::Failed;
  const TxStatus status = sendRaw(frame.frame());
  if (status != TxStatus::Sent || rate_ != car_.bitrate()) return TxStatus::Failed;
  car_.deliver(frame.frame(), micros());
  return TxStatus::Sent;
}

bool LoopbackDemoHal::canReceive(CanFrame &out) {
  CanFrame due;
  if (car_.poll(micros(), &due) && isOpen() && rate_ == car_.bitrate()) {
    sendRaw(due);
  }
  while (Mcp2515Hal::canReceive(out)) {
    if (car_.transmits(out.id)) return true;
  }
  return false;
}

void LoopbackDemoHal::keepCarClock() {
  CanFrame dropped;
  while (car_.poll(micros(), &dropped)) {
  }
}

#include "vecu.h"

#include <string.h>

#include "rx_order.h"

static bool due(uint32_t atUs, uint32_t nowUs) {
  return static_cast<int32_t>(nowUs - atUs) >= 0;
}

static bool earlier(uint32_t a, uint32_t b) {
  return static_cast<int32_t>(a - b) < 0;
}

void VirtualEcu::bind(const EcuSpec *spec, Bitrate busRate) {
  spec_ = spec;
  // Time one 8-byte frame occupies the wire (~115 bits with stuffing).
  frameUs_ = busRate == Bitrate::k500 ? 230 : 460;
  phase_ = Phase::Idle;
}

uint32_t VirtualEcu::gapUs() const {
  return stMinUs_ > frameUs_ ? stMinUs_ : frameUs_;
}

void VirtualEcu::deliver(const CanFrame &f, uint32_t nowUs) {
  if (spec_ == nullptr) return;
  const bool physical = f.id == spec_->physical;
  const bool functional =
      spec_->obdFunctional && f.id == CanId::std11(0x7DF);
  if (!physical && !functional) return;

  switch (pciOf(f)) {
    case Pci::Single: {
      const uint8_t len = singleFrameLength(f);
      if (len == 0) return;
      memcpy(req_, f.data + 1, len);
      reqLen_ = len;
      functional_ = !physical;
      answer(nowUs);
      return;
    }
    case Pci::First: {
      const uint16_t total = firstFrameLength(f);
      if (!physical || total < 7 || total > sizeof req_ || f.dlc < 8) return;
      memcpy(req_, f.data + 2, 6);
      reqLen_ = 6;
      reqExpected_ = total;
      reqSeq_ = 1;
      functional_ = false;
      phase_ = Phase::OweFc;
      dueUs_ = nowUs + spec_->latencyUs;
      return;
    }
    case Pci::Consecutive: {
      if (!physical || phase_ != Phase::Receiving) return;
      if ((f.data[0] & 0x0F) != (reqSeq_ & 0x0F)) {
        phase_ = Phase::Idle;
        return;
      }
      ++reqSeq_;
      uint16_t take = static_cast<uint16_t>(reqExpected_ - reqLen_);
      if (take > 7) take = 7;
      if (take > f.dlc - 1) take = static_cast<uint16_t>(f.dlc - 1);
      memcpy(req_ + reqLen_, f.data + 1, take);
      reqLen_ = static_cast<uint16_t>(reqLen_ + take);
      if (reqLen_ >= reqExpected_) answer(nowUs);
      return;
    }
    case Pci::FlowControl: {
      if (!physical || phase_ != Phase::AwaitFc || f.dlc < 3) return;
      const uint8_t status = f.data[0] & 0x0F;
      if (status == 1) {
        dueUs_ = nowUs + 1000000;
        return;
      }
      if (status != 0) {
        phase_ = Phase::Idle;
        return;
      }
      blockSize_ = f.data[1];
      blockLeft_ = blockSize_;
      const uint8_t st = f.data[2];
      if (st <= 0x7F) {
        stMinUs_ = static_cast<uint32_t>(st) * 1000;
      } else if (st >= 0xF1 && st <= 0xF9) {
        stMinUs_ = static_cast<uint32_t>(st - 0xF0) * 100;
      } else {
        stMinUs_ = 127000;
      }
      phase_ = Phase::SendingCf;
      dueUs_ = nowUs + frameUs_;
      return;
    }
    case Pci::Other:
      return;
  }
}

void VirtualEcu::answer(uint32_t nowUs) {
  phase_ = Phase::Idle;
  const uint8_t sid = req_[0];
  const Exchange *match = nullptr;
  for (uint8_t i = 0; i < spec_->exchangeCount && match == nullptr; ++i) {
    const Exchange &x = spec_->exchanges[i];
    if (x.requestLen <= reqLen_ && memcmp(x.request, req_, x.requestLen) == 0) {
      match = &x;
    }
  }
  if (functional_ && (match == nullptr || sid >= 0x10)) return;
  dueUs_ = nowUs + spec_->latencyUs;
  if (match == nullptr) {
    nrc_[0] = 0x7F;
    nrc_[1] = sid;
    nrc_[2] = sid == 0x22 ? 0x31 : 0x11;
    tx_ = nrc_;
    txLen_ = 3;
    pendingMs_ = 0;
    phase_ = Phase::OweAnswer;
    return;
  }
  if (match->responseLen == 0) return;
  tx_ = match->response;
  txLen_ = match->responseLen;
  pendingMs_ = match->pendingMs;
  phase_ = pendingMs_ > 0 ? Phase::OwePending : Phase::OweAnswer;
}

void VirtualEcu::singleFrame(CanFrame *out, const uint8_t *payload,
                             uint8_t len) const {
  out->id = spec_->reply;
  out->dlc = 8;
  memset(out->data, 0x00, sizeof out->data);
  out->data[0] = len;
  memcpy(out->data + 1, payload, len);
}

bool VirtualEcu::poll(uint32_t nowUs, CanFrame *out) {
  uint32_t at;
  if (!nextDue(&at) || !due(at, nowUs)) return false;
  switch (phase_) {
    case Phase::Idle:
    case Phase::Receiving:
      return false;
    case Phase::OweFc: {
      static const uint8_t kClearToSend[] = {0x30, 0x00, 0x00};
      out->id = spec_->reply;
      out->dlc = 8;
      memset(out->data, 0x00, sizeof out->data);
      memcpy(out->data, kClearToSend, sizeof kClearToSend);
      phase_ = Phase::Receiving;
      return true;
    }
    case Phase::OwePending: {
      const uint8_t pending[] = {0x7F, req_[0], 0x78};
      singleFrame(out, pending, sizeof pending);
      phase_ = Phase::OweAnswer;
      dueUs_ = at + static_cast<uint32_t>(pendingMs_) * 1000;
      return true;
    }
    case Phase::OweAnswer:
      if (txLen_ <= 7) {
        singleFrame(out, tx_, static_cast<uint8_t>(txLen_));
        phase_ = Phase::Idle;
        return true;
      }
      out->id = spec_->reply;
      out->dlc = 8;
      out->data[0] = static_cast<uint8_t>(0x10 | ((txLen_ >> 8) & 0x0F));
      out->data[1] = static_cast<uint8_t>(txLen_ & 0xFF);
      memcpy(out->data + 2, tx_, 6);
      txSent_ = 6;
      txSeq_ = 1;
      phase_ = Phase::AwaitFc;
      dueUs_ = at + 1000000;
      return true;
    case Phase::AwaitFc:
      phase_ = Phase::Idle;
      return false;
    case Phase::SendingCf: {
      out->id = spec_->reply;
      out->dlc = 8;
      memset(out->data, 0x00, sizeof out->data);
      out->data[0] = static_cast<uint8_t>(0x20 | (txSeq_ & 0x0F));
      uint16_t n = static_cast<uint16_t>(txLen_ - txSent_);
      if (n > 7) n = 7;
      memcpy(out->data + 1, tx_ + txSent_, n);
      txSent_ = static_cast<uint16_t>(txSent_ + n);
      ++txSeq_;
      if (txSent_ >= txLen_) {
        phase_ = Phase::Idle;
      } else if (blockSize_ != 0 && --blockLeft_ == 0) {
        phase_ = Phase::AwaitFc;
        dueUs_ = at + 1000000;
      } else {
        dueUs_ = at + gapUs();
      }
      return true;
    }
  }
  return false;
}

bool VirtualEcu::nextDue(uint32_t *atUs) const {
  if (phase_ == Phase::Idle || phase_ == Phase::Receiving) return false;
  *atUs = dueUs_;
  return true;
}

VirtualCar::VirtualCar(const CarSpec &spec) : spec_(spec) {
  if (spec_.ecuCount > kMaxEcus) spec_.ecuCount = kMaxEcus;
  if (spec_.trafficCount > kMaxTraffic) spec_.trafficCount = kMaxTraffic;
  for (uint8_t i = 0; i < spec_.ecuCount; ++i) {
    ecus_[i].bind(&spec_.ecus[i], spec_.bitrate);
  }
  for (uint8_t i = 0; i < spec_.trafficCount; ++i) {
    nextPeriodicUs_[i] = 700u + 1300u * i;
  }
}

void VirtualCar::deliver(const CanFrame &f, uint32_t nowUs) {
  for (uint8_t i = 0; i < spec_.ecuCount; ++i) ecus_[i].deliver(f, nowUs);
}

bool VirtualCar::poll(uint32_t nowUs, CanFrame *out) {
  for (;;) {
    int8_t ecu = -1;
    int8_t periodic = -1;
    uint32_t best = 0;
    bool found = false;
    for (uint8_t i = 0; i < spec_.ecuCount; ++i) {
      uint32_t at;
      if (ecus_[i].nextDue(&at) && due(at, nowUs) &&
          (!found || earlier(at, best))) {
        found = true;
        best = at;
        ecu = static_cast<int8_t>(i);
      }
    }
    for (uint8_t i = 0; i < spec_.trafficCount; ++i) {
      const uint32_t at = nextPeriodicUs_[i];
      if (due(at, nowUs) && (!found || earlier(at, best))) {
        found = true;
        best = at;
        ecu = -1;
        periodic = static_cast<int8_t>(i);
      }
    }
    if (!found) return false;
    if (ecu >= 0) {
      if (ecus_[ecu].poll(nowUs, out)) return true;
      continue;
    }
    const Periodic &p = spec_.traffic[periodic];
    const uint32_t period = static_cast<uint32_t>(p.periodMs) * 1000;
    uint32_t &next = nextPeriodicUs_[periodic];
    const uint32_t behind = nowUs - next;
    if (behind >= 2 * period) next += (behind / period - 1) * period;
    *out = p.frame;
    next += period;
    return true;
  }
}

bool VirtualCar::nextDue(uint32_t *atUs) const {
  bool found = false;
  for (uint8_t i = 0; i < spec_.ecuCount; ++i) {
    uint32_t at;
    if (ecus_[i].nextDue(&at) && (!found || earlier(at, *atUs))) {
      found = true;
      *atUs = at;
    }
  }
  for (uint8_t i = 0; i < spec_.trafficCount; ++i) {
    if (!found || earlier(nextPeriodicUs_[i], *atUs)) {
      found = true;
      *atUs = nextPeriodicUs_[i];
    }
  }
  return found;
}

bool VirtualCar::transmits(CanId id) const {
  for (uint8_t i = 0; i < spec_.ecuCount; ++i) {
    if (spec_.ecus[i].reply == id) return true;
  }
  for (uint8_t i = 0; i < spec_.trafficCount; ++i) {
    if (spec_.traffic[i].frame.id == id) return true;
  }
  return false;
}

SimBus::SimBus(VirtualCar &car, uint8_t rxSlots)
    : car_(car),
      rxSlots_(rxSlots < 2 ? 2 : (rxSlots > kMaxSlots ? kMaxSlots : rxSlots)) {}

void SimBus::clearSlots() {
  rxbFull_[0] = rxbFull_[1] = false;
  head_ = used_ = 0;
}

bool SimBus::open(Bitrate rate, CanMode mode, const Acceptance &acc,
                  uint32_t nowUs) {
  advance(nowUs);
  const bool reset = !open_ || rate != rate_ || !(acc == acc_);
  if (reset) clearSlots();
  open_ = true;
  rate_ = rate;
  mode_ = mode;
  acc_ = acc;
  health_ = reset ? CanHealth{} : CanHealth{health_.tec, health_.rec, 0, false};
  return reset;
}

void SimBus::close(uint32_t nowUs) {
  advance(nowUs);
  open_ = false;
  clearSlots();
}

TxStatus SimBus::send(const CanFrame &f, uint32_t nowUs) {
  advance(nowUs);
  if (!open_ || mode_ == CanMode::ListenOnly) return TxStatus::Failed;
  if (mode_ == CanMode::Loopback) {
    if (acc_.accepts(f.id)) arrive(f);
    return TxStatus::Sent;
  }
  if (rate_ != car_.bitrate()) {
    // Nobody can decode us: no ACK, an error frame, and one-shot gives up.
    if (health_.errorEvents < 0xFFFF) ++health_.errorEvents;
    health_.tec = static_cast<uint8_t>(health_.tec > 247 ? 255 : health_.tec + 8);
    return TxStatus::Failed;
  }
  car_.deliver(f, nowUs);
  return TxStatus::Sent;
}

void SimBus::advance(uint32_t nowUs) {
  CanFrame f;
  while (car_.poll(nowUs, &f)) {
    if (!open_ || mode_ == CanMode::Loopback) continue;
    if (rate_ != car_.bitrate()) {
      if (health_.errorEvents < 0xFFFF) ++health_.errorEvents;
      if (health_.rec < 255) ++health_.rec;
      continue;
    }
    if (acc_.accepts(f.id)) arrive(f);
  }
}

void SimBus::arrive(const CanFrame &f) {
  if (rxSlots_ == 2) {
    if (!rxbFull_[0]) {
      slots_[0] = f;
      rxbFull_[0] = true;
    } else if (!rxbFull_[1]) {
      slots_[1] = f;
      rxbFull_[1] = true;
    } else {
      health_.rxOverflow = true;
    }
    return;
  }
  if (used_ >= rxSlots_) {
    health_.rxOverflow = true;
    return;
  }
  slots_[(head_ + used_) % rxSlots_] = f;
  ++used_;
}

uint8_t SimBus::rxStatus(uint32_t nowUs) {
  advance(nowUs);
  if (rxSlots_ == 2) {
    return static_cast<uint8_t>((rxbFull_[0] ? kRxb0Full : 0) |
                                (rxbFull_[1] ? kRxb1Full : 0));
  }
  return used_ > 0 ? kRxb0Full : 0;
}

bool SimBus::readRxb(uint8_t n, CanFrame &out) {
  if (rxSlots_ == 2) {
    if (n > 1 || !rxbFull_[n]) return false;
    out = slots_[n];
    rxbFull_[n] = false;
    return true;
  }
  if (n != 0 || used_ == 0) return false;
  out = slots_[head_];
  head_ = static_cast<uint8_t>((head_ + 1) % rxSlots_);
  --used_;
  return true;
}

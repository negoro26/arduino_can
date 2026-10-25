#include "bus.h"

static void discardRestOfLine(BusSide &bus, int first) {
  if (first == '\r') return;
  const Deadline d = Deadline::after(bus.nowMs(), kInterruptLineRestMs);
  for (;;) {
    const int c = bus.readByte();
    if (c == '\r') return;
    if (c >= 0) continue;
    if (d.passed(bus.nowMs())) return;
    bus.idleUntil(d);
  }
}

Wake await(BusSide &bus, Deadline d, CanFrame *out) {
  for (;;) {
    const int c = bus.readByte();
    if (c >= 0 && c != '\n') {
      discardRestOfLine(bus, c);
      return Wake::Interrupted;
    }
    if (c >= 0) continue;
    if (bus.canReceive(*out)) return Wake::Frame;
    if (d.passed(bus.nowMs())) return Wake::Timeout;
    bus.idleUntil(d);
  }
}

bool LinkState::configure(BusSide &bus, Bitrate r, CanMode m,
                          const Acceptance &a) {
  if (!open || rate != r) talk = Talk::Unchecked;
  open = bus.canOpen(r, m, a);
  rate = r;
  acc = a;
  if (!open) talk = Talk::Unchecked;
  return open;
}

void LinkState::close(BusSide &bus) {
  bus.canClose();
  open = false;
  talk = Talk::Unchecked;
}

LbtVerdict listenBeforeTalk(BusSide &bus, LinkState &link, Bitrate rate,
                            bool *interrupted) {
  *interrupted = false;
  // Closing first makes the controller start from reset, so the error
  // counters measured below belong to this window only.
  link.close(bus);
  if (!link.configure(bus, rate, CanMode::ListenOnly, Acceptance::all())) {
    return LbtVerdict::Fault;
  }
  const Deadline d = Deadline::after(bus.nowMs(), kLbtWindowMs);
  CanFrame rx;
  const Wake w = await(bus, d, &rx);
  if (w == Wake::Interrupted) {
    *interrupted = true;
    return LbtVerdict::Silent;
  }
  if (w == Wake::Frame) return LbtVerdict::Verified;
  const CanHealth h = bus.canHealth();
  return (h.errorEvents > 0 || h.rec > 0) ? LbtVerdict::Mismatch
                                          : LbtVerdict::Silent;
}

bool RxLog::push(const CanFrame &f) {
  if (count >= kCapacity) return false;
  frames[count++] = f;
  return true;
}

namespace {

class OwedFrames {
 public:
  void expect(const CanFrame &firstFrame) {
    const uint16_t total = firstFrameLength(firstFrame);
    const uint16_t inFirst = static_cast<uint16_t>(firstFrame.dlc - 2);
    const uint16_t rest = total > inFirst ? total - inFirst : 0;
    const uint16_t frames = static_cast<uint16_t>((rest + 6) / 7);
    Entry *e = find(firstFrame.id);
    if (e == nullptr) {
      e = count_ < kSlots ? &entries_[count_++] : &entries_[kSlots - 1];
      e->id = firstFrame.id;
    }
    e->frames = frames;
  }
  void received(CanId id) {
    Entry *e = find(id);
    if (e != nullptr && e->frames > 0) --e->frames;
  }
  bool any() const {
    for (uint8_t i = 0; i < count_; ++i) {
      if (entries_[i].frames > 0) return true;
    }
    return false;
  }

 private:
  struct Entry {
    CanId id;
    uint16_t frames;
  };
  static constexpr uint8_t kSlots = 4;
  Entry *find(CanId id) {
    for (uint8_t i = 0; i < count_; ++i) {
      if (entries_[i].id == id) return &entries_[i];
    }
    return nullptr;
  }
  Entry entries_[kSlots] = {};
  uint8_t count_ = 0;
};

}  // namespace

Outcome converse(BusSide &bus, Guard &guard, LinkState &link,
                 const Settings &s, Protocol p, const HexRequest &req,
                 RxLog &log) {
  log.clear();
  const CanFrame request = requestFrame(s, p, req.bytes, req.len);
  ClearedFrame cleared;
  if (!guard.admit(request, &cleared)) return Outcome::Blocked;

  const Bitrate rate = bitrateOf(p);
  const Acceptance want = receiveFilter(s, p);
  if (!link.open || link.rate != rate ||
      link.talk != LinkState::Talk::Allowed) {
    bool interrupted = false;
    const LbtVerdict v = listenBeforeTalk(bus, link, rate, &interrupted);
    if (interrupted) return Outcome::Stopped;
    if (v == LbtVerdict::Fault) return Outcome::CanError;
    if (v == LbtVerdict::Mismatch) {
      link.talk = LinkState::Talk::Mismatch;
      return Outcome::CanError;
    }
    link.talk = LinkState::Talk::Allowed;
  }

  CanFrame rx;
  while (bus.canReceive(rx)) {
  }
  if (!link.configure(bus, rate, CanMode::Normal, want)) {
    return Outcome::CanError;
  }
  if (bus.canSend(cleared) != TxStatus::Sent) {
    link.configure(bus, rate, CanMode::ListenOnly, want);
    link.talk = LinkState::Talk::Unchecked;
    return Outcome::CanError;
  }
  if (!s.responses || req.maxResponses == 0) {
    link.configure(bus, rate, CanMode::ListenOnly, want);
    return Outcome::Sent;
  }

  OwedFrames owed;
  uint8_t counted = 0;
  bool stopped = false;
  bool full = false;
  Deadline d = Deadline::after(bus.nowMs(), firstWaitMs(s));
  for (;;) {
    const Wake w = await(bus, d, &rx);
    if (w == Wake::Interrupted) {
      stopped = true;
      break;
    }
    if (w == Wake::Timeout) break;
    if (!want.accepts(rx.id)) continue;
    if (isResponsePending(rx)) {
      if (!log.push(rx)) {
        full = true;
        break;
      }
      d = Deadline::after(bus.nowMs(), kPendingWaitMs);
      continue;
    }
    const bool shown = shownAsResponse(s, rx);
    if (shown && !log.push(rx)) {
      full = true;
      break;
    }
    const Pci pci = pciOf(rx);
    if (pci == Pci::First && s.cfc) {
      ClearedFrame fc;
      if (guard.admit(flowControlFrame(s, p, rx), &fc) &&
          bus.canSend(fc) == TxStatus::Sent) {
        owed.expect(rx);
      } else {
        link.talk = LinkState::Talk::Unchecked;
      }
    } else if (pci == Pci::Consecutive) {
      owed.received(rx.id);
    }
    if (shown) ++counted;
    if (req.maxResponses > 0 && counted >= req.maxResponses) break;
    d = Deadline::after(bus.nowMs(),
                        owed.any() ? firstWaitMs(s) : nextWaitMs(s));
  }
  if (bus.canHealth().rxOverflow) full = true;
  link.configure(bus, rate, CanMode::ListenOnly, want);

  if (stopped) return Outcome::Stopped;
  if (full) return Outcome::BufferFull;
  return log.count > 0 ? Outcome::Answered : Outcome::NoData;
}

Outcome search(BusSide &bus, Guard &guard, LinkState &link, Settings &s,
               const HexRequest &req, RxLog &log) {
  static const Protocol kOrder[] = {Protocol::Can11_500, Protocol::Can29_500,
                                    Protocol::Can11_250, Protocol::Can29_250};
  for (Protocol p : kOrder) {
    const Outcome o = converse(bus, guard, link, s, p, req, log);
    switch (o) {
      case Outcome::Answered:
      case Outcome::BufferFull:
      case Outcome::Sent:
        s.protocol = {p, true};
        return o;
      case Outcome::Blocked:
      case Outcome::Stopped:
        return o;
      case Outcome::NoData:
      case Outcome::CanError:
      case Outcome::UnableToConnect:
        break;
    }
  }
  log.clear();
  return Outcome::UnableToConnect;
}

bool surveyRate(BusSide &bus, Bitrate rate, uint16_t windowMs,
                RateReport *out) {
  *out = RateReport{};
  bus.canClose();
  if (!bus.canOpen(rate, CanMode::ListenOnly, Acceptance::all())) {
    out->setupFailed = true;
    bus.canClose();
    return true;
  }
  const Deadline d = Deadline::after(bus.nowMs(), windowMs);
  CanFrame rx;
  bool interrupted = false;
  for (;;) {
    const Wake w = await(bus, d, &rx);
    if (w == Wake::Timeout) break;
    if (w == Wake::Interrupted) {
      interrupted = true;
      break;
    }
    ++out->frames;
    out->ids.add(rx.id);
  }
  const CanHealth h = bus.canHealth();
  out->errors = static_cast<uint32_t>(h.errorEvents) + h.rec;
  bus.canClose();
  return !interrupted;
}

Wiring wiringVerdict(uint32_t frames, uint32_t errors) {
  if (frames > 0) return Wiring::BitrateFound;
  return errors > 0 ? Wiring::LiveBusUndecoded : Wiring::NotConnected;
}

Loopback loopbackRoundTrip(BusSide &bus, Guard &guard, Bitrate rate) {
  bus.canClose();
  if (!bus.canOpen(rate, CanMode::Loopback, Acceptance::all())) {
    return Loopback::SetupFailed;
  }
  const CanFrame probe = {CanId::std11(0x745), 8,
                          {0x02, 0x10, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00}};
  ClearedFrame cleared;
  Loopback result = Loopback::Fail;
  if (guard.admit(probe, &cleared) && bus.canSend(cleared) == TxStatus::Sent) {
    const Deadline d = Deadline::after(bus.nowMs(), 50);
    CanFrame rx;
    const Wake w = await(bus, d, &rx);
    if (w == Wake::Interrupted) {
      result = Loopback::Interrupted;
    } else if (w == Wake::Frame && sameFrame(rx, probe)) {
      result = Loopback::Pass;
    }
  }
  bus.canClose();
  return result;
}

#include "elm.h"

#include <string.h>

namespace {

class Line {
 public:
  Line &text(const char *s) {
    while (*s && len_ + 1 < sizeof buf_) buf_[len_++] = *s++;
    buf_[len_] = '\0';
    return *this;
  }
  Line &number(uint32_t v) {
    char digits[11];
    uint8_t n = 0;
    do {
      digits[n++] = static_cast<char>('0' + v % 10);
      v /= 10;
    } while (v != 0);
    char reversed[11];
    for (uint8_t i = 0; i < n; ++i) reversed[i] = digits[n - 1 - i];
    reversed[n] = '\0';
    return text(reversed);
  }
  Line &id(CanId id) {
    static const char kHex[] = "0123456789ABCDEF";
    const uint8_t nibbles = id.isExt() ? 8 : 3;
    char s[9];
    for (uint8_t i = 0; i < nibbles; ++i) {
      s[i] = kHex[(id.value() >> (4 * (nibbles - 1 - i))) & 0xF];
    }
    s[nibbles] = '\0';
    return text(s);
  }
  const char *str() const { return buf_; }

 private:
  char buf_[128] = {};
  size_t len_ = 0;
};

const Bitrate kSurveyRates[] = {Bitrate::k500, Bitrate::k250, Bitrate::k125,
                                Bitrate::k100};
constexpr uint16_t kSurveyWindowMs = 1000;
// ATMA output is batched so the USB link sees few large writes instead of one
// per frame; a partial batch is flushed after this much quiet time.
constexpr uint16_t kMonitorFlushMs = 10;

}  // namespace

void Elm::begin() {
  print("ELM327 v1.5");
  hal_.write(lineEnd(set_), static_cast<uint16_t>(strlen(lineEnd(set_))));
  hal_.write(">", 1);
}

void Elm::service() {
  const uint8_t switches = hal_.linkSwitches();
  if (switches != linkSwitches_) {
    linkSwitches_ = switches;
    line_.clear();
    last_[0] = '\0';
  }
  for (int c; (c = hal_.readByte()) >= 0;) onByte(static_cast<uint8_t>(c));
}

void Elm::onByte(uint8_t c) {
  if (c == '\n') return;
  if (set_.echo) {
    if (c == '\r') {
      const char *end = lineEnd(set_);
      hal_.write(end, static_cast<uint16_t>(strlen(end)));
    } else {
      hal_.write(reinterpret_cast<const char *>(&c), 1);
    }
  }
  if (c != '\r') {
    line_.push(c);
    return;
  }
  execute();
  line_.clear();
}

void Elm::execute() {
  const char *line = line_.text();
  if (!line_.valid()) {
    print("?");
  } else if (line_.blank()) {
    if (last_[0] == '\0') {
      print("?");
    } else {
      runRequest(last_);
    }
  } else if (line[0] == 'A' && line[1] == 'T') {
    runAt(line + 2);
  } else {
    runRequest(line);
  }
  const char *end = lineEnd(set_);
  hal_.write(end, static_cast<uint16_t>(strlen(end)));
  hal_.write(">", 1);
}

void Elm::runAt(const char *body) {
  AtArgs args{};
  const AtCommand *cmd = findAt(body, &args);
  AtReply reply = AtReply::Bad;
  if (cmd != nullptr && cmd->run != nullptr) {
    reply = cmd->run(*this, args);
  } else if (cmd != nullptr) {
    set_.*(cmd->flag) =
        cmd->shape == AtShape::Bit ? args.value != 0 : cmd->flagValue;
    reply = AtReply::Ok;
  }
  if (reply == AtReply::Ok) print("OK");
  if (reply == AtReply::Bad) print("?");
}

void Elm::runRequest(const char *line) {
  HexRequest req;
  if (!parseHexRequest(line, set_.caf, &req)) {
    print("?");
    return;
  }
  if (line != last_) strcpy(last_, line);

  Outcome outcome;
  const Protocol p = set_.protocol.current;
  if (p == Protocol::None) {
    // A request refused under every protocol must not even print
    // SEARCHING..., let alone listen-before-talk on each one.
    if (!searchable(guard_, set_, req)) {
      print("BLOCKED");
      return;
    }
    print("SEARCHING...");
    outcome = search(hal_, guard_, link_, set_, req, log_);
  } else if (!isCan(p)) {
    log_.clear();
    outcome = Outcome::UnableToConnect;
  } else {
    outcome = converse(hal_, guard_, link_, set_, p, req, log_);
  }
  printFrames();
  printOutcome(outcome);
}

void Elm::monitor() {
  const Protocol current = set_.protocol.current;
  const Protocol p = isCan(current) ? current : Protocol::Can11_500;
  // With automatic receive filtering a real ELM monitors every id; only an
  // explicit CRA / CF+CM narrows ATMA.
  const Acceptance acc = set_.rx.mode == RxFilterMode::Auto
                             ? Acceptance::all()
                             : receiveFilter(set_, p);
  link_.close(hal_);
  if (!hal_.canOpen(bitrateOf(p), CanMode::ListenOnly, acc)) {
    print("CAN ERROR");
    return;
  }
  FrameFormatter fmt(set_, true);
  const char *end = lineEnd(set_);
  const size_t endLen = strlen(end);
  char batch[256];
  size_t used = 0;
  Deadline flushAt = Deadline::never();
  for (;;) {
    CanFrame rx;
    const Wake w = await(hal_, flushAt, &rx);
    if (w == Wake::Interrupted) break;
    if (w == Wake::Timeout) {
      hal_.write(batch, static_cast<uint16_t>(used));
      hal_.flush();
      used = 0;
      flushAt = Deadline::never();
      continue;
    }
    guard_.heard(rx.id);
    if (!acc.accepts(rx.id)) continue;
    TextLine lines[2];
    const uint8_t n = fmt.format(rx, lines);
    for (uint8_t i = 0; i < n; ++i) {
      if (used + lines[i].len + endLen > sizeof batch) {
        hal_.write(batch, static_cast<uint16_t>(used));
        used = 0;
      }
      memcpy(batch + used, lines[i].text, lines[i].len);
      used += lines[i].len;
      memcpy(batch + used, end, endLen);
      used += endLen;
    }
    if (used > 0 && flushAt.isNever()) {
      flushAt = Deadline::after(hal_.nowMs(), kMonitorFlushMs);
    }
  }
  if (used > 0) hal_.write(batch, static_cast<uint16_t>(used));
  link_.close(hal_);
  print("STOPPED");
}

void Elm::survey() {
  link_.close(hal_);
  uint32_t totalErrors = 0;
  bool configured = false;
  bool found = false;
  Bitrate foundRate = Bitrate::k500;
  for (Bitrate rate : kSurveyRates) {
    RateReport report;
    if (!surveyRate(hal_, guard_, rate, kSurveyWindowMs, &report)) {
      print("STOPPED");
      return;
    }
    Line line;
    line.number(bitrateKbit(rate)).text(" kbit: ");
    if (report.setupFailed) {
      print(line.text("SETUP FAILED").str());
      continue;
    }
    configured = true;
    line.number(report.frames)
        .text(" frames, errors ")
        .number(report.errors);
    if (report.ids.count() > 0) {
      line.text(", ids");
      for (uint8_t i = 0; i < report.ids.count(); ++i) {
        line.text(" ").id(report.ids.at(i));
      }
      if (report.ids.isFull()) line.text(" ...");
    }
    print(line.str());
    totalErrors += report.errors;
    if (report.frames > 0) {
      found = true;
      foundRate = rate;
      break;
    }
  }
  if (!configured) {
    print("WIRING: UNKNOWN, the CAN controller could not be configured "
          "(run AT#SELFTEST)");
    return;
  }
  switch (wiringVerdict(found ? 1 : 0, totalErrors)) {
    case Wiring::BitrateFound: {
      Line line;
      line.text("WIRING: OK, bus decoding cleanly at ")
          .number(bitrateKbit(foundRate))
          .text(" kbit");
      print(line.str());
      break;
    }
    case Wiring::LiveBusUndecoded:
      print("WIRING: CONNECTED to a live bus, but nothing decoded:");
      print("swap CAN-H and CAN-L, or the bus runs at a rate not tried");
      break;
    case Wiring::NotConnected:
      print("WIRING: NOT CONNECTED, zero frames and zero errors at every rate:");
      print("wrong OBD pins, broken cable, or bus asleep (ignition on?)");
      break;
  }
}

void Elm::selfTest() {
  link_.close(hal_);
  char detail[40] = {};
  const bool present = hal_.canProbe(detail, sizeof detail);
  Line probe;
  probe.text("MCP2515 ").text(detail).text(present ? ": PASS" : ": FAIL");
  print(probe.str());
  uint8_t passed = 0;
  if (present) {
    for (Bitrate rate : kSurveyRates) {
      const Loopback result = loopbackRoundTrip(hal_, guard_, rate);
      if (result == Loopback::Interrupted) {
        print("STOPPED");
        return;
      }
      Line line;
      line.text("LOOPBACK ").number(bitrateKbit(rate)).text(" kbit: ");
      switch (result) {
        case Loopback::Pass: line.text("PASS"); ++passed; break;
        case Loopback::Fail: line.text("FAIL"); break;
        case Loopback::SetupFailed: line.text("SETUP FAILED"); break;
        case Loopback::Interrupted: break;
      }
      print(line.str());
    }
  }
  print(passed > 0 ? "SELFTEST: PASS" : "SELFTEST: FAIL");
}

void Elm::resetSettings() {
  set_ = Settings{};
  set_.protocol = storedProtocol_;
  link_.close(hal_);
}

void Elm::print(const char *text) {
  hal_.write(text, static_cast<uint16_t>(strlen(text)));
  const char *end = lineEnd(set_);
  hal_.write(end, static_cast<uint16_t>(strlen(end)));
}

void Elm::printFrames() {
  FrameFormatter fmt(set_, false);
  for (uint16_t i = 0; i < log_.count; ++i) {
    TextLine lines[2];
    const uint8_t n = fmt.format(log_.frames[i], lines);
    for (uint8_t j = 0; j < n; ++j) print(lines[j].text);
  }
}

void Elm::printOutcome(Outcome o) {
  switch (o) {
    case Outcome::Answered:
    case Outcome::Sent:
      break;
    case Outcome::NoData: print("NO DATA"); break;
    case Outcome::Blocked: print("BLOCKED"); break;
    case Outcome::CanError: print("CAN ERROR"); break;
    case Outcome::Stopped: print("STOPPED"); break;
    case Outcome::BufferFull: print("BUFFER FULL"); break;
    case Outcome::UnableToConnect: print("UNABLE TO CONNECT"); break;
  }
}

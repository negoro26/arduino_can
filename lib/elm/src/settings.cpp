#include "settings.h"

CanId headerId(const TxHeader &h, uint8_t canPriority, Protocol p) {
  if (!is29Bit(p)) return CanId::std11(h.bits);
  if (h.digits == 8) return CanId::ext29(h.bits);
  return CanId::ext29((static_cast<uint32_t>(canPriority & 0x1F) << 24) |
                      h.bits);
}

CanId requestId(const Settings &s, Protocol p) {
  if (s.header.digits != 0) return headerId(s.header, s.canPriority, p);
  if (is29Bit(p)) {
    return CanId::ext29(
        (static_cast<uint32_t>(s.canPriority & 0x1F) << 24) | 0xDB33F1u);
  }
  return CanId::std11(0x7DF);
}

Acceptance receiveFilter(const Settings &s, Protocol p) {
  const bool ext = is29Bit(p);
  if (s.rx.mode == RxFilterMode::Pattern) {
    return {false, ext, s.rx.id, s.rx.mask};
  }
  return ext ? Acceptance{false, true, 0x18DAF100u, 0x1FFFFF00u}
             : Acceptance{false, false, 0x7E8u, 0x7F8u};
}

static void padFrame(const Settings &s, CanFrame &f) {
  if (s.varDlc) return;
  for (uint8_t i = f.dlc; i < 8; ++i) f.data[i] = 0x00;
  f.dlc = 8;
}

CanFrame requestFrame(const Settings &s, Protocol p, const uint8_t *bytes,
                      uint8_t len) {
  CanFrame f{};
  f.id = requestId(s, p);
  uint8_t n = 0;
  if (s.caf) {
    if (len > 7) len = 7;
    f.data[n++] = len;
  } else if (len > 8) {
    len = 8;
  }
  for (uint8_t i = 0; i < len; ++i) f.data[n++] = bytes[i];
  f.dlc = n;
  padFrame(s, f);
  return f;
}

static CanId automaticFlowControlId(const Settings &s, Protocol p,
                                    CanId responder) {
  const uint32_t v = responder.value();
  if (!responder.isExt() && v >= 0x7E8 && v <= 0x7EF) {
    return CanId::std11(v - 8);
  }
  if (responder.isExt() && (v & 0x1FFFFF00u) == 0x18DAF100u) {
    return CanId::ext29(0x18DA00F1u | ((v & 0xFFu) << 8));
  }
  return requestId(s, p);
}

CanFrame flowControlFrame(const Settings &s, Protocol p,
                          const CanFrame &firstFrame) {
  CanFrame f{};
  if (s.fc.mode == 1) {
    f.id = headerId(s.fc.header, s.canPriority, p);
  } else {
    f.id = automaticFlowControlId(s, p, firstFrame.id);
  }
  if (s.fc.mode == 0) {
    f.data[0] = 0x30;
    f.data[1] = 0x00;
    f.data[2] = 0x00;
    f.dlc = 3;
  } else {
    for (uint8_t i = 0; i < s.fc.dataLen; ++i) f.data[i] = s.fc.data[i];
    f.dlc = s.fc.dataLen;
  }
  padFrame(s, f);
  return f;
}

uint16_t firstWaitMs(const Settings &s) {
  return static_cast<uint16_t>((s.stTicks ? s.stTicks : 0x32) * 4);
}

uint16_t nextWaitMs(const Settings &s) {
  const uint16_t st = firstWaitMs(s);
  const uint16_t cap = s.adaptive == 2 ? 25 : 50;
  if (s.adaptive == 0 || st < cap) return st;
  return cap;
}

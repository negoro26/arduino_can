#include "at_table.h"

#include <string.h>

#include "elm.h"

static int hexDigit(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  return -1;
}

static bool parseHex(const char *s, uint8_t n, uint32_t *out) {
  uint32_t v = 0;
  for (uint8_t i = 0; i < n; ++i) {
    const int d = hexDigit(s[i]);
    if (d < 0) return false;
    v = (v << 4) | static_cast<uint32_t>(d);
  }
  *out = v;
  return true;
}

static bool parseBytes(const char *s, uint8_t maxBytes, AtArgs *a) {
  const size_t n = strlen(s);
  if (n == 0 || n % 2 != 0 || n / 2 > maxBytes) return false;
  for (uint8_t i = 0; i < n / 2; ++i) {
    uint32_t v;
    if (!parseHex(s + 2 * i, 2, &v)) return false;
    a->bytes[i] = static_cast<uint8_t>(v);
  }
  a->count = static_cast<uint8_t>(n / 2);
  return true;
}

static bool parseShape(AtShape shape, const char *s, AtArgs *a) {
  const size_t n = strlen(s);
  switch (shape) {
    case AtShape::None:
      return n == 0;
    case AtShape::Bit:
    case AtShape::Digit012:
      if (n != 1 || s[0] < '0' || s[0] > (shape == AtShape::Bit ? '1' : '2')) {
        return false;
      }
      a->value = static_cast<uint32_t>(s[0] - '0');
      return true;
    case AtShape::Hex2:
      return n == 2 && parseHex(s, 2, &a->value);
    case AtShape::Dec4:
      if (n != 4) return false;
      a->value = 0;
      for (uint8_t i = 0; i < 4; ++i) {
        if (s[i] < '0' || s[i] > '9') return false;
        a->value = a->value * 10 + static_cast<uint32_t>(s[i] - '0');
      }
      return true;
    case AtShape::Header:
      if (n != 3 && n != 6 && n != 8) return false;
      a->count = static_cast<uint8_t>(n);
      return parseHex(s, static_cast<uint8_t>(n), &a->value);
    case AtShape::Bytes1to5:
      return parseBytes(s, 5, a);
    case AtShape::Bytes1to6:
      return parseBytes(s, 6, a);
    case AtShape::Pattern: {
      if (n != 3 && n != 8) return false;
      a->value = 0;
      a->mask = 0;
      for (size_t i = 0; i < n; ++i) {
        a->value <<= 4;
        a->mask <<= 4;
        if (s[i] == 'X') continue;
        const int d = hexDigit(s[i]);
        if (d < 0) return false;
        a->value |= static_cast<uint32_t>(d);
        a->mask |= 0xF;
      }
      a->count = static_cast<uint8_t>(n);
      return true;
    }
    case AtShape::ProtoSel: {
      if (strcmp(s, "00") == 0) s = "0";
      a->automatic = s[0] == 'A';
      if (a->automatic) ++s;
      if (strlen(s) != 1 || s[0] < '0' || s[0] > '9') return false;
      a->value = static_cast<uint32_t>(s[0] - '0');
      if (a->value == 0) a->automatic = true;
      return true;
    }
    case AtShape::TierWord:
      return parseTier(s, &a->tier);
  }
  return false;
}

const AtCommand *findAt(const char *body, AtArgs *args) {
  const AtCommand *best = nullptr;
  size_t bestLen = 0;
  for (uint8_t i = 0; i < AtTable::rowCount; ++i) {
    const AtCommand &row = AtTable::rows[i];
    const size_t n = strlen(row.name);
    if (strncmp(body, row.name, n) != 0) continue;
    if (best != nullptr && n <= bestLen) continue;
    AtArgs parsed{};
    if (!parseShape(row.shape, body + n, &parsed)) continue;
    best = &row;
    bestLen = n;
    *args = parsed;
  }
  return best;
}

static const char kBanner[] = "ELM327 v1.5";

static ProtocolSel selection(const AtArgs &a) {
  return {static_cast<Protocol>(a.value), a.automatic};
}

static TxHeader header(const AtArgs &a) { return {a.value, a.count}; }

static uint32_t fullMask(uint8_t digits) {
  return digits == 8 ? 0x1FFFFFFFu : 0x7FFu;
}

static AtReply ok(Elm &, const AtArgs &) { return AtReply::Ok; }

AtReply AtTable::busInitError(Elm &e, const AtArgs &) {
  e.print("BUS INIT: ...ERROR");
  return AtReply::Printed;
}

const AtCommand AtTable::rows[] = {
    {"", AtShape::None, ok, nullptr, false},
    {"Z", AtShape::None,
     [](Elm &e, const AtArgs &) {
       e.resetSettings();
       e.print("");
       e.print(kBanner);
       return AtReply::Printed;
     },
     nullptr, false},
    {"WS", AtShape::None,
     [](Elm &e, const AtArgs &) {
       e.resetSettings();
       e.print("");
       e.print(kBanner);
       return AtReply::Printed;
     },
     nullptr, false},
    {"D", AtShape::None,
     [](Elm &e, const AtArgs &) {
       e.resetSettings();
       return AtReply::Ok;
     },
     nullptr, false},
    {"I", AtShape::None,
     [](Elm &e, const AtArgs &) {
       e.print(kBanner);
       return AtReply::Printed;
     },
     nullptr, false},
    {"@1", AtShape::None,
     [](Elm &e, const AtArgs &) {
       e.print("OBDII to RS232 Interpreter");
       return AtReply::Printed;
     },
     nullptr, false},
    // Pin 16 (the ELM's voltage input) is not wired on this board. python-OBD
    // refuses to connect below 6 V, so a fixed nominal value is reported.
    {"RV", AtShape::None,
     [](Elm &e, const AtArgs &) {
       e.print("12.0V");
       return AtReply::Printed;
     },
     nullptr, false},

    {"E", AtShape::Bit, nullptr, &Settings::echo, false},
    {"L", AtShape::Bit, nullptr, &Settings::linefeeds, false},
    {"S", AtShape::Bit, nullptr, &Settings::spaces, false},
    {"H", AtShape::Bit, nullptr, &Settings::headers, false},
    {"R", AtShape::Bit, nullptr, &Settings::responses, false},
    {"V", AtShape::Bit, nullptr, &Settings::varDlc, false},
    {"D", AtShape::Bit, nullptr, &Settings::dlc, false},
    {"CAF", AtShape::Bit, nullptr, &Settings::caf, false},
    {"CFC", AtShape::Bit, nullptr, &Settings::cfc, false},
    {"AL", AtShape::None, nullptr, &Settings::allowLong, true},
    {"NL", AtShape::None, nullptr, &Settings::allowLong, false},

    {"AT", AtShape::Digit012,
     [](Elm &e, const AtArgs &a) {
       e.set_.adaptive = static_cast<uint8_t>(a.value);
       return AtReply::Ok;
     },
     nullptr, false},
    {"ST", AtShape::Hex2,
     [](Elm &e, const AtArgs &a) {
       e.set_.stTicks = a.value == 0 ? 0x32 : static_cast<uint8_t>(a.value);
       return AtReply::Ok;
     },
     nullptr, false},
    {"SH", AtShape::Header,
     [](Elm &e, const AtArgs &a) {
       e.set_.header = header(a);
       return AtReply::Ok;
     },
     nullptr, false},
    {"CP", AtShape::Hex2,
     [](Elm &e, const AtArgs &a) {
       e.set_.canPriority = static_cast<uint8_t>(a.value & 0x1F);
       return AtReply::Ok;
     },
     nullptr, false},
    {"SP", AtShape::ProtoSel,
     [](Elm &e, const AtArgs &a) {
       e.set_.protocol = selection(a);
       e.storedProtocol_ = selection(a);
       return AtReply::Ok;
     },
     nullptr, false},
    {"TP", AtShape::ProtoSel,
     [](Elm &e, const AtArgs &a) {
       e.set_.protocol = selection(a);
       return AtReply::Ok;
     },
     nullptr, false},
    {"DP", AtShape::None,
     [](Elm &e, const AtArgs &) {
       const ProtocolSel sel = e.set_.protocol;
       const char *name = protocolName(sel.current);
       if (name == nullptr) {
         e.print("AUTO");
       } else if (!sel.automatic) {
         e.print(name);
       } else {
         char text[40] = "AUTO, ";
         strncat(text, name, sizeof text - strlen(text) - 1);
         e.print(text);
       }
       return AtReply::Printed;
     },
     nullptr, false},
    {"DPN", AtShape::None,
     [](Elm &e, const AtArgs &) {
       char text[3];
       formatDpn(e.set_.protocol, text);
       e.print(text);
       return AtReply::Printed;
     },
     nullptr, false},
    {"PC", AtShape::None,
     [](Elm &e, const AtArgs &) {
       e.link_.close(e.hal_);
       return AtReply::Ok;
     },
     nullptr, false},

    {"AR", AtShape::None,
     [](Elm &e, const AtArgs &) {
       e.set_.rx = RxFilter{};
       return AtReply::Ok;
     },
     nullptr, false},
    {"CRA", AtShape::None,
     [](Elm &e, const AtArgs &) {
       e.set_.rx = RxFilter{};
       return AtReply::Ok;
     },
     nullptr, false},
    {"CRA", AtShape::Pattern,
     [](Elm &e, const AtArgs &a) {
       e.set_.rx = {RxFilterMode::Pattern, a.value, a.mask};
       return AtReply::Ok;
     },
     nullptr, false},
    {"CF", AtShape::Header,
     [](Elm &e, const AtArgs &a) {
       if (e.set_.rx.mode != RxFilterMode::Pattern) {
         e.set_.rx = {RxFilterMode::Pattern, 0, fullMask(a.count)};
       }
       e.set_.rx.id = a.value;
       return AtReply::Ok;
     },
     nullptr, false},
    {"CM", AtShape::Header,
     [](Elm &e, const AtArgs &a) {
       if (e.set_.rx.mode != RxFilterMode::Pattern) {
         e.set_.rx = {RxFilterMode::Pattern, 0, 0};
       }
       e.set_.rx.mask = a.value;
       return AtReply::Ok;
     },
     nullptr, false},

    {"FCSH", AtShape::Header,
     [](Elm &e, const AtArgs &a) {
       e.set_.fc.header = header(a);
       return AtReply::Ok;
     },
     nullptr, false},
    {"FCSD", AtShape::Bytes1to5,
     [](Elm &e, const AtArgs &a) {
       memcpy(e.set_.fc.data, a.bytes, a.count);
       e.set_.fc.dataLen = a.count;
       return AtReply::Ok;
     },
     nullptr, false},
    // FC modes 1 and 2 need their id/data defined first; the ELM answers "?"
    // otherwise, and ddt4all/pyren always define them before AT FC SM 1.
    {"FCSM", AtShape::Digit012,
     [](Elm &e, const AtArgs &a) {
       const FlowControl &fc = e.set_.fc;
       if (a.value >= 1 && fc.dataLen == 0) return AtReply::Bad;
       if (a.value == 1 && fc.header.digits == 0) return AtReply::Bad;
       e.set_.fc.mode = static_cast<uint8_t>(a.value);
       return AtReply::Ok;
     },
     nullptr, false},

    {"MA", AtShape::None,
     [](Elm &e, const AtArgs &) {
       e.monitor();
       return AtReply::Printed;
     },
     nullptr, false},
    {"CS", AtShape::None,
     [](Elm &e, const AtArgs &) {
       static const char kHex[] = "0123456789ABCDEF";
       const CanHealth h = e.hal_.canHealth();
       const char text[] = {'T', ':', kHex[h.tec >> 4], kHex[h.tec & 0xF],
                            ' ', 'R', ':', kHex[h.rec >> 4],
                            kHex[h.rec & 0xF], '\0'};
       e.print(text);
       return AtReply::Printed;
     },
     nullptr, false},

    // Accepted as no-ops so ddt4all/pyren adapter checkers see no "?": CAN
    // silent monitoring, turning off CAN extended addressing (it is never
    // on), CAN timing, and K-line parameters that cannot apply without a
    // K-line transceiver.
    {"M", AtShape::Bit, ok, nullptr, false},
    {"CSM", AtShape::Bit, ok, nullptr, false},
    {"CEA", AtShape::None, ok, nullptr, false},
    {"CV", AtShape::Dec4, ok, nullptr, false},
    {"IB", AtShape::Hex2, ok, nullptr, false},
    {"SW", AtShape::Hex2, ok, nullptr, false},
    {"IIA", AtShape::Hex2, ok, nullptr, false},
    {"WM", AtShape::Bytes1to6, ok, nullptr, false},
    {"KW", AtShape::None, ok, nullptr, false},
    {"KW", AtShape::Bit, ok, nullptr, false},
    {"BI", AtShape::None, ok, nullptr, false},
    {"SS", AtShape::None, ok, nullptr, false},
    {"SI", AtShape::None, AtTable::busInitError, nullptr, false},
    {"FI", AtShape::None, AtTable::busInitError, nullptr, false},

    {"#GUARD", AtShape::None,
     [](Elm &e, const AtArgs &) {
       e.print(tierName(e.guard_.tier()));
       return AtReply::Printed;
     },
     nullptr, false},
    // WRITE can be unlocked over USB only: the TCP link is open to anyone
    // with the access point's password.
    {"#GUARD", AtShape::TierWord,
     [](Elm &e, const AtArgs &a) {
       if (a.tier == Tier::Write && !e.hal_.linkTrusted()) return AtReply::Bad;
       e.guard_.setTier(a.tier);
       return AtReply::Ok;
     },
     nullptr, false},
    {"#WIFI", AtShape::None,
     [](Elm &e, const AtArgs &) {
       char line[64];
       for (uint8_t i = 0; e.hal_.wifiStatus(i, line, sizeof line); ++i) {
         e.print(line);
       }
       return AtReply::Printed;
     },
     nullptr, false},
    {"#SURVEY", AtShape::None,
     [](Elm &e, const AtArgs &) {
       e.survey();
       return AtReply::Printed;
     },
     nullptr, false},
    {"#SELFTEST", AtShape::None,
     [](Elm &e, const AtArgs &) {
       e.selfTest();
       return AtReply::Printed;
     },
     nullptr, false},
};

const uint8_t AtTable::rowCount = sizeof AtTable::rows / sizeof AtTable::rows[0];

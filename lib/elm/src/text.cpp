#include "text.h"

static const char kHex[] = "0123456789ABCDEF";

static int hexValue(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  return -1;
}

// ------------------------------------------------------------------- input

void LineBuffer::push(uint8_t raw) {
  if (raw == ' ' || raw < 0x20) return;
  if (raw >= 0x7F || len_ >= kLineMax) {
    invalid_ = true;
    return;
  }
  char c = static_cast<char>(raw);
  if (c >= 'a' && c <= 'z') c = static_cast<char>(c - 'a' + 'A');
  buf_[len_++] = c;
  buf_[len_] = '\0';
}

void LineBuffer::clear() {
  len_ = 0;
  buf_[0] = '\0';
  invalid_ = false;
}

bool parseHexRequest(const char *line, bool caf, HexRequest *out) {
  uint8_t n = 0;
  for (; line[n] != '\0'; ++n) {
    if (hexValue(line[n]) < 0 || n >= 17) return false;
  }
  out->maxResponses = -1;
  if (n % 2 == 1) {
    if (n == 1) return false;
    out->maxResponses = static_cast<int8_t>(hexValue(line[n - 1]));
    --n;
  }
  const uint8_t bytes = n / 2;
  if (bytes == 0 || bytes > (caf ? 7 : 8)) return false;
  for (uint8_t i = 0; i < bytes; ++i) {
    out->bytes[i] = static_cast<uint8_t>((hexValue(line[2 * i]) << 4) |
                                         hexValue(line[2 * i + 1]));
  }
  out->len = bytes;
  return true;
}

// ------------------------------------------------------------------ output

namespace {

class LineWriter {
 public:
  LineWriter(TextLine &line, bool spaces) : line_(line), spaces_(spaces) {
    line_.len = 0;
    line_.text[0] = '\0';
  }
  void raw(const char *s) {
    while (*s) put(*s++);
  }
  void separator() {
    if (spaces_ && line_.len > 0) put(' ');
  }
  void byte(uint8_t b) {
    separator();
    put(kHex[b >> 4]);
    put(kHex[b & 0x0F]);
  }
  void digit(uint8_t v) {
    separator();
    put(kHex[v & 0x0F]);
  }
  void bytes(const uint8_t *data, uint8_t n) {
    for (uint8_t i = 0; i < n; ++i) byte(data[i]);
  }
  void id(CanId id) {
    if (id.isExt()) {
      const uint32_t v = id.value();
      for (int shift = 24; shift >= 0; shift -= 8) {
        byte(static_cast<uint8_t>(v >> shift));
      }
    } else {
      const uint32_t v = id.value();
      separator();
      put(kHex[(v >> 8) & 0x07]);
      put(kHex[(v >> 4) & 0x0F]);
      put(kHex[v & 0x0F]);
    }
  }

 private:
  void put(char c) {
    if (line_.len + 1 < static_cast<uint8_t>(sizeof line_.text)) {
      line_.text[line_.len++] = c;
      line_.text[line_.len] = '\0';
    }
  }
  TextLine &line_;
  bool spaces_;
};

}  // namespace

bool shownAsResponse(const Settings &s, const CanFrame &f) {
  if (!s.caf || s.headers) return true;
  switch (pciOf(f)) {
    case Pci::Single: return singleFrameLength(f) != 0;
    case Pci::First: return f.dlc >= 2;
    case Pci::Consecutive: return true;
    default: return false;
  }
}

uint16_t *FrameFormatter::remainingFor(CanId id, bool create) {
  for (uint8_t i = 0; i < remainingCount_; ++i) {
    if (remaining_[i].id == id) return &remaining_[i].bytes;
  }
  if (!create) return nullptr;
  const uint8_t slot = remainingCount_ < 4 ? remainingCount_++ : 3;
  remaining_[slot] = {id, 0};
  return &remaining_[slot].bytes;
}

uint8_t FrameFormatter::format(const CanFrame &f, TextLine out[2]) {
  LineWriter w(out[0], s_.spaces);
  if (!s_.caf || s_.headers) {
    if (s_.headers) {
      w.id(f.id);
      if (s_.dlc) w.digit(f.dlc);
    }
    w.bytes(f.data, f.dlc);
    return 1;
  }
  switch (pciOf(f)) {
    case Pci::Single: {
      const uint8_t len = singleFrameLength(f);
      if (len == 0) break;
      w.bytes(f.data + 1, len);
      return 1;
    }
    case Pci::First: {
      if (f.dlc < 2) break;
      const uint16_t total = firstFrameLength(f);
      const char length[4] = {kHex[(total >> 8) & 0x0F],
                              kHex[(total >> 4) & 0x0F], kHex[total & 0x0F],
                              '\0'};
      w.raw(length);
      uint8_t shown = static_cast<uint8_t>(f.dlc - 2);
      if (shown > total) shown = static_cast<uint8_t>(total);
      *remainingFor(f.id, true) = static_cast<uint16_t>(total - shown);
      LineWriter w2(out[1], s_.spaces);
      w2.raw("0:");
      w2.bytes(f.data + 2, shown);
      return 2;
    }
    case Pci::Consecutive: {
      uint8_t shown = static_cast<uint8_t>(f.dlc - 1);
      uint16_t *left = remainingFor(f.id, false);
      if (left != nullptr) {
        if (*left < shown) shown = static_cast<uint8_t>(*left);
        *left = static_cast<uint16_t>(*left - shown);
      }
      const char label[3] = {kHex[f.data[0] & 0x0F], ':', '\0'};
      w.raw(label);
      w.bytes(f.data + 1, shown);
      return 1;
    }
    case Pci::FlowControl:
      if (!monitor_) return 0;
      w.raw("FC:");
      w.bytes(f.data, f.dlc);
      return 1;
    case Pci::Other:
      break;
  }
  if (!monitor_) return 0;
  w.bytes(f.data, f.dlc);
  w.raw(" <DATA ERROR");
  return 1;
}

const char *lineEnd(const Settings &s) { return s.linefeeds ? "\r\n" : "\r"; }

const char *protocolName(Protocol p) {
  switch (p) {
    case Protocol::None: return nullptr;
    case Protocol::J1850Pwm: return "SAE J1850 PWM";
    case Protocol::J1850Vpw: return "SAE J1850 VPW";
    case Protocol::Iso9141: return "ISO 9141-2";
    case Protocol::Kwp5Baud: return "ISO 14230-4 (KWP 5BAUD)";
    case Protocol::KwpFast: return "ISO 14230-4 (KWP FAST)";
    case Protocol::Can11_500: return "ISO 15765-4 (CAN 11/500)";
    case Protocol::Can29_500: return "ISO 15765-4 (CAN 29/500)";
    case Protocol::Can11_250: return "ISO 15765-4 (CAN 11/250)";
    case Protocol::Can29_250: return "ISO 15765-4 (CAN 29/250)";
  }
  return nullptr;
}

void formatDpn(ProtocolSel sel, char out[3]) {
  const char digit = static_cast<char>('0' + static_cast<uint8_t>(sel.current));
  uint8_t n = 0;
  if (sel.automatic) out[n++] = 'A';
  out[n++] = digit;
  out[n] = '\0';
}

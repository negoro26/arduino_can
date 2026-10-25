#ifndef ELM_SETTINGS_H
#define ELM_SETTINGS_H

#include "can_types.h"

// AT SH / AT FC SH value as typed. Interpretation is deferred to headerId(),
// which knows the protocol width:
//   3 hex digits -> 11-bit id (29-bit protocols: AT CP << 24 | bits)
//   6 hex digits -> AT CP << 24 | bits (pyren: AT CP 18 + AT SH DA5DF2)
//   8 hex digits -> the full 29-bit id (AT CP ignored)
struct TxHeader {
  uint32_t bits = 0;
  uint8_t digits = 0;  // 0 = unset
};

enum class RxFilterMode : uint8_t {
  Auto,
  Pattern,
};

struct RxFilter {
  RxFilterMode mode = RxFilterMode::Auto;
  uint32_t id = 0;
  uint32_t mask = 0;
};

struct FlowControl {
  uint8_t mode = 0;
  TxHeader header;
  uint8_t data[5] = {};
  uint8_t dataLen = 0;
};

struct Settings {
  bool echo = true;
  bool linefeeds = false;
  bool spaces = true;
  bool headers = false;
  bool dlc = false;
  bool responses = true;
  bool caf = true;
  bool cfc = true;
  bool allowLong = false;
  bool varDlc = false;
  uint8_t adaptive = 1;
  uint8_t stTicks = 0x32;    // AT ST, 4 ms units
  uint8_t canPriority = 0x18;  // AT CP: top 5 bits of a 29-bit id
  ProtocolSel protocol{Protocol::None, true};
  TxHeader header;
  RxFilter rx;
  FlowControl fc;
};

CanId headerId(const TxHeader &h, uint8_t canPriority, Protocol p);

// Id a request goes out on: AT SH if set, else 7DF / 18DB33F1 (functional).
CanId requestId(const Settings &s, Protocol p);

// Receive acceptance for request/response. Auto: 7E8/7F8 or 18DAF100/1FFFFF00
// by protocol width. Pattern: rx.id/rx.mask at that width.
Acceptance receiveFilter(const Settings &s, Protocol p);

// The exact frame for a request. CAF1: PCI byte + bytes (len <= 7). CAF0: the
// bytes as given (len <= 8). Padded to 8 with 00 unless V1.
CanFrame requestFrame(const Settings &s, Protocol p, const uint8_t *bytes,
                      uint8_t len);

// Automatic flow control answering `firstFrame` (CFC1, CAF0 and CAF1 alike).
//   SM0: id = responder-8 for 7E8..7EF, 18DAxxF1 for 18DAF1xx, else
//        requestId(); data 30 00 00
//   SM1: id = fc.header, data = fc.data
//   SM2: SM0's id, data = fc.data
// Padded to 8 unless V1. Precondition (enforced by the AT FC SM handler):
// SM1/SM2 have their inputs.
CanFrame flowControlFrame(const Settings &s, Protocol p,
                          const CanFrame &firstFrame);

uint16_t firstWaitMs(const Settings &s);
uint16_t nextWaitMs(const Settings &s);
constexpr uint16_t kPendingWaitMs = 5000;

#endif  // ELM_SETTINGS_H

#ifndef ELM_TEXT_H
#define ELM_TEXT_H

#include "can_types.h"
#include "settings.h"

constexpr uint8_t kLineMax = 64;

class LineBuffer {
 public:
  // Precondition: raw != '\r' and raw != '\n' (Elm handles those).
  //   ' ', 0x00-0x1F   -> dropped
  //   0x7F, >= 0x80    -> line becomes invalid (python-OBD's "\x7F\x7F\r"
  //                       baud probe must get "?", never a repeat)
  //   beyond kLineMax  -> line becomes invalid
  void push(uint8_t raw);
  void clear();
  bool blank() const { return len_ == 0 && !invalid_; }
  bool valid() const { return !invalid_; }
  const char *text() const { return buf_; }

 private:
  char buf_[kLineMax + 1] = {};
  uint8_t len_ = 0;
  bool invalid_ = false;
};

struct HexRequest {
  uint8_t bytes[8];
  uint8_t len;          // CAF1: 1..7 (PCI added later); CAF0: 1..8
  int8_t maxResponses;
};

// false -> print "?". Hex digits only; an odd count means the last digit is
// the response count; byte count within the CAF limit above.
bool parseHexRequest(const char *line, bool caf, HexRequest *out);

struct TextLine {
  char text[48];
  uint8_t len;
};

// CAF1 with H0 hides the PCI and shows only ISO-TP payload. Frames it cannot
// show that way (flow control, invalid PCI) are not responses: they do not
// count toward the response digit and do not prevent NO DATA.
bool shownAsResponse(const Settings &s, const CanFrame &f);

// Formats received frames. Stateful only for CAF1 + H0, where the padding of
// the last consecutive frame is trimmed using the first frame's length
// (tracked per responder id).
//   CAF0 or H1 : [id ][dlc ]all dlc bytes, PCI included, as received
//                id: "7E8" (11-bit) or "18 DA F1 10" (29-bit); dlc if D1
//   CAF1 and H0: SF -> payload bytes only
//                FF -> "LLL" then "0: " + up to 6 bytes   (2 lines)
//                CF -> "N: " + up to 7 bytes, N = sequence nibble
//                FC / invalid PCI -> nothing, or in monitor mode
//                "FC: " + bytes / bytes + " <DATA ERROR"
//   S0 removes every space ("0:490201...").
// No trailing space. Line ending is NOT included.
class FrameFormatter {
 public:
  FrameFormatter(const Settings &s, bool monitor) : s_(s), monitor_(monitor) {}
  uint8_t format(const CanFrame &f, TextLine out[2]);

 private:
  struct Remaining {
    CanId id;
    uint16_t bytes;
  };
  uint16_t *remainingFor(CanId id, bool create);

  const Settings &s_;
  bool monitor_;
  Remaining remaining_[4] = {};
  uint8_t remainingCount_ = 0;
};

const char *lineEnd(const Settings &s);

// "ISO 15765-4 (CAN 11/500)" ... ; nullptr for Protocol::None.
const char *protocolName(Protocol p);
// AT DPN: "6", "A6", "A0" (auto, not found yet).
void formatDpn(ProtocolSel sel, char out[3]);

#endif  // ELM_TEXT_H

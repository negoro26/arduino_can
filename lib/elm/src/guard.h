// The device-level write guard and the single transmit choke point.
//
// BusSide::canSend() accepts only a ClearedFrame, and only Guard::admit() can
// put bytes into one. Requests in both CAF modes, automatic flow control and
// the self-test loopback frame all reach the controller through admit().
//
// The guard classifies the EXACT bytes that will be on the wire, with data[0]
// read as the ISO-TP PCI. CAF1 frames are built (PCI included) before they are
// admitted, so one rule covers CAF0 and CAF1.

#ifndef ELM_GUARD_H
#define ELM_GUARD_H

#include "can_types.h"

// Ordered: a frame is admitted iff its tier <= the guard tier.
enum class Tier : uint8_t { Read = 0, Clear = 1, Write = 2 };

const char *tierName(Tier t);                 // "READ" / "CLEAR" / "WRITE"
bool parseTier(const char *word, Tier *out);  // exact, already uppercased

// Default-deny policy: anything not listed as READ or CLEAR is WRITE.
//   Read : 01 02 03 05 06 07 09 0A; 10 except sub 02/82/85; 12 17 18 19 1A
//          20 21 22 23 24 3E 81 82 83
//   Clear: 04, 14
Tier tierOfService(uint8_t sid, bool hasSub, uint8_t sub);

// Proof that a frame passed the guard. Non-copyable so it is used where it is
// made (admit -> canSend) rather than cached and replayed after the tier drops.
class ClearedFrame {
 public:
  ClearedFrame() = default;  // empty: every adapter refuses to send it
  ClearedFrame(const ClearedFrame &) = delete;
  ClearedFrame &operator=(const ClearedFrame &) = delete;

  bool valid() const { return valid_; }
  const CanFrame &frame() const { return frame_; }

 private:
  friend class Guard;
  CanFrame frame_{};
  bool valid_ = false;
};

// Lives in Elm next to (not inside) Settings: ATZ/ATWS/ATD cannot reset it;
// only power-up (construction) and AT#GUARD change the tier.
class Guard {
 public:
  Tier tier() const { return tier_; }
  void setTier(Tier t) { tier_ = t; }

  // Tier of one wire frame:
  //   PCI 3x (flow control) -> Read (always admitted)
  //   PCI 0x (single frame) -> tierOfService(data[1], len >= 2, data[2])
  //   PCI 1x (first frame)  -> tierOfService(data[2], true, data[3])
  //   PCI 2x (consecutive)  -> the tier of the last first frame on the same
  //                            id, else Write (no first frame -> deny)
  //   anything else         -> Write
  Tier classify(const CanFrame &f) const;
  bool permits(const CanFrame &f) const { return classify(f) <= tier_; }

  // THE choke point. Fills *out iff `f` may be transmitted. A first frame is
  // remembered (even when refused) so its consecutive frames inherit its
  // verdict; the comparison happens at admit time, so a CF sent after
  // `AT#GUARD READ` is judged against the new, lower tier.
  bool admit(const CanFrame &f, ClearedFrame *out);

 private:
  Tier tier_ = Tier::Read;  // power-up default
  CanId lastFirstId_;
  Tier lastFirstTier_ = Tier::Write;
  bool haveFirst_ = false;
};

#endif  // ELM_GUARD_H

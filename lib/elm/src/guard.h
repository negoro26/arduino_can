// The device-level write guard and the single transmit choke point.
//
// BusSide::canSend() accepts only a ClearedFrame, and only Guard::admit() can
// put bytes into one. Requests in both CAF modes, automatic flow control and
// the self-test loopback frame all reach the controller through admit().
//
// The guard classifies the EXACT bytes that will be on the wire, with data[0]
// read as the ISO-TP PCI. CAF1 frames are built (PCI included) before they are
// admitted, so one rule covers CAF0 and CAF1. Below WRITE only diagnostic ids
// pass at all: 11-bit 0x600-0x7FF, and 29-bit 18DAxxxx/18DBxxxx (ISO 15765-4
// normal fixed addressing).

#ifndef ELM_GUARD_H
#define ELM_GUARD_H

#include "can_types.h"

enum class Tier : uint8_t { Read = 0, Clear = 1, Write = 2 };

const char *tierName(Tier t);
bool parseTier(const char *word, Tier *out);  // exact, already uppercased

Tier tierOfService(uint8_t sid, bool hasSub, uint8_t sub);

// Proof that a frame passed the guard. Non-copyable so it is used where it is
// made (admit -> canSend) rather than cached and replayed after the tier drops.
class ClearedFrame {
 public:
  ClearedFrame() = default;
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

  Tier classify(const CanFrame &f) const;
  bool permits(const CanFrame &f) const { return classify(f) <= tier_; }

  // THE choke point. Fills *out iff `f` may be transmitted. A first frame is
  // remembered (even when refused) so its consecutive frames inherit its
  // verdict; the comparison happens at admit time, so a CF sent after
  // `AT#GUARD READ` is judged against the new, lower tier.
  bool admit(const CanFrame &f, ClearedFrame *out);

 private:
  Tier tier_ = Tier::Read;
  CanId lastFirstId_;
  Tier lastFirstTier_ = Tier::Write;
  bool haveFirst_ = false;
};

#endif  // ELM_GUARD_H

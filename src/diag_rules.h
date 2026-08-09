// Pure decision logic for the CAN probe, deliberately free of Arduino, SPI and
// MCP2515 dependencies so it compiles unchanged on the SAMD21 and on a host for
// unit testing (`pio test -e native`).
//
// The two rules here are the ones with real consequences:
//   - SeenIds backs the stage 4 safety interlock. If it ever reported "not
//     seen" for an ID a module is actually using, the sweep would transmit on
//     top of live operational traffic.
//   - looksLikeDiagReply decides what counts as an ECU answering. Too loose and
//     ordinary vehicle traffic is reported as a discovered ECU; too strict and
//     real ECUs are missed.

#ifndef DIAG_RULES_H
#define DIAG_RULES_H

#include <stddef.h>
#include <stdint.h>

// Diagnostic request IDs live in this range by convention. Operational traffic
// normally sits below it, but the interlock never relies on that -- it excludes
// whatever the car is measured to be using.
static const uint32_t DIAG_ID_FIRST = 0x700;
static const uint32_t DIAG_ID_LAST = 0x7FF;

inline bool isDiagRequestId(uint32_t id) {
  return id >= DIAG_ID_FIRST && id <= DIAG_ID_LAST;
}

// A UDS reply is recognised by structure rather than by CAN ID, so unrelated
// vehicle traffic is not mistaken for an answer. data[0] is the ISO-TP PCI
// byte, data[1] the service response:
//   0x50  positive response to 0x10 StartDiagnosticSession
//   0x7F  negative response -- the ECU refused, but it is present
//   0x61  positive response to 0x21 ReadDataByLocalIdentifier
inline bool looksLikeDiagReply(uint8_t dlc, const uint8_t *data) {
  if (data == NULL || dlc < 2) {
    return false;
  }
  const uint8_t sid = data[1];
  return sid == 0x50 || sid == 0x7F || sid == 0x61;
}

// Fixed-capacity set of CAN IDs observed during the passive survey. Fixed
// capacity because this runs on a 32 KB part and must never allocate; once full
// it stops recording, which callers can detect with isFull().
template <uint8_t CAPACITY>
class SeenIds {
 public:
  void clear() { count_ = 0; }

  bool contains(uint32_t id) const {
    for (uint8_t i = 0; i < count_; i++) {
      if (ids_[i] == id) {
        return true;
      }
    }
    return false;
  }

  // Returns true if the id was newly recorded. Duplicates and overflow both
  // return false; overflow is deliberately silent-but-detectable rather than
  // overwriting an earlier entry, because forgetting an in-use ID would weaken
  // the interlock.
  bool add(uint32_t id) {
    if (contains(id) || count_ >= CAPACITY) {
      return false;
    }
    ids_[count_++] = id;
    return true;
  }

  uint8_t count() const { return count_; }
  bool isFull() const { return count_ >= CAPACITY; }
  uint32_t at(uint8_t i) const { return ids_[i]; }
  static uint8_t capacity() { return CAPACITY; }

  // How many recorded ids fall inside the diagnostic request range, i.e. how
  // many the active sweep will have to skip.
  uint8_t diagRangeOverlap() const {
    uint8_t n = 0;
    for (uint8_t i = 0; i < count_; i++) {
      if (isDiagRequestId(ids_[i])) {
        n++;
      }
    }
    return n;
  }

 private:
  uint32_t ids_[CAPACITY];
  uint8_t count_ = 0;
};

#endif  // DIAG_RULES_H

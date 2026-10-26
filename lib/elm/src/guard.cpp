#include "guard.h"

#include <string.h>

const char *tierName(Tier t) {
  switch (t) {
    case Tier::Read: return "READ";
    case Tier::Clear: return "CLEAR";
    case Tier::Write: return "WRITE";
  }
  return "WRITE";
}

bool parseTier(const char *word, Tier *out) {
  static const Tier kTiers[] = {Tier::Read, Tier::Clear, Tier::Write};
  for (Tier t : kTiers) {
    if (strcmp(word, tierName(t)) == 0) {
      *out = t;
      return true;
    }
  }
  return false;
}

Tier tierOfService(uint8_t sid, bool hasSub, uint8_t sub) {
  switch (sid) {
    case 0x10: {
      // Programming sessions (UDS 02, or 82 with suppress-positive-response;
      // KWP2000 85) are the entry point to reflashing.
      const bool programming =
          hasSub && (sub == 0x02 || sub == 0x82 || sub == 0x85);
      return programming ? Tier::Write : Tier::Read;
    }
    case 0x01: case 0x02: case 0x03: case 0x05: case 0x06: case 0x07:
    case 0x09: case 0x0A:
    case 0x12: case 0x17: case 0x18: case 0x19: case 0x1A:
    case 0x20: case 0x21: case 0x22: case 0x23: case 0x24:
    case 0x3E: case 0x81: case 0x82: case 0x83:
      return Tier::Read;
    case 0x04: case 0x14:
      return Tier::Clear;
    default:
      return Tier::Write;
  }
}

static bool isDiagnosticId(CanId id) {
  if (!id.isExt()) return id.value() >= 0x600;
  const uint32_t top = id.value() >> 16;
  return top == 0x18DA || top == 0x18DB;
}

Tier Guard::classify(const CanFrame &f) const {
  // A raw frame on an operational id can drive the car whatever its PCI byte
  // looks like.
  if (!isDiagnosticId(f.id)) return Tier::Write;
  switch (pciOf(f)) {
    case Pci::FlowControl:
      return Tier::Read;
    case Pci::Single: {
      const uint8_t len = singleFrameLength(f);
      if (len == 0) return Tier::Write;
      return tierOfService(f.data[1], len >= 2, f.data[2]);
    }
    case Pci::First:
      if (f.dlc < 3) return Tier::Write;
      return tierOfService(f.data[2], f.dlc >= 4, f.data[3]);
    case Pci::Consecutive:
      return (haveFirst_ && lastFirstId_ == f.id) ? lastFirstTier_
                                                   : Tier::Write;
    case Pci::Other:
      break;
  }
  return Tier::Write;
}

bool Guard::admit(const CanFrame &f, ClearedFrame *out) {
  const Tier t = classify(f);
  const Pci pci = pciOf(f);
  if (pci == Pci::First) {
    haveFirst_ = true;
    lastFirstId_ = f.id;
    lastFirstTier_ = t;
  } else if (pci == Pci::Single && haveFirst_ && lastFirstId_ == f.id) {
    haveFirst_ = false;
  }
  if (t > tier_) return false;
  out->frame_ = f;
  out->valid_ = true;
  return true;
}

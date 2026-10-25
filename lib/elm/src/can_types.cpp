#include "can_types.h"

bool sameFrame(const CanFrame &a, const CanFrame &b) {
  if (a.id != b.id || a.dlc != b.dlc) return false;
  for (uint8_t i = 0; i < a.dlc; ++i) {
    if (a.data[i] != b.data[i]) return false;
  }
  return true;
}

Pci pciOf(const CanFrame &f) {
  if (f.dlc == 0) return Pci::Other;
  switch (f.data[0] >> 4) {
    case 0x0: return Pci::Single;
    case 0x1: return Pci::First;
    case 0x2: return Pci::Consecutive;
    case 0x3: return Pci::FlowControl;
    default: return Pci::Other;
  }
}

uint8_t singleFrameLength(const CanFrame &f) {
  if (pciOf(f) != Pci::Single) return 0;
  const uint8_t len = f.data[0] & 0x0F;
  return (len >= 1 && len + 1 <= f.dlc) ? len : 0;
}

uint16_t firstFrameLength(const CanFrame &f) {
  if (pciOf(f) != Pci::First || f.dlc < 2) return 0;
  return static_cast<uint16_t>(((f.data[0] & 0x0F) << 8) | f.data[1]);
}

bool isResponsePending(const CanFrame &f) {
  return singleFrameLength(f) == 3 && f.data[1] == 0x7F && f.data[3] == 0x78;
}

uint16_t bitrateKbit(Bitrate r) {
  switch (r) {
    case Bitrate::k500: return 500;
    case Bitrate::k250: return 250;
    case Bitrate::k125: return 125;
    case Bitrate::k100: return 100;
  }
  return 0;
}

bool Acceptance::accepts(CanId frameId) const {
  return any ||
         (frameId.isExt() == ext && ((frameId.value() ^ id) & mask) == 0);
}

bool Acceptance::operator==(const Acceptance &o) const {
  if (any || o.any) return any == o.any;
  return ext == o.ext && id == o.id && mask == o.mask;
}

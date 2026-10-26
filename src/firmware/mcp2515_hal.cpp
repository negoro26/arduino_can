#include "mcp2515_hal.h"

#include <SPI.h>
#include <string.h>

#include "tx_attempt.h"

namespace {

// Shield facts (ASX00005 schematic): CS = D3, 16 MHz crystal.
constexpr uint8_t kPinCs = 3;
// 8 MHz rather than the library's 10 MHz default: 10 MHz is the MCP2515
// datasheet maximum, and there is no reason to sit on the spec edge.
constexpr uint32_t kSpiHz = 8000000;

constexpr uint8_t kSpiRead = 0x03;
constexpr uint8_t kSpiBitModify = 0x05;
constexpr uint8_t kSpiReadStatus = 0xA0;

constexpr uint8_t kRegCanstat = 0x0E;
constexpr uint8_t kRegCanctrl = 0x0F;
constexpr uint8_t kRegTec = 0x1C;
constexpr uint8_t kRegRec = 0x1D;
constexpr uint8_t kRegCanintf = 0x2C;
constexpr uint8_t kRegEflg = 0x2D;
constexpr uint8_t kRegTxb0ctrl = 0x30;

constexpr uint8_t kIntfRx0 = 0x01;
constexpr uint8_t kIntfRx1 = 0x02;
constexpr uint8_t kIntfErr = 0x20;
constexpr uint8_t kEflgRxOverflow = 0xC0;  // RX1OVR | RX0OVR

constexpr uint8_t kTxbTxreq = 0x08;

constexpr uint8_t kCtrlReqopMask = 0xE0;
constexpr uint8_t kCtrlOsm = 0x08;
constexpr uint8_t kReqopNormal = 0x00;
constexpr uint8_t kReqopLoopback = 0x40;
constexpr uint8_t kReqopListenOnly = 0x60;
constexpr uint8_t kReqopConfig = 0x80;

// Waiting for bus idle plus one frame at 250 kbit fits easily; a frame still
// pending after this is aborted and reported as failed.
constexpr uint32_t kTxTimeoutUs = 10000;
constexpr uint32_t kModeChangeTimeoutMs = 10;

CAN_SPEED speedOf(Bitrate r) {
  switch (r) {
    case Bitrate::k500: return CAN_500KBPS;
    case Bitrate::k250: return CAN_250KBPS;
    case Bitrate::k125: return CAN_125KBPS;
    case Bitrate::k100: return CAN_100KBPS;
  }
  return CAN_500KBPS;
}

const SPISettings kSpi(kSpiHz, MSBFIRST, SPI_MODE0);

}  // namespace

Mcp2515Hal::Mcp2515Hal() : mcp_(kPinCs, kSpiHz, &SPI) {}

void Mcp2515Hal::begin() {
  Serial.begin(115200);  // native USB: the rate is ignored
  pinMode(kPinCs, OUTPUT);
  digitalWrite(kPinCs, HIGH);
  SPI.begin();
  mcp_.reset();
  canClose();
}

int Mcp2515Hal::readByte() { return Serial.available() > 0 ? Serial.read() : -1; }

void Mcp2515Hal::write(const char *bytes, uint16_t length) {
  Serial.write(reinterpret_cast<const uint8_t *>(bytes), length);
}

uint32_t Mcp2515Hal::nowMs() { return millis(); }

void Mcp2515Hal::idleUntil(Deadline) {}

uint8_t Mcp2515Hal::rawRead(uint8_t reg) {
  SPI.beginTransaction(kSpi);
  digitalWrite(kPinCs, LOW);
  SPI.transfer(kSpiRead);
  SPI.transfer(reg);
  const uint8_t v = SPI.transfer(0x00);
  digitalWrite(kPinCs, HIGH);
  SPI.endTransaction();
  return v;
}

void Mcp2515Hal::rawModify(uint8_t reg, uint8_t mask, uint8_t value) {
  SPI.beginTransaction(kSpi);
  digitalWrite(kPinCs, LOW);
  SPI.transfer(kSpiBitModify);
  SPI.transfer(reg);
  SPI.transfer(mask);
  SPI.transfer(value);
  digitalWrite(kPinCs, HIGH);
  SPI.endTransaction();
}

uint8_t Mcp2515Hal::readStatus() {
  SPI.beginTransaction(kSpi);
  digitalWrite(kPinCs, LOW);
  SPI.transfer(kSpiReadStatus);
  const uint8_t v = SPI.transfer(0x00);
  digitalWrite(kPinCs, HIGH);
  SPI.endTransaction();
  return v;
}

bool Mcp2515Hal::setMode(uint8_t reqop) {
  rawModify(kRegCanctrl, kCtrlReqopMask | kCtrlOsm, reqop);
  const uint32_t start = millis();
  while ((rawRead(kRegCanstat) & kCtrlReqopMask) != (reqop & kCtrlReqopMask)) {
    if (millis() - start > kModeChangeTimeoutMs) return false;
  }
  // One-shot is a safety property, so it is read back rather than assumed.
  return (rawRead(kRegCanctrl) & kCtrlOsm) == (reqop & kCtrlOsm);
}

bool Mcp2515Hal::configure(Bitrate rate, const Acceptance &acc) {
  if (mcp_.reset() != MCP2515::ERROR_OK) return false;
  if (mcp_.setBitrate(speedOf(rate), MCP_16MHZ) != MCP2515::ERROR_OK) {
    return false;
  }
  order_.reset();
  // After reset() every filter and mask is zero: both widths are accepted.
  if (acc.any) return true;
  const MCP2515::MASK masks[] = {MCP2515::MASK0, MCP2515::MASK1};
  for (MCP2515::MASK m : masks) {
    if (mcp_.setFilterMask(m, acc.ext, acc.mask) != MCP2515::ERROR_OK) {
      return false;
    }
  }
  const MCP2515::RXF filters[] = {MCP2515::RXF0, MCP2515::RXF1, MCP2515::RXF2,
                                  MCP2515::RXF3, MCP2515::RXF4, MCP2515::RXF5};
  for (MCP2515::RXF f : filters) {
    if (mcp_.setFilter(f, acc.ext, acc.id) != MCP2515::ERROR_OK) return false;
  }
  return true;
}

bool Mcp2515Hal::canOpen(Bitrate rate, CanMode mode, const Acceptance &acc) {
  if (!open_ || rate != rate_ || !(acc == acc_)) {
    open_ = false;
    if (!configure(rate, acc)) return false;
    rate_ = rate;
    acc_ = acc;
  }
  health_ = CanHealth{};
  rawModify(kRegEflg, kEflgRxOverflow, 0);
  rawModify(kRegCanintf, kIntfErr | kIntfMerr, 0);
  uint8_t reqop = kReqopListenOnly;
  if (mode == CanMode::Normal) reqop = kReqopNormal | kCtrlOsm;
  if (mode == CanMode::Loopback) reqop = kReqopLoopback | kCtrlOsm;
  open_ = setMode(reqop);
  return open_;
}

void Mcp2515Hal::canClose() {
  setMode(kReqopConfig);
  open_ = false;
}

TxStatus Mcp2515Hal::canSend(const ClearedFrame &frame) {
  if (!frame.valid()) return TxStatus::Failed;
  return sendRaw(frame.frame());
}

TxStatus Mcp2515Hal::sendRaw(const CanFrame &f) {
  struct can_frame tx;
  tx.can_id = f.id.isExt() ? (f.id.value() | CAN_EFF_FLAG) : f.id.value();
  tx.can_dlc = f.dlc;
  memcpy(tx.data, f.data, sizeof tx.data);
  for (uint8_t attempt = 1;; ++attempt) {
    // Clears (and counts) any earlier MERRF, so the read after the attempt
    // shows only errors raised during it.
    serviceErrors(rawRead(kRegCanintf));
    const uint8_t recBefore = rawRead(kRegRec);
    // Setting TXREQ clears ABTF/MLOA/TXERR, so the library's immediate flag
    // check is meaningless; completion is polled below instead.
    mcp_.sendMessage(MCP2515::TXB0, &tx);
    const uint32_t start = micros();
    uint8_t ctrl = rawRead(kRegTxb0ctrl);
    while ((ctrl & kTxbTxreq) && micros() - start < kTxTimeoutUs) {
      ctrl = rawRead(kRegTxb0ctrl);
    }
    if (ctrl & kTxbTxreq) {
      rawModify(kRegTxb0ctrl, kTxbTxreq, 0);
      return TxStatus::Failed;
    }
    const uint8_t recAfter = rawRead(kRegRec);
    const uint8_t intf = rawRead(kRegCanintf);
    switch (judgeTxAttempt(ctrl, recBefore, recAfter, intf, attempt)) {
      case TxVerdict::Sent: return TxStatus::Sent;
      case TxVerdict::Failed: return TxStatus::Failed;
      case TxVerdict::Retry: break;
    }
  }
}

void Mcp2515Hal::serviceErrors(uint8_t intf) {
  if (!(intf & (kIntfErr | kIntfMerr))) return;
  if (health_.errorEvents < 0xFFFF) ++health_.errorEvents;
  if (rawRead(kRegEflg) & kEflgRxOverflow) {
    health_.rxOverflow = true;
    rawModify(kRegEflg, kEflgRxOverflow, 0);
  }
  rawModify(kRegCanintf, kIntfErr | kIntfMerr, 0);
}

uint8_t Mcp2515Hal::Controller::rxStatus() {
  if (!firstUsed) {
    firstUsed = true;
    return firstStatus;
  }
  return hal.readStatus() & (kRxb0Full | kRxb1Full);
}

bool Mcp2515Hal::Controller::readRxb(uint8_t n, CanFrame &out) {
  struct can_frame rx;
  const MCP2515::RXBn buffer = n == 0 ? MCP2515::RXB0 : MCP2515::RXB1;
  if (hal.mcp_.readMessage(buffer, &rx) != MCP2515::ERROR_OK) {
    // An invalid DLC leaves the buffer flagged; free it or it blocks RX.
    hal.rawModify(kRegCanintf, n == 0 ? kIntfRx0 : kIntfRx1, 0);
    return false;
  }
  if (rx.can_id & CAN_RTR_FLAG) return false;  // remote frames carry no data
  out.id = (rx.can_id & CAN_EFF_FLAG) ? CanId::ext29(rx.can_id & CAN_EFF_MASK)
                                      : CanId::std11(rx.can_id & CAN_SFF_MASK);
  out.dlc = rx.can_dlc;
  memcpy(out.data, rx.data, sizeof out.data);
  return true;
}

bool Mcp2515Hal::canReceive(CanFrame &out) {
  const uint8_t intf = rawRead(kRegCanintf);
  serviceErrors(intf);
  const uint8_t rx = intf & (kIntfRx0 | kIntfRx1);
  if (rx == 0) return false;
  Controller c{*this, rx, false};
  return order_.read(c, out);
}

CanHealth Mcp2515Hal::canHealth() {
  serviceErrors(rawRead(kRegCanintf));
  CanHealth h = health_;
  h.tec = rawRead(kRegTec);
  h.rec = rawRead(kRegRec);
  return h;
}

bool Mcp2515Hal::canProbe(char *detail, uint8_t capacity) {
  open_ = false;
  mcp_.reset();
  const uint8_t canstat = rawRead(kRegCanstat);
  const uint8_t canctrl = rawRead(kRegCanctrl);
  static const char kHex[] = "0123456789ABCDEF";
  const char text[] = {'C', 'A', 'N', 'S', 'T', 'A', 'T', '=', kHex[canstat >> 4],
                       kHex[canstat & 0xF], ' ', 'C', 'A', 'N', 'C', 'T', 'R', 'L',
                       '=', kHex[canctrl >> 4], kHex[canctrl & 0xF], '\0'};
  if (capacity > 0) {
    strncpy(detail, text, capacity - 1);
    detail[capacity - 1] = '\0';
  }
  return canstat == 0x80 && canctrl == 0x87;
}

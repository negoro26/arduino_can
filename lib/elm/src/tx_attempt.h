// Judging one one-shot MCP2515 transmit attempt from the controller registers.
//
// In one-shot mode a lost arbitration sets ABTF as well as MLOA, and a frame
// that hit a bus error can end the same way: with nothing on the CAN pins at
// 500 kbit, TXB0CTRL reads ABTF|MLOA and REC rises by 9 per attempt. The flags
// alone cannot tell the two apart, so a retry also needs REC and MERRF to show
// no bus error during the attempt. Retrying after a bus error would put
// another possibly corrupting frame on the wire.

#ifndef ELM_TX_ATTEMPT_H
#define ELM_TX_ATTEMPT_H

#include <stdint.h>

constexpr uint8_t kTxbAbtf = 0x40;   // TXBnCTRL: transmission aborted
constexpr uint8_t kTxbMloa = 0x20;   // TXBnCTRL: arbitration lost
constexpr uint8_t kTxbTxerr = 0x10;  // TXBnCTRL: bus error while sending
constexpr uint8_t kIntfMerr = 0x80;  // CANINTF.MERRF

constexpr uint8_t kMaxTxAttempts = 3;

enum class TxVerdict : uint8_t { Sent, Retry, Failed };

// txbCtrl: TXBnCTRL once TXREQ has cleared. recBefore/recAfter: REC read just
// before TXREQ was set and after it cleared. canintfAfter: CANINTF after the
// attempt, with MERRF cleared before it. attempts: attempts made, this one
// included.
inline TxVerdict judgeTxAttempt(uint8_t txbCtrl, uint8_t recBefore,
                                uint8_t recAfter, uint8_t canintfAfter,
                                uint8_t attempts) {
  if (!(txbCtrl & (kTxbAbtf | kTxbMloa | kTxbTxerr))) return TxVerdict::Sent;
  const bool cleanLoss = (txbCtrl & kTxbMloa) && !(txbCtrl & kTxbTxerr) &&
                         recAfter <= recBefore && !(canintfAfter & kIntfMerr);
  return cleanLoss && attempts < kMaxTxAttempts ? TxVerdict::Retry
                                                : TxVerdict::Failed;
}

#endif  // ELM_TX_ATTEMPT_H

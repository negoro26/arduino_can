// Reading the MCP2515's two receive buffers in arrival order.
//
// With BUKT rollover a frame goes to RXB0, or to RXB1 when RXB0 is full. So
// after RXB0 is read, a full RXB1 holds a frame older than anything that can
// land in RXB0 from then on, and must be read next.

#ifndef ELM_RX_ORDER_H
#define ELM_RX_ORDER_H

#include "can_types.h"

constexpr uint8_t kRxb0Full = 0x01;  // CANINTF.RX0IF / READ STATUS bit 0
constexpr uint8_t kRxb1Full = 0x02;  // CANINTF.RX1IF / READ STATUS bit 1

class RolloverOrder {
 public:
  // Controller needs: uint8_t rxStatus() (kRxb0Full | kRxb1Full snapshot)
  // and bool readRxb(uint8_t n, CanFrame &out) (reads and frees buffer n).
  template <class Controller>
  bool read(Controller &c, CanFrame &out) {
    const uint8_t first = c.rxStatus();
    if (preferRxb1_ && (first & kRxb1Full)) {
      preferRxb1_ = false;
      return c.readRxb(1, out);
    }
    if (first & kRxb0Full) {
      const bool ok = c.readRxb(0, out);
      // Second snapshot: whatever sits in RXB1 now arrived before anything
      // that can land in the RXB0 we just freed.
      preferRxb1_ = (c.rxStatus() & kRxb1Full) != 0;
      return ok;
    }
    if (first & kRxb1Full) {
      preferRxb1_ = false;
      return c.readRxb(1, out);
    }
    return false;
  }

  void reset() { preferRxb1_ = false; }

 private:
  bool preferRxb1_ = false;
};

#endif  // ELM_RX_ORDER_H

// The firmware Hal: millis() + the MKR CAN shield's MCP2515 + the session
// link, native USB CDC or the WINC1500's TCP link (wifi_link.h) as LinkMux
// picks it.
//
// Every MCP2515 quirk the core must not know about is absorbed here:
//   - TXB0 only. The chip sends the highest-numbered pending buffer first, so
//     several buffers would reorder consecutive frames.
//   - One-shot mode set by a raw CANCTRL write. The library's
//     setNormalOneShotMode() compares CANSTAT.OPMOD against a value that
//     includes the OSM bit, so it always spins 10 ms and returns ERROR_FAIL.
//   - A frame is retried (3 attempts in all) only after a clean arbitration
//     loss, never after a bus error (tx_attempt.h).
//   - RX buffers are read in arrival order under BUKT rollover (rx_order.h).
//   - Registers the library keeps private (CANINTF, EFLG, TXB0CTRL, CANSTAT)
//     are read with raw SPI, as the probe did.
// The INT pin (D7) is not used: polling CANINTF is one 3-byte SPI read
// (a few us against ~230 us between consecutive frames), and it keeps the
// receive path independent of a pin the bring-up probe never exercised.

#ifndef MCP2515_HAL_H
#define MCP2515_HAL_H

#include <Arduino.h>
#include <mcp2515.h>

#include "hal.h"
#include "link_mux.h"
#include "rx_order.h"
#include "wifi_link.h"

class Mcp2515Hal : public Hal {
 public:
  Mcp2515Hal();
  // USB first, then the WiFi access point; USB works whatever WiFi does.
  void begin();
  // Between commands only (see LinkMux::poll).
  void pollLinks();

  int readByte() override;
  void write(const char *bytes, uint16_t length) override;
  void flush() override;
  bool linkTrusted() override;
  uint8_t linkSwitches() override;
  bool wifiStatus(uint8_t index, char *out, uint8_t capacity) override;
  uint32_t nowMs() override;
  void idleUntil(Deadline d) override;

  bool canOpen(Bitrate rate, CanMode mode, const Acceptance &acc) override;
  void canClose() override;
  TxStatus canSend(const ClearedFrame &frame) override;
  bool canReceive(CanFrame &out) override;
  CanHealth canHealth() override;
  bool canProbe(char *detail, uint8_t capacity) override;

 protected:
  // Unguarded TXB0 send. Only LoopbackDemoHal uses it, with the controller
  // forced into loopback mode so nothing reaches the wire.
  TxStatus sendRaw(const CanFrame &f);
  bool isOpen() const { return open_; }

 private:
  class Usb final : public ByteLink {
   public:
    uint16_t read(uint8_t *out, uint16_t capacity) override;
    void write(const char *bytes, uint16_t length) override;
  };

  struct Controller {
    Mcp2515Hal &hal;
    uint8_t firstStatus;
    bool firstUsed;
    uint8_t rxStatus();
    bool readRxb(uint8_t n, CanFrame &out);
  };

  uint8_t rawRead(uint8_t reg);
  void rawModify(uint8_t reg, uint8_t mask, uint8_t value);
  uint8_t readStatus();
  bool setMode(uint8_t reqop);
  bool configure(Bitrate rate, const Acceptance &acc);
  void serviceErrors(uint8_t intf);

  MCP2515 mcp_;
  RolloverOrder order_;
  bool open_ = false;
  Bitrate rate_ = Bitrate::k500;
  Acceptance acc_ = Acceptance::all();
  CanHealth health_{};
  Usb usb_;
  WifiLink wifi_;
  LinkMux links_;
};

#endif  // MCP2515_HAL_H

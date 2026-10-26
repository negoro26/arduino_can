// The WINC1500 as a WiFi ELM327 dongle: WPA/WPA2 access point (the WINC's
// M2M_WIFI_SEC_WPA_PSK offers both), AP at 192.168.0.10, TCP port 35000, one
// client (the WINC admits one station).
//
// WiFi101 AP-mode quirks absorbed here:
//   - server.available() hands out a new client once, then only clients with
//     data, so the accepted WiFiClient is kept.
//   - When the station leaves the AP the library closes no socket, and the
//     firmware may clear the listening one (WiFi101 issue #46): the client is
//     stopped and the server restarted on WL_AP_CONNECTED -> WL_AP_LISTENING.
//   - Reads go in blocks: each read() call pumps WINC events over SPI.

#ifndef WIFI_LINK_H
#define WIFI_LINK_H

#include <WiFi101.h>

#include "link_mux.h"

class WifiLink final : public TcpLink {
 public:
  WifiLink() : server_(kPort) {}

  // Starts the access point. With no WINC, or an AP the firmware refuses, it
  // stays off (status lines say why) and never touches the WINC again.
  void begin();

  bool accept() override;
  bool connected() override;
  uint16_t read(uint8_t *out, uint16_t capacity) override;
  void write(const char *bytes, uint16_t length) override;

  bool statusLine(uint8_t index, char *out, uint8_t capacity);

 private:
  static constexpr uint16_t kPort = 35000;

  WiFiServer server_;
  WiFiClient client_;
  bool on_ = false;
  uint8_t apStatus_ = WL_IDLE_STATUS;
  const char *offReason_ = "not started";
  char firmware_[12] = {};
};

#endif  // WIFI_LINK_H

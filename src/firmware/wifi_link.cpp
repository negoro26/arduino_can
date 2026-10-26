#include "wifi_link.h"

#include <stdio.h>
#include <string.h>

#ifndef ELM_WIFI_SSID
#define ELM_WIFI_SSID "MKR-OBDII"
#endif
#ifndef ELM_WIFI_PASSWORD
#define ELM_WIFI_PASSWORD "obd2mkr1000"
#endif

// The WINC refuses a shorter WPA key at run time (WL_AP_FAILED).
static_assert(sizeof(ELM_WIFI_PASSWORD) - 1 >= 8,
              "ELM_WIFI_PASSWORD needs at least 8 characters");
// 802.11's limit; WiFi101 copies the SSID into a 33-byte field unchecked.
static_assert(sizeof(ELM_WIFI_SSID) - 1 <= 32,
              "ELM_WIFI_SSID holds at most 32 characters");

namespace {

// Phone OBD apps default to 192.168.0.10:35000, the WiFi ELM327 convention.
const IPAddress kApIp(192, 168, 0, 10);
constexpr uint8_t kChannel = 6;

}  // namespace

void WifiLink::begin() {
  if (WiFi.status() == WL_NO_SHIELD) {
    offReason_ = "no WINC1500 found";
    return;
  }
  strncpy(firmware_, WiFi.firmwareVersion(), sizeof firmware_ - 1);
  WiFi.config(kApIp);
  if (WiFi.beginAP(ELM_WIFI_SSID, ELM_WIFI_PASSWORD, kChannel) != WL_AP_LISTENING) {
    offReason_ = "access point refused (WPA/WPA2 needs WINC firmware 19.5.2+)";
    return;
  }
  apStatus_ = WL_AP_LISTENING;
  server_.begin();
  on_ = true;
}

bool WifiLink::accept() {
  if (!on_) return false;
  const uint8_t status = WiFi.status();
  if (apStatus_ == WL_AP_CONNECTED && status == WL_AP_LISTENING) {
    client_.stop();
    server_.begin();
  }
  apStatus_ = status;
  WiFiClient fresh = server_.available();
  if (!fresh || fresh == client_) return false;
  client_.stop();
  client_ = fresh;
  return true;
}

bool WifiLink::connected() {
  return on_ && client_.connected() && WiFi.status() == WL_AP_CONNECTED;
}

uint16_t WifiLink::read(uint8_t *out, uint16_t capacity) {
  const int n = client_.read(out, capacity);
  return n > 0 ? static_cast<uint16_t>(n) : 0;
}

void WifiLink::write(const char *bytes, uint16_t length) {
  if (client_.write(reinterpret_cast<const uint8_t *>(bytes), length) != length) {
    client_.stop();
  }
}

bool WifiLink::statusLine(uint8_t index, char *out, uint8_t capacity) {
  if (index == 0) {
    if (on_) {
      snprintf(out, capacity, "WIFI AP %s WPA/WPA2 %d.%d.%d.%d:%u", ELM_WIFI_SSID,
               kApIp[0], kApIp[1], kApIp[2], kApIp[3], kPort);
    } else {
      snprintf(out, capacity, "WIFI OFF: %s", offReason_);
    }
    return true;
  }
  if (index == 1 && firmware_[0] != '\0') {
    snprintf(out, capacity, "WINC firmware %s", firmware_);
    return true;
  }
  if (index == 2 && on_) {
    if (connected()) {
      const IPAddress ip = client_.remoteIP();
      snprintf(out, capacity, "CLIENT %d.%d.%d.%d connected", ip[0], ip[1],
               ip[2], ip[3]);
    } else {
      snprintf(out, capacity, "CLIENT none");
    }
    return true;
  }
  return false;
}

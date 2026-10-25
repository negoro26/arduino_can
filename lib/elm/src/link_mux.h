// One ELM session, two links. While a TCP client is connected it is the
// session (its bytes in, all output to it) and USB input stays unread in the
// USB buffer; otherwise USB is. A new TCP connection replaces the old one
// (phones reconnect without closing). Only poll() switches, and only between
// commands, so a command's whole answer goes to the link that asked.
//
// TCP output is held and sent in chunks: WiFi101 refuses a write over 1400
// bytes, and each write is one SPI command to the WINC. A chunk goes out when
// it holds the `>` prompt, reaches 1400 bytes, or its first byte has waited
// 20 ms (checked by write() and poll(), which run between receive windows
// only), or on flush().

#ifndef ELM_LINK_MUX_H
#define ELM_LINK_MUX_H

#include <stdint.h>

#include "hal.h"

class ByteLink {
 public:
  // Up to `capacity` received bytes into `out`; 0 if none. Never blocks.
  virtual uint16_t read(uint8_t *out, uint16_t capacity) = 0;
  virtual void write(const char *bytes, uint16_t length) = 0;

 protected:
  ~ByteLink() = default;
};

class TcpLink : public ByteLink {
 public:
  // Takes a waiting connection, closing any older one; true if it did.
  virtual bool accept() = 0;
  virtual bool connected() = 0;

 protected:
  ~TcpLink() = default;
};

class LinkMux {
 public:
  static constexpr uint16_t kTcpChunk = 1400;
  static constexpr uint32_t kTcpFlushMs = 20;

  LinkMux(ByteLink &usb, TcpLink &tcp) : usb_(usb), tcp_(tcp) {}

  // Between commands only, after Elm::service() has drained the input.
  void poll(uint32_t nowMs);

  bool onTcp() const { return onTcp_; }
  int readByte();
  void write(const char *bytes, uint16_t length, uint32_t nowMs);
  void flush();
  // When the held TCP output is due; never() if nothing is held.
  Deadline flushDue() const;

 private:
  void switchTo(bool tcp);

  ByteLink &usb_;
  TcpLink &tcp_;
  bool onTcp_ = false;
  uint8_t in_[64];
  uint8_t inLen_ = 0;
  uint8_t inPos_ = 0;
  char out_[kTcpChunk];
  uint16_t outLen_ = 0;
  uint32_t firstOutMs_ = 0;
};

#endif  // ELM_LINK_MUX_H

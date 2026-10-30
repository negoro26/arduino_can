#include "link_mux.h"

#include <string.h>

void LinkMux::poll(uint32_t nowMs) {
  if (tcp_.accept()) {
    switchTo(true);
  } else if (onTcp_ && !tcp_.connected()) {
    switchTo(false);
  } else if (flushDue().passed(nowMs)) {
    flush();
  }
}

void LinkMux::switchTo(bool tcp) {
  onTcp_ = tcp;
  ++switches_;
  inLen_ = 0;
  inPos_ = 0;
  outLen_ = 0;
}

int LinkMux::readByte() {
  if (inPos_ == inLen_) {
    ByteLink &session = onTcp_ ? static_cast<ByteLink &>(tcp_) : usb_;
    inPos_ = 0;
    inLen_ = static_cast<uint8_t>(session.read(in_, sizeof in_));
    if (inLen_ == 0) return onTcp_ && !tcp_.connected() ? kLinkClosed : -1;
  }
  return in_[inPos_++];
}

void LinkMux::write(const char *bytes, uint16_t length, uint32_t nowMs) {
  if (!onTcp_) {
    usb_.write(bytes, length);
    return;
  }
  while (length > 0) {
    if (outLen_ == 0) firstOutMs_ = nowMs;
    uint16_t n = static_cast<uint16_t>(kTcpChunk - outLen_);
    if (n > length) n = length;
    memcpy(out_ + outLen_, bytes, n);
    outLen_ = static_cast<uint16_t>(outLen_ + n);
    const bool prompt = memchr(bytes, '>', n) != nullptr;
    bytes += n;
    length = static_cast<uint16_t>(length - n);
    if (prompt || outLen_ == kTcpChunk) flush();
  }
  if (flushDue().passed(nowMs)) flush();
}

void LinkMux::flush() {
  if (outLen_ == 0) return;
  tcp_.write(out_, outLen_);
  outLen_ = 0;
}

Deadline LinkMux::flushDue() const {
  return outLen_ == 0 ? Deadline::never()
                      : Deadline::after(firstOutMs_, kTcpFlushMs);
}

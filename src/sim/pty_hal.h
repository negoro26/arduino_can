#ifndef PTY_HAL_H
#define PTY_HAL_H

#include <stdint.h>
#include <time.h>

#include "link_mux.h"
#include "pty_link.h"
#include "sim_can_hal.h"
#include "tcp_server.h"

class PtyHal final : public SimCanHal {
 public:
  // 32 RX slots rather than the chip's two, so a late wakeup on a busy host
  // cannot fake BUFFER FULL.
  PtyHal(const CarSpec &car, const char *linkPath)
      : SimCanHal(car, 32), pty_(linkPath), links_(pty_, tcp_) {}

  bool open();
  bool listen(const char *hostPort) { return tcp_.open(hostPort); }

  // Between commands: sleeps until the session link or the TCP listener has
  // something, held TCP output is due, or 1 s passes.
  void waitForInput();
  void pollLinks() { links_.poll(nowMs()); }

  int readByte() override { return links_.readByte(); }
  void write(const char *bytes, uint16_t length) override {
    links_.write(bytes, length, nowMs());
  }
  void flush() override { links_.flush(); }
  bool linkTrusted() override { return !links_.onTcp(); }
  uint8_t linkSwitches() override { return links_.switches(); }
  bool wifiStatus(uint8_t index, char *out, uint8_t capacity) override {
    return tcp_.statusLine(index, out, capacity);
  }
  uint32_t nowMs() override;
  void idleUntil(Deadline d) override;

 protected:
  uint32_t nowUs() override;

 private:
  uint64_t elapsedUs() const;
  void waitReadable(int64_t timeoutUs, bool listener);

  PtyLink pty_;
  TcpServer tcp_;
  LinkMux links_;
  timespec start_{};
};

#endif  // PTY_HAL_H

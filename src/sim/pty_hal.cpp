#include "pty_hal.h"

#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <termios.h>
#include <unistd.h>

bool PtyHal::open() {
  clock_gettime(CLOCK_MONOTONIC, &start_);
  master_ = posix_openpt(O_RDWR | O_NOCTTY);
  if (master_ < 0 || grantpt(master_) != 0 || unlockpt(master_) != 0) {
    perror("posix_openpt");
    return false;
  }
  const char *name = ptsname(master_);
  if (name == nullptr) {
    perror("ptsname");
    return false;
  }
  slave_ = ::open(name, O_RDWR | O_NOCTTY);
  if (slave_ < 0) {
    perror(name);
    return false;
  }
  termios t;
  if (tcgetattr(slave_, &t) != 0) {
    perror("tcgetattr");
    return false;
  }
  cfmakeraw(&t);
  if (tcsetattr(slave_, TCSANOW, &t) != 0) {
    perror("tcsetattr");
    return false;
  }
  if (fcntl(master_, F_SETFL, fcntl(master_, F_GETFL) | O_NONBLOCK) != 0) {
    perror("fcntl");
    return false;
  }
  if (unlink(link_) != 0 && errno != ENOENT) {
    perror(link_);
    return false;
  }
  if (symlink(name, link_) != 0) {
    perror(link_);
    return false;
  }
  return true;
}

uint64_t PtyHal::elapsedUs() const {
  timespec now;
  clock_gettime(CLOCK_MONOTONIC, &now);
  const int64_t ns = static_cast<int64_t>(now.tv_sec - start_.tv_sec) * 1000000000 +
                     (now.tv_nsec - start_.tv_nsec);
  return static_cast<uint64_t>(ns / 1000);
}

uint32_t PtyHal::nowUs() { return static_cast<uint32_t>(elapsedUs()); }

uint32_t PtyHal::nowMs() { return static_cast<uint32_t>(elapsedUs() / 1000); }

int PtyHal::readByte() {
  if (inPos_ >= inLen_) {
    const ssize_t n = read(master_, in_, sizeof in_);
    if (n <= 0) return -1;
    inLen_ = n;
    inPos_ = 0;
  }
  return static_cast<uint8_t>(in_[inPos_++]);
}

void PtyHal::write(const char *bytes, uint16_t length) {
  while (length > 0) {
    const ssize_t n = ::write(master_, bytes, length);
    if (n > 0) {
      bytes += n;
      length = static_cast<uint16_t>(length - n);
      continue;
    }
    if (n < 0 && errno != EAGAIN && errno != EINTR) return;
    // The client is not reading (or has gone): give it a second, then drop
    // the rest rather than freeze the simulator.
    pollfd p{master_, POLLOUT, 0};
    if (poll(&p, 1, 1000) <= 0) return;
  }
}

void PtyHal::waitReadable(int64_t timeoutUs) {
  if (inPos_ < inLen_) return;
  pollfd p{master_, POLLIN, 0};
  if (timeoutUs < 0) {
    ppoll(&p, 1, nullptr, nullptr);
    return;
  }
  const timespec t{static_cast<time_t>(timeoutUs / 1000000),
                   static_cast<long>(timeoutUs % 1000000) * 1000};
  ppoll(&p, 1, &t, nullptr);
}

void PtyHal::idleUntil(Deadline d) {
  int64_t timeoutUs = -1;
  if (!d.isNever()) {
    const int32_t ms = static_cast<int32_t>(d.atMs() - nowMs());
    timeoutUs = ms > 0 ? static_cast<int64_t>(ms) * 1000 : 0;
  }
  uint32_t busAt;
  if (nextBusEvent(&busAt)) {
    const int32_t us = static_cast<int32_t>(busAt - nowUs());
    const int64_t untilBus = us > 0 ? us : 0;
    if (timeoutUs < 0 || untilBus < timeoutUs) timeoutUs = untilBus;
  }
  waitReadable(timeoutUs);
}

void PtyHal::waitForInput() {
  waitReadable(1000000);
  uint32_t ignored;
  nextBusEvent(&ignored);
}

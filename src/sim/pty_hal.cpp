#include "pty_hal.h"

#include <poll.h>

bool PtyHal::open() {
  clock_gettime(CLOCK_MONOTONIC, &start_);
  return pty_.open();
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

// Only the session's link is watched: the PTY's bytes stay unread while a TCP
// client holds the session, and must not wake every wait.
void PtyHal::waitReadable(int64_t timeoutUs, bool listener) {
  if (links_.onTcp() && !tcp_.connected()) return;
  pollfd fds[2];
  nfds_t n = 0;
  fds[n++] = {links_.onTcp() ? tcp_.clientFd() : pty_.fd(), POLLIN, 0};
  if (listener && tcp_.listenFd() >= 0) fds[n++] = {tcp_.listenFd(), POLLIN, 0};
  if (timeoutUs < 0) {
    ppoll(fds, n, nullptr, nullptr);
    return;
  }
  const timespec t{static_cast<time_t>(timeoutUs / 1000000),
                   static_cast<long>(timeoutUs % 1000000) * 1000};
  ppoll(fds, n, &t, nullptr);
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
  waitReadable(timeoutUs, false);
}

void PtyHal::waitForInput() {
  int64_t timeoutUs = 1000000;
  const Deadline flush = links_.flushDue();
  if (!flush.isNever()) {
    const int32_t ms = static_cast<int32_t>(flush.atMs() - nowMs());
    const int64_t untilFlush = ms > 0 ? static_cast<int64_t>(ms) * 1000 : 0;
    if (untilFlush < timeoutUs) timeoutUs = untilFlush;
  }
  waitReadable(timeoutUs, true);
  uint32_t ignored;
  nextBusEvent(&ignored);
}

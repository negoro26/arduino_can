#include "pty_link.h"

#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <termios.h>
#include <unistd.h>

bool PtyLink::open() {
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
  if (unlink(path_) != 0 && errno != ENOENT) {
    perror(path_);
    return false;
  }
  if (symlink(name, path_) != 0) {
    perror(path_);
    return false;
  }
  return true;
}

uint16_t PtyLink::read(uint8_t *out, uint16_t capacity) {
  const ssize_t n = ::read(master_, out, capacity);
  return n > 0 ? static_cast<uint16_t>(n) : 0;
}

void PtyLink::write(const char *bytes, uint16_t length) {
  while (length > 0) {
    const ssize_t n = ::write(master_, bytes, length);
    if (n > 0) {
      bytes += n;
      length = static_cast<uint16_t>(length - n);
      continue;
    }
    if (n < 0 && errno != EAGAIN && errno != EINTR) return;
    pollfd p{master_, POLLOUT, 0};
    if (poll(&p, 1, 1000) <= 0) return;
  }
}

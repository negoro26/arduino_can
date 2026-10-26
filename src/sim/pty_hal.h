#ifndef PTY_HAL_H
#define PTY_HAL_H

#include <sys/types.h>
#include <time.h>

#include "sim_can_hal.h"

class PtyHal final : public SimCanHal {
 public:
  // 32 RX slots: host scheduling jitter must not fake BUFFER FULL.
  PtyHal(const CarSpec &car, const char *linkPath)
      : SimCanHal(car, 32), link_(linkPath) {}

  // Opens the PTY in raw mode and points linkPath at its slave. false (with
  // a message on stderr) on failure.
  bool open();

  void waitForInput();

  int readByte() override;
  void write(const char *bytes, uint16_t length) override;
  uint32_t nowMs() override;
  void idleUntil(Deadline d) override;

 protected:
  uint32_t nowUs() override;

 private:
  uint64_t elapsedUs() const;
  void waitReadable(int64_t timeoutUs);

  const char *link_;
  int master_ = -1;
  // Held open for the simulator's lifetime: while no slave fd is open, reads
  // on the master fail with EIO, and clients open and close the port often.
  int slave_ = -1;
  timespec start_{};
  char in_[256];
  ssize_t inLen_ = 0;
  ssize_t inPos_ = 0;
};

#endif  // PTY_HAL_H

#ifndef TEST_HAL_H
#define TEST_HAL_H

#include <stdio.h>
#include <unity.h>

#include <algorithm>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "elm.h"
#include "renault_scenario.h"
#include "sim_can_hal.h"

// A SimCanHal on virtual time: idleUntil() jumps the clock to the next
// deadline, input or bus event, so a wait that would hang fails the test.
class VirtualTimeHal : public SimCanHal {
 public:
  VirtualTimeHal(const CarSpec &car, uint8_t rxSlots) : SimCanHal(car, rxSlots) {}

  // Every frame the bus carried to the car, with its virtual time in us.
  // Loopback frames and sends the bus refused (listen-only, wrong bitrate)
  // never reach it.
  const std::vector<std::pair<uint32_t, CanFrame>> &wire() const { return wire_; }
  // canSend() calls with a cleared frame, whether or not the bus took them.
  size_t sendAttempts() const { return sendAttempts_; }
  // The controller's mode; empty while it is closed.
  std::optional<CanMode> controllerMode() const { return mode_; }
  uint32_t now() const { return nowUs_; }
  void sleepMs(uint32_t ms) { sleepUs(ms * 1000); }
  void sleepUs(uint32_t us) { nowUs_ += us; }
  void setCanOpenFails(bool fails) { canOpenFails_ = fails; }

  bool canOpen(Bitrate rate, CanMode mode, const Acceptance &acc) override {
    if (canOpenFails_ || !SimCanHal::canOpen(rate, mode, acc)) {
      mode_.reset();
      return false;
    }
    mode_ = mode;
    return true;
  }

  void canClose() override {
    SimCanHal::canClose();
    mode_.reset();
  }

  TxStatus canSend(const ClearedFrame &f) override {
    const TxStatus status = SimCanHal::canSend(f);
    if (f.valid()) ++sendAttempts_;
    if (status == TxStatus::Sent && mode_ == CanMode::Normal) {
      wire_.push_back({nowUs_, f.frame()});
    }
    return status;
  }

  uint32_t nowMs() override {
    if (++clockReads_ > 100000) {
      TEST_FAIL_MESSAGE("virtual time never advances: 100000 clock reads without idling");
    }
    return nowUs_ / 1000;
  }

  void idleUntil(Deadline d) override {
    clockReads_ = 0;
    bool found = false;
    uint32_t target = 0;
    auto consider = [&](uint32_t t) {
      if (!found || t < target) target = t;
      found = true;
    };
    if (!d.isNever()) consider(d.atMs() * 1000);
    uint32_t at;
    if (nextBusEvent(&at)) consider(at);
    if (nextInputUs(&at)) consider(at);
    if (!found) {
      TEST_FAIL_MESSAGE("the adapter waits forever: idleUntil(never) with nothing scheduled");
    }
    if (target <= nowUs_) {
      if (++stalls_ > 100000) {
        TEST_FAIL_MESSAGE("virtual time never advances: 100000 idleUntil() calls wake at the current time");
      }
      return;
    }
    stalls_ = 0;
    nowUs_ = target;
  }

 protected:
  ~VirtualTimeHal() = default;
  // When the next scheduled input arrives, false if none is.
  virtual bool nextInputUs(uint32_t *atUs) const = 0;
  uint32_t nowUs() override { return nowUs_; }

 private:
  uint32_t nowUs_ = 0;
  uint32_t clockReads_ = 0;
  uint32_t stalls_ = 0;
  bool canOpenFails_ = false;
  std::optional<CanMode> mode_;
  size_t sendAttempts_ = 0;
  std::vector<std::pair<uint32_t, CanFrame>> wire_;
};

class TestHal final : public VirtualTimeHal {
 public:
  explicit TestHal(const CarSpec &car, uint8_t rxSlots = 2) : VirtualTimeHal(car, rxSlots) {}

  void type(const std::string &s) { typeAt(now() / 1000, s); }
  void typeAt(uint32_t ms, const std::string &s) {
    const uint32_t at = std::max(ms * 1000, now());
    auto pos = std::upper_bound(
        input_.begin(), input_.end(), at,
        [](uint32_t t, const std::pair<uint32_t, char> &e) { return t < e.first; });
    for (char c : s) pos = std::next(input_.insert(pos, {at, c}));
  }
  const std::string &output() const { return output_; }
  void clearOutput() { output_.clear(); }
  void setWriteCostUs(uint16_t perByte) { writeCostUs_ = perByte; }
  void setWifiStatus(std::vector<std::string> lines) { wifi_ = std::move(lines); }

  int readByte() override {
    if (input_.empty() || input_.front().first > now()) return -1;
    const char c = input_.front().second;
    input_.erase(input_.begin());
    return static_cast<uint8_t>(c);
  }

  void write(const char *bytes, uint16_t length) override {
    output_.append(bytes, length);
    sleepUs(static_cast<uint32_t>(length) * writeCostUs_);
  }

  void flush() override {}
  bool linkTrusted() override { return true; }
  uint8_t linkSwitches() override { return 0; }
  bool wifiStatus(uint8_t index, char *out, uint8_t capacity) override {
    if (index >= wifi_.size()) return false;
    snprintf(out, capacity, "%s", wifi_[index].c_str());
    return true;
  }

 protected:
  bool nextInputUs(uint32_t *atUs) const override {
    if (input_.empty()) return false;
    *atUs = input_.front().first;
    return true;
  }

 private:
  uint16_t writeCostUs_ = 0;
  std::vector<std::string> wifi_;
  std::vector<std::pair<uint32_t, char>> input_;
  std::string output_;
};

inline void run(TestHal &hal, Elm &elm, const std::string &line) {
  hal.type(line + "\r");
  elm.service();
}

inline std::string ask(TestHal &hal, Elm &elm, const std::string &line) {
  hal.clearOutput();
  run(hal, elm, line);
  return hal.output();
}

#endif  // TEST_HAL_H

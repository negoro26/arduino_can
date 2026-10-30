#ifndef TEST_HAL_H
#define TEST_HAL_H

#include <stdio.h>
#include <unity.h>

#include <algorithm>
#include <string>
#include <utility>
#include <vector>

#include "elm.h"
#include "renault_scenario.h"
#include "sim_can_hal.h"

class TestHal final : public SimCanHal {
 public:
  explicit TestHal(const CarSpec &car, uint8_t rxSlots = 2)
      : SimCanHal(car, rxSlots) {}

  void type(const std::string &s) { typeAt(nowUs_ / 1000, s); }
  void typeAt(uint32_t ms, const std::string &s) {
    const uint32_t at = std::max(ms * 1000, nowUs_);
    auto pos = std::upper_bound(
        input_.begin(), input_.end(), at,
        [](uint32_t t, const std::pair<uint32_t, char> &e) { return t < e.first; });
    for (char c : s) pos = std::next(input_.insert(pos, {at, c}));
  }
  const std::string &output() const { return output_; }
  void clearOutput() { output_.clear(); }
  // Every frame the adapter transmitted, with its virtual time in us.
  const std::vector<std::pair<uint32_t, CanFrame>> &wire() const {
    return wire_;
  }
  void setWriteCostUs(uint16_t perByte) { writeCostUs_ = perByte; }
  uint32_t now() const { return nowUs_; }
  void sleepMs(uint32_t ms) { nowUs_ += ms * 1000; }
  void setCanOpenFails(bool fails) { canOpenFails_ = fails; }
  void setLinkTrusted(bool trusted) { linkTrusted_ = trusted; }
  void setWifiStatus(std::vector<std::string> lines) { wifi_ = std::move(lines); }

  bool canOpen(Bitrate rate, CanMode mode, const Acceptance &acc) override {
    return !canOpenFails_ && SimCanHal::canOpen(rate, mode, acc);
  }

  TxStatus canSend(const ClearedFrame &f) override {
    if (f.valid()) wire_.push_back({nowUs_, f.frame()});
    return SimCanHal::canSend(f);
  }

  int readByte() override {
    if (input_.empty() || input_.front().first > nowUs_) return -1;
    const char c = input_.front().second;
    input_.erase(input_.begin());
    return static_cast<uint8_t>(c);
  }

  void write(const char *bytes, uint16_t length) override {
    output_.append(bytes, length);
    nowUs_ += static_cast<uint32_t>(length) * writeCostUs_;
  }

  void flush() override {}
  bool linkTrusted() override { return linkTrusted_; }
  uint8_t linkSwitches() override { return 0; }
  bool wifiStatus(uint8_t index, char *out, uint8_t capacity) override {
    if (index >= wifi_.size()) return false;
    snprintf(out, capacity, "%s", wifi_[index].c_str());
    return true;
  }

  uint32_t nowMs() override {
    if (++clockReads_ > 100000) {
      TEST_FAIL_MESSAGE("100000 clock reads without idling: a wait loop is not using await()");
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
    uint32_t bus;
    if (nextBusEvent(&bus)) consider(bus);
    if (!input_.empty()) consider(input_.front().first);
    if (!found) {
      TEST_FAIL_MESSAGE("idleUntil(never) with nothing scheduled: the adapter would hang");
    }
    if (target <= nowUs_) {
      if (++stalls_ > 100000) TEST_FAIL_MESSAGE("idleUntil() never lets time advance");
      return;
    }
    stalls_ = 0;
    nowUs_ = target;
  }

 protected:
  uint32_t nowUs() override { return nowUs_; }

 private:
  uint32_t nowUs_ = 0;
  uint32_t clockReads_ = 0;
  uint32_t stalls_ = 0;
  uint16_t writeCostUs_ = 0;
  bool canOpenFails_ = false;
  bool linkTrusted_ = true;
  std::vector<std::string> wifi_;
  std::vector<std::pair<uint32_t, char>> input_;
  std::string output_;
  std::vector<std::pair<uint32_t, CanFrame>> wire_;
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

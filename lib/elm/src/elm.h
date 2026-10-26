#ifndef ELM_ELM_H
#define ELM_ELM_H

#include "at_table.h"
#include "bus.h"
#include "guard.h"
#include "hal.h"
#include "settings.h"
#include "text.h"

class Elm {
 public:
  // No I/O, so safe for a static global.
  explicit Elm(Hal &hal) : hal_(hal) {}

  // Power-up: prints "ELM327 v1.5" and the first prompt. Call once, after the
  // Hal is ready.
  void begin();

  // Processes every byte the Hal has buffered, then returns. A CR runs that
  // line to completion before the next byte is read, so input that arrives
  // during a CAN wait is seen by await() (-> STOPPED), not by service().
  void service();

 private:
  friend struct AtTable;

  void onByte(uint8_t c);
  void execute();
  void runAt(const char *body);
  void runRequest(const char *line);

  void monitor();
  void survey();
  void selfTest();

  void resetSettings();

  void print(const char *text);
  void printFrames();
  void printOutcome(Outcome o);

  Hal &hal_;

  Guard guard_;
  ProtocolSel storedProtocol_{Protocol::None, true};  // RAM stand-in for EEPROM

  Settings set_;

  LinkState link_;
  LineBuffer line_;
  char last_[kLineMax + 1] = {};
  uint8_t linkSwitches_ = 0;
  RxLog log_;
};

#endif  // ELM_ELM_H

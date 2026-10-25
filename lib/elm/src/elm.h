// The ELM327 session: the only public entry point of the core.
//
// Execution model: each completed input line runs synchronously to completion
// -- including its '>' prompt -- inside service(); waits are local loops
// around await() that poll the serial input, the CAN controller and the clock.

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

  // LF: ignored. Echo (E1): the raw byte, a CR as lineEnd(set_). CR: execute.
  void onByte(uint8_t c);
  // Runs line_ (or the repeat buffer on a bare CR) and prints exactly one
  // prompt, always last -- no handler or procedure prints a prompt.
  void execute();
  void runAt(const char *body);
  void runRequest(const char *line);

  // Long-running AT procedures (blocking; every wait is await()).
  void monitor();   // ATMA
  void survey();    // AT#SURVEY
  void selfTest();  // AT#SELFTEST

  // ATZ / ATWS / ATD: set_ = Settings{} plus the stored protocol; link closed.
  void resetSettings();

  // Output: the only code that writes to the Hal.
  void print(const char *text);  // text + lineEnd(set_)
  void printFrames();
  void printOutcome(Outcome o);

  Hal &hal_;

  // Power-up-only state. Not reachable from resetSettings().
  Guard guard_;
  ProtocolSel storedProtocol_{Protocol::None, true};  // RAM stand-in for EEPROM

  Settings set_;

  LinkState link_;
  LineBuffer line_;
  // Bare-CR repeat buffer. Holds the last CAN request only: repeating an AT
  // command would let a stray CR re-run ATZ or restart ATMA.
  char last_[kLineMax + 1] = {};
  RxLog log_;
};

#endif  // ELM_ELM_H

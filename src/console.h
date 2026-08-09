// Keyboard command decoding for the serial console.
//
// Deliberately free of Arduino and Serial dependencies so the same code runs on
// the SAMD21 and under `pio test -e native`. loop() supplies the byte; this
// header decides what it means. Nothing here touches hardware, which is the
// only reason it can be tested at all.

#ifndef CONSOLE_H
#define CONSOLE_H

#include <stdint.h>

enum class Command : uint8_t {
  None,          // nothing to do: no byte waiting, or an unrecognised one
  RunSweep,      // stages 1-3: probe, loopback, listen-only survey
  RunDiscovery,  // stage 4: the active sweep that transmits on the bus
};

// Maps one byte from Serial.read() to a command.
//
// `raw` is an int rather than a char because Serial.read() returns -1 when the
// buffer is empty, and that must not be mistaken for a keypress.
//
// Letters are accepted in either case. Everything else -- line endings, digits,
// control bytes -- is None, so a terminal that appends CR or CRLF cannot
// trigger a second, phantom command.
inline Command commandForKey(int raw) {
  if (raw < 0 || raw > 255) {
    return Command::None;
  }

  uint8_t c = static_cast<uint8_t>(raw);
  if (c >= 'A' && c <= 'Z') {
    c = static_cast<uint8_t>(c - 'A' + 'a');
  }

  switch (c) {
    case 'q':
      return Command::RunSweep;
    case 's':
      return Command::RunDiscovery;
    default:
      return Command::None;
  }
}

#endif  // CONSOLE_H

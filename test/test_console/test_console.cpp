// Host-side tests for keyboard command decoding.
//
// These run with `pio test -e native`: no board, no Serial, no car. They exist
// because two of the three bugs in this console were about the wrong byte
// reaching the wrong handler, and that is decidable without hardware.

#include <unity.h>

#include "console.h"

void setUp(void) {}
void tearDown(void) {}

// --------------------------------------------------- the two real commands

static void q_runs_the_passive_sweep(void) {
  TEST_ASSERT_TRUE(commandForKey('q') == Command::RunSweep);
}

static void s_runs_the_active_discovery(void) {
  TEST_ASSERT_TRUE(commandForKey('s') == Command::RunDiscovery);
}

// The two commands must never resolve to each other. This is the bug that
// actually happened: one handler consumed the other's byte, and stage 4 became
// unreachable for a while.
static void the_two_commands_are_distinct(void) {
  TEST_ASSERT_FALSE(commandForKey('q') == Command::RunDiscovery);
  TEST_ASSERT_FALSE(commandForKey('s') == Command::RunSweep);
}

static void uppercase_is_accepted_for_both(void) {
  TEST_ASSERT_TRUE(commandForKey('Q') == Command::RunSweep);
  TEST_ASSERT_TRUE(commandForKey('S') == Command::RunDiscovery);
}

// ------------------------------------------------------- nothing waiting

// Serial.read() returns -1 on an empty buffer. If that were treated as a
// keypress the sweep would fire continuously on its own.
static void empty_buffer_is_not_a_command(void) {
  TEST_ASSERT_TRUE(commandForKey(-1) == Command::None);
}

// ------------------------------------------------- what a terminal sends

// Tera Term sends the letter followed by CR; some terminals send CRLF. Neither
// may register as a command, or every keypress becomes two.
static void line_endings_are_not_commands(void) {
  TEST_ASSERT_TRUE(commandForKey('\r') == Command::None);
  TEST_ASSERT_TRUE(commandForKey('\n') == Command::None);
}

static void unrelated_keys_are_not_commands(void) {
  TEST_ASSERT_TRUE(commandForKey('x') == Command::None);
  TEST_ASSERT_TRUE(commandForKey('1') == Command::None);
  TEST_ASSERT_TRUE(commandForKey(' ') == Command::None);
  TEST_ASSERT_TRUE(commandForKey(0x1B) == Command::None);  // ESC
}

// ------------------------------------------------------------ boundaries

static void byte_range_boundaries_are_not_commands(void) {
  TEST_ASSERT_TRUE(commandForKey(0) == Command::None);
  TEST_ASSERT_TRUE(commandForKey(255) == Command::None);
  TEST_ASSERT_TRUE(commandForKey(256) == Command::None);  // out of byte range
  TEST_ASSERT_TRUE(commandForKey(-2) == Command::None);
}

// Every byte that is not one of the four accepted characters must be None.
// A loop is worth it here: it proves there is no accidental second mapping
// hiding somewhere in the 256 possible values.
static void exactly_four_bytes_are_commands(void) {
  int commands = 0;
  for (int b = 0; b <= 255; b++) {
    if (commandForKey(b) != Command::None) {
      commands++;
    }
  }
  TEST_ASSERT_EQUAL_INT(4, commands);  // q, Q, s, S
}

int main(int, char **) {
  UNITY_BEGIN();

  RUN_TEST(q_runs_the_passive_sweep);
  RUN_TEST(s_runs_the_active_discovery);
  RUN_TEST(the_two_commands_are_distinct);
  RUN_TEST(uppercase_is_accepted_for_both);

  RUN_TEST(empty_buffer_is_not_a_command);

  RUN_TEST(line_endings_are_not_commands);
  RUN_TEST(unrelated_keys_are_not_commands);

  RUN_TEST(byte_range_boundaries_are_not_commands);
  RUN_TEST(exactly_four_bytes_are_commands);

  return UNITY_END();
}

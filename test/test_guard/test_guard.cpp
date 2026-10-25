// The device-level write guard: service table, ISO-TP frame classification,
// and the tier's lifetime (only power-up and AT#GUARD change it).

#include <unity.h>

#include "test_hal.h"

void setUp(void) {}
void tearDown(void) {}

static const CanId k7E0 = CanId::std11(0x7E0);

static CanFrame frame(CanId id, std::initializer_list<uint8_t> bytes) {
  CanFrame f{};
  f.id = id;
  for (uint8_t b : bytes) f.data[f.dlc++] = b;
  return f;
}

static void read_services(void) {
  const uint8_t reads[] = {0x01, 0x02, 0x03, 0x05, 0x06, 0x07, 0x09, 0x0A,
                           0x12, 0x17, 0x18, 0x19, 0x1A, 0x20, 0x21, 0x22,
                           0x23, 0x24, 0x3E, 0x81, 0x82, 0x83};
  for (uint8_t sid : reads) {
    TEST_ASSERT_TRUE_MESSAGE(tierOfService(sid, false, 0) == Tier::Read, "read sid");
  }
}

static void clear_services(void) {
  TEST_ASSERT_TRUE(tierOfService(0x14, true, 0xFF) == Tier::Clear);
  TEST_ASSERT_TRUE(tierOfService(0x04, false, 0) == Tier::Clear);
}

static void write_services_and_default_deny(void) {
  const uint8_t writes[] = {0x08, 0x11, 0x27, 0x28, 0x2C, 0x2E, 0x2F, 0x30, 0x31,
                            0x34, 0x35, 0x36, 0x37, 0x3B, 0x3D, 0x85, 0x86, 0x87,
                            0x00, 0x0B, 0x40, 0xFF};
  for (uint8_t sid : writes) {
    TEST_ASSERT_TRUE_MESSAGE(tierOfService(sid, true, 0x01) == Tier::Write, "write sid");
  }
}

static void session_control_is_read_except_programming(void) {
  TEST_ASSERT_TRUE(tierOfService(0x10, true, 0x01) == Tier::Read);
  TEST_ASSERT_TRUE(tierOfService(0x10, true, 0x03) == Tier::Read);
  TEST_ASSERT_TRUE(tierOfService(0x10, true, 0xC0) == Tier::Read);
  TEST_ASSERT_TRUE(tierOfService(0x10, true, 0x02) == Tier::Write);
  TEST_ASSERT_TRUE(tierOfService(0x10, true, 0x82) == Tier::Write);
  TEST_ASSERT_TRUE(tierOfService(0x10, true, 0x85) == Tier::Write);
}

static void tier_words(void) {
  Tier t;
  TEST_ASSERT_TRUE(parseTier("CLEAR", &t) && t == Tier::Clear);
  TEST_ASSERT_FALSE(parseTier("ALL", &t));
  TEST_ASSERT_EQUAL_STRING("WRITE", tierName(Tier::Write));
}

static void single_frame_sid_is_byte_one(void) {
  Guard g;
  ClearedFrame out;
  TEST_ASSERT_TRUE(g.admit(frame(k7E0, {0x03, 0x22, 0xF1, 0x90, 0, 0, 0, 0}), &out));
  TEST_ASSERT_TRUE(out.valid());
  ClearedFrame refused;
  TEST_ASSERT_FALSE(g.admit(frame(k7E0, {0x03, 0x14, 0xFF, 0x00, 0, 0, 0, 0}), &refused));
  TEST_ASSERT_FALSE(refused.valid());
  TEST_ASSERT_FALSE(g.permits(frame(k7E0, {0x02, 0x10, 0x02})));
}

static void invalid_single_frame_is_denied(void) {
  Guard g;
  g.setTier(Tier::Clear);
  TEST_ASSERT_FALSE(g.permits(frame(k7E0, {0x00, 0x22})));
  TEST_ASSERT_FALSE(g.permits(frame(k7E0, {0x07, 0x22, 0xF1})));
}

static void first_frame_sid_is_byte_two(void) {
  Guard g;
  TEST_ASSERT_TRUE(g.permits(frame(k7E0, {0x10, 0x0A, 0x22, 0xF1, 0x90, 0xF1, 0x8A, 0xF1})));
  TEST_ASSERT_FALSE(g.permits(frame(k7E0, {0x10, 0x0A, 0x2E, 0xF1, 0x90, 0x41, 0x42, 0x43})));
}

static void consecutive_frames_inherit_the_first_frame(void) {
  Guard g;
  ClearedFrame out;
  const CanFrame cf = frame(k7E0, {0x21, 0x44, 0x45, 0x46, 0, 0, 0, 0});
  TEST_ASSERT_FALSE_MESSAGE(g.permits(cf), "no first frame yet: default deny");

  TEST_ASSERT_TRUE(g.admit(frame(k7E0, {0x10, 0x0A, 0x22, 0xF1, 0x90, 0xF1, 0x8A, 0xF1}), &out));
  ClearedFrame cleared;
  TEST_ASSERT_TRUE(g.admit(cf, &cleared));
  TEST_ASSERT_FALSE_MESSAGE(g.permits(frame(CanId::std11(0x745), {0x21, 0x00})),
                            "a CF on another id does not inherit");

  ClearedFrame refusedFirst;
  TEST_ASSERT_FALSE(g.admit(frame(k7E0, {0x10, 0x0A, 0x2E, 0xF1, 0x90, 0x41, 0x42, 0x43}), &refusedFirst));
  TEST_ASSERT_FALSE_MESSAGE(g.permits(cf), "CFs of a refused first frame stay refused");
}

static void consecutive_frame_is_judged_against_the_current_tier(void) {
  Guard g;
  g.setTier(Tier::Write);
  ClearedFrame out;
  TEST_ASSERT_TRUE(g.admit(frame(k7E0, {0x10, 0x0A, 0x2E, 0xF1, 0x90, 0x41, 0x42, 0x43}), &out));
  g.setTier(Tier::Read);
  TEST_ASSERT_FALSE(g.permits(frame(k7E0, {0x21, 0x44, 0x45, 0x46, 0, 0, 0, 0})));
}

static void single_frame_ends_a_segmented_request(void) {
  Guard g;
  ClearedFrame a, b;
  TEST_ASSERT_TRUE(g.admit(frame(k7E0, {0x10, 0x0A, 0x22, 0xF1, 0x90, 0xF1, 0x8A, 0xF1}), &a));
  TEST_ASSERT_TRUE(g.admit(frame(k7E0, {0x02, 0x3E, 0x00}), &b));
  TEST_ASSERT_FALSE(g.permits(frame(k7E0, {0x21, 0x44})));
}

static void flow_control_is_always_allowed(void) {
  Guard g;
  TEST_ASSERT_TRUE(g.permits(frame(k7E0, {0x30, 0x00, 0x00, 0, 0, 0, 0, 0})));
  TEST_ASSERT_TRUE(g.permits(frame(k7E0, {0x31, 0x00, 0x00})));
}

static void other_pci_and_empty_frames_are_denied(void) {
  Guard g;
  g.setTier(Tier::Clear);
  TEST_ASSERT_FALSE(g.permits(frame(k7E0, {0x40, 0x22})));
  TEST_ASSERT_FALSE(g.permits(frame(k7E0, {})));
}

static void tier_survives_reset_commands_and_is_settable(void) {
  TestHal hal(renaultDaciaCarQuiet());
  Elm elm(hal);
  elm.begin();
  TEST_ASSERT_EQUAL_STRING("AT#GUARD\rREAD\r\r>", ask(hal, elm, "AT#GUARD").c_str());
  TEST_ASSERT_EQUAL_STRING("AT#GUARD CLEAR\rOK\r\r>", ask(hal, elm, "AT#GUARD CLEAR").c_str());
  for (const char *reset : {"ATZ", "ATWS", "ATD"}) {
    run(hal, elm, reset);
    TEST_ASSERT_EQUAL_STRING("AT#GUARD\rCLEAR\r\r>", ask(hal, elm, "AT#GUARD").c_str());
  }
  TEST_ASSERT_EQUAL_STRING("AT#GUARD ALL\r?\r\r>", ask(hal, elm, "AT#GUARD ALL").c_str());
  run(hal, elm, "AT#GUARD WRITE");
  TEST_ASSERT_EQUAL_STRING("AT#GUARD\rWRITE\r\r>", ask(hal, elm, "AT#GUARD").c_str());
}

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(read_services);
  RUN_TEST(clear_services);
  RUN_TEST(write_services_and_default_deny);
  RUN_TEST(session_control_is_read_except_programming);
  RUN_TEST(tier_words);
  RUN_TEST(single_frame_sid_is_byte_one);
  RUN_TEST(invalid_single_frame_is_denied);
  RUN_TEST(first_frame_sid_is_byte_two);
  RUN_TEST(consecutive_frames_inherit_the_first_frame);
  RUN_TEST(consecutive_frame_is_judged_against_the_current_tier);
  RUN_TEST(single_frame_ends_a_segmented_request);
  RUN_TEST(flow_control_is_always_allowed);
  RUN_TEST(other_pci_and_empty_frames_are_denied);
  RUN_TEST(tier_survives_reset_commands_and_is_settable);
  return UNITY_END();
}

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

static bool admittedAt(Tier tier, const CanFrame &f) {
  Guard g;
  g.setTier(tier);
  ClearedFrame out;
  return g.admit(f, &out);
}

static const char *sidName(uint8_t sid) {
  static char name[8];
  snprintf(name, sizeof name, "sid %02X", sid);
  return name;
}

static void read_services_pass_at_read(void) {
  const uint8_t reads[] = {0x01, 0x02, 0x03, 0x05, 0x06, 0x07, 0x09, 0x0A,
                           0x10, 0x12, 0x17, 0x18, 0x19, 0x1A, 0x20, 0x21,
                           0x22, 0x23, 0x24, 0x3E, 0x81, 0x82, 0x83};
  for (uint8_t sid : reads) {
    TEST_ASSERT_TRUE_MESSAGE(admittedAt(Tier::Read, frame(k7E0, {0x02, sid, 0x01})), sidName(sid));
  }
}

static void clear_services_need_clear(void) {
  for (const CanFrame &f : {frame(k7E0, {0x04, 0x14, 0xFF, 0xFF, 0xFF}), frame(k7E0, {0x01, 0x04})}) {
    TEST_ASSERT_FALSE(admittedAt(Tier::Read, f));
    TEST_ASSERT_TRUE(admittedAt(Tier::Clear, f));
  }
}

static void write_and_unknown_services_need_write(void) {
  const uint8_t writes[] = {0x08, 0x11, 0x27, 0x28, 0x2C, 0x2E, 0x2F, 0x30, 0x31,
                            0x34, 0x35, 0x36, 0x37, 0x3B, 0x3D, 0x85, 0x86, 0x87,
                            0x00, 0x0B, 0x40, 0xFF};
  for (uint8_t sid : writes) {
    TEST_ASSERT_FALSE_MESSAGE(admittedAt(Tier::Clear, frame(k7E0, {0x02, sid, 0x01})), sidName(sid));
    TEST_ASSERT_TRUE_MESSAGE(admittedAt(Tier::Write, frame(k7E0, {0x02, sid, 0x01})), sidName(sid));
  }
}

static void session_control_is_read_except_programming(void) {
  for (uint8_t session : {0x01, 0x03, 0xC0}) {
    TEST_ASSERT_TRUE(admittedAt(Tier::Read, frame(k7E0, {0x02, 0x10, session})));
  }
  for (uint8_t programming : {0x02, 0x82, 0x85}) {
    TEST_ASSERT_FALSE(admittedAt(Tier::Clear, frame(k7E0, {0x02, 0x10, programming})));
  }
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

static void flow_control_is_allowed_at_read(void) {
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

static void only_diagnostic_ids_pass_below_write(void) {
  const CanFrame flowControlLooking =
      frame(CanId::std11(0x1F6), {0x30, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77});
  const CanFrame read = frame(CanId::std11(0x5DE), {0x03, 0x22, 0xF1, 0x90});
  Guard g;
  for (Tier t : {Tier::Read, Tier::Clear}) {
    g.setTier(t);
    TEST_ASSERT_FALSE(g.permits(flowControlLooking));
    TEST_ASSERT_FALSE(g.permits(read));
  }
  g.setTier(Tier::Write);
  ClearedFrame a, b;
  TEST_ASSERT_TRUE(g.admit(flowControlLooking, &a));
  TEST_ASSERT_TRUE(g.admit(read, &b));
}

static void reads_on_diagnostic_ids_pass_at_read(void) {
  Guard g;
  const CanId ids[] = {k7E0, CanId::std11(0x605), CanId::ext29(0x18DA10F1),
                       CanId::ext29(0x18DB33F1)};
  for (CanId id : ids) {
    ClearedFrame out;
    TEST_ASSERT_TRUE(g.admit(frame(id, {0x03, 0x22, 0xF1, 0x90}), &out));
  }
  TEST_ASSERT_FALSE(g.permits(frame(CanId::ext29(0x0CF00400), {0x03, 0x22, 0xF1, 0x90})));
}

static void raw_frame_on_an_operational_id_is_blocked(void) {
  TestHal hal(renaultDaciaCarQuiet());
  Elm elm(hal);
  elm.begin();
  for (const char *cmd : {"ATE0", "ATSP6", "ATCAF0", "ATSH1F6"}) run(hal, elm, cmd);
  TEST_ASSERT_EQUAL_STRING("BLOCKED\r\r>", ask(hal, elm, "3011223344556677").c_str());
  TEST_ASSERT_EQUAL_size_t(0, hal.wire().size());
}

static const CanFrame kRead622 = frame(CanId::std11(0x622), {0x03, 0x22, 0xF1, 0x90});

static void ids_heard_as_traffic_are_refused_below_write(void) {
  Guard g;
  TEST_ASSERT_TRUE(g.permits(kRead622));
  g.heard(CanId::std11(0x622));
  for (Tier t : {Tier::Read, Tier::Clear}) {
    g.setTier(t);
    TEST_ASSERT_FALSE(g.permits(kRead622));
    TEST_ASSERT_TRUE(g.permits(frame(k7E0, {0x03, 0x22, 0xF1, 0x90})));
  }
  g.setTier(Tier::Write);
  ClearedFrame out;
  TEST_ASSERT_TRUE(g.admit(kRead622, &out));
}

static void the_guard_keeps_128_heard_ids(void) {
  Guard g;
  for (int i = 0; i < 1000; ++i) g.heard(CanId::std11(0x600));
  for (uint32_t id = 0x601; id < 0x680; ++id) g.heard(CanId::std11(id));
  for (uint32_t id = 0x600; id < 0x680; ++id) {
    TEST_ASSERT_FALSE(g.permits(frame(CanId::std11(id), {0x03, 0x22, 0xF1, 0x90})));
  }
}

static void traffic_outside_the_diagnostic_range_leaves_room_for_heard_ids(void) {
  Guard g;
  for (uint32_t id = 0x100; id < 0x100 + 200; ++id) g.heard(CanId::std11(id));
  g.heard(CanId::std11(0x60D));
  TEST_ASSERT_FALSE(g.permits(frame(CanId::std11(0x60D), {0x03, 0x22, 0xF1, 0x90})));
}

static void listen_before_talk_traffic_id_needs_write(void) {
  TestHal hal(renaultDaciaCar());
  Elm elm(hal);
  elm.begin();
  for (const char *cmd : {"ATE0", "ATSP6", "ATSH622"}) run(hal, elm, cmd);
  // The listen-before-talk window hears the car's 622 broadcast.
  TEST_ASSERT_EQUAL_STRING("BLOCKED\r\r>", ask(hal, elm, "22F190").c_str());
  run(hal, elm, "AT#GUARD CLEAR");
  TEST_ASSERT_EQUAL_STRING("BLOCKED\r\r>", ask(hal, elm, "22F190").c_str());
  TEST_ASSERT_EQUAL_size_t(0, hal.wire().size());
  run(hal, elm, "AT#GUARD WRITE");
  TEST_ASSERT_EQUAL_STRING("NO DATA\r\r>", ask(hal, elm, "22F190").c_str());
  TEST_ASSERT_EQUAL_size_t(1, hal.wire().size());
  TEST_ASSERT_TRUE(hal.wire()[0].second.id == CanId::std11(0x622));
  for (const char *cmd : {"AT#GUARD READ", "ATSH7E0"}) run(hal, elm, cmd);
  TEST_ASSERT_EQUAL_STRING("41 00 BE 1F A8 12\r\r>", ask(hal, elm, "0100").c_str());
}

// A 6xx id too slow for the 60 ms listen-before-talk window to catch.
static const Periodic kSlow60D[] = {
    {{CanId::std11(0x60D), 8, {0x00, 0x11, 0, 0, 0, 0, 0, 0}}, 2000}};

static CarSpec carWithSlow60D() {
  CarSpec car = renaultDaciaCarQuiet();
  car.traffic = kSlow60D;
  car.trafficCount = 1;
  return car;
}

static void monitored_traffic_id_is_refused_until_power_up(void) {
  TestHal hal(carWithSlow60D());
  Elm elm(hal);
  elm.begin();
  for (const char *cmd : {"ATE0", "ATSP6"}) run(hal, elm, cmd);
  hal.type("ATMA\r");
  hal.typeAt(hal.now() / 1000 + 25, "\r");
  elm.service();
  for (const char *cmd : {"ATZ", "ATWS", "ATD", "ATE0", "ATSP6", "ATSH60D"}) run(hal, elm, cmd);
  TEST_ASSERT_EQUAL_STRING("BLOCKED\r\r>", ask(hal, elm, "22F190").c_str());
  TEST_ASSERT_EQUAL_size_t(0, hal.wire().size());
  run(hal, elm, "ATSH7E0");
  TEST_ASSERT_EQUAL_STRING("41 00 BE 1F A8 12\r\r>", ask(hal, elm, "0100").c_str());
}

static void surveyed_traffic_id_is_refused(void) {
  TestHal hal(carWithSlow60D());
  Elm elm(hal);
  elm.begin();
  for (const char *cmd : {"ATE0", "AT#SURVEY", "ATSP6", "ATSH60D"}) run(hal, elm, cmd);
  TEST_ASSERT_EQUAL_STRING("BLOCKED\r\r>", ask(hal, elm, "22F190").c_str());
  TEST_ASSERT_EQUAL_size_t(0, hal.wire().size());
}

static void answers_to_a_request_are_not_traffic(void) {
  TestHal hal(renaultDaciaCarQuiet());
  Elm elm(hal);
  elm.begin();
  for (const char *cmd : {"ATE0", "ATSP6", "ATSH7E0"}) run(hal, elm, cmd);
  TEST_ASSERT_EQUAL_STRING("41 00 BE 1F A8 12\r\r>", ask(hal, elm, "0100").c_str());
  run(hal, elm, "ATSH7E8");
  TEST_ASSERT_EQUAL_STRING("NO DATA\r\r>", ask(hal, elm, "0100").c_str());
  TEST_ASSERT_TRUE(hal.wire().back().second.id == CanId::std11(0x7E8));
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
  RUN_TEST(read_services_pass_at_read);
  RUN_TEST(clear_services_need_clear);
  RUN_TEST(write_and_unknown_services_need_write);
  RUN_TEST(session_control_is_read_except_programming);
  RUN_TEST(single_frame_sid_is_byte_one);
  RUN_TEST(invalid_single_frame_is_denied);
  RUN_TEST(first_frame_sid_is_byte_two);
  RUN_TEST(consecutive_frames_inherit_the_first_frame);
  RUN_TEST(consecutive_frame_is_judged_against_the_current_tier);
  RUN_TEST(single_frame_ends_a_segmented_request);
  RUN_TEST(flow_control_is_allowed_at_read);
  RUN_TEST(other_pci_and_empty_frames_are_denied);
  RUN_TEST(only_diagnostic_ids_pass_below_write);
  RUN_TEST(reads_on_diagnostic_ids_pass_at_read);
  RUN_TEST(raw_frame_on_an_operational_id_is_blocked);
  RUN_TEST(ids_heard_as_traffic_are_refused_below_write);
  RUN_TEST(the_guard_keeps_128_heard_ids);
  RUN_TEST(traffic_outside_the_diagnostic_range_leaves_room_for_heard_ids);
  RUN_TEST(listen_before_talk_traffic_id_needs_write);
  RUN_TEST(monitored_traffic_id_is_refused_until_power_up);
  RUN_TEST(surveyed_traffic_id_is_refused);
  RUN_TEST(answers_to_a_request_are_not_traffic);
  RUN_TEST(tier_survives_reset_commands_and_is_settable);
  return UNITY_END();
}

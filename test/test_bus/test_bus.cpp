#include <unity.h>

#include <string>

#include "test_hal.h"

void setUp(void) {}
void tearDown(void) {}

static const uint8_t kPid00[] = {0x01, 0x00};
static const uint8_t kPid00Ok[] = {0x41, 0x00, 0xBE, 0x1F, 0xA8, 0x12};
static const Exchange kPid00Only[] = {{kPid00, sizeof kPid00, kPid00Ok, sizeof kPid00Ok, 0}};
static const EcuSpec kThreeObdEcus[] = {
    {"ECM", CanId::std11(0x7E0), CanId::std11(0x7E8), true, kPid00Only, 1, 2000},
    {"TCM", CanId::std11(0x7E1), CanId::std11(0x7E9), true, kPid00Only, 1, 2000},
    {"ABS", CanId::std11(0x7E2), CanId::std11(0x7EA), true, kPid00Only, 1, 2000},
};
static const CarSpec kThreeObdCar = {kThreeObdEcus, 3, nullptr, 0, Bitrate::k500};

static const Periodic kEvery50Ms[] = {
    {{CanId::std11(0x181), 8, {0x0B, 0xB8, 0x00, 0x00, 0x1E, 0x00, 0x00, 0x00}}, 50}};
static const Periodic kEverySecond[] = {
    {{CanId::std11(0x181), 8, {0x0B, 0xB8, 0x00, 0x00, 0x1E, 0x00, 0x00, 0x00}}, 1000}};

static CarSpec quietCarWith(const Periodic *traffic) {
  CarSpec car = renaultDaciaCarQuiet();
  car.traffic = traffic;
  car.trafficCount = 1;
  return car;
}

static void expectListenOnly(TestHal &hal, const char *after) {
  TEST_ASSERT_TRUE_MESSAGE(hal.controllerMode() == CanMode::ListenOnly, after);
}

static void the_controller_is_listen_only_at_every_prompt(void) {
  TestHal hal(renaultDaciaCarQuiet());
  Elm elm(hal);
  elm.begin();
  runAll(hal, elm, {"ATE0", "ATSH7E0", "ATSP8"});
  TEST_ASSERT_EQUAL_STRING("CAN ERROR\r\r>", ask(hal, elm, "0100").c_str());
  TEST_ASSERT_EQUAL_size_t_MESSAGE(1, hal.sendAttempts(), "a silent bus at the wrong bitrate");
  expectListenOnly(hal, "after a send that failed");
  runAll(hal, elm, {"ATSP6", "0100"});
  expectListenOnly(hal, "after an answer");
  run(hal, elm, "09021");
  expectListenOnly(hal, "after a count-digit stop");
  runAll(hal, elm, {"ATR0", "0100"});
  expectListenOnly(hal, "after a send with responses off");
}

static void a_byte_during_listen_before_talk_stops_with_nothing_sent(void) {
  TestHal hal(renaultDaciaCar());
  Elm elm(hal);
  elm.begin();
  runAll(hal, elm, {"ATE0", "ATSP8", "ATSH7E0"});
  hal.clearOutput();
  hal.type("0100\r");
  hal.typeAt(hal.now() / 1000 + 20, "\r");
  elm.service();
  TEST_ASSERT_EQUAL_STRING("STOPPED\r\r>", hal.output().c_str());
  TEST_ASSERT_EQUAL_size_t(0, hal.sendAttempts());
}

static void a_byte_during_the_protocol_search_stops_it(void) {
  TestHal hal(renaultDaciaCarQuiet(Bitrate::k250));
  Elm elm(hal);
  elm.begin();
  runAll(hal, elm, {"ATE0", "ATSP0"});
  hal.clearOutput();
  hal.type("0100\r");
  hal.typeAt(hal.now() / 1000 + 20, "\r");
  elm.service();
  TEST_ASSERT_EQUAL_STRING("SEARCHING...\rSTOPPED\r\r>", hal.output().c_str());
  TEST_ASSERT_EQUAL_size_t(0, hal.sendAttempts());
  TEST_ASSERT_EQUAL_STRING("A0\r\r>", ask(hal, elm, "ATDPN").c_str());
}

static void consecutive_frames_owed_after_flow_control_get_the_full_at_st(void) {
  TestHal hal(renaultDaciaCarQuiet());
  Elm elm(hal);
  elm.begin();
  pyrenSetup(hal, elm, "7E0", "7E8");
  run(hal, elm, "AT FC SD 30 00 64");
  TEST_ASSERT_EQUAL_STRING_MESSAGE("0322F190\r"
                                   "101462F190555531\r"
                                   "2135534441473535\r"
                                   "2231323334353637\r"
                                   "\r>",
                                   ask(hal, elm, "0322F190").c_str(),
                                   "frames 100 ms apart outlast the 50 ms adaptive window");
}

static void a_frame_the_controller_drops_ends_in_buffer_full(void) {
  TestHal hal(kThreeObdCar);
  Elm elm(hal);
  elm.begin();
  runAll(hal, elm, {"ATE0", "ATH1", "ATSP6"});
  TEST_ASSERT_EQUAL_STRING("7E8 06 41 00 BE 1F A8 12 00\r"
                           "7E9 06 41 00 BE 1F A8 12 00\r"
                           "BUFFER FULL\r\r>",
                           ask(hal, elm, "0100").c_str());
}

static void cfc0_sends_no_flow_control(void) {
  TestHal hal(renaultDaciaCarQuiet());
  Elm elm(hal);
  elm.begin();
  pyrenSetup(hal, elm, "7E0", "7E8");
  run(hal, elm, "AT CFC0");
  TEST_ASSERT_EQUAL_STRING("0322F190\r101462F190555531\r\r>", ask(hal, elm, "0322F190").c_str());
  TEST_ASSERT_EQUAL_size_t_MESSAGE(1, hal.wire().size(), "only the request");
}

static void listen_before_talk_hears_a_50_ms_periodic_at_any_phase(void) {
  for (uint32_t phaseMs = 0; phaseMs < 50; phaseMs += 10) {
    TestHal hal(quietCarWith(kEvery50Ms));
    Elm elm(hal);
    elm.begin();
    runAll(hal, elm, {"ATE0", "ATSP8", "ATSH7E0"});
    hal.sleepMs(phaseMs);
    TEST_ASSERT_EQUAL_STRING("CAN ERROR\r\r>", ask(hal, elm, "0100").c_str());
    TEST_ASSERT_EQUAL_size_t(0, hal.sendAttempts());
  }
}

static void listen_before_talk_counts_only_errors_inside_its_window(void) {
  TestHal hal(quietCarWith(kEverySecond));
  Elm elm(hal);
  elm.begin();
  runAll(hal, elm, {"ATE0", "ATSP8", "ATSH7E0"});
  TEST_ASSERT_EQUAL_STRING_MESSAGE("CAN ERROR\r\r>", ask(hal, elm, "0100").c_str(),
                                   "the window hears the 500 kbit frame as errors");
  TEST_ASSERT_EQUAL_size_t(0, hal.sendAttempts());
  hal.sleepMs(1100);
  TEST_ASSERT_EQUAL_STRING("CAN ERROR\r\r>", ask(hal, elm, "0100").c_str());
  TEST_ASSERT_EQUAL_size_t_MESSAGE(1, hal.sendAttempts(),
                                   "a window between two frames hears nothing, as on a silent bus");
}

static void frames_left_from_an_earlier_request_are_not_printed(void) {
  TestHal hal(renaultDaciaCarQuiet());
  Elm elm(hal);
  elm.begin();
  runAll(hal, elm, {"ATE0", "ATH1", "ATSP6", "ATSH7E0"});
  TEST_ASSERT_EQUAL_STRING("7E8 10 14 49 02 01 55 55 31\r\r>", ask(hal, elm, "09021").c_str());
  hal.sleepMs(10);
  TEST_ASSERT_EQUAL_STRING_MESSAGE("7E8 06 41 00 BE 1F A8 12 00\r\r>", ask(hal, elm, "0100").c_str(),
                                   "the rest of the VIN came after the digit ended the wait");
}

static void a_count_digit_of_0_sends_without_waiting(void) {
  TestHal hal(renaultDaciaCarQuiet());
  Elm elm(hal);
  elm.begin();
  runAll(hal, elm, {"ATE0", "ATSP6", "ATSH7E0", "0100"});
  const size_t sent = hal.wire().size();
  const uint32_t start = hal.now();
  TEST_ASSERT_EQUAL_STRING("\r>", ask(hal, elm, "01000").c_str());
  TEST_ASSERT_EQUAL_UINT32(start, hal.now());
  TEST_ASSERT_EQUAL_size_t(sent + 1, hal.wire().size());
}

static void k_line_protocols_are_unable_to_connect(void) {
  TestHal hal(renaultDaciaCarQuiet());
  Elm elm(hal);
  elm.begin();
  run(hal, elm, "ATE0");
  for (const char *protocol : {"ATSP1", "ATSP2", "ATSP3", "ATSP4", "ATSP5"}) {
    run(hal, elm, protocol);
    TEST_ASSERT_EQUAL_STRING_MESSAGE("UNABLE TO CONNECT\r\r>", ask(hal, elm, "0100").c_str(),
                                     protocol);
  }
  TEST_ASSERT_EQUAL_size_t(0, hal.sendAttempts());
}

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(the_controller_is_listen_only_at_every_prompt);
  RUN_TEST(a_byte_during_listen_before_talk_stops_with_nothing_sent);
  RUN_TEST(a_byte_during_the_protocol_search_stops_it);
  RUN_TEST(consecutive_frames_owed_after_flow_control_get_the_full_at_st);
  RUN_TEST(a_frame_the_controller_drops_ends_in_buffer_full);
  RUN_TEST(cfc0_sends_no_flow_control);
  RUN_TEST(listen_before_talk_hears_a_50_ms_periodic_at_any_phase);
  RUN_TEST(listen_before_talk_counts_only_errors_inside_its_window);
  RUN_TEST(frames_left_from_an_earlier_request_are_not_printed);
  RUN_TEST(a_count_digit_of_0_sends_without_waiting);
  RUN_TEST(k_line_protocols_are_unable_to_connect);
  return UNITY_END();
}

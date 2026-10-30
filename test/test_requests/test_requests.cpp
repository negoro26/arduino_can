#include <unity.h>

#include <string>

#include "test_hal.h"

void setUp(void) {}
void tearDown(void) {}

static const uint8_t kPid00[] = {0x01, 0x00};
static const uint8_t kPid00Ok[] = {0x41, 0x00, 0xBE, 0x1F, 0xA8, 0x12};
static const uint8_t kVin[] = {0x09, 0x02};
static const uint8_t kVinOk[] = {0x49, 0x02, 0x01, 'U', 'U', '1', '5', 'S', 'D', 'A', 'G',
                                 '5', '5', '1', '2', '3', '4', '5', '6', '7'};
static const Exchange kObd[] = {{kPid00, sizeof kPid00, kPid00Ok, sizeof kPid00Ok, 0},
                                {kVin, sizeof kVin, kVinOk, sizeof kVinOk, 0}};
static const EcuSpec kEcm29 = {"ECM", CanId::ext29(0x18DA10F1), CanId::ext29(0x18DAF110),
                               false, kObd, 2, 2000};
static const Periodic kJ1939Traffic[] = {
    {{CanId::ext29(0x0CF00400), 8, {0xF0, 0xFF, 0x7D, 0, 0, 0, 0, 0xFF}}, 10}};
static const CarSpec kCar29 = {&kEcm29, 1, kJ1939Traffic, 1, Bitrate::k500};

static void a_negative_answer_ends_the_wait_unlike_response_pending(void) {
  TestHal hal(renaultDaciaCarQuiet());
  Elm elm(hal);
  elm.begin();
  runAll(hal, elm, {"ATE0", "ATS0", "ATCAF0", "ATSP6", "ATSH752", "ATCRA772", "ATST19"});
  TEST_ASSERT_EQUAL_STRING("037F223100000000\r\r>", ask(hal, elm, "0322F190").c_str());
  TEST_ASSERT_TRUE_MESSAGE(hal.now() - hal.wire().back().first < 110000,
                           "7F 22 31 ends the request; only 7F xx 78 holds it for 5 s");
}

static void an_overlong_line_answers_question_mark(void) {
  TestHal hal(renaultDaciaCarQuiet());
  Elm elm(hal);
  elm.begin();
  run(hal, elm, "ATE0");
  TEST_ASSERT_EQUAL_STRING("?\r\r>", ask(hal, elm, std::string(200, '0')).c_str());
  TEST_ASSERT_EQUAL_STRING("ELM327 v1.5\r\r>", ask(hal, elm, "ATI").c_str());
}

static void a_request_too_long_for_one_frame_is_refused_and_not_sent(void) {
  TestHal hal(renaultDaciaCarQuiet());
  Elm elm(hal);
  elm.begin();
  runAll(hal, elm, {"ATE0", "ATSP6", "ATSH7E0"});
  TEST_ASSERT_EQUAL_STRING_MESSAGE("?\r\r>", ask(hal, elm, "22F1901122334455").c_str(),
                                   "CAF1 leaves room for 7 bytes after the PCI");
  run(hal, elm, "ATCAF0");
  TEST_ASSERT_EQUAL_STRING("?\r\r>", ask(hal, elm, "112233445566778899").c_str());
  TEST_ASSERT_EQUAL_size_t(0, hal.sendAttempts());

  TEST_ASSERT_EQUAL_STRING("04 62 F1 A0 44 00 00 00\r\r>",
                           ask(hal, elm, "0322F1A000000000").c_str());
  TEST_ASSERT_EQUAL_size_t(1, hal.wire().size());
  TEST_ASSERT_TRUE(sameBytes(hal.wire()[0].second, {0x03, 0x22, 0xF1, 0xA0, 0, 0, 0, 0}));
}

static void v1_sends_the_frame_at_its_own_length(void) {
  TestHal hal(renaultDaciaCarQuiet());
  Elm elm(hal);
  elm.begin();
  runAll(hal, elm, {"ATE0", "ATSP6", "ATCAF0", "ATV1", "ATSH745", "ATCRA765"});
  run(hal, elm, "0210C0");
  TEST_ASSERT_EQUAL_size_t(1, hal.wire().size());
  TEST_ASSERT_TRUE(hal.wire()[0].second.id == CanId::std11(0x745));
  TEST_ASSERT_TRUE(sameBytes(hal.wire()[0].second, {0x02, 0x10, 0xC0}));
}

static void a_29_bit_request_goes_out_on_the_header_at_full_width(void) {
  TestHal hal(renaultDaciaCarQuiet());
  Elm elm(hal);
  elm.begin();
  runAll(hal, elm, {"ATE0", "ATSP7", "0100"});
  TEST_ASSERT_TRUE_MESSAGE(hal.wire().back().second.id == CanId::ext29(0x18DB33F1),
                           "no header: the 29-bit functional id");
  runAll(hal, elm, {"ATCP18", "ATSH DA5DF2", "0100"});
  TEST_ASSERT_TRUE_MESSAGE(hal.wire().back().second.id == CanId::ext29(0x18DA5DF2),
                           "6 digits after AT CP 18, as pyren sets it");

  TestHal slow(renaultDaciaCarQuiet(Bitrate::k250));
  Elm elm250(slow);
  elm250.begin();
  runAll(slow, elm250, {"ATE0", "ATSP9", "ATSH18DA10F1", "0100"});
  TEST_ASSERT_EQUAL_size_t(1, slow.wire().size());
  TEST_ASSERT_TRUE(slow.wire()[0].second.id == CanId::ext29(0x18DA10F1));
}

static void a_29_bit_responder_prints_its_four_byte_id_and_gets_flow_control(void) {
  TestHal hal(kCar29);
  Elm elm(hal);
  elm.begin();
  runAll(hal, elm, {"ATE0", "ATSP7", "ATH1", "ATSH DA10F1"});
  TEST_ASSERT_EQUAL_STRING_MESSAGE("18 DA F1 10 06 41 00 BE 1F A8 12 00\r\r>",
                                   ask(hal, elm, "0100").c_str(),
                                   "the 0CF00400 traffic is not an answer");
  TEST_ASSERT_EQUAL_STRING("18 DA F1 10 10 14 49 02 01 55 55 31\r"
                           "18 DA F1 10 21 35 53 44 41 47 35 35\r"
                           "18 DA F1 10 22 31 32 33 34 35 36 37\r\r>",
                           ask(hal, elm, "0902").c_str());
  const CanFrame &fc = hal.wire().back().second;
  TEST_ASSERT_TRUE(fc.id == CanId::ext29(0x18DA10F1));
  TEST_ASSERT_TRUE(sameBytes(fc, {0x30, 0x00, 0x00, 0, 0, 0, 0, 0}));
}

static void flow_control_for_a_responder_outside_7E8_goes_to_the_header(void) {
  TestHal hal(renaultDaciaCarQuiet());
  Elm elm(hal);
  elm.begin();
  runAll(hal, elm, {"ATE0", "ATSP6", "ATH1", "ATSH745", "ATCRA765"});
  TEST_ASSERT_EQUAL_STRING("765 10 1A 61 80 82 00 44 66\r"
                           "765 21 27 44 32 31 33 82 00\r"
                           "765 22 38 71 38 00 A7 75 00\r"
                           "765 23 00 00 00 00 00 00 00\r\r>",
                           ask(hal, elm, "2180").c_str());
  const CanFrame &fc = hal.wire().back().second;
  TEST_ASSERT_TRUE(fc.id == CanId::std11(0x745));
  TEST_ASSERT_TRUE(sameBytes(fc, {0x30, 0x00, 0x00, 0, 0, 0, 0, 0}));
}

static void flow_control_modes_1_and_2_send_the_set_data(void) {
  TestHal hal(renaultDaciaCarQuiet());
  Elm elm(hal);
  elm.begin();
  runAll(hal, elm, {"ATE0", "ATSP6", "ATH1", "ATSH7E0", "ATFCSH7E5", "ATFCSD30080A", "ATFCSM1"});
  run(hal, elm, "0902");
  const CanFrame &sm1 = hal.wire().back().second;
  TEST_ASSERT_TRUE_MESSAGE(sm1.id == CanId::std11(0x7E5), "SM1: on the AT FC SH id");
  TEST_ASSERT_TRUE(sameBytes(sm1, {0x30, 0x08, 0x0A, 0, 0, 0, 0, 0}));

  run(hal, elm, "ATFCSM2");
  TEST_ASSERT_EQUAL_STRING("7E8 10 14 49 02 01 55 55 31\r"
                           "7E8 21 35 53 44 41 47 35 35\r"
                           "7E8 22 31 32 33 34 35 36 37\r\r>",
                           ask(hal, elm, "0902").c_str());
  const CanFrame &sm2 = hal.wire().back().second;
  TEST_ASSERT_TRUE_MESSAGE(sm2.id == CanId::std11(0x7E0), "SM2: on the automatic id");
  TEST_ASSERT_TRUE(sameBytes(sm2, {0x30, 0x08, 0x0A, 0, 0, 0, 0, 0}));
}

static void wait_windows_follow_at_st_and_adaptive_timing(void) {
  TestHal hal(renaultDaciaCarQuiet());
  Elm elm(hal);
  elm.begin();
  runAll(hal, elm, {"ATE0", "ATSP6", "ATSH7E0", "0100"});
  const struct {
    const char *setting;
    uint32_t afterAnswerMs;
  } windows[] = {{"ATAT1", 50}, {"ATAT2", 25}, {"ATAT0", 200}, {"ATAT1", 50}, {"ATST05", 20}};
  for (const auto &w : windows) {
    run(hal, elm, w.setting);
    run(hal, elm, "0100");
    TEST_ASSERT_UINT32_WITHIN_MESSAGE(1000, 2000 + w.afterAnswerMs * 1000,
                                      hal.now() - hal.wire().back().first, w.setting);
  }
  runAll(hal, elm, {"ATST00", "ATSH742"});
  TEST_ASSERT_EQUAL_STRING("NO DATA\r\r>", ask(hal, elm, "0100").c_str());
  TEST_ASSERT_UINT32_WITHIN(1000, 200000, hal.now() - hal.wire().back().first);
  run(hal, elm, "ATSTFF");
  TEST_ASSERT_EQUAL_STRING("NO DATA\r\r>", ask(hal, elm, "0100").c_str());
  TEST_ASSERT_UINT32_WITHIN(1000, 1020000, hal.now() - hal.wire().back().first);
}

static void headers_dlc_and_spaces_shape_each_printed_line(void) {
  TestHal hal(renaultDaciaCarQuiet());
  Elm elm(hal);
  elm.begin();
  runAll(hal, elm, {"ATE0", "ATSP6", "ATH1", "ATD1"});
  TEST_ASSERT_EQUAL_STRING("7E8 8 06 41 00 BE 1F A8 12 00\r\r>", ask(hal, elm, "0100").c_str());
  runAll(hal, elm, {"ATD0", "ATS0"});
  TEST_ASSERT_EQUAL_STRING("7E8064100BE1FA81200\r\r>", ask(hal, elm, "0100").c_str());
}

static void a_multi_frame_answer_drops_the_last_frame_padding(void) {
  TestHal hal(renaultDaciaCarQuiet());
  Elm elm(hal);
  elm.begin();
  runAll(hal, elm, {"ATE0", "ATSP6", "ATSH7E0"});
  TEST_ASSERT_EQUAL_STRING("00B\r0: 62 F1 94 53 31 38\r1: 30 30 36 37 41\r\r>",
                           ask(hal, elm, "22F194").c_str());
}

static void spaces_off_keeps_the_sequence_prefix(void) {
  TestHal hal(renaultDaciaCarQuiet());
  Elm elm(hal);
  elm.begin();
  runAll(hal, elm, {"ATE0", "ATSP6", "ATS0"});
  TEST_ASSERT_EQUAL_STRING("014\r0:490201555531\r1:35534441473535\r2:31323334353637\r\r>",
                           ask(hal, elm, "0902").c_str());
}

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(a_negative_answer_ends_the_wait_unlike_response_pending);
  RUN_TEST(an_overlong_line_answers_question_mark);
  RUN_TEST(a_request_too_long_for_one_frame_is_refused_and_not_sent);
  RUN_TEST(v1_sends_the_frame_at_its_own_length);
  RUN_TEST(a_29_bit_request_goes_out_on_the_header_at_full_width);
  RUN_TEST(a_29_bit_responder_prints_its_four_byte_id_and_gets_flow_control);
  RUN_TEST(flow_control_for_a_responder_outside_7E8_goes_to_the_header);
  RUN_TEST(flow_control_modes_1_and_2_send_the_set_data);
  RUN_TEST(wait_windows_follow_at_st_and_adaptive_timing);
  RUN_TEST(headers_dlc_and_spaces_shape_each_printed_line);
  RUN_TEST(a_multi_frame_answer_drops_the_last_frame_padding);
  RUN_TEST(spaces_off_keeps_the_sequence_prefix);
  return UNITY_END();
}

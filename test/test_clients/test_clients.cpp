#include <unity.h>

#include <string>

#include "test_hal.h"

void setUp(void) {}
void tearDown(void) {}

static void runAll(TestHal &hal, Elm &elm, std::initializer_list<const char *> lines) {
  for (const char *line : lines) run(hal, elm, line);
}

// pyren mod_elm init_can + set_can_addr (ddt4all's are the same commands).
static void pyrenSetup(TestHal &hal, Elm &elm, const char *tx, const char *rx) {
  runAll(hal, elm, {"AT WS", "AT E1", "AT S0", "AT H0", "AT L0", "AT AL", "AT CAF0",
                    "AT CFC1"});
  run(hal, elm, std::string("AT SH ") + tx);
  run(hal, elm, std::string("AT FC SH ") + tx);
  runAll(hal, elm, {"AT FC SD 30 00 00", "AT FC SM 1", "AT ST FF", "AT AT 0", "AT SP 6",
                    "AT AT 1"});
  run(hal, elm, std::string("AT CRA ") + rx);
}

static bool sameBytes(const CanFrame &f, std::initializer_list<uint8_t> bytes) {
  if (f.dlc != bytes.size()) return false;
  uint8_t i = 0;
  for (uint8_t b : bytes) {
    if (f.data[i++] != b) return false;
  }
  return true;
}

static void pyren_reads_multi_frame_vin_raw_with_automatic_flow_control(void) {
  TestHal hal(renaultDaciaCarQuiet());
  Elm elm(hal);
  elm.begin();
  pyrenSetup(hal, elm, "7E0", "7E8");
  TEST_ASSERT_EQUAL_STRING("0322F190\r"
                           "101462F190555531\r"
                           "2135534441473535\r"
                           "2231323334353637\r"
                           "\r>",
                           ask(hal, elm, "0322F190").c_str());
  const auto &wire = hal.wire();
  TEST_ASSERT_EQUAL_size_t(2, wire.size());
  TEST_ASSERT_TRUE(wire[0].second.id == CanId::std11(0x7E0));
  TEST_ASSERT_TRUE(sameBytes(wire[0].second, {0x03, 0x22, 0xF1, 0x90, 0, 0, 0, 0}));
  TEST_ASSERT_TRUE(wire[1].second.id == CanId::std11(0x7E0));
  TEST_ASSERT_TRUE(sameBytes(wire[1].second, {0x30, 0x00, 0x00, 0, 0, 0, 0, 0}));
  TEST_ASSERT_EQUAL_UINT32(wire[0].first + 2000, wire[1].first);
}

static void ddt4all_reads_kwp_identification_2180(void) {
  TestHal hal(renaultDaciaCarQuiet());
  Elm elm(hal);
  elm.begin();
  pyrenSetup(hal, elm, "745", "765");
  TEST_ASSERT_EQUAL_STRING("0210C0\r0250C00000000000\r\r>", ask(hal, elm, "0210C0").c_str());
  TEST_ASSERT_EQUAL_STRING("022180\r"
                           "101A618082004466\r"
                           "2127443231338200\r"
                           "2238713800A77500\r"
                           "2300000000000000\r"
                           "\r>",
                           ask(hal, elm, "022180").c_str());
  TEST_ASSERT_TRUE(hal.wire().back().second.id == CanId::std11(0x745));
  TEST_ASSERT_EQUAL_HEX8(0x30, hal.wire().back().second.data[0]);
}

static void airbag_response_pending_then_final_answer_in_one_reply(void) {
  TestHal hal(renaultDaciaCarQuiet());
  Elm elm(hal);
  elm.begin();
  pyrenSetup(hal, elm, "752", "772");
  const uint32_t start = hal.now();
  TEST_ASSERT_EQUAL_STRING("0322F18A\r"
                           "037F227800000000\r"
                           "100862F18A54454D\r"
                           "2149430000000000\r"
                           "\r>",
                           ask(hal, elm, "0322F18A").c_str());
  TEST_ASSERT_TRUE(hal.now() - start > 300000);
  TEST_ASSERT_EQUAL_STRING("021003\r037F107800000000\r065003003201F400\r\r>",
                           ask(hal, elm, "021003").c_str());
}

static void absent_ecu_answers_no_data(void) {
  TestHal hal(renaultDaciaCarQuiet());
  Elm elm(hal);
  elm.begin();
  pyrenSetup(hal, elm, "742", "762");
  TEST_ASSERT_EQUAL_STRING("021003\rNO DATA\r\r>", ask(hal, elm, "021003").c_str());
}

static void pyren_count_digit_and_responses_off(void) {
  TestHal hal(renaultDaciaCarQuiet());
  Elm elm(hal);
  elm.begin();
  pyrenSetup(hal, elm, "7E0", "7E8");
  run(hal, elm, "0322F190");
  // pyren's cached frame count: stop as soon as 3 frames are in.
  uint32_t start = hal.now();
  TEST_ASSERT_EQUAL_STRING("0322F1903\r101462F190555531\r2135534441473535\r2231323334353637\r\r>",
                           ask(hal, elm, "0322F1903").c_str());
  TEST_ASSERT_TRUE(hal.now() - start < 3000);
  start = hal.now();
  TEST_ASSERT_EQUAL_STRING("AT R0\rOK\r\r>", ask(hal, elm, "AT R0").c_str());
  TEST_ASSERT_EQUAL_STRING("023E00\r\r>", ask(hal, elm, "023E00").c_str());
  TEST_ASSERT_TRUE(hal.now() == start);
  TEST_ASSERT_TRUE(sameBytes(hal.wire().back().second, {0x02, 0x3E, 0x00, 0, 0, 0, 0, 0}));
}

static void python_obd_connects_with_search(void) {
  TestHal hal(renaultDaciaCarQuiet());
  Elm elm(hal);
  elm.begin();
  run(hal, elm, "ATZ");
  TEST_ASSERT_EQUAL_STRING("ATE0\rOK\r\r>", ask(hal, elm, "ATE0").c_str());
  TEST_ASSERT_EQUAL_STRING("OK\r\r>", ask(hal, elm, "ATH1").c_str());
  TEST_ASSERT_EQUAL_STRING("OK\r\r>", ask(hal, elm, "ATL0").c_str());
  TEST_ASSERT_EQUAL_STRING("12.0V\r\r>", ask(hal, elm, "AT RV").c_str());
  TEST_ASSERT_EQUAL_STRING("OK\r\r>", ask(hal, elm, "ATSP0").c_str());
  TEST_ASSERT_EQUAL_STRING("SEARCHING...\r7E8 06 41 00 BE 1F A8 12 00\r\r>",
                           ask(hal, elm, "0100").c_str());
  TEST_ASSERT_TRUE(hal.wire()[0].second.id == CanId::std11(0x7DF));
  TEST_ASSERT_EQUAL_STRING("A6\r\r>", ask(hal, elm, "ATDPN").c_str());
}

static void python_obd_count_digit_and_bare_cr_repeat(void) {
  TestHal hal(renaultDaciaCarQuiet());
  Elm elm(hal);
  elm.begin();
  runAll(hal, elm, {"ATZ", "ATE0", "ATH1", "ATL0", "ATSP0", "0100"});
  uint32_t start = hal.now();
  TEST_ASSERT_EQUAL_STRING("7E8 04 41 0C 0B B8 00 00 00\r\r>", ask(hal, elm, "010C1").c_str());
  TEST_ASSERT_TRUE(hal.now() - start < 3000);
  start = hal.now();
  TEST_ASSERT_EQUAL_STRING("7E8 04 41 0C 0B B8 00 00 00\r\r>", ask(hal, elm, "").c_str());
  TEST_ASSERT_TRUE(hal.now() - start < 3000);
  TEST_ASSERT_EQUAL_STRING("7E8 10 14 49 02 01 55 55 31\r"
                           "7E8 21 35 53 44 41 47 35 35\r"
                           "7E8 22 31 32 33 34 35 36 37\r\r>",
                           ask(hal, elm, "09023").c_str());
  TEST_ASSERT_TRUE(hal.wire().back().second.id == CanId::std11(0x7E0));
  TEST_ASSERT_EQUAL_STRING("7E8 03 41 05 5A 00 00 00 00\r\r>", ask(hal, elm, "0105").c_str());
  TEST_ASSERT_EQUAL_STRING("7E8 06 43 02 03 35 01 15 00\r\r>", ask(hal, elm, "03").c_str());
}

static void headers_off_multi_frame_layout(void) {
  TestHal hal(renaultDaciaCarQuiet());
  Elm elm(hal);
  elm.begin();
  runAll(hal, elm, {"ATE0", "ATSP6"});
  TEST_ASSERT_EQUAL_STRING("014\r0: 49 02 01 55 55 31\r1: 35 53 44 41 47 35 35\r"
                           "2: 31 32 33 34 35 36 37\r\r>",
                           ask(hal, elm, "0902").c_str());
  TEST_ASSERT_EQUAL_STRING("41 0C 0B B8\r\r>", ask(hal, elm, "010C").c_str());
}

static void byte_during_a_wait_stops_and_the_rest_of_its_line_is_dropped(void) {
  TestHal hal(renaultDaciaCarQuiet());
  Elm elm(hal);
  elm.begin();
  pyrenSetup(hal, elm, "752", "772");
  hal.clearOutput();
  hal.type("0322F18A\r");
  // ddt4all interrupts with "AT\r" while the airbag's 7F 22 78 holds the line.
  hal.typeAt(hal.now() / 1000 + 150, "AT\r");
  elm.service();
  TEST_ASSERT_EQUAL_STRING("0322F18A\r037F227800000000\rSTOPPED\r\r>", hal.output().c_str());
  TEST_ASSERT_EQUAL_STRING("ATI\rELM327 v1.5\r\r>", ask(hal, elm, "ATI").c_str());
}

static void input_after_the_100ms_window_is_a_new_command(void) {
  TestHal hal(renaultDaciaCarQuiet());
  Elm elm(hal);
  elm.begin();
  pyrenSetup(hal, elm, "752", "772");
  hal.clearOutput();
  const uint32_t t0 = hal.now() / 1000;
  hal.type("0322F18A\r");
  hal.typeAt(t0 + 150, "X");
  hal.typeAt(t0 + 300, "ATI\r");
  elm.service();
  TEST_ASSERT_EQUAL_STRING("0322F18A\r037F227800000000\rSTOPPED\r\r>", hal.output().c_str());
  hal.clearOutput();
  hal.sleepMs(100);
  elm.service();
  TEST_ASSERT_EQUAL_STRING("ATI\rELM327 v1.5\r\r>", hal.output().c_str());
}

static void monitor_prints_traffic_until_a_byte_arrives(void) {
  TestHal hal(renaultDaciaCar());
  Elm elm(hal);
  elm.begin();
  runAll(hal, elm, {"ATE0", "ATH1", "ATSP6"});
  hal.clearOutput();
  hal.type("ATMA\r");
  // tools/canscan.py stops ATMA with a lone CR.
  hal.typeAt(hal.now() / 1000 + 25, "\r");
  elm.service();
  TEST_ASSERT_EQUAL_STRING("181 0B B8 00 00 1E 00 00 00\r"
                           "354 00 00 00 00 00 00 00 00\r"
                           "622 00 81 00 00 00 00 00 00\r"
                           "5DE 00 00 00 00 00 00 00 00\r"
                           "181 0B B8 00 00 1E 00 00 00\r"
                           "181 0B B8 00 00 1E 00 00 00\r"
                           "354 00 00 00 00 00 00 00 00\r"
                           "STOPPED\r\r>",
                           hal.output().c_str());
  TEST_ASSERT_EQUAL_size_t(0, hal.sendAttempts());
}

static void wrong_bitrate_is_can_error_with_nothing_transmitted(void) {
  TestHal hal(renaultDaciaCar());
  Elm elm(hal);
  elm.begin();
  pyrenSetup(hal, elm, "745", "765");
  run(hal, elm, "AT SP 8");
  TEST_ASSERT_EQUAL_STRING("0210C0\rCAN ERROR\r\r>", ask(hal, elm, "0210C0").c_str());
  TEST_ASSERT_EQUAL_size_t(0, hal.sendAttempts());
  run(hal, elm, "AT SP 6");
  TEST_ASSERT_EQUAL_STRING("0210C0\r0250C00000000000\r\r>", ask(hal, elm, "0210C0").c_str());
}

static void wrong_bitrate_on_a_silent_bus_fails_after_one_attempt(void) {
  TestHal hal(renaultDaciaCarQuiet());
  Elm elm(hal);
  elm.begin();
  pyrenSetup(hal, elm, "745", "765");
  run(hal, elm, "AT SP 8");
  TEST_ASSERT_EQUAL_STRING("0210C0\rCAN ERROR\r\r>", ask(hal, elm, "0210C0").c_str());
  TEST_ASSERT_EQUAL_size_t(1, hal.sendAttempts());
  TEST_ASSERT_EQUAL_size_t(0, hal.wire().size());
}

static void blocked_request_touches_nothing(void) {
  TestHal hal(renaultDaciaCarQuiet());
  Elm elm(hal);
  elm.begin();
  pyrenSetup(hal, elm, "745", "765");
  const uint32_t start = hal.now();
  TEST_ASSERT_EQUAL_STRING("0314FF00\rBLOCKED\r\r>", ask(hal, elm, "0314FF00").c_str());
  TEST_ASSERT_EQUAL_STRING("042E010203\rBLOCKED\r\r>", ask(hal, elm, "042E010203").c_str());
  TEST_ASSERT_EQUAL_STRING("100A2E0102030405\rBLOCKED\r\r>",
                           ask(hal, elm, "100A2E0102030405").c_str());
  TEST_ASSERT_EQUAL_STRING("2106070809\rBLOCKED\r\r>", ask(hal, elm, "2106070809").c_str());
  TEST_ASSERT_EQUAL_size_t(0, hal.wire().size());
  TEST_ASSERT_EQUAL_UINT32(start, hal.now());
}

static void blocked_request_skips_the_search(void) {
  TestHal hal(renaultDaciaCarQuiet());
  Elm elm(hal);
  elm.begin();
  runAll(hal, elm, {"ATE0", "ATSP0"});
  const uint32_t start = hal.now();
  TEST_ASSERT_EQUAL_STRING("BLOCKED\r\r>", ask(hal, elm, "14FF00").c_str());
  TEST_ASSERT_EQUAL_STRING("BLOCKED\r\r>", ask(hal, elm, "04").c_str());
  // Refused as 11-bit 1F6 and as 29-bit 180001F6.
  run(hal, elm, "ATSH1F6");
  TEST_ASSERT_EQUAL_STRING("BLOCKED\r\r>", ask(hal, elm, "0100").c_str());
  TEST_ASSERT_EQUAL_size_t(0, hal.wire().size());
  TEST_ASSERT_EQUAL_UINT32(start, hal.now());
  TEST_ASSERT_EQUAL_STRING("A0\r\r>", ask(hal, elm, "ATDPN").c_str());
}

static void search_with_a_header_skips_protocols_the_guard_refuses(void) {
  TestHal hal(renaultDaciaCarQuiet(Bitrate::k250));
  Elm elm(hal);
  elm.begin();
  // Protocol 7 would send 180007E0, which the guard refuses.
  runAll(hal, elm, {"ATE0", "ATSP0", "ATSH7E0"});
  TEST_ASSERT_EQUAL_STRING("SEARCHING...\r41 00 BE 1F A8 12\r\r>", ask(hal, elm, "0100").c_str());
  TEST_ASSERT_EQUAL_STRING("A8\r\r>", ask(hal, elm, "ATDPN").c_str());
  for (const auto &sent : hal.wire()) TEST_ASSERT_TRUE(sent.second.id == CanId::std11(0x7E0));
}

static void search_with_skipped_protocols_and_no_answer_is_unable_to_connect(void) {
  const struct {
    const char *header;
    CanId id;
  } cases[] = {{"7E5", CanId::std11(0x7E5)}, {"18DA10F1", CanId::ext29(0x18DA10F1)}};
  for (const auto &c : cases) {
    TestHal hal(renaultDaciaCarQuiet(Bitrate::k250));
    Elm elm(hal);
    elm.begin();
    runAll(hal, elm, {"ATE0", "ATSP0"});
    run(hal, elm, std::string("ATSH") + c.header);
    TEST_ASSERT_EQUAL_STRING("SEARCHING...\rUNABLE TO CONNECT\r\r>", ask(hal, elm, "0100").c_str());
    TEST_ASSERT_EQUAL_size_t_MESSAGE(2, hal.sendAttempts(), "one try at each bitrate");
    TEST_ASSERT_EQUAL_size_t_MESSAGE(1, hal.wire().size(), "only the 250 kbit try reaches the car");
    TEST_ASSERT_TRUE(hal.wire()[0].second.id == c.id);
  }
}

static void guard_clear_lets_the_clear_request_through(void) {
  TestHal hal(renaultDaciaCarQuiet());
  Elm elm(hal);
  elm.begin();
  runAll(hal, elm, {"ATE0", "ATSP6", "ATSH7E0"});
  TEST_ASSERT_EQUAL_STRING("BLOCKED\r\r>", ask(hal, elm, "14FFFFFF").c_str());
  TEST_ASSERT_EQUAL_STRING("OK\r\r>", ask(hal, elm, "AT#GUARD CLEAR").c_str());
  TEST_ASSERT_EQUAL_STRING("CLEAR\r\r>", ask(hal, elm, "AT#GUARD").c_str());
  TEST_ASSERT_EQUAL_STRING("54\r\r>", ask(hal, elm, "14FFFFFF").c_str());
  TEST_ASSERT_TRUE(sameBytes(hal.wire().back().second, {0x04, 0x14, 0xFF, 0xFF, 0xFF, 0, 0, 0}));
  TEST_ASSERT_EQUAL_STRING("BLOCKED\r\r>", ask(hal, elm, "2E010203").c_str());
}

static const uint8_t kBigReq[] = {0x22, 0xF1, 0x99};
static const uint8_t kBigRsp[64] = {0x62, 0xF1, 0x99, 0x01, 0x02, 0x03, 0x04, 0x05,
                                    0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D,
                                    0x0E, 0x0F, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15,
                                    0x16, 0x17, 0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D,
                                    0x1E, 0x1F, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25,
                                    0x26, 0x27, 0x28, 0x29, 0x2A, 0x2B, 0x2C, 0x2D,
                                    0x2E, 0x2F, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35,
                                    0x36, 0x37, 0x38, 0x39, 0x3A, 0x3B, 0x3C, 0x3D};
static const Exchange kBig[] = {{kBigReq, sizeof kBigReq, kBigRsp, sizeof kBigRsp, 0}};
static const EcuSpec kBigEcu = {"BIG", CanId::std11(0x7E0), CanId::std11(0x7E8), false,
                                kBig, 1, 2000};

static void ten_frame_burst_with_slow_output_loses_nothing(void) {
  TestHal hal({&kBigEcu, 1, nullptr, 0, Bitrate::k500});
  Elm elm(hal);
  elm.begin();
  pyrenSetup(hal, elm, "7E0", "7E8");
  // 1 ms per byte written: one echoed or printed line inside the burst would
  // let three consecutive frames (230 us apart) hit the two RX buffers.
  hal.setWriteCostUs(1000);
  const std::string out = ask(hal, elm, "0322F199");

  std::string expected = "0322F199\r104062F199010203\r";
  static const char kHex[] = "0123456789ABCDEF";
  for (uint8_t seq = 1, at = 6; at < sizeof kBigRsp; ++seq) {
    std::string line = {'2', kHex[seq & 0x0F]};
    for (uint8_t i = 0; i < 7; ++i, ++at) {
      const uint8_t b = at < sizeof kBigRsp ? kBigRsp[at] : 0x00;
      line += kHex[b >> 4];
      line += kHex[b & 0x0F];
    }
    expected += line + "\r";
  }
  expected += "\r>";
  TEST_ASSERT_EQUAL_STRING(expected.c_str(), out.c_str());
  TEST_ASSERT_TRUE(out.find("BUFFER FULL") == std::string::npos);
  TEST_ASSERT_EQUAL_INT(12, std::count(out.begin(), out.end(), '\r'));
}

static void survey_finds_the_bitrate_and_lists_ids(void) {
  TestHal hal(renaultDaciaCar(Bitrate::k250));
  Elm elm(hal);
  elm.begin();
  run(hal, elm, "ATE0");
  const std::string out = ask(hal, elm, "AT#SURVEY");
  TEST_ASSERT_EQUAL_STRING_LEN("500 kbit: 0 frames, errors ", out.c_str(), 27);
  TEST_ASSERT_TRUE(out.find("\r250 kbit: 180 frames, errors 0, ids 181 354 622 5DE\r"
                            "WIRING: OK, bus decoding cleanly at 250 kbit\r\r>") !=
                   std::string::npos);
  TEST_ASSERT_EQUAL_size_t(0, hal.sendAttempts());
}

static void survey_on_a_silent_bus_reports_not_connected(void) {
  TestHal hal(renaultDaciaCarQuiet());
  Elm elm(hal);
  elm.begin();
  run(hal, elm, "ATE0");
  TEST_ASSERT_EQUAL_STRING(
      "500 kbit: 0 frames, errors 0\r250 kbit: 0 frames, errors 0\r"
      "125 kbit: 0 frames, errors 0\r100 kbit: 0 frames, errors 0\r"
      "WIRING: NOT CONNECTED, zero frames and zero errors at every rate:\r"
      "wrong OBD pins, broken cable, or bus asleep (ignition on?)\r\r>",
      ask(hal, elm, "AT#SURVEY").c_str());
}

static void survey_with_an_unconfigurable_controller_reports_unknown(void) {
  TestHal hal(renaultDaciaCarQuiet());
  Elm elm(hal);
  elm.begin();
  run(hal, elm, "ATE0");
  hal.setCanOpenFails(true);
  const std::string out = ask(hal, elm, "AT#SURVEY");
  TEST_ASSERT_EQUAL_STRING(
      "500 kbit: SETUP FAILED\r250 kbit: SETUP FAILED\r"
      "125 kbit: SETUP FAILED\r100 kbit: SETUP FAILED\r"
      "WIRING: UNKNOWN, the CAN controller could not be configured "
      "(run AT#SELFTEST)\r\r>",
      out.c_str());
  TEST_ASSERT_TRUE(out.find("NOT CONNECTED") == std::string::npos);
}

static void selftest_round_trips_a_frame_at_every_rate(void) {
  TestHal hal(renaultDaciaCar());
  Elm elm(hal);
  elm.begin();
  run(hal, elm, "ATE0");
  TEST_ASSERT_EQUAL_STRING("MCP2515 SIMULATED: PASS\rLOOPBACK 500 kbit: PASS\r"
                           "LOOPBACK 250 kbit: PASS\rLOOPBACK 125 kbit: PASS\r"
                           "LOOPBACK 100 kbit: PASS\rSELFTEST: PASS\r\r>",
                           ask(hal, elm, "AT#SELFTEST").c_str());
  TEST_ASSERT_TRUE_MESSAGE(hal.wire().empty(), "loopback frames never reach the car");
}

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(pyren_reads_multi_frame_vin_raw_with_automatic_flow_control);
  RUN_TEST(ddt4all_reads_kwp_identification_2180);
  RUN_TEST(airbag_response_pending_then_final_answer_in_one_reply);
  RUN_TEST(absent_ecu_answers_no_data);
  RUN_TEST(pyren_count_digit_and_responses_off);
  RUN_TEST(python_obd_connects_with_search);
  RUN_TEST(python_obd_count_digit_and_bare_cr_repeat);
  RUN_TEST(headers_off_multi_frame_layout);
  RUN_TEST(byte_during_a_wait_stops_and_the_rest_of_its_line_is_dropped);
  RUN_TEST(input_after_the_100ms_window_is_a_new_command);
  RUN_TEST(monitor_prints_traffic_until_a_byte_arrives);
  RUN_TEST(wrong_bitrate_is_can_error_with_nothing_transmitted);
  RUN_TEST(wrong_bitrate_on_a_silent_bus_fails_after_one_attempt);
  RUN_TEST(blocked_request_touches_nothing);
  RUN_TEST(blocked_request_skips_the_search);
  RUN_TEST(search_with_a_header_skips_protocols_the_guard_refuses);
  RUN_TEST(search_with_skipped_protocols_and_no_answer_is_unable_to_connect);
  RUN_TEST(guard_clear_lets_the_clear_request_through);
  RUN_TEST(ten_frame_burst_with_slow_output_loses_nothing);
  RUN_TEST(survey_finds_the_bitrate_and_lists_ids);
  RUN_TEST(survey_on_a_silent_bus_reports_not_connected);
  RUN_TEST(survey_with_an_unconfigurable_controller_reports_unknown);
  RUN_TEST(selftest_round_trips_a_frame_at_every_rate);
  return UNITY_END();
}

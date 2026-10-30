#include <unity.h>

#include <string>

#include "test_hal.h"

void setUp(void) {}
void tearDown(void) {}

static std::string reply(TestHal &hal, Elm &elm, const std::string &line) {
  const std::string out = ask(hal, elm, line);
  const std::string echo = line + "\r";
  std::string body = out.compare(0, echo.size(), echo) == 0 ? out.substr(echo.size()) : out;
  if (body.size() >= 2 && body.compare(body.size() - 2, 2, "\r>") == 0) {
    body.erase(body.size() - 2);
  }
  return body;
}

static void same_name_rows_are_told_apart_by_shape(void) {
  TestHal hal(renaultDaciaCarQuiet());
  Elm elm(hal);
  elm.begin();
  runAll(hal, elm, {"ATE0", "ATSP6", "ATSH7E0", "ATH1", "ATD1"});
  TEST_ASSERT_EQUAL_STRING_MESSAGE("7E8 8 06 41 00 BE 1F A8 12 00\r", reply(hal, elm, "0100").c_str(),
                                   "ATD1 shows the DLC");
  runAll(hal, elm, {"ATD0", "ATCRA 7EX"});
  TEST_ASSERT_EQUAL_STRING("7E8 06 41 00 BE 1F A8 12 00\r", reply(hal, elm, "0100").c_str());
  run(hal, elm, "ATCRA 7FX");
  TEST_ASSERT_EQUAL_STRING("NO DATA\r", reply(hal, elm, "0100").c_str());
  run(hal, elm, "ATCRA");
  TEST_ASSERT_EQUAL_STRING("7E8 06 41 00 BE 1F A8 12 00\r", reply(hal, elm, "0100").c_str());
}

static void malformed_arguments_answer_question_mark(void) {
  TestHal hal(renaultDaciaCarQuiet());
  Elm elm(hal);
  elm.begin();
  for (const char *cmd : {"ATE2", "ATAT3", "ATST1", "ATSH7E", "ATSPA", "ATSPB", "ATFCSD",
                          "ATFCSD301", "ATBRD23", "ATPPFFOFF"}) {
    TEST_ASSERT_EQUAL_STRING_MESSAGE("?\r", reply(hal, elm, cmd).c_str(), cmd);
  }
}

static void protocol_selection_forms(void) {
  TestHal hal(renaultDaciaCarQuiet());
  Elm elm(hal);
  elm.begin();
  const struct {
    const char *select;
    const char *dpn;
  } forms[] = {{"ATSP6", "6\r"}, {"ATSP00", "A0\r"}, {"ATSPA6", "A6\r"}, {"ATTP8", "8\r"}};
  for (const auto &f : forms) {
    run(hal, elm, f.select);
    TEST_ASSERT_EQUAL_STRING_MESSAGE(f.dpn, reply(hal, elm, "ATDPN").c_str(), f.select);
  }
}

// Every command ddt4all's and pyren's adapter checkers flag 'P' (and their
// CAN setup) must not answer "?", or the user is told the adapter is fake.
static void client_setup_commands_are_accepted(void) {
  TestHal hal(renaultDaciaCarQuiet());
  Elm elm(hal);
  elm.begin();
  const char *commands[] = {
      "ATZ", "ATE1", "ATL0", "ATAL", "ATCAF0", "ATH0", "ATIB 10", "ATSH 012",
      "ATST FF", "ATSW 96", "ATSW 34", "ATWM 817AF13E", "ATWS",
      "ATFC SD 300000", "ATFC SH 012", "ATFC SH 00112233", "ATFC SM 1",
      "ATCRA 012", "AT WS", "AT E1", "AT S0", "AT H0", "AT L0", "AT AL",
      "AT CAF0", "AT CFC1", "AT CFC0", "AT SH 7E0", "AT CP 18", "AT SH DA5DF2",
      "AT FC SH 7E0", "AT FC SD 30 00 00", "AT FC SM 1", "AT ST FF", "AT AT 0",
      "AT SP 6", "AT SP 8", "AT SP 7", "AT SP 9", "AT AT 1", "AT CRA 7E8",
      "AT CRA 18DAF110", "AT ST 4B", "ATSTFF", "AT R0", "AT R1", "AT", "ATD0",
      "ATD1", "ATCSM1", "ATCEA", "ATCV 1250", "ATKW0", "ATKW",
      "ATBI", "ATSS", "ATIIA 01", "ATM0", "ATNL", "ATV0", "ATV1", "ATAR",
      "ATCM 123", "ATCM 12345678", "ATCF 123", "ATCF 12345678", "ATSP A6",
      "ATTP A6", "ATTP 6", "ATSP 00", "ATS1", "ATAT1", "ATPC", "ATD"};
  for (const char *cmd : commands) {
    const std::string r = reply(hal, elm, cmd);
    TEST_ASSERT_TRUE_MESSAGE(r.find('?') == std::string::npos, cmd);
  }
}

static void identity_and_fixed_answers(void) {
  TestHal hal(renaultDaciaCarQuiet());
  Elm elm(hal);
  elm.begin();
  TEST_ASSERT_EQUAL_STRING("ELM327 v1.5\r\r>", hal.output().c_str());
  TEST_ASSERT_EQUAL_STRING("\rELM327 v1.5\r", reply(hal, elm, "ATZ").c_str());
  TEST_ASSERT_EQUAL_STRING("ELM327 v1.5\r", reply(hal, elm, "ATI").c_str());
  TEST_ASSERT_EQUAL_STRING("OBDII to RS232 Interpreter\r", reply(hal, elm, "AT@1").c_str());
  TEST_ASSERT_EQUAL_STRING("12.0V\r", reply(hal, elm, "AT RV").c_str());
  TEST_ASSERT_EQUAL_STRING("BUS INIT: ...ERROR\r", reply(hal, elm, "ATSI").c_str());
  TEST_ASSERT_EQUAL_STRING("BUS INIT: ...ERROR\r", reply(hal, elm, "ATFI").c_str());
  TEST_ASSERT_EQUAL_STRING("?\r", reply(hal, elm, "STI").c_str());
  TEST_ASSERT_EQUAL_STRING("?\r", reply(hal, elm, "ATBRD 23").c_str());
  TEST_ASSERT_EQUAL_STRING("?\r", reply(hal, elm, "ATXYZ").c_str());
  TEST_ASSERT_EQUAL_STRING("T:00 R:00\r", reply(hal, elm, "ATCS").c_str());
}

static void extended_addressing_can_only_be_turned_off(void) {
  TestHal hal(renaultDaciaCarQuiet());
  Elm elm(hal);
  elm.begin();
  TEST_ASSERT_EQUAL_STRING("OK\r", reply(hal, elm, "ATCEA").c_str());
  TEST_ASSERT_EQUAL_STRING("?\r", reply(hal, elm, "ATCEA 12").c_str());
}

static void flow_control_modes_need_their_inputs(void) {
  TestHal hal(renaultDaciaCarQuiet());
  Elm elm(hal);
  elm.begin();
  TEST_ASSERT_EQUAL_STRING("?\r", reply(hal, elm, "AT FC SM 1").c_str());
  TEST_ASSERT_EQUAL_STRING("?\r", reply(hal, elm, "AT FC SM 2").c_str());
  TEST_ASSERT_EQUAL_STRING("OK\r", reply(hal, elm, "AT FC SD 30 00 00").c_str());
  TEST_ASSERT_EQUAL_STRING("OK\r", reply(hal, elm, "AT FC SM 2").c_str());
  TEST_ASSERT_EQUAL_STRING("?\r", reply(hal, elm, "AT FC SM 1").c_str());
  TEST_ASSERT_EQUAL_STRING("OK\r", reply(hal, elm, "AT FC SH 7E0").c_str());
  TEST_ASSERT_EQUAL_STRING("OK\r", reply(hal, elm, "AT FC SM 1").c_str());
  TEST_ASSERT_EQUAL_STRING("OK\r", reply(hal, elm, "AT FC SM 0").c_str());
}

static void protocol_display(void) {
  TestHal hal(renaultDaciaCarQuiet());
  Elm elm(hal);
  elm.begin();
  TEST_ASSERT_EQUAL_STRING("A0\r", reply(hal, elm, "ATDPN").c_str());
  TEST_ASSERT_EQUAL_STRING("AUTO\r", reply(hal, elm, "ATDP").c_str());
  run(hal, elm, "ATSP6");
  TEST_ASSERT_EQUAL_STRING("6\r", reply(hal, elm, "ATDPN").c_str());
  TEST_ASSERT_EQUAL_STRING("ISO 15765-4 (CAN 11/500)\r", reply(hal, elm, "ATDP").c_str());
  run(hal, elm, "ATSPA7");
  TEST_ASSERT_EQUAL_STRING("A7\r", reply(hal, elm, "ATDPN").c_str());
  TEST_ASSERT_EQUAL_STRING("AUTO, ISO 15765-4 (CAN 29/500)\r", reply(hal, elm, "ATDP").c_str());
}

static void reset_restores_every_setting_but_keeps_the_stored_protocol(void) {
  TestHal hal(renaultDaciaCarQuiet());
  Elm elm(hal);
  elm.begin();
  for (const char *cmd : {"ATE0", "ATL1", "ATS0", "ATH1", "ATCAF0", "ATSP6", "ATTP9",
                          "ATST10"}) {
    run(hal, elm, cmd);
  }
  hal.clearOutput();
  run(hal, elm, "ATZ");
  TEST_ASSERT_EQUAL_STRING("\rELM327 v1.5\r\r>", hal.output().c_str());
  TEST_ASSERT_EQUAL_STRING("0100\r41 00 BE 1F A8 12\r\r>", ask(hal, elm, "0100").c_str());
  TEST_ASSERT_EQUAL_STRING("ATDPN\r6\r\r>", ask(hal, elm, "ATDPN").c_str());
}

static void at_d_resets_without_banner(void) {
  TestHal hal(renaultDaciaCarQuiet());
  Elm elm(hal);
  elm.begin();
  run(hal, elm, "ATE0");
  TEST_ASSERT_EQUAL_STRING("OK\r\r>", ask(hal, elm, "ATD").c_str());
  TEST_ASSERT_EQUAL_STRING("ATI\rELM327 v1.5\r\r>", ask(hal, elm, "ATI").c_str());
}

static void at_sp_keeps_headers_and_flow_control(void) {
  TestHal hal(renaultDaciaCarQuiet());
  Elm elm(hal);
  elm.begin();
  // ddt4all sends SH / FC before AT SP; AT SP must not undo them.
  for (const char *cmd : {"ATE0", "ATCAF0", "ATS0", "AT SH 745", "AT FC SH 745",
                          "AT FC SD 30 08 00", "AT FC SM 1", "AT SP 6", "AT CRA 765"}) {
    run(hal, elm, cmd);
  }
  TEST_ASSERT_EQUAL_STRING("101A618082004466\r"
                           "2127443231338200\r"
                           "2238713800A77500\r"
                           "2300000000000000\r\r>",
                           ask(hal, elm, "022180").c_str());
  const CanFrame &fc = hal.wire().back().second;
  TEST_ASSERT_TRUE(fc.id == CanId::std11(0x745));
  TEST_ASSERT_TRUE(sameBytes(fc, {0x30, 0x08, 0x00, 0, 0, 0, 0, 0}));
}

static void linefeeds_and_echo(void) {
  TestHal hal(renaultDaciaCarQuiet());
  Elm elm(hal);
  elm.begin();
  TEST_ASSERT_EQUAL_STRING("ATL1\rOK\r\n\r\n>", ask(hal, elm, "ATL1").c_str());
  TEST_ASSERT_EQUAL_STRING("ati\r\nELM327 v1.5\r\n\r\n>", ask(hal, elm, "ati").c_str());
  TEST_ASSERT_EQUAL_STRING("AT E0\r\nOK\r\n\r\n>", ask(hal, elm, "AT E0").c_str());
  TEST_ASSERT_EQUAL_STRING("OK\r\r>", ask(hal, elm, "ATL0").c_str());
}

static void python_obd_baud_probe_gets_a_question_mark(void) {
  TestHal hal(renaultDaciaCarQuiet());
  Elm elm(hal);
  elm.begin();
  runAll(hal, elm, {"ATE0", "ATSP6", "0100"});
  const size_t sent = hal.wire().size();
  for (const std::string probe : {"\x7F\x7F\r", "\xC3\r"}) {
    hal.clearOutput();
    hal.type(probe);
    elm.service();
    TEST_ASSERT_EQUAL_STRING("?\r\r>", hal.output().c_str());
  }
  TEST_ASSERT_EQUAL_size_t_MESSAGE(sent, hal.wire().size(), "a probe never repeats 0100");
}

static void bare_cr_without_a_previous_request_answers_question_mark(void) {
  TestHal hal(renaultDaciaCarQuiet());
  Elm elm(hal);
  elm.begin();
  run(hal, elm, "ATI");
  TEST_ASSERT_EQUAL_STRING("\r?\r\r>", ask(hal, elm, "").c_str());
}

static void line_feeds_from_the_host_are_ignored(void) {
  TestHal hal(renaultDaciaCarQuiet());
  Elm elm(hal);
  elm.begin();
  hal.clearOutput();
  hal.type("ATI\r\n");
  elm.service();
  TEST_ASSERT_EQUAL_STRING("ATI\rELM327 v1.5\r\r>", hal.output().c_str());
}

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(same_name_rows_are_told_apart_by_shape);
  RUN_TEST(malformed_arguments_answer_question_mark);
  RUN_TEST(protocol_selection_forms);
  RUN_TEST(client_setup_commands_are_accepted);
  RUN_TEST(identity_and_fixed_answers);
  RUN_TEST(extended_addressing_can_only_be_turned_off);
  RUN_TEST(flow_control_modes_need_their_inputs);
  RUN_TEST(protocol_display);
  RUN_TEST(reset_restores_every_setting_but_keeps_the_stored_protocol);
  RUN_TEST(at_d_resets_without_banner);
  RUN_TEST(at_sp_keeps_headers_and_flow_control);
  RUN_TEST(linefeeds_and_echo);
  RUN_TEST(python_obd_baud_probe_gets_a_question_mark);
  RUN_TEST(bare_cr_without_a_previous_request_answers_question_mark);
  RUN_TEST(line_feeds_from_the_host_are_ignored);
  return UNITY_END();
}

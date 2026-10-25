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

static void longest_matching_name_wins(void) {
  AtArgs a{};
  const AtCommand *cmd = findAt("CFC1", &a);
  TEST_ASSERT_NOT_NULL(cmd);
  TEST_ASSERT_EQUAL_STRING("CFC", cmd->name);
  TEST_ASSERT_EQUAL_UINT32(1, a.value);
  TEST_ASSERT_EQUAL_STRING("CSM", findAt("CSM1", &a)->name);
  TEST_ASSERT_EQUAL_STRING("DPN", findAt("DPN", &a)->name);
  TEST_ASSERT_EQUAL_STRING("SH", findAt("SH7E0", &a)->name);
  TEST_ASSERT_EQUAL_UINT32(0x7E0, a.value);
  TEST_ASSERT_EQUAL_UINT8(3, a.count);
  TEST_ASSERT_EQUAL_STRING("", findAt("", &a)->name);
  TEST_ASSERT_EQUAL_STRING("AT", findAt("AT1", &a)->name);
}

static void same_name_rows_are_told_apart_by_shape(void) {
  AtArgs a{};
  const AtCommand *reset = findAt("D", &a);
  const AtCommand *dlc = findAt("D1", &a);
  TEST_ASSERT_NOT_NULL(reset);
  TEST_ASSERT_NOT_NULL(dlc);
  TEST_ASSERT_TRUE(reset->shape == AtShape::None);
  TEST_ASSERT_TRUE(dlc->shape == AtShape::Bit);
  TEST_ASSERT_TRUE(findAt("CRA", &a)->shape == AtShape::None);
  TEST_ASSERT_TRUE(findAt("CRA7EX", &a)->shape == AtShape::Pattern);
  TEST_ASSERT_EQUAL_HEX32(0x7E0, a.value);
  TEST_ASSERT_EQUAL_HEX32(0xFF0, a.mask);
}

static void arguments_are_validated(void) {
  AtArgs a{};
  TEST_ASSERT_NULL(findAt("E2", &a));
  TEST_ASSERT_NULL(findAt("AT3", &a));
  TEST_ASSERT_NULL(findAt("ST1", &a));
  TEST_ASSERT_NULL(findAt("SH7E", &a));
  TEST_ASSERT_NULL(findAt("SPA", &a));
  TEST_ASSERT_NULL(findAt("SPB", &a));
  TEST_ASSERT_NULL(findAt("FCSD", &a));
  TEST_ASSERT_NULL(findAt("FCSD301", &a));
  TEST_ASSERT_NULL(findAt("BRD23", &a));
  TEST_ASSERT_NULL(findAt("PPFFOFF", &a));
}

static void protocol_selection_forms(void) {
  AtArgs a{};
  TEST_ASSERT_NOT_NULL(findAt("SP6", &a));
  TEST_ASSERT_EQUAL_UINT32(6, a.value);
  TEST_ASSERT_FALSE(a.automatic);
  TEST_ASSERT_NOT_NULL(findAt("SPA6", &a));
  TEST_ASSERT_EQUAL_UINT32(6, a.value);
  TEST_ASSERT_TRUE(a.automatic);
  TEST_ASSERT_NOT_NULL(findAt("SP00", &a));
  TEST_ASSERT_EQUAL_UINT32(0, a.value);
  TEST_ASSERT_TRUE(a.automatic);
  TEST_ASSERT_NOT_NULL(findAt("TP8", &a));
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
      "ATD1", "ATCSM1", "ATCEA", "ATCEA 01", "ATCV 1250", "ATKW0", "ATKW",
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
  for (const char *cmd : {"ATE0", "ATL1", "ATS0", "ATH1", "ATCAF0", "ATSP8", "ATTP9",
                          "ATST10"}) {
    run(hal, elm, cmd);
  }
  hal.clearOutput();
  run(hal, elm, "ATZ");
  TEST_ASSERT_EQUAL_STRING("\rELM327 v1.5\r\r>", hal.output().c_str());
  TEST_ASSERT_EQUAL_STRING("ATDPN\r8\r\r>", ask(hal, elm, "ATDPN").c_str());
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
                          "AT FC SD 30 00 00", "AT FC SM 1", "AT SP 6", "AT CRA 765"}) {
    run(hal, elm, cmd);
  }
  TEST_ASSERT_EQUAL_STRING("0250C00000000000\r\r>", ask(hal, elm, "0210C0").c_str());
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
  hal.clearOutput();
  hal.type(std::string("\x7F\x7F\r"));
  elm.service();
  TEST_ASSERT_EQUAL_STRING("\x7F\x7F\r?\r\r>", hal.output().c_str());
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
  RUN_TEST(longest_matching_name_wins);
  RUN_TEST(same_name_rows_are_told_apart_by_shape);
  RUN_TEST(arguments_are_validated);
  RUN_TEST(protocol_selection_forms);
  RUN_TEST(client_setup_commands_are_accepted);
  RUN_TEST(identity_and_fixed_answers);
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

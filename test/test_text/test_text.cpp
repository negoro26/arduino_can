// ISO-TP PCI rules, the input grammar and the response formatter.

#include <unity.h>

#include <string>

#include "settings.h"
#include "text.h"

void setUp(void) {}
void tearDown(void) {}

static CanFrame frame(CanId id, std::initializer_list<uint8_t> bytes) {
  CanFrame f{};
  f.id = id;
  for (uint8_t b : bytes) f.data[f.dlc++] = b;
  return f;
}

static std::string formatted(FrameFormatter &fmt, const CanFrame &f) {
  TextLine lines[2];
  const uint8_t n = fmt.format(f, lines);
  std::string s;
  for (uint8_t i = 0; i < n; ++i) s += std::string(lines[i].text) + "|";
  return s;
}

static const CanId k7E8 = CanId::std11(0x7E8);

// ----------------------------------------------------------------- PCI

static void pci_classes_come_from_the_high_nibble(void) {
  TEST_ASSERT_TRUE(pciOf(frame(k7E8, {0x02, 0x50, 0xC0})) == Pci::Single);
  TEST_ASSERT_TRUE(pciOf(frame(k7E8, {0x10, 0x14})) == Pci::First);
  TEST_ASSERT_TRUE(pciOf(frame(k7E8, {0x21})) == Pci::Consecutive);
  TEST_ASSERT_TRUE(pciOf(frame(k7E8, {0x30, 0, 0})) == Pci::FlowControl);
  TEST_ASSERT_TRUE(pciOf(frame(k7E8, {0x45})) == Pci::Other);
  TEST_ASSERT_TRUE(pciOf(frame(k7E8, {})) == Pci::Other);
}

static void single_frame_length_must_fit_the_frame(void) {
  TEST_ASSERT_EQUAL_UINT8(2, singleFrameLength(frame(k7E8, {0x02, 0x50, 0xC0})));
  TEST_ASSERT_EQUAL_UINT8(0, singleFrameLength(frame(k7E8, {0x00, 0x50})));
  TEST_ASSERT_EQUAL_UINT8(0, singleFrameLength(frame(k7E8, {0x03, 0x50, 0xC0})));
}

static void first_frame_length_is_twelve_bits(void) {
  TEST_ASSERT_EQUAL_UINT16(0x014, firstFrameLength(frame(k7E8, {0x10, 0x14, 0x62})));
  TEST_ASSERT_EQUAL_UINT16(0xFFF, firstFrameLength(frame(k7E8, {0x1F, 0xFF, 0x62})));
}

static void response_pending_is_exactly_7F_xx_78(void) {
  TEST_ASSERT_TRUE(isResponsePending(frame(k7E8, {0x03, 0x7F, 0x22, 0x78, 0, 0, 0, 0})));
  TEST_ASSERT_FALSE(isResponsePending(frame(k7E8, {0x03, 0x7F, 0x22, 0x31, 0, 0, 0, 0})));
  TEST_ASSERT_FALSE(isResponsePending(frame(k7E8, {0x04, 0x7F, 0x22, 0x78, 0x00})));
}

// --------------------------------------------------------------- input

static void line_buffer_uppercases_and_drops_spaces(void) {
  LineBuffer line;
  for (char c : std::string("at fc sd 30 00 00\t")) line.push(static_cast<uint8_t>(c));
  TEST_ASSERT_TRUE(line.valid());
  TEST_ASSERT_EQUAL_STRING("ATFCSD300000", line.text());
}

static void line_buffer_rejects_del_and_high_bytes(void) {
  LineBuffer line;
  line.push(0x7F);
  line.push(0x7F);
  TEST_ASSERT_FALSE(line.valid());
  TEST_ASSERT_FALSE(line.blank());
  line.clear();
  TEST_ASSERT_TRUE(line.blank());
  line.push(0xC3);
  TEST_ASSERT_FALSE(line.valid());
}

static void line_buffer_rejects_overlong_lines(void) {
  LineBuffer line;
  for (int i = 0; i < kLineMax; ++i) line.push('0');
  TEST_ASSERT_TRUE(line.valid());
  line.push('0');
  TEST_ASSERT_FALSE(line.valid());
}

static void hex_request_with_and_without_count_digit(void) {
  HexRequest r;
  TEST_ASSERT_TRUE(parseHexRequest("0100", true, &r));
  TEST_ASSERT_EQUAL_UINT8(2, r.len);
  TEST_ASSERT_EQUAL_INT8(-1, r.maxResponses);
  TEST_ASSERT_TRUE(parseHexRequest("0322F1903", false, &r));
  TEST_ASSERT_EQUAL_UINT8(4, r.len);
  TEST_ASSERT_EQUAL_INT8(3, r.maxResponses);
  TEST_ASSERT_EQUAL_HEX8(0x90, r.bytes[3]);
}

static void hex_request_limits_depend_on_caf(void) {
  HexRequest r;
  TEST_ASSERT_FALSE(parseHexRequest("1122334455667788", true, &r));
  TEST_ASSERT_TRUE(parseHexRequest("1122334455667788", false, &r));
  TEST_ASSERT_FALSE(parseHexRequest("112233445566778899", false, &r));
  TEST_ASSERT_FALSE(parseHexRequest("0", true, &r));
  TEST_ASSERT_FALSE(parseHexRequest("", true, &r));
  TEST_ASSERT_FALSE(parseHexRequest("STI", true, &r));
}

// ------------------------------------------------------------- framing

static void caf1_request_gets_pci_and_padding(void) {
  Settings s;
  const uint8_t req[] = {0x01, 0x00};
  const CanFrame f = requestFrame(s, Protocol::Can11_500, req, 2);
  TEST_ASSERT_TRUE(f.id == CanId::std11(0x7DF));
  TEST_ASSERT_EQUAL_UINT8(8, f.dlc);
  const uint8_t want[] = {0x02, 0x01, 0x00, 0, 0, 0, 0, 0};
  TEST_ASSERT_EQUAL_HEX8_ARRAY(want, f.data, 8);
}

static void caf0_request_is_sent_as_given_and_v1_skips_padding(void) {
  Settings s;
  s.caf = false;
  s.varDlc = true;
  s.header = {0x745, 3};
  const uint8_t req[] = {0x02, 0x10, 0xC0};
  const CanFrame f = requestFrame(s, Protocol::Can11_500, req, 3);
  TEST_ASSERT_TRUE(f.id == CanId::std11(0x745));
  TEST_ASSERT_EQUAL_UINT8(3, f.dlc);
  TEST_ASSERT_EQUAL_HEX8(0x02, f.data[0]);
}

static void headers_follow_the_protocol_width(void) {
  Settings s;
  TEST_ASSERT_TRUE(requestId(s, Protocol::Can29_500) == CanId::ext29(0x18DB33F1));
  s.header = {0xDA5DF2, 6};
  TEST_ASSERT_TRUE(requestId(s, Protocol::Can29_500) == CanId::ext29(0x18DA5DF2));
  s.header = {0x18DA10F1, 8};
  TEST_ASSERT_TRUE(requestId(s, Protocol::Can29_250) == CanId::ext29(0x18DA10F1));
  s.header = {0x7E0, 3};
  TEST_ASSERT_TRUE(requestId(s, Protocol::Can11_250) == CanId::std11(0x7E0));
}

static void default_receive_filter_is_the_obd_responder_range(void) {
  Settings s;
  const Acceptance a11 = receiveFilter(s, Protocol::Can11_500);
  TEST_ASSERT_TRUE(a11.accepts(CanId::std11(0x7E8)));
  TEST_ASSERT_TRUE(a11.accepts(CanId::std11(0x7EF)));
  TEST_ASSERT_FALSE(a11.accepts(CanId::std11(0x7E0)));
  TEST_ASSERT_FALSE(a11.accepts(CanId::ext29(0x7E8)));
  const Acceptance a29 = receiveFilter(s, Protocol::Can29_500);
  TEST_ASSERT_TRUE(a29.accepts(CanId::ext29(0x18DAF110)));
  TEST_ASSERT_FALSE(a29.accepts(CanId::ext29(0x18DA10F1)));
}

static void automatic_flow_control_ids(void) {
  Settings s;
  CanFrame ff = frame(k7E8, {0x10, 0x14, 0x62, 0xF1, 0x90, 0x55, 0x55, 0x31});
  CanFrame fc = flowControlFrame(s, Protocol::Can11_500, ff);
  TEST_ASSERT_TRUE(fc.id == CanId::std11(0x7E0));
  const uint8_t want[] = {0x30, 0x00, 0x00, 0, 0, 0, 0, 0};
  TEST_ASSERT_EQUAL_UINT8(8, fc.dlc);
  TEST_ASSERT_EQUAL_HEX8_ARRAY(want, fc.data, 8);

  ff.id = CanId::ext29(0x18DAF110);
  fc = flowControlFrame(s, Protocol::Can29_500, ff);
  TEST_ASSERT_TRUE(fc.id == CanId::ext29(0x18DA10F1));

  ff.id = CanId::std11(0x765);
  s.header = {0x745, 3};
  fc = flowControlFrame(s, Protocol::Can11_500, ff);
  TEST_ASSERT_TRUE(fc.id == CanId::std11(0x745));
}

static void user_flow_control_modes(void) {
  Settings s;
  s.fc.header = {0x745, 3};
  s.fc.data[0] = 0x30;
  s.fc.data[1] = 0x08;
  s.fc.data[2] = 0x0A;
  s.fc.dataLen = 3;
  const CanFrame ff = frame(k7E8, {0x10, 0x14, 0x62, 0xF1, 0x90, 0x55, 0x55, 0x31});
  s.fc.mode = 1;
  CanFrame fc = flowControlFrame(s, Protocol::Can11_500, ff);
  TEST_ASSERT_TRUE(fc.id == CanId::std11(0x745));
  TEST_ASSERT_EQUAL_HEX8(0x08, fc.data[1]);
  s.fc.mode = 2;
  fc = flowControlFrame(s, Protocol::Can11_500, ff);
  TEST_ASSERT_TRUE(fc.id == CanId::std11(0x7E0));
  TEST_ASSERT_EQUAL_HEX8(0x0A, fc.data[2]);
}

static void wait_windows(void) {
  Settings s;
  TEST_ASSERT_EQUAL_UINT16(200, firstWaitMs(s));
  TEST_ASSERT_EQUAL_UINT16(50, nextWaitMs(s));
  s.adaptive = 2;
  TEST_ASSERT_EQUAL_UINT16(25, nextWaitMs(s));
  s.adaptive = 0;
  TEST_ASSERT_EQUAL_UINT16(200, nextWaitMs(s));
  s.adaptive = 1;
  s.stTicks = 0x05;
  TEST_ASSERT_EQUAL_UINT16(20, nextWaitMs(s));
  s.stTicks = 0xFF;
  TEST_ASSERT_EQUAL_UINT16(1020, firstWaitMs(s));
}

// -------------------------------------------------------------- output

static void caf0_h0_s0_prints_every_byte_raw(void) {
  Settings s;
  s.caf = false;
  s.spaces = false;
  FrameFormatter fmt(s, false);
  TEST_ASSERT_EQUAL_STRING("037F227800000000|",
                           formatted(fmt, frame(k7E8, {0x03, 0x7F, 0x22, 0x78, 0, 0, 0, 0})).c_str());
}

static void h1_prints_id_and_all_bytes(void) {
  Settings s;
  s.headers = true;
  FrameFormatter fmt(s, false);
  const CanFrame f = frame(k7E8, {0x06, 0x41, 0x00, 0xBE, 0x1F, 0xA8, 0x12, 0x00});
  TEST_ASSERT_EQUAL_STRING("7E8 06 41 00 BE 1F A8 12 00|", formatted(fmt, f).c_str());
  s.dlc = true;
  TEST_ASSERT_EQUAL_STRING("7E8 8 06 41 00 BE 1F A8 12 00|", formatted(fmt, f).c_str());
  s.dlc = false;
  s.spaces = false;
  TEST_ASSERT_EQUAL_STRING("7E8064100BE1FA81200|", formatted(fmt, f).c_str());
}

static void h1_prints_29_bit_ids_as_four_bytes(void) {
  Settings s;
  s.headers = true;
  FrameFormatter fmt(s, false);
  const CanFrame f = frame(CanId::ext29(0x18DAF110), {0x02, 0x50, 0x03});
  TEST_ASSERT_EQUAL_STRING("18 DA F1 10 02 50 03|", formatted(fmt, f).c_str());
}

static void caf1_h0_single_frame_shows_payload_only(void) {
  Settings s;
  FrameFormatter fmt(s, false);
  TEST_ASSERT_EQUAL_STRING("41 0C 0B B8|",
                           formatted(fmt, frame(k7E8, {0x04, 0x41, 0x0C, 0x0B, 0xB8, 0, 0, 0})).c_str());
}

static void caf1_h0_multi_frame_trims_the_last_padding(void) {
  Settings s;
  FrameFormatter fmt(s, false);
  TEST_ASSERT_EQUAL_STRING(
      "014|0: 49 02 01 55 55 31|",
      formatted(fmt, frame(k7E8, {0x10, 0x14, 0x49, 0x02, 0x01, 0x55, 0x55, 0x31})).c_str());
  TEST_ASSERT_EQUAL_STRING(
      "1: 35 53 44 41 47 35 35|",
      formatted(fmt, frame(k7E8, {0x21, 0x35, 0x53, 0x44, 0x41, 0x47, 0x35, 0x35})).c_str());
  TEST_ASSERT_EQUAL_STRING(
      "2: 31 32 33 34 35 36 37|",
      formatted(fmt, frame(k7E8, {0x22, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37})).c_str());

  FrameFormatter short_(s, false);
  formatted(short_, frame(k7E8, {0x10, 0x09, 0x62, 0xF1, 0x94, 0x53, 0x31, 0x38}));
  TEST_ASSERT_EQUAL_STRING(
      "1: 30 30 36|",
      formatted(short_, frame(k7E8, {0x21, 0x30, 0x30, 0x36, 0x00, 0x00, 0x00, 0x00})).c_str());
}

static void caf1_h0_s0_keeps_the_sequence_prefix(void) {
  Settings s;
  s.spaces = false;
  FrameFormatter fmt(s, false);
  TEST_ASSERT_EQUAL_STRING(
      "014|0:490201555531|",
      formatted(fmt, frame(k7E8, {0x10, 0x14, 0x49, 0x02, 0x01, 0x55, 0x55, 0x31})).c_str());
}

static void caf1_h0_hides_flow_control_except_when_monitoring(void) {
  Settings s;
  const CanFrame fc = frame(CanId::std11(0x765), {0x30, 0x00, 0x00, 0, 0, 0, 0, 0});
  FrameFormatter response(s, false);
  TEST_ASSERT_EQUAL_STRING("", formatted(response, fc).c_str());
  TEST_ASSERT_FALSE(shownAsResponse(s, fc));
  FrameFormatter monitor(s, true);
  TEST_ASSERT_EQUAL_STRING("FC: 30 00 00 00 00 00 00 00|", formatted(monitor, fc).c_str());
  const CanFrame junk = frame(CanId::std11(0x181), {0xAB, 0x01});
  TEST_ASSERT_EQUAL_STRING("AB 01 <DATA ERROR|", formatted(monitor, junk).c_str());
  s.headers = true;
  TEST_ASSERT_TRUE(shownAsResponse(s, fc));
}

static void protocol_numbers_and_names(void) {
  char dpn[3];
  formatDpn({Protocol::Can11_500, true}, dpn);
  TEST_ASSERT_EQUAL_STRING("A6", dpn);
  formatDpn({Protocol::Can29_250, false}, dpn);
  TEST_ASSERT_EQUAL_STRING("9", dpn);
  formatDpn({Protocol::None, true}, dpn);
  TEST_ASSERT_EQUAL_STRING("A0", dpn);
  TEST_ASSERT_EQUAL_STRING("ISO 15765-4 (CAN 11/500)", protocolName(Protocol::Can11_500));
  TEST_ASSERT_EQUAL_STRING("ISO 15765-4 (CAN 29/250)", protocolName(Protocol::Can29_250));
}

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(pci_classes_come_from_the_high_nibble);
  RUN_TEST(single_frame_length_must_fit_the_frame);
  RUN_TEST(first_frame_length_is_twelve_bits);
  RUN_TEST(response_pending_is_exactly_7F_xx_78);
  RUN_TEST(line_buffer_uppercases_and_drops_spaces);
  RUN_TEST(line_buffer_rejects_del_and_high_bytes);
  RUN_TEST(line_buffer_rejects_overlong_lines);
  RUN_TEST(hex_request_with_and_without_count_digit);
  RUN_TEST(hex_request_limits_depend_on_caf);
  RUN_TEST(caf1_request_gets_pci_and_padding);
  RUN_TEST(caf0_request_is_sent_as_given_and_v1_skips_padding);
  RUN_TEST(headers_follow_the_protocol_width);
  RUN_TEST(default_receive_filter_is_the_obd_responder_range);
  RUN_TEST(automatic_flow_control_ids);
  RUN_TEST(user_flow_control_modes);
  RUN_TEST(wait_windows);
  RUN_TEST(caf0_h0_s0_prints_every_byte_raw);
  RUN_TEST(h1_prints_id_and_all_bytes);
  RUN_TEST(h1_prints_29_bit_ids_as_four_bytes);
  RUN_TEST(caf1_h0_single_frame_shows_payload_only);
  RUN_TEST(caf1_h0_multi_frame_trims_the_last_padding);
  RUN_TEST(caf1_h0_s0_keeps_the_sequence_prefix);
  RUN_TEST(caf1_h0_hides_flow_control_except_when_monitoring);
  RUN_TEST(protocol_numbers_and_names);
  return UNITY_END();
}

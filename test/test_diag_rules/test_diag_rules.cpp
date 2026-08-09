// Host-side tests for the probe's decision logic. These run on the laptop with
// `pio test -e native` -- no board, no car.
//
// They defend two contracts that have physical consequences: the stage 4 safety
// interlock must never forget an in-use CAN ID, and responder detection must
// not confuse ordinary vehicle traffic for an ECU answering.

#include <unity.h>

#include "diag_rules.h"

void setUp(void) {}
void tearDown(void) {}

// ------------------------------------------------ looksLikeDiagReply

static void positive_response_to_start_session_is_a_reply(void) {
  const uint8_t f[8] = {0x03, 0x50, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00};
  TEST_ASSERT_TRUE(looksLikeDiagReply(8, f));
}

static void byte99_is_not_a_reply(void) {
  const uint8_t f[8] = {0x03, 0x99, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00};
  TEST_ASSERT_FALSE(looksLikeDiagReply(8, f));
}

static void negative_response_still_means_the_ecu_exists(void) {
  // 7F 10 12 = service 0x10 refused, sub-function not supported. The ECU is
  // present and must be reported, otherwise a refusing module looks absent.
  const uint8_t f[8] = {0x03, 0x7F, 0x10, 0x12, 0x00, 0x00, 0x00, 0x00};
  TEST_ASSERT_TRUE(looksLikeDiagReply(8, f));
}

static void positive_response_to_read_data_is_a_reply(void) {
  const uint8_t f[8] = {0x03, 0x61, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00};
  TEST_ASSERT_TRUE(looksLikeDiagReply(8, f));
}

static void ordinary_vehicle_traffic_is_not_a_reply(void) {
  // Engine/wheel-speed style broadcast. If this matched, the active sweep would
  // report the whole car as ECUs answering.
  const uint8_t f[8] = {0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC, 0xDE, 0xF0};
  TEST_ASSERT_FALSE(looksLikeDiagReply(8, f));
}

static void a_request_echo_is_not_a_reply(void) {
  // Our own transmitted 02 10 C0: service byte 0x10, not a response SID.
  const uint8_t f[8] = {0x02, 0x10, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00};
  TEST_ASSERT_FALSE(looksLikeDiagReply(8, f));
}

static void frames_too_short_to_carry_a_service_id_are_rejected(void) {
  const uint8_t f[2] = {0x03, 0x50};
  TEST_ASSERT_FALSE(looksLikeDiagReply(1, f)); // dlc 1: no data[1]
  TEST_ASSERT_FALSE(looksLikeDiagReply(0, f));
  TEST_ASSERT_TRUE(looksLikeDiagReply(2, f)); // dlc 2: boundary, accepted
}

static void null_payload_is_rejected(void) {
  TEST_ASSERT_FALSE(looksLikeDiagReply(8, NULL));
}

// ------------------------------------------------------ isDiagRequestId

static void diag_range_boundaries_are_inclusive(void) {
  TEST_ASSERT_FALSE(isDiagRequestId(0x6FF));
  TEST_ASSERT_TRUE(isDiagRequestId(0x700));
  TEST_ASSERT_TRUE(isDiagRequestId(0x7FF));
  TEST_ASSERT_FALSE(isDiagRequestId(0x800));
}

// ------------------------------------------------------------- SeenIds

static void recorded_ids_are_reported_as_seen(void) {
  SeenIds<8> seen;
  seen.add(0x1F6);
  seen.add(0x354);
  TEST_ASSERT_TRUE(seen.contains(0x1F6));
  TEST_ASSERT_TRUE(seen.contains(0x354));
  TEST_ASSERT_FALSE(seen.contains(0x745));
}

static void duplicates_do_not_consume_capacity(void) {
  SeenIds<8> seen;
  TEST_ASSERT_TRUE(seen.add(0x1F6));
  TEST_ASSERT_FALSE(seen.add(0x1F6));
  TEST_ASSERT_EQUAL_UINT8(1, seen.count());
}

static void overflow_keeps_earlier_ids_rather_than_overwriting(void) {
  // The interlock's failure mode matters: dropping a NEW id means the sweep may
  // transmit on it, but silently replacing an ALREADY-SEEN id would be worse,
  // because that id is known to be in use.
  SeenIds<2> seen;
  seen.add(0x100);
  seen.add(0x200);
  TEST_ASSERT_TRUE(seen.isFull());
  TEST_ASSERT_FALSE(seen.add(0x300));
  TEST_ASSERT_TRUE(seen.contains(0x100));
  TEST_ASSERT_TRUE(seen.contains(0x200));
  TEST_ASSERT_FALSE(seen.contains(0x300));
  TEST_ASSERT_EQUAL_UINT8(2, seen.count());
}

static void clear_resets_the_set(void) {
  SeenIds<4> seen;
  seen.add(0x123);
  seen.clear();
  TEST_ASSERT_EQUAL_UINT8(0, seen.count());
  TEST_ASSERT_FALSE(seen.contains(0x123));
}

static void overlap_counts_only_ids_in_the_diagnostic_range(void) {
  SeenIds<8> seen;
  seen.add(0x1F6); // operational
  seen.add(0x6FF); // just below the range
  seen.add(0x700); // in range
  seen.add(0x765); // in range
  seen.add(0x800); // above the range
  TEST_ASSERT_EQUAL_UINT8(2, seen.diagRangeOverlap());
}

static void an_empty_set_reports_no_overlap(void) {
  SeenIds<8> seen;
  TEST_ASSERT_EQUAL_UINT8(0, seen.diagRangeOverlap());
  TEST_ASSERT_FALSE(seen.isFull());
}

int main(int, char **) {
  UNITY_BEGIN();

  RUN_TEST(positive_response_to_start_session_is_a_reply);
  RUN_TEST(byte99_is_not_a_reply);
  RUN_TEST(negative_response_still_means_the_ecu_exists);
  RUN_TEST(positive_response_to_read_data_is_a_reply);
  RUN_TEST(ordinary_vehicle_traffic_is_not_a_reply);
  RUN_TEST(a_request_echo_is_not_a_reply);
  RUN_TEST(frames_too_short_to_carry_a_service_id_are_rejected);
  RUN_TEST(null_payload_is_rejected);

  RUN_TEST(diag_range_boundaries_are_inclusive);

  RUN_TEST(recorded_ids_are_reported_as_seen);
  RUN_TEST(duplicates_do_not_consume_capacity);
  RUN_TEST(overflow_keeps_earlier_ids_rather_than_overwriting);
  RUN_TEST(clear_resets_the_set);
  RUN_TEST(overlap_counts_only_ids_in_the_diagnostic_range);
  RUN_TEST(an_empty_set_reports_no_overlap);

  return UNITY_END();
}

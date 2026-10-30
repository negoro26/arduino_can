#include <unity.h>

#include "tx_attempt.h"

void setUp(void) {}
void tearDown(void) {}

// Register bytes as the MCP2515 datasheet gives them: TXBnCTRL ABTF 0x40,
// MLOA 0x20, TXERR 0x10; CANINTF MERRF 0x80.

static void clean_completion_is_sent(void) {
  TEST_ASSERT_TRUE(judgeTxAttempt(0x00, 0, 0, 0, 1) == TxVerdict::Sent);
  TEST_ASSERT_TRUE_MESSAGE(judgeTxAttempt(0x00, 0, 9, 0x80, 1) == TxVerdict::Sent,
                           "an error on another frame does not undo our ACKed frame");
}

static void clean_arbitration_loss_is_retried(void) {
  TEST_ASSERT_TRUE(judgeTxAttempt(0x60, 5, 5, 0, 1) == TxVerdict::Retry);
  TEST_ASSERT_TRUE(judgeTxAttempt(0x20, 5, 5, 0, 1) == TxVerdict::Retry);
  TEST_ASSERT_TRUE_MESSAGE(judgeTxAttempt(0x60, 5, 4, 0, 2) == TxVerdict::Retry,
                           "a frame received meanwhile lowers REC");
}

static void third_attempt_is_the_last(void) {
  TEST_ASSERT_TRUE(judgeTxAttempt(0x60, 0, 0, 0, 2) == TxVerdict::Retry);
  TEST_ASSERT_TRUE(judgeTxAttempt(0x60, 0, 0, 0, 3) == TxVerdict::Failed);
}

static void lost_arbitration_with_a_rec_rise_fails(void) {
  TEST_ASSERT_TRUE_MESSAGE(judgeTxAttempt(0x60, 0, 9, 0, 1) == TxVerdict::Failed,
                           "measured on the board with nothing on the CAN pins at 500 kbit");
  TEST_ASSERT_TRUE(judgeTxAttempt(0x60, 9, 10, 0, 1) == TxVerdict::Failed);
}

static void lost_arbitration_with_merrf_fails(void) {
  TEST_ASSERT_TRUE(judgeTxAttempt(0x60, 0, 0, 0x80, 1) == TxVerdict::Failed);
}

static void txerr_fails(void) {
  TEST_ASSERT_TRUE(judgeTxAttempt(0x50, 0, 0, 0, 1) == TxVerdict::Failed);
  TEST_ASSERT_TRUE(judgeTxAttempt(0x70, 0, 0, 0, 1) == TxVerdict::Failed);
  TEST_ASSERT_TRUE(judgeTxAttempt(0x10, 0, 0, 0, 1) == TxVerdict::Failed);
}

static void abort_without_lost_arbitration_fails(void) {
  TEST_ASSERT_TRUE(judgeTxAttempt(0x40, 0, 0, 0, 1) == TxVerdict::Failed);
}

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(clean_completion_is_sent);
  RUN_TEST(clean_arbitration_loss_is_retried);
  RUN_TEST(third_attempt_is_the_last);
  RUN_TEST(lost_arbitration_with_a_rec_rise_fails);
  RUN_TEST(lost_arbitration_with_merrf_fails);
  RUN_TEST(txerr_fails);
  RUN_TEST(abort_without_lost_arbitration_fails);
  return UNITY_END();
}

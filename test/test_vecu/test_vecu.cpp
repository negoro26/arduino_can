// The virtual ECU's ISO-TP server, the virtual car and the simulated wire.

#include <unity.h>

#include <vector>

#include "renault_scenario.h"
#include "rx_order.h"
#include "vecu.h"

void setUp(void) {}
void tearDown(void) {}

static CanFrame frame(uint32_t id, std::initializer_list<uint8_t> bytes) {
  CanFrame f{};
  f.id = CanId::std11(id);
  for (uint8_t b : bytes) f.data[f.dlc++] = b;
  while (f.dlc < 8) f.data[f.dlc++] = 0x00;
  return f;
}

struct Timed {
  uint32_t atUs;
  CanFrame frame;
};

// Runs the car until `untilUs`, collecting every frame at its due time.
static std::vector<Timed> runCar(VirtualCar &car, uint32_t fromUs, uint32_t untilUs) {
  std::vector<Timed> out;
  uint32_t now = fromUs;
  for (;;) {
    CanFrame f;
    while (car.poll(now, &f)) out.push_back({now, f});
    uint32_t next;
    if (!car.nextDue(&next) || next > untilUs) break;
    now = next;
  }
  return out;
}

static const uint8_t kReq[] = {0x10, 0x03};
static const uint8_t kRsp[] = {0x50, 0x03};
static const uint8_t kLongReq[] = {0x2E, 0xF1, 0x90};
static const uint8_t kLongRsp[] = {0x62, 0xF1, 0x90, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10,
                                   11, 12, 13, 14, 15, 16, 17, 18, 19, 20};
static const uint8_t kVinReq[] = {0x22, 0xF1, 0x90};
static const Exchange kExchanges[] = {
    {kReq, sizeof kReq, kRsp, sizeof kRsp, 0},
    {kLongReq, sizeof kLongReq, kRsp, sizeof kRsp, 0},
    {kVinReq, sizeof kVinReq, kLongRsp, sizeof kLongRsp, 0},
};
static const EcuSpec kEcu = {"TEST", CanId::std11(0x7E0), CanId::std11(0x7E8), true,
                             kExchanges, 3, 2000};
static const CarSpec kCar = {&kEcu, 1, nullptr, 0, Bitrate::k500};

static void single_frame_request_is_answered_after_the_latency(void) {
  VirtualCar car(kCar);
  car.deliver(frame(0x7E0, {0x02, 0x10, 0x03}), 1000);
  const std::vector<Timed> got = runCar(car, 1000, 100000);
  TEST_ASSERT_EQUAL_size_t(1, got.size());
  TEST_ASSERT_EQUAL_UINT32(3000, got[0].atUs);
  TEST_ASSERT_TRUE(got[0].frame.id == CanId::std11(0x7E8));
  const uint8_t want[] = {0x02, 0x50, 0x03, 0, 0, 0, 0, 0};
  TEST_ASSERT_EQUAL_HEX8_ARRAY(want, got[0].frame.data, 8);
}

static void long_answer_waits_for_flow_control_then_streams(void) {
  VirtualCar car(kCar);
  car.deliver(frame(0x7E0, {0x03, 0x22, 0xF1, 0x90}), 0);
  std::vector<Timed> got = runCar(car, 0, 10000);
  TEST_ASSERT_EQUAL_size_t(1, got.size());
  TEST_ASSERT_EQUAL_HEX8(0x10, got[0].frame.data[0]);
  TEST_ASSERT_EQUAL_HEX8(23, got[0].frame.data[1]);

  car.deliver(frame(0x7E0, {0x30, 0x00, 0x00}), 2000);
  got = runCar(car, 2000, 100000);
  TEST_ASSERT_EQUAL_size_t(3, got.size());
  TEST_ASSERT_EQUAL_UINT32(2230, got[0].atUs);
  TEST_ASSERT_EQUAL_UINT32(2460, got[1].atUs);
  TEST_ASSERT_EQUAL_HEX8(0x21, got[0].frame.data[0]);
  TEST_ASSERT_EQUAL_HEX8(0x23, got[2].frame.data[0]);
  TEST_ASSERT_EQUAL_HEX8(20, got[2].frame.data[3]);
  TEST_ASSERT_EQUAL_HEX8(0x00, got[2].frame.data[4]);
}

static void flow_control_block_size_and_stmin_are_honoured(void) {
  VirtualCar car(kCar);
  car.deliver(frame(0x7E0, {0x03, 0x22, 0xF1, 0x90}), 0);
  runCar(car, 0, 10000);
  car.deliver(frame(0x7E0, {0x30, 0x02, 0x0A}), 5000);
  std::vector<Timed> got = runCar(car, 5000, 100000);
  TEST_ASSERT_EQUAL_size_t(2, got.size());
  TEST_ASSERT_EQUAL_UINT32(5230, got[0].atUs);
  TEST_ASSERT_EQUAL_UINT32(15230, got[1].atUs);
  car.deliver(frame(0x7E0, {0x30, 0x00, 0x00}), 20000);
  got = runCar(car, 20000, 100000);
  TEST_ASSERT_EQUAL_size_t(1, got.size());
  TEST_ASSERT_EQUAL_HEX8(0x23, got[0].frame.data[0]);
}

static void missing_flow_control_abandons_the_answer(void) {
  VirtualCar car(kCar);
  car.deliver(frame(0x7E0, {0x03, 0x22, 0xF1, 0x90}), 0);
  runCar(car, 0, 2000000);
  uint32_t next;
  TEST_ASSERT_FALSE(car.nextDue(&next));
  car.deliver(frame(0x7E0, {0x30, 0x00, 0x00}), 2000000);
  TEST_ASSERT_EQUAL_size_t(0, runCar(car, 2000000, 3000000).size());
}

static void segmented_request_is_reassembled(void) {
  VirtualCar car(kCar);
  car.deliver(frame(0x7E0, {0x10, 0x09, 0x2E, 0xF1, 0x90, 0x41, 0x42, 0x43}), 0);
  std::vector<Timed> got = runCar(car, 0, 10000);
  TEST_ASSERT_EQUAL_size_t(1, got.size());
  const uint8_t fc[] = {0x30, 0x00, 0x00};
  TEST_ASSERT_EQUAL_HEX8_ARRAY(fc, got[0].frame.data, 3);
  car.deliver(frame(0x7E0, {0x21, 0x44, 0x45, 0x46}), 3000);
  got = runCar(car, 3000, 100000);
  TEST_ASSERT_EQUAL_size_t(1, got.size());
  TEST_ASSERT_EQUAL_HEX8(0x50, got[0].frame.data[1]);
}

static void unknown_requests_get_negative_responses(void) {
  VirtualCar car(kCar);
  car.deliver(frame(0x7E0, {0x02, 0x11, 0x01}), 0);
  std::vector<Timed> got = runCar(car, 0, 100000);
  const uint8_t nrc11[] = {0x03, 0x7F, 0x11, 0x11};
  TEST_ASSERT_EQUAL_HEX8_ARRAY(nrc11, got.at(0).frame.data, 4);
  car.deliver(frame(0x7E0, {0x03, 0x22, 0xF1, 0x87}), 100000);
  got = runCar(car, 100000, 200000);
  const uint8_t nrc31[] = {0x03, 0x7F, 0x22, 0x31};
  TEST_ASSERT_EQUAL_HEX8_ARRAY(nrc31, got.at(0).frame.data, 4);
}

static void functional_requests_reach_only_obd_services(void) {
  VirtualCar car(renaultDaciaCarQuiet());
  car.deliver(frame(0x7DF, {0x02, 0x10, 0x03}), 0);
  car.deliver(frame(0x7DF, {0x02, 0x01, 0x42}), 0);
  TEST_ASSERT_EQUAL_size_t(0, runCar(car, 0, 100000).size());
  car.deliver(frame(0x7DF, {0x02, 0x01, 0x0C}), 100000);
  const std::vector<Timed> got = runCar(car, 100000, 200000);
  TEST_ASSERT_EQUAL_size_t(1, got.size());
  TEST_ASSERT_TRUE(got[0].frame.id == CanId::std11(0x7E8));
  const uint8_t rpm[] = {0x04, 0x41, 0x0C, 0x0B, 0xB8};
  TEST_ASSERT_EQUAL_HEX8_ARRAY(rpm, got[0].frame.data, 5);
}

static void airbag_answers_pending_first(void) {
  VirtualCar car(renaultDaciaCarQuiet());
  car.deliver(frame(0x752, {0x03, 0x22, 0xF1, 0x8A}), 0);
  const std::vector<Timed> got = runCar(car, 0, 1000000);
  TEST_ASSERT_EQUAL_size_t(2, got.size());
  const uint8_t pending[] = {0x03, 0x7F, 0x22, 0x78};
  TEST_ASSERT_EQUAL_HEX8_ARRAY(pending, got[0].frame.data, 4);
  TEST_ASSERT_EQUAL_UINT32(303000, got[1].atUs);
  // 8 payload bytes: one more than a single frame holds.
  const uint8_t final[] = {0x10, 0x08, 0x62, 0xF1, 0x8A, 'T', 'E', 'M'};
  TEST_ASSERT_EQUAL_HEX8_ARRAY(final, got[1].frame.data, 8);
  car.deliver(frame(0x752, {0x03, 0x22, 0xF1, 0x90}), 1000000);
  const std::vector<Timed> unknown = runCar(car, 1000000, 2000000);
  TEST_ASSERT_EQUAL_size_t(1, unknown.size());
  TEST_ASSERT_EQUAL_HEX8(0x31, unknown[0].frame.data[3]);
}

static void abs_ignores_extended_session(void) {
  VirtualCar car(renaultDaciaCarQuiet());
  car.deliver(frame(0x740, {0x02, 0x10, 0x03}), 0);
  TEST_ASSERT_EQUAL_size_t(0, runCar(car, 0, 1000000).size());
}

static void periodic_traffic_skips_backlog_after_a_pause(void) {
  VirtualCar car(renaultDaciaCar());
  const std::vector<Timed> got = runCar(car, 1800000000u, 1800000000u);
  // Half an hour late: at most two frames per source, not 180000.
  TEST_ASSERT_TRUE(got.size() <= 8);
  TEST_ASSERT_TRUE(car.transmits(CanId::std11(0x622)));
  TEST_ASSERT_TRUE(car.transmits(CanId::std11(0x765)));
  TEST_ASSERT_FALSE(car.transmits(CanId::std11(0x745)));
}

// ----------------------------------------------------------------- SimBus

struct BusController {
  SimBus &bus;
  uint32_t nowUs;
  uint8_t rxStatus() { return bus.rxStatus(nowUs); }
  bool readRxb(uint8_t n, CanFrame &out) { return bus.readRxb(n, out); }
};

static void two_buffers_overflow_on_the_third_frame(void) {
  VirtualCar car(kCar);
  SimBus bus(car, 2);
  bus.open(Bitrate::k500, CanMode::Normal, Acceptance::all(), 0);
  car.deliver(frame(0x7E0, {0x03, 0x22, 0xF1, 0x90}), 0);
  // The first frame lands at 2000 us; this flow control releases the CFs.
  bus.send(frame(0x7E0, {0x30, 0x00, 0x00}), 2000);
  bus.advance(3000);
  TEST_ASSERT_TRUE(bus.health().rxOverflow);
  TEST_ASSERT_EQUAL_UINT8(kRxb0Full | kRxb1Full, bus.rxStatus(3000));
}

static void rollover_order_returns_frames_oldest_first(void) {
  VirtualCar car(kCar);
  SimBus bus(car, 2);
  bus.open(Bitrate::k500, CanMode::Loopback, Acceptance::all(), 0);
  RolloverOrder order;
  BusController c{bus, 0};
  CanFrame got;
  bus.send(frame(0x100, {1}), 0);  // -> RXB0
  bus.send(frame(0x101, {2}), 0);  // -> RXB1 (rollover)
  TEST_ASSERT_TRUE(order.read(c, got));
  TEST_ASSERT_EQUAL_HEX32(0x100, got.id.value());
  bus.send(frame(0x102, {3}), 0);  // -> RXB0 again, newer than RXB1
  TEST_ASSERT_TRUE(order.read(c, got));
  TEST_ASSERT_EQUAL_HEX32(0x101, got.id.value());
  TEST_ASSERT_TRUE(order.read(c, got));
  TEST_ASSERT_EQUAL_HEX32(0x102, got.id.value());
  TEST_ASSERT_FALSE(order.read(c, got));
}

static void wrong_bitrate_sees_errors_and_cannot_send(void) {
  VirtualCar car(renaultDaciaCar());
  SimBus bus(car, 2);
  bus.open(Bitrate::k250, CanMode::ListenOnly, Acceptance::all(), 0);
  bus.advance(60000);
  TEST_ASSERT_EQUAL_UINT8(0, bus.rxStatus(60000));
  TEST_ASSERT_TRUE(bus.health().errorEvents > 0);
  TEST_ASSERT_TRUE(bus.health().rec > 0);
  bus.open(Bitrate::k250, CanMode::Normal, Acceptance::all(), 60000);
  TEST_ASSERT_TRUE(bus.send(frame(0x7E0, {0x02, 0x10, 0xC0}), 60000) == TxStatus::Failed);
}

static void listen_only_cannot_send_and_acceptance_filters(void) {
  VirtualCar car(renaultDaciaCar());
  SimBus bus(car, 32);
  const Acceptance only622 = {false, false, 0x622, 0x7FF};
  bus.open(Bitrate::k500, CanMode::ListenOnly, only622, 0);
  TEST_ASSERT_TRUE(bus.send(frame(0x7E0, {0x02, 0x10, 0xC0}), 0) == TxStatus::Failed);
  for (uint32_t t = 0; t <= 200000; t += 10000) bus.advance(t);
  CanFrame got;
  int frames = 0;
  while (bus.readRxb(0, got)) {
    TEST_ASSERT_EQUAL_HEX32(0x622, got.id.value());
    ++frames;
  }
  TEST_ASSERT_EQUAL_INT(4, frames);
  TEST_ASSERT_FALSE(bus.health().rxOverflow);
}

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(single_frame_request_is_answered_after_the_latency);
  RUN_TEST(long_answer_waits_for_flow_control_then_streams);
  RUN_TEST(flow_control_block_size_and_stmin_are_honoured);
  RUN_TEST(missing_flow_control_abandons_the_answer);
  RUN_TEST(segmented_request_is_reassembled);
  RUN_TEST(unknown_requests_get_negative_responses);
  RUN_TEST(functional_requests_reach_only_obd_services);
  RUN_TEST(airbag_answers_pending_first);
  RUN_TEST(abs_ignores_extended_session);
  RUN_TEST(periodic_traffic_skips_backlog_after_a_pause);
  RUN_TEST(two_buffers_overflow_on_the_third_frame);
  RUN_TEST(rollover_order_returns_frames_oldest_first);
  RUN_TEST(wrong_bitrate_sees_errors_and_cannot_send);
  RUN_TEST(listen_only_cannot_send_and_acceptance_filters);
  return UNITY_END();
}

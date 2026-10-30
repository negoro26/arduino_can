#include <string.h>
#include <unity.h>

#include <algorithm>
#include <string>
#include <utility>
#include <vector>

#include "link_mux.h"
#include "test_hal.h"

void setUp(void) {}
void tearDown(void) {}

static uint16_t take(std::string &from, uint8_t *out, uint16_t capacity) {
  const uint16_t n = static_cast<uint16_t>(std::min<size_t>(from.size(), capacity));
  memcpy(out, from.data(), n);
  from.erase(0, n);
  return n;
}

class FakeUsb final : public ByteLink {
 public:
  std::string in;
  std::string out;

  uint16_t read(uint8_t *buf, uint16_t capacity) override {
    return take(in, buf, capacity);
  }
  void write(const char *bytes, uint16_t length) override {
    out.append(bytes, length);
  }
};

class FakeTcp final : public TcpLink {
 public:
  std::string in;
  // One entry per write (one send to the WINC): connection number, bytes.
  std::vector<std::pair<int, std::string>> sent;
  bool open = false;
  int connections = 0;

  void connect(const std::string &data) {
    waiting_ = true;
    waitingData_ = data;
  }

  bool accept() override {
    if (!waiting_) return false;
    waiting_ = false;
    open = true;
    in = waitingData_;
    ++connections;
    return true;
  }
  bool connected() override { return open; }
  uint16_t read(uint8_t *buf, uint16_t capacity) override {
    return open ? take(in, buf, capacity) : 0;
  }
  void write(const char *bytes, uint16_t length) override {
    if (open) sent.push_back({connections, std::string(bytes, length)});
  }

  std::string all() const {
    std::string s;
    for (const auto &w : sent) s += w.second;
    return s;
  }

 private:
  bool waiting_ = false;
  std::string waitingData_;
};

static std::string drain(LinkMux &mux) {
  std::string s;
  for (int c; (c = mux.readByte()) >= 0;) s += static_cast<char>(c);
  return s;
}

static void usb_is_the_session_without_a_client(void) {
  FakeUsb usb;
  FakeTcp tcp;
  LinkMux mux(usb, tcp);
  usb.in = "ATI\r";
  mux.poll(0);
  TEST_ASSERT_FALSE(mux.onTcp());
  TEST_ASSERT_EQUAL_STRING("ATI\r", drain(mux).c_str());
  mux.write("OK", 2, 0);
  TEST_ASSERT_EQUAL_STRING("OK", usb.out.c_str());
  TEST_ASSERT_TRUE(mux.flushDue().isNever());
}

static void tcp_takes_over_while_connected(void) {
  FakeUsb usb;
  FakeTcp tcp;
  LinkMux mux(usb, tcp);
  usb.in = "ATI\r";
  tcp.connect("ATZ\r");
  mux.poll(0);
  TEST_ASSERT_TRUE(mux.onTcp());
  TEST_ASSERT_EQUAL_STRING("ATZ\r", drain(mux).c_str());
  TEST_ASSERT_EQUAL_STRING("ATI\r", usb.in.c_str());
  mux.write("ELM327 v1.5\r\r>", 14, 0);
  TEST_ASSERT_EQUAL_size_t(1, tcp.sent.size());
  TEST_ASSERT_EQUAL_STRING("ELM327 v1.5\r\r>", tcp.sent[0].second.c_str());
  TEST_ASSERT_EQUAL_STRING("", usb.out.c_str());
  mux.poll(1);
  TEST_ASSERT_TRUE(mux.onTcp());
}

static void usb_resumes_after_the_client_closes(void) {
  FakeUsb usb;
  FakeTcp tcp;
  LinkMux mux(usb, tcp);
  tcp.connect("");
  mux.poll(0);
  usb.in = "ATI\r";
  TEST_ASSERT_EQUAL_INT(-1, mux.readByte());
  tcp.open = false;
  TEST_ASSERT_EQUAL_INT(kLinkClosed, mux.readByte());
  mux.poll(1);
  TEST_ASSERT_FALSE(mux.onTcp());
  TEST_ASSERT_EQUAL_STRING("ATI\r", drain(mux).c_str());
}

static void a_new_connection_replaces_the_old_one(void) {
  FakeUsb usb;
  FakeTcp tcp;
  LinkMux mux(usb, tcp);
  tcp.connect("ATZ\rAT");
  mux.poll(0);
  TEST_ASSERT_EQUAL_INT('A', mux.readByte());
  mux.write("AT", 2, 0);
  tcp.connect("ATI\r");
  mux.poll(5);
  TEST_ASSERT_TRUE(mux.onTcp());
  TEST_ASSERT_EQUAL_STRING("ATI\r", drain(mux).c_str());
  mux.write("ELM327 v1.5\r\r>", 14, 6);
  TEST_ASSERT_EQUAL_size_t(1, tcp.sent.size());
  TEST_ASSERT_EQUAL_INT(2, tcp.sent[0].first);
  TEST_ASSERT_EQUAL_STRING("ELM327 v1.5\r\r>", tcp.sent[0].second.c_str());
}

static void tcp_output_waits_for_the_prompt(void) {
  FakeUsb usb;
  FakeTcp tcp;
  LinkMux mux(usb, tcp);
  tcp.connect("");
  mux.poll(0);
  mux.write("41 00 BE 3F A8 13\r", 18, 0);
  mux.write("\r", 1, 1);
  TEST_ASSERT_EQUAL_size_t(0, tcp.sent.size());
  mux.write(">", 1, 2);
  TEST_ASSERT_EQUAL_size_t(1, tcp.sent.size());
  TEST_ASSERT_EQUAL_STRING("41 00 BE 3F A8 13\r\r>", tcp.sent[0].second.c_str());
  TEST_ASSERT_TRUE(mux.flushDue().isNever());
}

static void tcp_output_goes_in_1400_byte_chunks(void) {
  FakeUsb usb;
  FakeTcp tcp;
  LinkMux mux(usb, tcp);
  tcp.connect("");
  mux.poll(0);
  const std::string big(3000, 'A');
  mux.write(big.data(), static_cast<uint16_t>(big.size()), 0);
  TEST_ASSERT_EQUAL_size_t(2, tcp.sent.size());
  TEST_ASSERT_EQUAL_size_t(1400, tcp.sent[0].second.size());
  TEST_ASSERT_EQUAL_size_t(1400, tcp.sent[1].second.size());
  mux.write(">", 1, 1);
  TEST_ASSERT_EQUAL_size_t(3, tcp.sent.size());
  TEST_ASSERT_EQUAL_size_t(201, tcp.sent[2].second.size());
}

static void held_tcp_output_goes_after_20_ms(void) {
  FakeUsb usb;
  FakeTcp tcp;
  LinkMux mux(usb, tcp);
  tcp.connect("");
  mux.poll(100);
  mux.write("A", 1, 100);
  TEST_ASSERT_EQUAL_UINT32(120, mux.flushDue().atMs());
  mux.poll(119);
  TEST_ASSERT_EQUAL_size_t(0, tcp.sent.size());
  mux.poll(120);
  TEST_ASSERT_EQUAL_size_t(1, tcp.sent.size());
  TEST_ASSERT_EQUAL_STRING("A", tcp.sent[0].second.c_str());

  mux.write("T", 1, 200);
  mux.write("I", 1, 220);
  TEST_ASSERT_EQUAL_size_t(2, tcp.sent.size());
  TEST_ASSERT_EQUAL_STRING("TI", tcp.sent[1].second.c_str());

  mux.write("X", 1, 300);
  mux.flush();
  TEST_ASSERT_EQUAL_size_t(3, tcp.sent.size());
  TEST_ASSERT_EQUAL_STRING("X", tcp.sent[2].second.c_str());
}

class MuxHal final : public VirtualTimeHal {
 public:
  explicit MuxHal(const CarSpec &car) : VirtualTimeHal(car, 2), mux(usb, tcp) {}

  FakeUsb usb;
  FakeTcp tcp;
  LinkMux mux;
  uint32_t hangUpAtMs = UINT32_MAX;

  void poll() { mux.poll(nowMs()); }

  int readByte() override {
    if (now() >= hangUpAtMs * 1000ull) tcp.open = false;
    return mux.readByte();
  }
  void write(const char *bytes, uint16_t length) override {
    mux.write(bytes, length, nowMs());
  }
  void flush() override { mux.flush(); }
  bool linkTrusted() override { return !mux.onTcp(); }
  uint8_t linkSwitches() override { return mux.switches(); }
  bool wifiStatus(uint8_t index, char *out, uint8_t capacity) override {
    if (index > 0) return false;
    snprintf(out, capacity, "WIFI TEST");
    return true;
  }

 protected:
  bool nextInputUs(uint32_t *atUs) const override {
    if (hangUpAtMs == UINT32_MAX || !tcp.open) return false;
    *atUs = hangUpAtMs * 1000;
    return true;
  }
};

static void elm_answers_on_the_session_link_only(void) {
  MuxHal hal(renaultDaciaCarQuiet());
  Elm elm(hal);
  elm.begin();
  TEST_ASSERT_EQUAL_STRING("ELM327 v1.5\r\r>", hal.usb.out.c_str());
  hal.usb.out.clear();

  hal.usb.in = "ATI\r";
  hal.tcp.connect("ATI\rATE0\r");
  hal.poll();
  elm.service();
  TEST_ASSERT_EQUAL_size_t(2, hal.tcp.sent.size());
  TEST_ASSERT_EQUAL_STRING("ATI\rELM327 v1.5\r\r>", hal.tcp.sent[0].second.c_str());
  TEST_ASSERT_EQUAL_STRING("ATE0\rOK\r\r>", hal.tcp.sent[1].second.c_str());
  TEST_ASSERT_EQUAL_STRING("", hal.usb.out.c_str());
  TEST_ASSERT_EQUAL_STRING("ATI\r", hal.usb.in.c_str());

  hal.tcp.open = false;
  elm.service();
  hal.poll();
  elm.service();
  // Echo stays off: switching links does not reset the settings.
  TEST_ASSERT_EQUAL_STRING("ELM327 v1.5\r\r>", hal.usb.out.c_str());
}

static void guard_write_is_refused_over_tcp(void) {
  MuxHal hal(renaultDaciaCarQuiet());
  Elm elm(hal);
  elm.begin();
  hal.tcp.connect("ATE0\rAT#GUARD WRITE\rAT#GUARD\rAT#GUARD CLEAR\rAT#GUARD\r");
  hal.poll();
  elm.service();
  TEST_ASSERT_EQUAL_STRING("ATE0\rOK\r\r>?\r\r>READ\r\r>OK\r\r>CLEAR\r\r>", hal.tcp.all().c_str());

  hal.tcp.open = false;
  elm.service();
  hal.poll();
  hal.usb.in = "AT#GUARD WRITE\rAT#GUARD\r";
  elm.service();
  TEST_ASSERT_EQUAL_STRING("ELM327 v1.5\r\r>OK\r\r>WRITE\r\r>", hal.usb.out.c_str());
}

static void a_phone_taking_the_session_drops_write_to_read(void) {
  MuxHal hal(renaultDaciaCarQuiet());
  Elm elm(hal);
  elm.begin();
  hal.usb.out.clear();
  hal.usb.in = "ATE0\rAT#GUARD WRITE\rAT#GUARD\r";
  elm.service();
  TEST_ASSERT_EQUAL_STRING("ATE0\rOK\r\r>OK\r\r>WRITE\r\r>", hal.usb.out.c_str());

  hal.tcp.connect("AT#GUARD\r");
  hal.poll();
  elm.service();
  TEST_ASSERT_EQUAL_STRING("READ\r\r>", hal.tcp.all().c_str());

  hal.tcp.open = false;
  elm.service();
  hal.poll();
  hal.usb.out.clear();
  hal.usb.in = "AT#GUARD\r";
  elm.service();
  TEST_ASSERT_EQUAL_STRING("READ\r\r>", hal.usb.out.c_str());
}

static void a_write_service_from_the_phone_is_blocked(void) {
  MuxHal hal(renaultDaciaCarQuiet());
  Elm elm(hal);
  elm.begin();
  hal.usb.out.clear();
  hal.usb.in = "ATE0\rATSP6\rATSH7E0\rAT#GUARD WRITE\r1101\r";
  elm.service();
  TEST_ASSERT_EQUAL_STRING("ATE0\rOK\r\r>OK\r\r>OK\r\r>OK\r\r>7F 11 11\r\r>",
                           hal.usb.out.c_str());

  hal.tcp.connect("ATSH7E0\r1101\r");
  hal.poll();
  elm.service();
  TEST_ASSERT_EQUAL_STRING("OK\r\r>BLOCKED\r\r>", hal.tcp.all().c_str());
}

static void clear_set_over_usb_survives_a_phone_taking_the_session(void) {
  MuxHal hal(renaultDaciaCarQuiet());
  Elm elm(hal);
  elm.begin();
  hal.usb.in = "ATE0\rAT#GUARD CLEAR\r";
  elm.service();

  hal.tcp.connect("AT#GUARD\r");
  hal.poll();
  elm.service();
  TEST_ASSERT_EQUAL_STRING("CLEAR\r\r>", hal.tcp.all().c_str());
}

static void a_line_the_phone_left_unfinished_does_not_run_on_usb(void) {
  MuxHal hal(renaultDaciaCarQuiet());
  Elm elm(hal);
  elm.begin();
  hal.usb.out.clear();
  hal.tcp.connect("ATE0\rAT#GUARD WRITE");
  hal.poll();
  elm.service();

  hal.tcp.open = false;
  elm.service();
  hal.poll();
  hal.usb.in = "\rAT#GUARD\r";
  elm.service();
  TEST_ASSERT_EQUAL_STRING("?\r\r>READ\r\r>", hal.usb.out.c_str());
}

static void a_bare_cr_on_usb_does_not_repeat_the_phone_request(void) {
  MuxHal hal(renaultDaciaCarQuiet());
  Elm elm(hal);
  elm.begin();
  hal.usb.out.clear();
  hal.tcp.connect("ATE0\rATSP6\r0100\r");
  hal.poll();
  elm.service();
  TEST_ASSERT_EQUAL_STRING("ATE0\rOK\r\r>OK\r\r>41 00 BE 1F A8 12\r\r>",
                           hal.tcp.all().c_str());

  hal.tcp.open = false;
  elm.service();
  hal.poll();
  hal.usb.in = "\r";
  elm.service();
  TEST_ASSERT_EQUAL_STRING("?\r\r>", hal.usb.out.c_str());
}

static void the_phone_first_command_ignores_a_line_half_typed_on_usb(void) {
  MuxHal hal(renaultDaciaCarQuiet());
  Elm elm(hal);
  elm.begin();
  hal.usb.in = "ATI";
  elm.service();

  hal.tcp.connect("ATZ\r");
  hal.poll();
  elm.service();
  TEST_ASSERT_EQUAL_STRING("ATZ\r\rELM327 v1.5\r\r>", hal.tcp.all().c_str());
}

static void monitor_over_tcp_streams_and_stops_when_the_client_goes(void) {
  MuxHal hal(renaultDaciaCar());
  Elm elm(hal);
  elm.begin();
  hal.tcp.connect("ATE0\rATH1\rATSP6\r");
  hal.poll();
  elm.service();
  hal.tcp.sent.clear();

  hal.hangUpAtMs = hal.nowMs() + 25;
  hal.tcp.in = "ATMA\r";
  elm.service();
  TEST_ASSERT_TRUE(hal.tcp.sent.size() >= 2);
  for (const auto &w : hal.tcp.sent) {
    TEST_ASSERT_NULL(strchr(w.second.c_str(), '>'));
  }
  TEST_ASSERT_EQUAL_STRING("181 0B B8 00 00 1E 00 00 00\r"
                           "354 00 00 00 00 00 00 00 00\r",
                           hal.tcp.all().substr(0, 56).c_str());

  hal.poll();
  TEST_ASSERT_FALSE(hal.mux.onTcp());
  hal.usb.in = "ATI\r";
  elm.service();
  TEST_ASSERT_EQUAL_STRING("ELM327 v1.5\r\r>ELM327 v1.5\r\r>", hal.usb.out.c_str());
}

static void at_wifi_prints_the_hal_lines(void) {
  TestHal hal(renaultDaciaCarQuiet());
  Elm elm(hal);
  elm.begin();
  hal.setWifiStatus({"WIFI AP MKR-OBDII WPA/WPA2 192.168.0.10:35000",
                     "WINC firmware 19.6.1", "CLIENT 192.168.0.100 connected"});
  TEST_ASSERT_EQUAL_STRING(
      "AT#WIFI\rWIFI AP MKR-OBDII WPA/WPA2 192.168.0.10:35000\r"
      "WINC firmware 19.6.1\rCLIENT 192.168.0.100 connected\r\r>",
      ask(hal, elm, "AT#WIFI").c_str());
  hal.setWifiStatus(
      {"WIFI OFF: access point refused (WPA/WPA2 needs WINC firmware 19.5.2+)"});
  TEST_ASSERT_EQUAL_STRING(
      "AT#WIFI\r"
      "WIFI OFF: access point refused (WPA/WPA2 needs WINC firmware 19.5.2+)\r\r>",
      ask(hal, elm, "AT#WIFI").c_str());
  hal.setWifiStatus({"WIFI OFF: no WINC1500 found"});
  TEST_ASSERT_EQUAL_STRING("AT#WIFI\rWIFI OFF: no WINC1500 found\r\r>",
                           ask(hal, elm, "AT#WIFI").c_str());
}

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(usb_is_the_session_without_a_client);
  RUN_TEST(tcp_takes_over_while_connected);
  RUN_TEST(usb_resumes_after_the_client_closes);
  RUN_TEST(a_new_connection_replaces_the_old_one);
  RUN_TEST(tcp_output_waits_for_the_prompt);
  RUN_TEST(tcp_output_goes_in_1400_byte_chunks);
  RUN_TEST(held_tcp_output_goes_after_20_ms);
  RUN_TEST(elm_answers_on_the_session_link_only);
  RUN_TEST(guard_write_is_refused_over_tcp);
  RUN_TEST(a_phone_taking_the_session_drops_write_to_read);
  RUN_TEST(a_write_service_from_the_phone_is_blocked);
  RUN_TEST(clear_set_over_usb_survives_a_phone_taking_the_session);
  RUN_TEST(a_line_the_phone_left_unfinished_does_not_run_on_usb);
  RUN_TEST(a_bare_cr_on_usb_does_not_repeat_the_phone_request);
  RUN_TEST(the_phone_first_command_ignores_a_line_half_typed_on_usb);
  RUN_TEST(monitor_over_tcp_streams_and_stops_when_the_client_goes);
  RUN_TEST(at_wifi_prints_the_hal_lines);
  return UNITY_END();
}

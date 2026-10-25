#include <string.h>
#include <unity.h>

#include <algorithm>
#include <string>
#include <utility>
#include <vector>

#include "link_mux.h"

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

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(usb_is_the_session_without_a_client);
  RUN_TEST(tcp_takes_over_while_connected);
  RUN_TEST(usb_resumes_after_the_client_closes);
  RUN_TEST(a_new_connection_replaces_the_old_one);
  RUN_TEST(tcp_output_waits_for_the_prompt);
  RUN_TEST(tcp_output_goes_in_1400_byte_chunks);
  RUN_TEST(held_tcp_output_goes_after_20_ms);
  return UNITY_END();
}

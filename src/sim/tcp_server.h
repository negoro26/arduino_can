// The simulator's WiFi: a TCP listener standing in for the WINC1500's, one
// client at a time.

#ifndef TCP_SERVER_H
#define TCP_SERVER_H

#include "link_mux.h"

class TcpServer final : public TcpLink {
 public:
  // Listens on "HOST:PORT" (IPv4 address). false (with a message on stderr)
  // if it is malformed or cannot be bound.
  bool open(const char *hostPort);
  // -1 while not listening / no client.
  int listenFd() const { return listen_; }
  int clientFd() const { return client_; }

  bool accept() override;
  bool connected() override { return client_ >= 0; }
  uint16_t read(uint8_t *out, uint16_t capacity) override;
  void write(const char *bytes, uint16_t length) override;

  bool statusLine(uint8_t index, char *out, uint8_t capacity);

 private:
  void closeClient();

  int listen_ = -1;
  int client_ = -1;
  char address_[24] = {};
  char peer_[24] = {};
};

#endif  // TCP_SERVER_H

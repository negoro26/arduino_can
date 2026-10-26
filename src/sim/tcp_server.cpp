#include "tcp_server.h"

#include <arpa/inet.h>
#include <errno.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

bool TcpServer::open(const char *hostPort) {
  const char *colon = strrchr(hostPort, ':');
  char host[16] = {};
  unsigned long port = 0;
  if (colon != nullptr && static_cast<size_t>(colon - hostPort) < sizeof host) {
    memcpy(host, hostPort, static_cast<size_t>(colon - hostPort));
    char *end = nullptr;
    port = strtoul(colon + 1, &end, 10);
    if (end == colon + 1 || *end != '\0' || port > 65535) port = 0;
  }
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(static_cast<uint16_t>(port));
  if (port == 0 || inet_pton(AF_INET, host, &addr.sin_addr) != 1) {
    fprintf(stderr, "--tcp %s: expected IPV4:PORT\n", hostPort);
    return false;
  }
  listen_ = socket(AF_INET, SOCK_STREAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
  const int one = 1;
  if (listen_ < 0 ||
      setsockopt(listen_, SOL_SOCKET, SO_REUSEADDR, &one, sizeof one) != 0 ||
      bind(listen_, reinterpret_cast<sockaddr *>(&addr), sizeof addr) != 0 ||
      listen(listen_, 1) != 0) {
    perror(hostPort);
    return false;
  }
  snprintf(address_, sizeof address_, "%s:%lu", host, port);
  return true;
}

bool TcpServer::accept() {
  if (listen_ < 0) return false;
  bool took = false;
  for (;;) {
    sockaddr_in peer{};
    socklen_t len = sizeof peer;
    const int fd = accept4(listen_, reinterpret_cast<sockaddr *>(&peer), &len,
                           SOCK_NONBLOCK | SOCK_CLOEXEC);
    if (fd < 0) return took;
    closeClient();
    client_ = fd;
    const int one = 1;
    setsockopt(client_, IPPROTO_TCP, TCP_NODELAY, &one, sizeof one);
    char ip[INET_ADDRSTRLEN] = {};
    inet_ntop(AF_INET, &peer.sin_addr, ip, sizeof ip);
    snprintf(peer_, sizeof peer_, "%s:%u", ip, ntohs(peer.sin_port));
    took = true;
  }
}

void TcpServer::closeClient() {
  if (client_ < 0) return;
  close(client_);
  client_ = -1;
}

uint16_t TcpServer::read(uint8_t *out, uint16_t capacity) {
  if (client_ < 0) return 0;
  const ssize_t n = recv(client_, out, capacity, 0);
  if (n > 0) return static_cast<uint16_t>(n);
  if (n == 0 || (errno != EAGAIN && errno != EINTR)) closeClient();
  return 0;
}

void TcpServer::write(const char *bytes, uint16_t length) {
  while (client_ >= 0 && length > 0) {
    const ssize_t n = send(client_, bytes, length, MSG_NOSIGNAL);
    if (n > 0) {
      bytes += n;
      length = static_cast<uint16_t>(length - n);
      continue;
    }
    pollfd p{client_, POLLOUT, 0};
    if ((n < 0 && errno != EAGAIN && errno != EINTR) || poll(&p, 1, 1000) <= 0) {
      closeClient();
    }
  }
}

bool TcpServer::statusLine(uint8_t index, char *out, uint8_t capacity) {
  if (listen_ < 0) {
    if (index > 0) return false;
    snprintf(out, capacity, "WIFI OFF: simulator started without --tcp");
    return true;
  }
  switch (index) {
    case 0:
      snprintf(out, capacity, "WIFI SIM TCP %s", address_);
      return true;
    case 1:
      if (client_ >= 0) {
        snprintf(out, capacity, "CLIENT %s connected", peer_);
      } else {
        snprintf(out, capacity, "CLIENT none");
      }
      return true;
    default:
      return false;
  }
}

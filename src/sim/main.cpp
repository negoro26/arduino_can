#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "elm.h"
#include "pty_hal.h"
#include "renault_scenario.h"

namespace {

const char *g_link = "/tmp/elm327-sim";

void removeLinkAndExit(int) {
  unlink(g_link);
  _exit(0);
}

int usage(const char *program) {
  fprintf(stderr,
          "usage: %s [--link PATH] [--tcp HOST:PORT] [--car-bitrate 500|250] "
          "[--quiet]\n",
          program);
  return 2;
}

}  // namespace

int main(int argc, char **argv) {
  Bitrate rate = Bitrate::k500;
  bool quiet = false;
  const char *tcp = nullptr;
  for (int i = 1; i < argc; ++i) {
    if (strcmp(argv[i], "--link") == 0 && i + 1 < argc) {
      g_link = argv[++i];
    } else if (strcmp(argv[i], "--tcp") == 0 && i + 1 < argc) {
      tcp = argv[++i];
    } else if (strcmp(argv[i], "--car-bitrate") == 0 && i + 1 < argc) {
      const char *value = argv[++i];
      if (strcmp(value, "500") == 0) {
        rate = Bitrate::k500;
      } else if (strcmp(value, "250") == 0) {
        rate = Bitrate::k250;
      } else {
        return usage(argv[0]);
      }
    } else if (strcmp(argv[i], "--quiet") == 0) {
      quiet = true;
    } else {
      return usage(argv[0]);
    }
  }

  static PtyHal hal(quiet ? renaultDaciaCarQuiet(rate) : renaultDaciaCar(rate),
                    g_link);
  if (tcp != nullptr && !hal.listen(tcp)) return 1;
  if (!hal.open()) return 1;
  struct sigaction sa;
  memset(&sa, 0, sizeof sa);
  sa.sa_handler = removeLinkAndExit;
  sigaction(SIGINT, &sa, nullptr);
  sigaction(SIGTERM, &sa, nullptr);

  static Elm elm(hal);
  elm.begin();
  if (tcp != nullptr) {
    printf("ready %s tcp %s\n", g_link, tcp);
  } else {
    printf("ready %s\n", g_link);
  }
  fflush(stdout);
  for (;;) {
    hal.waitForInput();
    hal.pollLinks();
    elm.service();
  }
}

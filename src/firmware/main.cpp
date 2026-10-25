// ELM327 v1.5-compatible adapter (CAN protocols only) on an Arduino MKR1000 +
// MKR CAN shield, over native USB CDC. Built with -DELM_LOOPBACK_DEMO it
// answers as a virtual Renault/Dacia through the MCP2515 in loopback mode.

#include <Arduino.h>

#include "elm.h"

#ifdef ELM_LOOPBACK_DEMO
#include "loopback_demo_hal.h"
#include "renault_scenario.h"
static LoopbackDemoHal hal(renaultDaciaCar());
#else
#include "mcp2515_hal.h"
static Mcp2515Hal hal;
#endif

static Elm elm(hal);

void setup() {
  hal.begin();
  elm.begin();
}

void loop() {
  elm.service();
#ifdef ELM_LOOPBACK_DEMO
  hal.keepCarClock();
#endif
}

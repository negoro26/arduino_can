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

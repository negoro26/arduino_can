// Ids from pyren's dnat/snat tables and ddt4all's projects.json; payloads are
// invented but shaped like the real services, so pyren, ddt4all, python-OBD
// and tools/canscan.py parse them.

#ifndef RENAULT_SCENARIO_H
#define RENAULT_SCENARIO_H

#include "vecu.h"

CarSpec renaultDaciaCar(Bitrate rate = Bitrate::k500);

CarSpec renaultDaciaCarQuiet(Bitrate rate = Bitrate::k500);

#endif  // RENAULT_SCENARIO_H

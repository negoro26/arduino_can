// The Dacia Sandero II / Renault virtual car.
//
// Ids from pyren's dnat/snat tables and ddt4all's projects.json; payloads are
// invented but shaped like the real services, so pyren, ddt4all, python-OBD
// and tools/canscan.py parse them. One definition serves the PTY simulator,
// the transcript tests and the loopback demo firmware.
//
//   ECM    7E0 -> 7E8  UDS + OBD-II (also answers OBD-II on 7DF)
//   UCH    745 -> 765  KWP2000 on CAN
//   ABS    740 -> 760  KWP2000 on CAN (10 03 gets no answer at all)
//   Airbag 752 -> 772  UDS, every known request first answered 7F <sid> 78,
//                      the final answer 300 ms later
//   EPS    742 -> 762  absent
// Background traffic: 181 every 10 ms, 354 every 20 ms, 622 every 50 ms,
// 5DE every 100 ms (622 is also a request id in the scanner's Renault table).

#ifndef RENAULT_SCENARIO_H
#define RENAULT_SCENARIO_H

#include "vecu.h"

CarSpec renaultDaciaCar(Bitrate rate = Bitrate::k500);

// Same ECUs, no background traffic: for tests that assert exact wire logs or
// a "silent bus" listen-before-talk verdict.
CarSpec renaultDaciaCarQuiet(Bitrate rate = Bitrate::k500);

#endif  // RENAULT_SCENARIO_H

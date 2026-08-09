// Feasibility bring-up for "ddt4all -> ELM327 emulator -> MCP2515" on an
// Arduino MKR1000 + official MKR CAN shield (ASX00005).
//
// Stages 1-3 are strictly passive and run automatically. Stage 4 transmits and
// is gated behind BOTH a manual keypress and an automatic safety interlock.
//   1. SPI/register probe   -- is the MCP2515 there at all?
//   2. LOOPBACK self-test   -- arbitrary 11-bit ID + 8 data bytes through the
//                              controller and back. Never reaches the wire.
//   3. LISTEN-ONLY sniff    -- 500 then 250 kbit. Listen-only means the MCP2515
//                              never transmits and never even ACKs, so this
//                              physically cannot disturb a vehicle bus. Records
//                              which IDs real modules are using.
//   4. ACTIVE ECU DISCOVERY -- sweeps diagnostic request IDs sending UDS
//                              StartDiagnosticSession, to find which modules
//                              speak diagnostics over CAN rather than K-line.
//                              INTERLOCK: refuses to run unless stage 3 saw a
//                              live bus, and skips every ID that stage 3
//                              observed in normal traffic, so it can never
//                              collide with a module's operational messages.
//
// Shield facts (fixed by the PCB, per the ASX00005 schematic):
//   CS = D3, INT = D7, MCP2515 crystal = 16 MHz, transceiver = TJA1049T/3
//   (STB tied low via R8 = 0R), onboard 120R terminator R3, transceiver VDD
//   fed from the header 5V pin, so USB power alone is sufficient.

#include <Arduino.h>
#include <SPI.h>
#include <mcp2515.h>

#include "console.h"
#include "diag_rules.h"

static const uint8_t PIN_CAN_CS = 3;
static const uint8_t PIN_CAN_INT = 7;

// 8 MHz rather than the library's 10 MHz default: 10 MHz is the MCP2515
// datasheet maximum, and there is no reason to sit on the spec edge here.
static MCP2515 mcp2515(PIN_CAN_CS, 8000000, &SPI);

static const uint8_t REG_CANSTAT = 0x0E;
static const uint8_t REG_CANCTRL = 0x0F;
static const uint8_t CANINTF_ERRIF = 0x20;
static const uint8_t CANINTF_MERRF = 0x80;

// ------------------------------------------------- observed-bus state

static SeenIds<64> g_seen;
static bool g_busAlive = false;
static CAN_SPEED g_busSpeed = CAN_500KBPS;
static const __FlashStringHelper *g_busSpeedLabel = nullptr;

// ------------------------------------------------------------- raw SPI

static void spiBegin() {
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(PIN_CAN_CS, LOW);
}

static void spiEnd() {
  digitalWrite(PIN_CAN_CS, HIGH);
  SPI.endTransaction();
}

// Independent of the library, so stage 1 stays a hardware question rather than
// a library question.
static uint8_t rawRegRead(uint8_t reg) {
  spiBegin();
  SPI.transfer(0x03); // READ
  SPI.transfer(reg);
  uint8_t v = SPI.transfer(0x00);
  spiEnd();
  return v;
}

// No raw writes: every register this probe modifies is reachable through the
// library (clearERRIF/clearMERR/setNormalOneShotMode). The read helper above
// exists only so stage 1 can question the hardware without going through the
// library it is about to depend on.

static void printHex2(uint8_t v) {
  if (v < 0x10) {
    Serial.print('0');
  }
  Serial.print(v, HEX);
}

// ---------------------------------------------------------------- stage 1

static bool stageProbe() {
  Serial.println(F("[1] MCP2515 presence"));
  mcp2515.reset();
  delay(10);

  uint8_t canstat = rawRegRead(REG_CANSTAT);
  uint8_t canctrl = rawRegRead(REG_CANCTRL);
  Serial.print(F("    CANSTAT=0x"));
  Serial.print(canstat, HEX);
  Serial.print(F(" CANCTRL=0x"));
  Serial.println(canctrl, HEX);

  bool ok = (canstat == 0x80) && (canctrl == 0x87);
  Serial.println(ok ? F("    PASS: controller present, in config mode")
                    : F("    FAIL: no controller / wrong reset state"));
  return ok;
}

// ---------------------------------------------------------------- stage 2

// Loopback proves SPI framing, CNF1/2/3 acceptance, TX load, RX readback and
// payload fidelity. It does NOT prove the crystal is 16 MHz and does NOT
// exercise the transceiver: loopback never puts a bit on the wire, so both
// would still pass here and only fail against a real bus.
static bool loopbackAt(CAN_SPEED speed, const __FlashStringHelper *label) {
  Serial.print(F("    "));
  Serial.print(label);
  Serial.print(F(": "));

  if (mcp2515.reset() != MCP2515::ERROR_OK ||
      mcp2515.setBitrate(speed, MCP_16MHZ) != MCP2515::ERROR_OK ||
      mcp2515.setLoopbackMode() != MCP2515::ERROR_OK) {
    Serial.println(F("setup failed"));
    return false;
  }

  struct can_frame tx;
  tx.can_id = 0x745; // a real Renault diagnostic request id from ddt4all's db
  tx.can_dlc = 8;
  const uint8_t payload[8] = {0x02, 0x10, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00};
  memcpy(tx.data, payload, sizeof(payload));

  if (mcp2515.sendMessage(&tx) != MCP2515::ERROR_OK) {
    Serial.println(F("sendMessage failed"));
    return false;
  }

  struct can_frame rx;
  unsigned long deadline = millis() + 500;
  while (millis() < deadline) {
    if (mcp2515.readMessage(&rx) == MCP2515::ERROR_OK) {
      bool match = (rx.can_id == tx.can_id) && (rx.can_dlc == tx.can_dlc) &&
                   (memcmp(rx.data, tx.data, tx.can_dlc) == 0);
      Serial.println(match ? F("PASS (id + 8 bytes round-tripped)")
                           : F("FAIL corrupt round-trip"));
      return match;
    }
  }
  Serial.println(F("FAIL timeout: nothing looped back"));
  return false;
}

// Bitrates the probe understands, shared by stages 2 and 3 so the two cannot
// drift apart. 125 and 100 kbit matter because a Renault comfort/multimedia bus
// may run there; without them "wrong bitrate" looks like "wrong pins".
struct Candidate {
  CAN_SPEED speed;
  const __FlashStringHelper *label;
};

static const Candidate CANDIDATES[] = {
    {CAN_500KBPS, F("500 kbit")},
    {CAN_250KBPS, F("250 kbit")},
    {CAN_125KBPS, F("125 kbit")},
    {CAN_100KBPS, F("100 kbit")},
};
static const uint8_t CANDIDATE_COUNT =
    sizeof(CANDIDATES) / sizeof(CANDIDATES[0]);

// Passes if AT LEAST ONE bitrate round-trips a frame, because that is all this
// stage claims to prove: SPI framing, TX load, RX readback and payload
// fidelity. Requiring every bitrate would let one bad CNF table entry veto the
// bus survey even though the rate the car actually uses works fine.
static bool stageLoopback() {
  Serial.println(F("[2] Loopback self-test (raw arbitrary-ID frames)"));

  uint8_t passed = 0;
  for (uint8_t i = 0; i < CANDIDATE_COUNT; i++) {
    if (loopbackAt(CANDIDATES[i].speed, CANDIDATES[i].label)) {
      passed++;
    }
  }

  if (passed == 0) {
    Serial.println(
        F("    FAIL: no bitrate round-tripped - controller unusable"));
    return false;
  }
  if (passed < CANDIDATE_COUNT) {
    Serial.print(F("    WARNING: only "));
    Serial.print(passed);
    Serial.print('/');
    Serial.print(CANDIDATE_COUNT);
    Serial.println(F(" bitrates usable; the frame path itself works, so"));
    Serial.println(
        F("    continuing. Stage 3 will report any rate it cannot set."));
  }
  return true;
}

// ---------------------------------------------------------------- stage 3

// Result of one listen-only window. `errors` is the discriminator that answers
// "are my pins even on a CAN bus?": at the WRONG bitrate on a LIVE bus the
// controller cannot decode anything, so it racks up receive errors. On
// disconnected or dead wires it sees nothing at all and stays clean.
struct SniffResult {
  uint32_t frames;
  uint32_t errors; // ERRIF/MERRF events observed during the window
  uint8_t rec;     // receive error counter
  uint8_t eflg;    // error flag register
};

static SniffResult sniff(CAN_SPEED speed, const __FlashStringHelper *label,
                         unsigned long windowMs) {
  SniffResult r = {0, 0, 0, 0};

  Serial.print(F("    "));
  Serial.print(label);
  Serial.print(F(": "));

  mcp2515.reset(); // also zeroes TEC/REC
  if (mcp2515.setBitrate(speed, MCP_16MHZ) != MCP2515::ERROR_OK ||
      mcp2515.setListenOnlyMode() != MCP2515::ERROR_OK) {
    Serial.println(F("setup failed"));
    return r;
  }

  uint8_t before = g_seen.count();

  struct can_frame rx;
  unsigned long deadline = millis() + windowMs;
  while (millis() < deadline) {
    const uint8_t intf = mcp2515.getInterrupts();
    if (intf & (CANINTF_ERRIF | CANINTF_MERRF)) {
      r.errors++;
      mcp2515.clearERRIF();
      mcp2515.clearMERR();
    }
    if (mcp2515.readMessage(&rx) != MCP2515::ERROR_OK) {
      continue;
    }
    r.frames++;
    g_seen.add(rx.can_id);
  }

  r.rec = mcp2515.errorCountRX();
  r.eflg = mcp2515.getErrorFlags();

  Serial.print(r.frames);
  Serial.print(F(" frames, err="));
  Serial.print(r.errors);
  Serial.print(F(" REC="));
  Serial.print(r.rec);
  Serial.print(F(" EFLG=0x"));
  Serial.print(r.eflg, HEX);

  if (r.frames == 0) {
    Serial.println();
    return r;
  }

  Serial.print(F("  ids: "));
  for (uint8_t i = before; i < g_seen.count(); i++) {
    Serial.print(F("0x"));
    Serial.print(g_seen.at(i), HEX);
    Serial.print(' ');
  }
  if (g_seen.isFull()) {
    Serial.print(F("(list full)"));
  }
  Serial.println();
  return r;
}

static void stageSniff() {
  Serial.println(F("[3] Listen-only bus survey (cannot disturb the vehicle)"));
  g_seen.clear();
  g_busAlive = false;

  uint32_t totalErrors = 0;
  for (uint8_t i = 0; i < CANDIDATE_COUNT; i++) {
    SniffResult result = sniff(CANDIDATES[i].speed, CANDIDATES[i].label, 2500);
    totalErrors += result.errors + result.rec;
    if (result.frames > 0) {
      g_busAlive = true;
      g_busSpeed = CANDIDATES[i].speed;
      g_busSpeedLabel = CANDIDATES[i].label;
      break; // decoding cleanly; no need to try slower rates
    }
  }

  Serial.println();
  if (!g_busAlive) {
    // This is the wiring verdict. Errors without frames means the transceiver
    // IS seeing differential activity it cannot decode, which proves the pins
    // are on a live bus. Total silence means they are not.
    if (totalErrors > 0) {
      Serial.println(F("    WIRING: pins look CONNECTED to a live bus"));
      Serial.println(F("      -> bus activity seen but nothing decoded."));
      Serial.println(
          F("      -> try swapping CAN-H and CAN-L, or the bus runs"));
      Serial.println(F("         at a rate not tried here."));
    } else {
      Serial.println(F("    WIRING: pins look NOT CONNECTED"));
      Serial.println(
          F("      -> zero frames AND zero errors at every bitrate."));
      Serial.println(
          F("      -> wrong OBD pins, broken cable, or bus asleep."));
      Serial.println(F("      -> check ignition is in position II."));
    }
    Serial.println(F("    stage 4 will refuse to transmit"));
    return;
  }

  // Report any operational traffic that overlaps the diagnostic ID range. Those
  // IDs get excluded from the active sweep.
  const uint8_t overlap = g_seen.diagRangeOverlap();
  Serial.print(F("    WIRING: OK - bus decoding cleanly at "));
  Serial.println(g_busSpeedLabel);
  Serial.print(F("    "));
  Serial.print(g_seen.count());
  Serial.print(F(" distinct ids, "));
  Serial.print(overlap);
  Serial.println(F(" of them inside 0x700-0x7FF (will be skipped)"));
}

// ---------------------------------------------------------------- stage 4

// For each candidate request ID we send one single-frame UDS
// StartDiagnosticSession (0x10 0xC0). Service 0x10 opens a session and writes
// nothing; it is in ddt4all's own safe_commands whitelist.
//
// Responders are recognised by looksLikeDiagReply() in diag_rules.h, which is
// unit-tested on the host.

static void stageDiscover() {
  Serial.println(F("[4] ACTIVE ECU discovery - this TRANSMITS on the bus"));

  // INTERLOCK 1: never transmit into an unknown bus. If stage 3 saw nothing we
  // do not know the bitrate, do not know the wiring is right, and cannot know
  // which IDs are in use -- so transmitting would be guessing.
  if (!g_busAlive) {
    Serial.println(F("    REFUSED: stage 3 saw no traffic."));
    Serial.println(F("    Fix wiring/ignition until stage 3 reports frames."));
    return;
  }

  mcp2515.reset();
  if (mcp2515.setBitrate(g_busSpeed, MCP_16MHZ) != MCP2515::ERROR_OK) {
    Serial.println(F("    setup failed"));
    return;
  }
  // One-shot mode: never retry a frame nobody ACKs, so a silent responder
  // cannot wedge the TX buffers or leave us hammering the bus.
  //
  // The return value is deliberately ignored. setNormalOneShotMode() programs
  // CANCTRL correctly -- setMode() writes with mask REQOP|OSM (0xE8) and value
  // 0x08, giving REQOP=normal plus OSM=1 -- but then verifies by comparing
  // CANSTAT & CANSTAT_OPMOD (0xE0) against that same 0x08. OPMOD can never
  // carry the OSM bit, so the comparison is 0x00 == 0x08 and the call always
  // reports ERROR_FAIL after spinning for 10 ms. The chip is configured; only
  // the library's self-check is wrong. Verified against CANCTRL below.
  mcp2515.setNormalOneShotMode();

  const uint8_t canctrl = rawRegRead(REG_CANCTRL);
  if ((canctrl & 0x08) == 0) {
    Serial.println(F("    setup failed: one-shot mode not active"));
    return;
  }

  Serial.print(F("    sweeping 0x"));
  Serial.print(DIAG_ID_FIRST, HEX);
  Serial.print(F("-0x"));
  Serial.print(DIAG_ID_LAST, HEX);
  Serial.print(F(" at "));
  Serial.println(g_busSpeedLabel);

  struct can_frame tx;
  tx.can_dlc = 8;
  const uint8_t payload[8] = {0x02, 0x10, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00};
  memcpy(tx.data, payload, sizeof(payload));

  uint16_t found = 0;
  uint16_t skipped = 0;
  for (uint16_t id = DIAG_ID_FIRST; id <= DIAG_ID_LAST; id++) {
    // INTERLOCK 2: if a real module is already broadcasting on this ID, do not
    // transmit on it. This is what prevents us injecting bytes that another
    // module would read as operational signal data.
    if (g_seen.contains(id)) {
      skipped++;
      continue;
    }

    tx.can_id = id;
    mcp2515.sendMessage(&tx); // one-shot; a missing ACK is expected, not fatal

    struct can_frame rx;
    unsigned long deadline = millis() + 40;
    while (millis() < deadline) {
      if (mcp2515.readMessage(&rx) != MCP2515::ERROR_OK) {
        continue;
      }
      if (rx.can_id == id || !looksLikeDiagReply(rx.can_dlc, rx.data)) {
        continue; // our own frame, or unrelated vehicle traffic
      }
      found++;
      Serial.print(F("    ECU: request 0x"));
      Serial.print(id, HEX);
      Serial.print(F(" -> response 0x"));
      Serial.print(rx.can_id, HEX);
      Serial.print(F("  ["));
      for (uint8_t i = 0; i < rx.can_dlc; i++) {
        printHex2(rx.data[i]);
        if (i + 1 < rx.can_dlc) {
          Serial.print(' ');
        }
      }
      Serial.print(F("]  "));
      Serial.println(rx.data[1] == 0x7F ? F("(refused - but the ECU is there)")
                                        : F("(session opened)"));
    }
  }

  Serial.print(F("    "));
  Serial.print(found);
  Serial.print(F(" responder(s), "));
  Serial.print(skipped);
  Serial.println(F(" id(s) skipped as in-use"));
  Serial.println();

  if (found > 0) {
    Serial.println(F("    => These modules speak diagnostics over CAN."));
    Serial.println(F("       ddt4all can reach them via an ELM327 bridge."));
  } else {
    Serial.println(F("    => Bus is alive but nothing answered a diagnostic"));
    Serial.println(F("       request. These modules are most likely K-line,"));
    Serial.println(F("       which an MCP2515 can never reach."));
  }
}

// ----------------------------------------------------------------- main

void setup() {
  Serial.begin(115200);
  pinMode(PIN_CAN_CS, OUTPUT);
  digitalWrite(PIN_CAN_CS, HIGH);
  pinMode(PIN_CAN_INT, INPUT_PULLUP);
  SPI.begin();
}

void loop() {
  // Re-run the passive sweep continuously: native USB does not reset the MCU
  // when the port is opened, so a one-shot report at boot would be missed.
  Serial.println();
  Serial.println(F("=== MKR1000 + MKR CAN shield bring-up ==="));

  unsigned long idleUntil = millis() + 3000;
  Serial.println(F("Send 'q' to arm stages 1, 2 and 3"));
  while (millis() < idleUntil) {
    if (Serial.available() && commandForKey(Serial.read()) == Command::RunSweep) {
      while (Serial.available()) {
        Serial.read(); // drain the rest of the line
      }
      if (stageProbe() && stageLoopback()) {
        stageSniff();
      } else {
        Serial.println(F(" Stage 1 or 2 failed "));
      }
      Serial.println(F("    (send 's' to arm stage 4: active ECU discovery)"));
      break;
    }
  }
  Serial.println(F("=== end sweep ==="));

  unsigned long idleUntil2 = millis() + 3000;
  while (millis() < idleUntil2) {
    if (Serial.available() &&
        commandForKey(Serial.read()) == Command::RunDiscovery) {
      while (Serial.available()) {
        Serial.read(); // drain the rest of the line
      }
      Serial.println();
      stageDiscover();
      break;
    }
  }
}

// Matching (findAt): the body is the normalized line after "AT" (uppercase, no
// spaces). Among rows whose name is a prefix of the body AND whose shape
// parses the rest, the LONGEST name wins. Row order is irrelevant.
//   "CFC1"  -> CFC/Bit   (CF/Header rejects "C1")
//   "CSM1"  -> CSM/Bit   (CS/None rejects "M1")
//   "D"     -> D/None    (reset)       "D1" -> D/Bit (DLC display)
//   "DPN"   -> DPN/None                "SH7E0" -> SH/Header (S/Bit rejects)
//   ""      -> ""/None   ("AT" alone answers OK)
// No row accepts -> "?". A handler may also answer "?" for semantic errors
// (FC SM 1 without FC SH + FC SD).

#ifndef ELM_AT_TABLE_H
#define ELM_AT_TABLE_H

#include <stdint.h>

#include "guard.h"
#include "settings.h"

class Elm;

enum class AtShape : uint8_t {
  None,
  Bit,
  Digit012,
  Hex2,
  Dec4,
  Header,     // 3, 6 or 8 hex digits
  Bytes1to5,
  Bytes1to6,
  Pattern,    // 3 or 8 chars of hex or X (CRA)
  ProtoSel,   // h | Ah | 00, h in 0-9 (A, B, C rejected)
  TierWord,
};

// Parsed argument; which fields are meaningful depends on the shape.
struct AtArgs {
  uint32_t value;    // Bit, Digit012, Hex2, Dec4, Header/Pattern id bits, protocol
  uint32_t mask;     // Pattern: 1 where not X
  uint8_t bytes[6];  // Bytes*
  uint8_t count;     // number of bytes, or of hex digits (Header/Pattern)
  bool automatic;    // ProtoSel "A" prefix or 0
  Tier tier;         // TierWord
};

enum class AtReply : uint8_t {
  Ok,       // Elm prints "OK"
  Bad,      // Elm prints "?"
  Printed,  // handler printed its own lines
};

using AtHandler = AtReply (*)(Elm &elm, const AtArgs &args);

struct AtCommand {
  const char *name;
  AtShape shape;
  AtHandler run;          // nullptr for flag rows
  bool Settings::*flag;   // flag rows: Bit sets it from the argument,
  bool flagValue;         // None sets it to flagValue
};

const AtCommand *findAt(const char *body, AtArgs *args);

// Absent on purpose (-> "?"): BRD BRT (USB CDC has no baud), MR MT, PP PPS,
// @2 @3, LP, IGN, RA SR, MP DM1 JE JS (J1939), RTR, BD, and STI (not AT).
struct AtTable {
  static const AtCommand rows[];
  static const uint8_t rowCount;
  // K-line init needs a K-line transceiver this board does not have. pyren
  // and ddt4all skip a K-line ECU when the reply contains "ERROR".
  static AtReply busInitError(Elm &e, const AtArgs &);
};

#endif  // ELM_AT_TABLE_H

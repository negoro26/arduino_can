#ifndef ELM_BUS_H
#define ELM_BUS_H

#include "can_types.h"
#include "guard.h"
#include "hal.h"
#include "settings.h"
#include "text.h"

enum class Wake : uint8_t { Frame, Interrupted, Timeout };

// After an interrupting byte, the rest of that line is discarded if its CR
// arrives within this window. ddt4all stops ATMA by writing "AT\r": without
// this, the leftover "T\r" would answer "?" and that "?" would be read as the
// reply to ddt4all's next command.
constexpr uint16_t kInterruptLineRestMs = 100;

// The ONLY blocking primitive. Order per iteration:
//   1. a serial byte other than LF -> Interrupted (byte consumed and
//      or kLinkClosed                 discarded, plus the rest of its line)
//   2. a CAN frame                 -> Frame (polled before the clock, so a
//                                     frame at the deadline is not lost)
//   3. deadline passed             -> Timeout
//   4. bus.idleUntil(d)            -> the only place test time advances
Wake await(BusSide &bus, Deadline d, CanFrame *out);

// Between transactions the controller sits in ListenOnly: it never ACKs or
// error-flags at the prompt, so a silent-at-connect wrong bitrate cannot start
// disturbing the bus when traffic appears later.
struct LinkState {
  enum class Talk : uint8_t { Unchecked, Allowed, Mismatch };
  bool open = false;
  Bitrate rate = Bitrate::k500;
  Acceptance acc = Acceptance::all();
  Talk talk = Talk::Unchecked;

  bool configure(BusSide &bus, Bitrate r, CanMode m, const Acceptance &a);
  void close(BusSide &bus);
};

enum class LbtVerdict : uint8_t { Verified, Silent, Mismatch, Fault };

// The full kLbtWindowMs in ListenOnly + Acceptance::all() from a freshly reset
// controller. Every frame's id goes to guard.heard(). Frames decoded ->
// Verified; errors (errorEvents or rec) without frames -> Mismatch; nothing ->
// Silent; controller refused -> Fault.
LbtVerdict listenBeforeTalk(BusSide &bus, Guard &guard, LinkState &link,
                            Bitrate rate, bool *interrupted);
constexpr uint16_t kLbtWindowMs = 60;

struct RxLog {
  // One maximal ISO-TP message (4095 bytes) is 586 frames.
  static constexpr uint16_t kCapacity = 600;
  CanFrame frames[kCapacity];
  uint16_t count = 0;
  void clear() { count = 0; }
  bool push(const CanFrame &f);
};

enum class Outcome : uint8_t {
  Answered,
  NoData,
  Sent,
  Blocked,
  CanError,
  Stopped,
  BufferFull,
  UnableToConnect,
};

// Send one request on protocol p and collect the answer. Blocks until done.
//
//   f = requestFrame(); guard.admit(f) refused        -> Blocked
//   rate unverified: listenBeforeTalk()
//       interrupted -> Stopped; Mismatch/Fault        -> CanError (nothing sent)
//       guard.admit(f) again, it may have heard f.id  -> Blocked
//   drain stale frames; Normal (one-shot) with receiveFilter()
//   canSend fails -> verdict dropped                  -> CanError
//   R0 / count digit 0                                -> Sent
//   listen (deadlines below); FC on every first frame (CFC1), via the guard
//   back to ListenOnly -> Answered / NoData / Stopped / BufferFull
//
// Deadlines: ST before the first frame; after a frame the adaptive window,
// except that while consecutive frames are owed after OUR flow control the
// full ST applies (deliberate deviation from AT1/AT2: an ECU honouring STmin
// can be slower than 25-50 ms between frames); 5 s after each 7F xx 78, which
// does not count toward the response digit. Reaching the digit stops at once.
Outcome converse(BusSide &bus, Guard &guard, LinkState &link,
                 const Settings &s, Protocol p, const HexRequest &req,
                 RxLog &log);

// What AT SP 0 tries, in this order.
constexpr Protocol kSearchOrder[] = {Protocol::Can11_500, Protocol::Can29_500,
                                     Protocol::Can11_250, Protocol::Can29_250};

// Whether the guard permits req's frame under at least one searched protocol.
// The id differs per protocol: ATSH 7E0 is 29-bit 180007E0 on 7 and 9.
bool searchable(const Guard &guard, const Settings &s, const HexRequest &req);

// AT SP 0 search on the first request with no protocol found yet. Elm prints
// "SEARCHING..." before calling. converse() per protocol in kSearchOrder, so
// the guard and listen-before-talk apply to every attempt and a mismatched
// bitrate is skipped without transmitting. A protocol whose frame the guard
// refuses is skipped too. The first answer wins and sets s.protocol.current.
// Stopped ends the search at once. Every protocol refused -> Blocked; nothing
// found otherwise -> UnableToConnect.
Outcome search(BusSide &bus, Guard &guard, LinkState &link, Settings &s,
               const HexRequest &req, RxLog &log);

struct RateReport {
  bool setupFailed = false;
  uint32_t frames = 0;
  uint32_t errors = 0;
  SeenIds<16> ids;
};

// false if interrupted. Every frame's id goes to guard.heard().
bool surveyRate(BusSide &bus, Guard &guard, Bitrate rate, uint16_t windowMs,
                RateReport *out);

enum class Wiring : uint8_t { BitrateFound, LiveBusUndecoded, NotConnected };
Wiring wiringVerdict(uint32_t frames, uint32_t errors);

enum class Loopback : uint8_t { Pass, Fail, SetupFailed, Interrupted };
Loopback loopbackRoundTrip(BusSide &bus, Guard &guard, Bitrate rate);

#endif  // ELM_BUS_H

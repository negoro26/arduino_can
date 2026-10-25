// Every blocking conversation with the CAN controller.
//
// All functions take BusSide&, never Hal&: they CANNOT write serial output, so
// "no USB write while a burst may be arriving" holds by construction. They
// collect what they observe (RxLog, reports) and return; Elm prints afterwards.
//
// Every wait is a loop around await(), the single blocking primitive.

#ifndef ELM_BUS_H
#define ELM_BUS_H

#include "can_types.h"
#include "guard.h"
#include "hal.h"
#include "settings.h"
#include "text.h"

// ------------------------------------------------------------------ await

enum class Wake : uint8_t { Frame, Interrupted, Timeout };

// After an interrupting byte, the rest of that line is discarded if its CR
// arrives within this window. ddt4all stops ATMA by writing "AT\r": without
// this, the leftover "T\r" would answer "?" and that "?" would be read as the
// reply to ddt4all's next command.
constexpr uint16_t kInterruptLineRestMs = 100;

// The ONLY blocking primitive. Order per iteration:
//   1. a serial byte other than LF -> Interrupted (byte consumed and
//                                     discarded, plus the rest of its line)
//   2. a CAN frame                 -> Frame (polled before the clock, so a
//                                     frame at the deadline is not lost)
//   3. deadline passed             -> Timeout
//   4. bus.idleUntil(d)            -> the only place test time advances
Wake await(BusSide &bus, Deadline d, CanFrame *out);

// ------------------------------------------------------------- link state

// What the controller is configured as. Between transactions the controller
// sits in ListenOnly: it never ACKs or error-flags at the prompt, so a
// silent-at-connect wrong bitrate cannot start disturbing the bus when traffic
// appears later.
struct LinkState {
  enum class Talk : uint8_t { Unchecked, Allowed, Mismatch };
  bool open = false;
  Bitrate rate = Bitrate::k500;
  Acceptance acc = Acceptance::all();
  Talk talk = Talk::Unchecked;  // listen-before-talk verdict for `rate`

  bool configure(BusSide &bus, Bitrate r, CanMode m, const Acceptance &a);
  void close(BusSide &bus);  // canClose(); open = false; talk = Unchecked
};

enum class LbtVerdict : uint8_t { Verified, Silent, Mismatch, Fault };

// ~60 ms ListenOnly + Acceptance::all() from a freshly reset controller.
// Frames decoded -> Verified; errors (errorEvents or rec) without frames ->
// Mismatch; nothing -> Silent; controller refused -> Fault.
LbtVerdict listenBeforeTalk(BusSide &bus, LinkState &link, Bitrate rate,
                            bool *interrupted);
constexpr uint16_t kLbtWindowMs = 60;

// --------------------------------------------------------- one transaction

// Accepted frames of one request, printed by Elm after the window closes.
struct RxLog {
  // One maximal ISO-TP message (4095 bytes) is 586 frames.
  static constexpr uint16_t kCapacity = 600;
  CanFrame frames[kCapacity];
  uint16_t count = 0;
  void clear() { count = 0; }
  bool push(const CanFrame &f);  // false when full
};

enum class Outcome : uint8_t {
  Answered,         // >= 1 frame in the log
  NoData,           // "NO DATA"
  Sent,             // R0 or count digit 0: transmitted, did not listen
  Blocked,          // "BLOCKED"  -- guard refused; nothing touched the bus
  CanError,         // "CAN ERROR" -- LBT mismatch, TX failure, controller fault
  Stopped,          // "STOPPED"  -- after any logged frames
  BufferFull,       // "BUFFER FULL" -- after the logged frames
  UnableToConnect,  // "UNABLE TO CONNECT" -- search found nothing
};

// Send one request on protocol p and collect the answer. Blocks until done.
//
//   f = requestFrame(); guard.admit(f) refused        -> Blocked
//   rate unverified: listenBeforeTalk()
//       interrupted -> Stopped; Mismatch/Fault        -> CanError (nothing sent)
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

// AT SP 0 search on the first request with no protocol found yet. Elm prints
// "SEARCHING..." before calling. converse() per protocol 6, 7, 8, 9, so the
// guard and listen-before-talk apply to every attempt and a mismatched bitrate
// is skipped without transmitting. The first answer wins and sets
// s.protocol.current. Blocked / Stopped end the search at once. Nothing found
// -> UnableToConnect.
Outcome search(BusSide &bus, Guard &guard, LinkState &link, Settings &s,
               const HexRequest &req, RxLog &log);

// --------------------------------------------------- AT#SURVEY / SELFTEST

// Fixed-capacity id set (from the probe). Overflow keeps the earliest ids and
// is detectable.
template <uint8_t Capacity>
class SeenIds {
 public:
  bool contains(CanId id) const {
    for (uint8_t i = 0; i < count_; ++i) {
      if (ids_[i] == id) return true;
    }
    return false;
  }
  bool add(CanId id) {  // false on duplicate or when full
    if (contains(id) || count_ >= Capacity) return false;
    ids_[count_++] = id;
    return true;
  }
  uint8_t count() const { return count_; }
  bool isFull() const { return count_ >= Capacity; }
  CanId at(uint8_t i) const { return ids_[i]; }

 private:
  CanId ids_[Capacity];
  uint8_t count_ = 0;
};

struct RateReport {
  bool setupFailed = false;  // canOpen refused the rate; nothing was heard
  uint32_t frames = 0;
  uint32_t errors = 0;  // errorEvents + rec, as the probe's stage 3 counted
  SeenIds<16> ids;
};

// ListenOnly + Acceptance::all() for windowMs. false if interrupted.
bool surveyRate(BusSide &bus, Bitrate rate, uint16_t windowMs,
                RateReport *out);

// The probe's stage-3 wiring verdict: frames -> the rate is found; errors
// without frames -> the pins are on a live bus but nothing decodes (swap H/L
// or another rate); nothing at all -> not connected.
enum class Wiring : uint8_t { BitrateFound, LiveBusUndecoded, NotConnected };
Wiring wiringVerdict(uint32_t frames, uint32_t errors);

// The probe's stage 2: Loopback mode, send 745 02 10 C0 00.. (admitted by the
// guard like any frame), expect it back bit-exact within 50 ms.
enum class Loopback : uint8_t { Pass, Fail, SetupFailed, Interrupted };
Loopback loopbackRoundTrip(BusSide &bus, Guard &guard, Bitrate rate);

#endif  // ELM_BUS_H

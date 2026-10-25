#include "renault_scenario.h"

namespace {

#define EXCHANGE(req, rsp, pendingMs) \
  { req, sizeof req, rsp, sizeof rsp, pendingMs }
#define SILENT(req) \
  { req, sizeof req, nullptr, 0, 0 }
#define COUNT(a) static_cast<uint8_t>(sizeof a / sizeof a[0])
#define VIN 'U', 'U', '1', '5', 'S', 'D', 'A', 'G', '5', '5', '1', '2', '3', '4', '5', '6', '7'

const uint8_t kSession01[] = {0x10, 0x01};
const uint8_t kSession01Ok[] = {0x50, 0x01, 0x00, 0x32, 0x01, 0xF4};
const uint8_t kSession03[] = {0x10, 0x03};
const uint8_t kSession03Ok[] = {0x50, 0x03, 0x00, 0x32, 0x01, 0xF4};
const uint8_t kSession03Refused[] = {0x7F, 0x10, 0x12};
const uint8_t kSessionC0[] = {0x10, 0xC0};
const uint8_t kSessionC0Ok[] = {0x50, 0xC0};
const uint8_t kSession81[] = {0x10, 0x81};
const uint8_t kSession81Ok[] = {0x50, 0x81};
const uint8_t kTester00[] = {0x3E, 0x00};
const uint8_t kTester00Ok[] = {0x7E, 0x00};
const uint8_t kTester01[] = {0x3E, 0x01};
const uint8_t kTester01Ok[] = {0x7E, 0x01};
const uint8_t kReadDtcUds[] = {0x19, 0x02, 0xAF};
const uint8_t kClearUds[] = {0x14, 0xFF, 0xFF, 0xFF};
const uint8_t kClearUdsOk[] = {0x54};
const uint8_t kReadDtcKwp[] = {0x17, 0xFF, 0x00};
const uint8_t kClearKwp[] = {0x14, 0xFF, 0x00};
const uint8_t kClearKwpOk[] = {0x54, 0xFF, 0x00};
const uint8_t kIdent2180[] = {0x21, 0x80};

const uint8_t kDidPart[] = {0x22, 0xF1, 0x87};
const uint8_t kDidPartOk[] = {0x62, 0xF1, 0x87, '2', '3', '7', '1', '0',
                              '7', '6', '3', '1', 'R'};
const uint8_t kDidSupplier[] = {0x22, 0xF1, 0x8A};
const uint8_t kEcmSupplierOk[] = {0x62, 0xF1, 0x8A, 'C', 'O', 'N', 'T', 'I',
                                  'N', 'E', 'N', 'T', 'A', 'L'};
const uint8_t kDidSoft[] = {0x22, 0xF1, 0x94};
const uint8_t kDidSoftOk[] = {0x62, 0xF1, 0x94, 'S', '1', '8',
                              '0', '0', '6', '7', 'A'};
const uint8_t kDidVersion[] = {0x22, 0xF1, 0x95};
const uint8_t kDidVersionOk[] = {0x62, 0xF1, 0x95, '0', 'A', '0', '4'};
const uint8_t kDidDiag[] = {0x22, 0xF1, 0xA0};
const uint8_t kDidDiagOk[] = {0x62, 0xF1, 0xA0, 0x44};
const uint8_t kDidVin[] = {0x22, 0xF1, 0x90};
const uint8_t kDidVinOk[] = {0x62, 0xF1, 0x90, VIN};
const uint8_t kEcmDtcOk[] = {0x59, 0x02, 0xFF, 0x03, 0x35, 0x1C,
                             0x2F, 0x01, 0x15, 0x64, 0x08};
const uint8_t kPid00[] = {0x01, 0x00};
const uint8_t kPid00Ok[] = {0x41, 0x00, 0xBE, 0x1F, 0xA8, 0x12};
const uint8_t kPid01[] = {0x01, 0x01};
const uint8_t kPid01Ok[] = {0x41, 0x01, 0x82, 0x07, 0x65, 0x00};
const uint8_t kPid05[] = {0x01, 0x05};
const uint8_t kPid05Ok[] = {0x41, 0x05, 0x5A};
const uint8_t kPid0C[] = {0x01, 0x0C};
const uint8_t kPid0COk[] = {0x41, 0x0C, 0x0B, 0xB8};
const uint8_t kPid0D[] = {0x01, 0x0D};
const uint8_t kPid0DOk[] = {0x41, 0x0D, 0x00};
const uint8_t kMode03[] = {0x03};
const uint8_t kMode03Ok[] = {0x43, 0x02, 0x03, 0x35, 0x01, 0x15};
const uint8_t kMode07[] = {0x07};
const uint8_t kMode07Ok[] = {0x47, 0x00};
const uint8_t kMode0A[] = {0x0A};
const uint8_t kMode0AOk[] = {0x4A, 0x00};
const uint8_t kMode0900[] = {0x09, 0x00};
const uint8_t kMode0900Ok[] = {0x49, 0x00, 0x40, 0x00, 0x00, 0x00};
const uint8_t kMode04[] = {0x04};
const uint8_t kMode04Ok[] = {0x44};
const uint8_t kMode0902[] = {0x09, 0x02};
const uint8_t kMode0902Ok[] = {0x49, 0x02, 0x01, VIN};

const Exchange kEcm[] = {
    EXCHANGE(kSession01, kSession01Ok, 0),
    EXCHANGE(kSession03, kSession03Ok, 0),
    EXCHANGE(kTester00, kTester00Ok, 0),
    EXCHANGE(kDidPart, kDidPartOk, 0),
    EXCHANGE(kDidSupplier, kEcmSupplierOk, 0),
    EXCHANGE(kDidSoft, kDidSoftOk, 0),
    EXCHANGE(kDidVersion, kDidVersionOk, 0),
    EXCHANGE(kDidDiag, kDidDiagOk, 0),
    EXCHANGE(kDidVin, kDidVinOk, 0),
    EXCHANGE(kReadDtcUds, kEcmDtcOk, 0),
    EXCHANGE(kClearUds, kClearUdsOk, 0),
    EXCHANGE(kPid00, kPid00Ok, 0),
    EXCHANGE(kPid01, kPid01Ok, 0),
    EXCHANGE(kPid05, kPid05Ok, 0),
    EXCHANGE(kPid0C, kPid0COk, 0),
    EXCHANGE(kPid0D, kPid0DOk, 0),
    EXCHANGE(kMode03, kMode03Ok, 0),
    EXCHANGE(kMode0900, kMode0900Ok, 0),
    EXCHANGE(kMode07, kMode07Ok, 0),
    EXCHANGE(kMode0A, kMode0AOk, 0),
    EXCHANGE(kMode04, kMode04Ok, 0),
    EXCHANGE(kMode0902, kMode0902Ok, 0),
};

const uint8_t kUchIdentOk[] = {
    0x61, 0x80, 0x82, 0x00, 0x44, 0x66, 0x27, 0x44, 0x32, 0x31, 0x33, 0x82, 0x00,
    0x38, 0x71, 0x38, 0x00, 0xA7, 0x75, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
const uint8_t kUchDtcOk[] = {0x57, 0x01, 0x93, 0x23, 0xE0};

const Exchange kUch[] = {
    EXCHANGE(kSession03, kSession03Refused, 0),
    EXCHANGE(kSessionC0, kSessionC0Ok, 0),
    EXCHANGE(kSession81, kSession81Ok, 0),
    EXCHANGE(kTester01, kTester01Ok, 0),
    EXCHANGE(kIdent2180, kUchIdentOk, 0),
    EXCHANGE(kReadDtcKwp, kUchDtcOk, 0),
    EXCHANGE(kClearKwp, kClearKwpOk, 0),
};

const uint8_t kAbsIdentOk[] = {0x61, 0x80, 0x01, 0x02, 0x03, 0x04, 0x05,
                               0x44, 0x42, 0x4F, 0x53, 0x01, 0x02, 0x03,
                               0x04, 0x05, 0x00, 0x12, 0x34, 0x56};
const uint8_t kAbsDtcOk[] = {0x57, 0x00};

const Exchange kAbs[] = {
    SILENT(kSession03),
    EXCHANGE(kSessionC0, kSessionC0Ok, 0),
    EXCHANGE(kSession81, kSession81Ok, 0),
    EXCHANGE(kIdent2180, kAbsIdentOk, 0),
    EXCHANGE(kReadDtcKwp, kAbsDtcOk, 0),
    EXCHANGE(kClearKwp, kClearKwpOk, 0),
};

const uint8_t kAirbagSupplierOk[] = {0x62, 0xF1, 0x8A, 'T', 'E', 'M', 'I', 'C'};
const uint8_t kAirbagDtcOk[] = {0x59, 0x02, 0xFF, 0x9A, 0x01, 0x11, 0x08};
constexpr uint16_t kAirbagPendingMs = 300;

const Exchange kAirbag[] = {
    EXCHANGE(kSession01, kSession01Ok, kAirbagPendingMs),
    EXCHANGE(kSession03, kSession03Ok, kAirbagPendingMs),
    EXCHANGE(kDidSupplier, kAirbagSupplierOk, kAirbagPendingMs),
    EXCHANGE(kReadDtcUds, kAirbagDtcOk, kAirbagPendingMs),
    EXCHANGE(kClearUds, kClearUdsOk, kAirbagPendingMs),
};

const EcuSpec kEcus[] = {
    {"ECM", CanId::std11(0x7E0), CanId::std11(0x7E8), true, kEcm, COUNT(kEcm),
     2000},
    {"UCH", CanId::std11(0x745), CanId::std11(0x765), false, kUch,
     COUNT(kUch), 5000},
    {"ABS", CanId::std11(0x740), CanId::std11(0x760), false, kAbs,
     COUNT(kAbs), 5000},
    {"Airbag", CanId::std11(0x752), CanId::std11(0x772), false, kAirbag,
     COUNT(kAirbag), 3000},
};

const Periodic kTraffic[] = {
    {{CanId::std11(0x181), 8, {0x0B, 0xB8, 0x00, 0x00, 0x1E, 0x00, 0x00, 0x00}},
     10},
    {{CanId::std11(0x354), 8, {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}},
     20},
    {{CanId::std11(0x622), 8, {0x00, 0x81, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}},
     50},
    {{CanId::std11(0x5DE), 8, {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}},
     100},
};

}  // namespace

CarSpec renaultDaciaCar(Bitrate rate) {
  return {kEcus, COUNT(kEcus), kTraffic, COUNT(kTraffic), rate};
}

CarSpec renaultDaciaCarQuiet(Bitrate rate) {
  return {kEcus, COUNT(kEcus), nullptr, 0, rate};
}

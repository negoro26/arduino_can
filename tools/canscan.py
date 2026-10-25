#!/usr/bin/env python3
# /// script
# requires-python = ">=3.9"
# dependencies = ["pyserial>=3.5"]
# ///
"""Database-free Renault/Dacia + generic OBD-II scanner for ELM327-compatible adapters.

Reads identification and DTCs from every Renault/Dacia diagnostic address and from
generic OBD-II over any ELM327-compatible serial adapter (the arduino_can firmware
or a real ELM327), without the proprietary DDT2000/CLIP databases.

    uv run tools/canscan.py /dev/ttyACM0
    uv run --with obd tools/canscan.py /dev/ttyACM0    # adds generic DTC descriptions
"""
from __future__ import annotations

import argparse
import dataclasses
import json
import re
import string
import sys
import time
from collections import Counter
from dataclasses import dataclass, field
from typing import Callable, Optional

import serial

try:
    # python-OBD is GPL-2.0-or-later: imported at runtime when installed, never copied.
    from obd.codes import DTC as OBD_DESCRIPTIONS
except ImportError:
    OBD_DESCRIPTIONS = {}

# Source: ddt4all src/ddt4all/resources/projects.json (projects.All dnat/snat/addressing),
# commit e7b582257e156f1cf2956831874c26305403c9b9. (request id, response id, module names)
RENAULT_ECUS = [
    (0x605, 0x585, "Nav4 / Tel"),
    (0x607, 0x587, "CCGW"),
    (0x60B, 0x58B, "Emergency Service"),
    (0x622, 0x484, "Steering Column"),
    (0x62F, 0x5AF, "SWSP"),
    (0x637, 0x5B7, "Touchpad, Telephone Key pad"),
    (0x638, 0x5B8, "HeadUnit"),
    (0x63A, 0x5BA, "Central Control Unit"),
    (0x63B, 0x5BB, "Central Display Module"),
    (0x641, 0x5C1, "Cluster"),
    (0x644, 0x5C4, "Rear View Cam"),
    (0x646, 0x5C6, "HERMES"),
    (0x650, 0x5D0, "LEFT_HEAD_LAMP / HLM_LT"),
    (0x656, 0x5D6, "RIGHT_HEAD_LAMP / HLM_RT"),
    (0x6BC, 0x69C, "Accessory Switch Module"),
    (0x704, 0x724, "ESUS-VADA"),
    (0x705, 0x725, "DDCM"),
    (0x706, 0x726, "PDCM"),
    (0x707, 0x727, "LKS"),
    (0x708, 0x728, "DIU-MID"),
    (0x709, 0x729, "eRDCR / ASD-R"),
    (0x70A, 0x72A, "PPS"),
    (0x70B, 0x72B, "DDR"),
    (0x70C, 0x700, "IDM-CDM"),
    (0x70E, 0x70F, "EBA / E-ACT-EBA_29b"),
    (0x710, 0x730, "GW"),
    (0x711, 0x701, "Nav2 / SOUND (AMP) / ASBM"),
    (0x711, 0x731, "Additional Heater"),
    (0x712, 0x732, "Audio"),
    (0x713, 0x733, "PBD/PTL"),
    (0x714, 0x704, "Tuner Box"),
    (0x714, 0x734, "MIU"),
    (0x716, 0x736, "MemoSeatPassenger - LDCM"),
    (0x717, 0x737, "Magic Folding-RDCM"),
    (0x718, 0x738, "Tachometer"),
    (0x719, 0x739, "VCCU"),
    (0x71A, 0x73A, "Magic Bumper"),
    (0x723, 0x735, "FRRAD"),
    (0x73A, 0x4A7, "HMI Gateway"),
    (0x73D, 0x73E, "SOW Rear Left"),
    (0x73F, 0x761, "IPA / VSP"),
    (0x740, 0x760, "ABS-VDC / ABS-VDC_29b"),
    (0x742, 0x762, "EPS"),
    (0x742, 0x771, "ODS"),
    (0x743, 0x763, "TDB"),
    (0x744, 0x764, "HVAC"),
    (0x745, 0x765, "UCH"),
    (0x746, 0x726, "SCCM / SCCM_29"),
    (0x746, 0x783, "DCM Nissan"),
    (0x747, 0x767, "Navigation-UCC-ITM"),
    (0x748, 0x768, "4WD"),
    (0x749, 0x74A, "ANC"),
    (0x74B, 0x76B, "TCU"),
    (0x74C, 0x76C, "Renault RemoteEntry"),
    (0x74D, 0x76D, "UPC-EMM / USM_29"),
    (0x74E, 0x76E, "Parking Sonar"),
    (0x74F, 0x757, "Intelligent Bat Sensor"),
    (0x74F, 0x76F, "ADP"),
    (0x750, 0x770, "HFCK"),
    (0x751, 0x771, "HLS"),
    (0x752, 0x772, "Airbag-SRS"),
    (0x753, 0x773, "4WS"),
    (0x754, 0x774, "PSB DR"),
    (0x755, 0x775, "APB"),
    (0x756, 0x776, "Superviseur-DCDC / DC-DC Conv / Superviseur-DCDC_29b"),
    (0x758, 0x778, "TPMS"),
    (0x759, 0x779, "eRDCL - ASDL / ASD-L"),
    (0x75A, 0x77E, "PEB"),
    (0x75C, 0x77C, "AFS"),
    (0x75D, 0x77D, "ACC"),
    (0x780, 0x7A0, "C-Box"),
    (0x781, 0x7C2, "ST2"),
    (0x782, 0x7A2, "AAU"),
    (0x785, 0x7A5, "INSULATOR"),
    (0x786, 0x7A6, "DMS"),
    (0x788, 0x7A8, "SR"),
    (0x78A, 0x7D7, "CSHV Airbag Vehicle stopped"),
    (0x78B, 0x7AB, "AFBCU"),
    (0x790, 0x791, "EMCU"),
    (0x792, 0x793, "BCB / BCB-OBC_29b"),
    (0x795, 0x7B5, "BSW"),
    (0x796, 0x7B6, "LBC2 / LBC2_29b"),
    (0x797, 0x79A, "VCM"),
    (0x798, 0x794, "ST1"),
    (0x799, 0x7B9, "HFM"),
    (0x79A, 0x4B3, "Transfert Case"),
    (0x79B, 0x7BB, "LBC (EV)"),
    (0x79C, 0x79D, "Tacograph"),
    (0x79C, 0x7BC, "UBP"),
    (0x79D, 0x7BD, "SCU"),
    (0x79F, 0x7A7, "Steering Wheel Switch"),
    (0x7A1, 0x7A9, "EPS (HEV)"),
    (0x7A4, 0x7AC, "DLOCK"),
    (0x7AA, 0x7AD, "WCGS"),
    (0x7B7, 0x7BA, "AVM"),
    (0x7C3, 0x7C9, "ADAS-Sub"),
    (0x7C5, 0x7C4, "e-shifter"),
    (0x7C6, 0x7C7, "TransAxle Control Unit"),
    (0x7C6, 0x7D1, "WSU"),
    (0x7C7, 0x7D3, "SIB"),
    (0x7CA, 0x7DA, "DCM Renault"),
    (0x7CD, 0x7CF, "Engineering"),
    (0x7D4, 0x7D5, "SOW Rear Right"),
    (0x7D6, 0x7D2, "CAN Adapter - BAC / AAM"),
    (0x7D8, 0x7C8, "PLC-PLGW"),
    (0x7D9, 0x7DB, "PTC AMP"),
    (0x7DC, 0x7DD, "SMB / FCAM"),
    (0x7E0, 0x7E8, "ECM_29 / ECM"),
    (0x7E1, 0x7E9, "AT_29b / AT"),
    (0x7E2, 0x7EA, "LNG"),
    (0x7E3, 0x7EB, "MC HEV / Fuel System CMG4"),
    (0x7E4, 0x7EC, "Selective Catalytic Reduction / (H) EVC / (H) EVC-HCM-VCM_29b"),
    (0x7E5, 0x7ED, "Common Powertrain / LBC (HEV) / LBC (HEV)_29b"),
    (0x7E6, 0x7EE, "UDM / UDM_29b"),
]

BAUD = 38400
AT_TIMEOUT = 2.0
REQUEST_TIMEOUT = 15.0

READ_SERVICES = {0x10, 0x19, 0x22, 0x21, 0x17, 0x01, 0x03, 0x07, 0x09, 0x0A}
CLEAR_SERVICES = {0x14, 0x04}

# With FC SM 0 the adapter answers a first frame from 7E8-7EF with flow control on 7E0-7E7.
OBD_TX_IDS = {0x7DF, *range(0x7E0, 0x7E8)}
OBD_DTC_MODES = (("03", "stored"), ("07", "pending"), ("0A", "permanent"))

UDS_IDENT = (
    ("F187", "part number"),
    ("F18A", "supplier"),
    ("F194", "software number"),
    ("F195", "software version"),
    ("F1A0", "diagnostic version"),
)
UDS_STATUS_WORDS = ((0, "active"), (3, "stored"), (2, "pending"))

NRC_NAMES = {
    0x11: "serviceNotSupported",
    0x12: "subFunctionNotSupported",
    0x13: "incorrectLength",
    0x22: "conditionsNotCorrect",
    0x31: "requestOutOfRange",
    0x33: "securityAccessDenied",
    0x78: "responsePending",
    0x7E: "subFunctionNotSupportedInActiveSession",
    0x7F: "serviceNotSupportedInActiveSession",
}


def _is_hex(text: str) -> bool:
    return bool(text) and all(c in string.hexdigits for c in text)


def fmt_id(can_id: int) -> str:
    return f"{can_id:03X}" if can_id <= 0x7FF else f"{can_id:08X}"


def parse_frame(line: str) -> Optional[tuple[int, bytes]]:
    """`7E8 06 41 00 ...` or `18 DA F1 10 ...` (spaces optional) -> (id, data); None for status text."""
    hexits = line.replace(" ", "")
    if not _is_hex(hexits):
        return None
    id_len = 3 if len(hexits) % 2 else 8
    data = hexits[id_len:]
    if not 2 <= len(data) <= 16:
        return None
    return int(hexits[:id_len], 16), bytes.fromhex(data)


def parse_frames(lines: list[str]) -> dict[int, list[bytes]]:
    """Raw CAN frames (PCI byte included) per responder id, in arrival order."""
    frames: dict[int, list[bytes]] = {}
    for line in lines:
        frame = parse_frame(line)
        if frame is not None:
            frames.setdefault(frame[0], []).append(frame[1])
    return frames


def status_lines(lines: list[str]) -> list[str]:
    """The adapter's own messages: NO DATA, CAN ERROR, BLOCKED, STOPPED, ?, ..."""
    return [line for line in lines if parse_frame(line) is None]


def is_pending(message: bytes) -> bool:
    return len(message) >= 3 and message[0] == 0x7F and message[2] == 0x78


def reassemble(frames: list[bytes]) -> list[bytes]:
    """Complete ISO-TP messages from one responder's frames, without flow control or response-pending."""
    messages: list[bytes] = []
    partial = bytearray()
    expected = 0
    seq = 0
    for frame in frames:
        kind, low = frame[0] >> 4, frame[0] & 0x0F
        if kind == 0 and 0 < low < len(frame):
            messages.append(bytes(frame[1:1 + low]))
            expected = 0
        elif kind == 1 and len(frame) >= 2:
            expected = (low << 8) | frame[1]
            partial = bytearray(frame[2:])
            seq = 1
        elif kind == 2 and expected and low == seq:
            partial += frame[1:]
            seq = (seq + 1) % 16
            if len(partial) >= expected:
                messages.append(bytes(partial[:expected]))
                expected = 0
        elif kind == 2:
            expected = 0
    return [m for m in messages if not is_pending(m)]


@dataclass
class Reply:
    """One request's outcome: final messages per responder id (a key even if only partial or
    pending frames arrived) and the adapter's status lines."""

    messages: dict[int, list[bytes]]
    statuses: list[str]

    @classmethod
    def parse(cls, lines: list[str]) -> Reply:
        frames = parse_frames(lines)
        return cls({rid: reassemble(f) for rid, f in frames.items()}, status_lines(lines))

    def answers(self, sid: int) -> dict[int, bytes]:
        """Each responder's first positive or negative answer to service `sid`."""
        found = {}
        for rid, messages in self.messages.items():
            for message in messages:
                if message[0] == sid + 0x40 or message[:2] == bytes((0x7F, sid)):
                    found[rid] = message
                    break
        return found

    def answer(self, rx: int, sid: int) -> Optional[bytes]:
        return self.answers(sid).get(rx)


def survey_ids(lines: list[str]) -> list[int]:
    """CAN id of every frame line printed by ATMA (an `FC:` prefix or trailing error text is allowed)."""
    ids = []
    for line in lines:
        tokens = line.replace("FC:", " ").split()
        if tokens and len(tokens[0]) == 3 and _is_hex(tokens[0]):
            ids.append(int(tokens[0], 16))
        elif len(tokens) >= 4 and all(len(t) == 2 and _is_hex(t) for t in tokens[:4]):
            ids.append(int("".join(tokens[:4]), 16))
    return ids


@dataclass
class Dtc:
    code: str
    raw: str
    sae: str
    status: Optional[int] = None
    summary: str = ""
    description: str = ""


def sae_code(b0: int, b1: int) -> str:
    """SAE J2012 reading of a two-byte DTC: 03 35 -> P0335."""
    return f"{'PCBU'[b0 >> 6]}{(b0 >> 4) & 3}{b0 & 0x0F:X}{b1:02X}"


def uds_status_summary(status: int) -> str:
    return ", ".join(word for bit, word in UDS_STATUS_WORDS if status >> bit & 1)


def decode_uds_dtcs(payload: bytes) -> list[Dtc]:
    """`59 02 <availability mask>` + [DTC hi, DTC mid, FTB, status] records."""
    if payload[:2] != b"\x59\x02":
        return []
    records = payload[3:]
    dtcs = []
    for i in range(0, len(records) - 3, 4):
        hi, mid, ftb, status = records[i:i + 4]
        sae = sae_code(hi, mid)
        dtcs.append(Dtc(code=f"{sae}-{ftb:02X}", raw=f"{hi:02X}{mid:02X}{ftb:02X}", sae=sae,
                        status=status, summary=uds_status_summary(status)))
    return dtcs


def decode_kwp_dtcs(payload: bytes) -> list[Dtc]:
    """`57 <count>` + [DTC hi, DTC lo, status] records; the status meaning is database-defined."""
    if len(payload) < 2 or payload[0] != 0x57:
        return []
    records = payload[2:2 + 3 * payload[1]]
    dtcs = []
    for i in range(0, len(records) - 2, 3):
        hi, lo, status = records[i:i + 3]
        sae = sae_code(hi, lo)
        dtcs.append(Dtc(code=f"DTC {hi:02X}{lo:02X} status 0x{status:02X} ({sae} if SAE)",
                        raw=f"{hi:02X}{lo:02X}", sae=sae, status=status))
    return dtcs


def decode_obd_dtcs(payload: bytes) -> list[Dtc]:
    """Mode 03/07/0A answer on CAN: `43|47|4A <count>` + two-byte DTCs; 00 00 pairs are skipped."""
    if len(payload) < 2 or payload[0] not in (0x43, 0x47, 0x4A):
        return []
    data = payload[2:2 + 2 * payload[1]]
    dtcs = []
    for b0, b1 in zip(data[::2], data[1::2]):
        if b0 or b1:
            sae = sae_code(b0, b1)
            dtcs.append(Dtc(code=sae, raw=f"{b0:02X}{b1:02X}", sae=sae,
                            description=OBD_DESCRIPTIONS.get(sae, "")))
    return dtcs


def decode_vin(payload: bytes) -> Optional[str]:
    """`49 02 01` + 17 ASCII characters."""
    if payload[:3] != b"\x49\x02\x01":
        return None
    return payload[3:].strip(b"\x00").decode("ascii", "replace")


def text_or_hex(data: bytes) -> str:
    text = data.rstrip(b"\x00\xff ")
    if text and all(0x20 <= c < 0x7F for c in text):
        return text.decode("ascii").strip()
    return data.hex(" ").upper()


def decode_2180(payload: bytes) -> dict[str, str]:
    """Renault KWP identification `61 80 ...`; offsets as in ddt4all ecu_scanner check_ecu."""
    if payload[:2] != b"\x61\x80" or len(payload) < 20:
        return {}
    return {
        "diag version": str(payload[7]),
        "supplier": text_or_hex(payload[8:11]),
        "part number": payload[11:16].hex().upper(),
        "soft": payload[16:18].hex().upper(),
        "version": payload[18:20].hex().upper(),
    }


def nrc_name(nrc: int) -> str:
    return NRC_NAMES.get(nrc, "unknown")


def negative_text(message: bytes) -> str:
    nrc = message[2] if len(message) > 2 else 0
    return f"{nrc:02X} {nrc_name(nrc)}"


@dataclass(frozen=True)
class Dialect:
    name: str
    session: str
    leave: str
    read_dtcs: str
    decode: Callable[[bytes], list[Dtc]]
    clear: str


UDS = Dialect("UDS", "1003", "1001", "1902AF", decode_uds_dtcs, "14FFFFFF")
KWP = Dialect("KWP", "10C0", "1081", "17FF00", decode_kwp_dtcs, "14FF00")
DIALECTS = {d.name: d for d in (UDS, KWP)}


@dataclass
class EcuResult:
    name: str
    tx: str
    rx: str
    protocol: str = "?"
    ident: dict[str, str] = field(default_factory=dict)
    dtcs: Optional[list[Dtc]] = None
    notes: list[str] = field(default_factory=list)


@dataclass
class ObdResult:
    id: str
    name: str
    mil: Optional[bool] = None
    dtc_count: Optional[int] = None
    vin: Optional[str] = None
    dtcs: dict[str, list[Dtc]] = field(default_factory=dict)


@dataclass
class Report:
    port: str
    adapter: str = ""
    guard: Optional[str] = None
    survey_frames: int = 0
    survey_ids: list[str] = field(default_factory=list)
    obd_skipped: str = ""
    obd: list[ObdResult] = field(default_factory=list)
    renault_skipped: str = ""
    ecus: list[EcuResult] = field(default_factory=list)
    skipped_ecus: list[str] = field(default_factory=list)
    statuses: dict[str, int] = field(default_factory=dict)
    notes: list[str] = field(default_factory=list)


class AdapterTimeout(Exception):
    pass


class Elm:
    """The serial adapter: one command at a time, each answered up to the `>` prompt."""

    def __init__(self, port: serial.Serial, verbose: bool = False):
        self.port = port
        self.verbose = verbose
        self.statuses: Counter[str] = Counter()
        self.rejected: list[str] = []

    def _trace(self, direction: str, text: str) -> None:
        if self.verbose:
            print(f"{direction} {text}", file=sys.stderr)

    def _write(self, cmd: str) -> None:
        self._trace(">", cmd or "<CR>")
        self.port.write(cmd.encode("ascii") + b"\r")

    def _read_prompt(self, cmd: str, timeout: float, received: bytes = b"") -> list[str]:
        buf = bytearray(received)
        deadline = time.monotonic() + timeout
        while b">" not in buf:
            if time.monotonic() > deadline:
                raise AdapterTimeout(f"adapter gave no '>' prompt within {timeout:g} s after {cmd or '<CR>'}")
            buf += self.port.read(self.port.in_waiting or 1)
        text = buf[:buf.index(b">")].decode("ascii", "replace").replace("\x00", "")
        lines = [line.strip() for line in re.split(r"[\r\n]", text) if line.strip()]
        for line in lines:
            self._trace("<", line)
        return lines

    def send(self, cmd: str, timeout: float = AT_TIMEOUT) -> list[str]:
        self.port.reset_input_buffer()
        self._write(cmd)
        echo = cmd.replace(" ", "").upper()
        return [line for line in self._read_prompt(cmd, timeout) if line.replace(" ", "").upper() != echo]

    def at(self, cmd: str) -> list[str]:
        lines = self.send(cmd)
        if "?" in lines and cmd not in self.rejected:
            self.rejected.append(cmd)
        return lines

    def request(self, req: str, clear: bool = False) -> Reply:
        sid = int(req[:2], 16)
        if sid not in READ_SERVICES and not (clear and sid in CLEAR_SERVICES):
            raise ValueError(f"service {sid:02X} is not allowed")
        reply = Reply.parse(self.send(req, REQUEST_TIMEOUT))
        self.statuses.update(reply.statuses)
        return reply

    def monitor(self, seconds: float) -> list[str]:
        """ATMA for `seconds`, then a lone CR, which the adapter discards after printing STOPPED."""
        self.port.reset_input_buffer()
        self._write("ATMA")
        buf = bytearray()
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline and b">" not in buf:
            buf += self.port.read(self.port.in_waiting or 1)
        if b">" not in buf:
            self._write("")
        return [line for line in self._read_prompt("ATMA", AT_TIMEOUT, bytes(buf)) if line != "ATMA"]


def init_adapter(elm: Elm, args: argparse.Namespace, report: Report) -> None:
    elm.send("ATZ")
    st = max(1, min(0xFF, args.timeout_ms // 4))
    for cmd in ("ATE0", "ATL0", "ATS1", "ATH1", "ATCAF1", "ATAL", "ATCFC1",
                f"ATSP{args.protocol}", f"ATST{st:02X}", "ATAT1"):
        elm.at(cmd)
    report.adapter = " ".join(elm.send("ATI"))
    guard = elm.send("AT#GUARD")
    if guard and guard != ["?"]:
        report.guard = " ".join(guard)


def passive_survey(elm: Elm, listen_ms: int, report: Report) -> set[int]:
    ids = survey_ids(elm.monitor(listen_ms / 1000))
    seen = set(ids)
    report.survey_frames = len(ids)
    report.survey_ids = [fmt_id(i) for i in sorted(seen)]
    if not ids:
        report.notes.append("no CAN frames during the passive survey: ignition off? wrong pins? "
                            "wrong bitrate? (continuing: a gatewayed bus can be quiet)")
    return seen


def obd_request(elm: Elm, req: str, notes: list[str], clear: bool = False) -> Reply:
    reply = elm.request(req, clear)
    notes.extend(f"OBD-II {req}: {s}" for s in reply.statuses)
    for rid, answer in reply.answers(int(req[:2], 16)).items():
        if answer[0] == 0x7F:
            notes.append(f"OBD-II {req}: {fmt_id(rid)} negative {negative_text(answer)}")
    return reply


def read_obd_dtcs(elm: Elm, notes: list[str]) -> dict[int, dict[str, list[Dtc]]]:
    found: dict[int, dict[str, list[Dtc]]] = {}
    for mode, label in OBD_DTC_MODES:
        for rid, answer in obd_request(elm, mode, notes).answers(int(mode, 16)).items():
            if answer[0] != 0x7F:
                found.setdefault(rid, {})[label] = decode_obd_dtcs(answer)
    return found


def select_obd(elm: Elm) -> None:
    for cmd in ("ATSH7DF", "ATAR", "ATFCSM0"):
        elm.at(cmd)


def scan_obd(elm: Elm, report: Report) -> None:
    select_obd(elm)
    results = {
        rid: ObdResult(fmt_id(rid), "engine" if rid == 0x7E8 else "")
        for rid, answer in obd_request(elm, "0100", report.notes).answers(0x01).items()
        if answer[0] == 0x41
    }
    if not results:
        return
    for rid, answer in obd_request(elm, "0101", report.notes).answers(0x01).items():
        if rid in results and answer[:2] == b"\x41\x01" and len(answer) >= 3:
            results[rid].mil = bool(answer[2] & 0x80)
            results[rid].dtc_count = answer[2] & 0x7F
    for rid, answer in obd_request(elm, "0902", report.notes).answers(0x09).items():
        if rid in results:
            results[rid].vin = decode_vin(answer)
    for rid, dtcs in read_obd_dtcs(elm, report.notes).items():
        if rid in results:
            results[rid].dtcs = dtcs
    report.obd = [results[rid] for rid in sorted(results)]


def select_ecu(elm: Elm, tx: int, rx: int) -> None:
    for cmd in (f"ATSH{tx:03X}", f"ATFCSH{tx:03X}", f"ATCRA{rx:03X}"):
        elm.at(cmd)


def ask(elm: Elm, req: str, rx: int, notes: list[str]) -> tuple[bool, Optional[bytes]]:
    """Send one request; returns whether `rx` sent anything, and its final answer."""
    reply = elm.request(req)
    notes.extend(f"{req}: {s}" for s in reply.statuses)
    answer = reply.answer(rx, int(req[:2], 16))
    if answer is not None and answer[0] == 0x7F:
        notes.append(f"{req}: negative {negative_text(answer)}")
    return rx in reply.messages, answer


def open_session(elm: Elm, rx: int, dialect: Dialect, notes: list[str]) -> tuple[bool, bool]:
    heard, answer = ask(elm, dialect.session, rx, notes)
    return heard, answer is not None and answer[:2] == bytes.fromhex("50" + dialect.session[2:])


def read_dtcs(elm: Elm, rx: int, dialect: Dialect, notes: list[str]) -> Optional[list[Dtc]]:
    _, answer = ask(elm, dialect.read_dtcs, rx, notes)
    if answer is None or answer[0] == 0x7F:
        return None
    return dialect.decode(answer)


def identify(elm: Elm, rx: int, dialect: Dialect, ecu: EcuResult) -> None:
    if dialect is UDS:
        for did, label in UDS_IDENT:
            _, answer = ask(elm, "22" + did, rx, ecu.notes)
            if answer is not None and answer[:3] == bytes.fromhex("62" + did):
                ecu.ident[label] = text_or_hex(answer[3:])
        return
    _, answer = ask(elm, "2180", rx, ecu.notes)
    if answer is not None and answer[:2] == b"\x61\x80":
        ecu.ident["2180"] = answer.hex(" ").upper()
        ecu.ident.update(decode_2180(answer))


def scan_ecu(elm: Elm, tx: int, rx: int, name: str) -> Optional[EcuResult]:
    ecu = EcuResult(name, fmt_id(tx), fmt_id(rx))
    heard = False
    for dialect in (UDS, KWP):
        answered, opened = open_session(elm, rx, dialect, ecu.notes)
        heard = heard or answered
        if opened:
            ecu.protocol = dialect.name
            identify(elm, rx, dialect, ecu)
            ecu.dtcs = read_dtcs(elm, rx, dialect, ecu.notes)
            ask(elm, dialect.leave, rx, ecu.notes)
            return ecu
    return ecu if heard else None


def scan_renault(elm: Elm, report: Report, seen: set[int], only: Optional[set[int]]) -> None:
    targets = []
    for tx, rx, name in RENAULT_ECUS:
        if only is not None and tx not in only:
            continue
        if tx in seen:
            report.skipped_ecus.append(f"{fmt_id(tx)}->{fmt_id(rx)} {name}")
        else:
            targets.append((tx, rx, name))
    if not targets:
        return
    elm.at("ATFCSD300000")
    for i, (tx, rx, name) in enumerate(targets):
        select_ecu(elm, tx, rx)
        if i == 0:
            elm.at("ATFCSM1")
        ecu = scan_ecu(elm, tx, rx, name)
        if ecu is not None:
            report.ecus.append(ecu)


def scan(elm: Elm, args: argparse.Namespace) -> Report:
    report = Report(port=args.port)
    init_adapter(elm, args, report)
    print(f"canscan: passive survey ({args.listen_ms} ms)...", file=sys.stderr)
    seen = passive_survey(elm, args.listen_ms, report)

    live_obd_ids = sorted(seen & OBD_TX_IDS)
    if args.no_obd:
        report.obd_skipped = "--no-obd"
    elif live_obd_ids:
        report.obd_skipped = "id " + " ".join(fmt_id(i) for i in live_obd_ids) + " seen on the bus"
    else:
        print("canscan: generic OBD-II...", file=sys.stderr)
        scan_obd(elm, report)

    if args.no_renault:
        report.renault_skipped = "--no-renault"
    else:
        print("canscan: Renault/Dacia modules...", file=sys.stderr)
        scan_renault(elm, report, seen, args.only)

    if elm.rejected:
        report.notes.append("adapter answered ? to: " + ", ".join(elm.rejected))
    report.statuses = dict(elm.statuses)
    return report


def clear_outcome(reply: Reply, rx: int, sid: int) -> str:
    if "BLOCKED" in reply.statuses:
        return "BLOCKED: the adapter's guard refused to transmit the clear request"
    answer = reply.answer(rx, sid)
    if answer is None:
        return "no answer" + (f" ({', '.join(reply.statuses)})" if reply.statuses else "")
    if answer[0] == 0x7F:
        return f"refused, negative {negative_text(answer)}"
    return "cleared"


def clear_ecu(elm: Elm, ecu: EcuResult) -> str:
    dialect = DIALECTS[ecu.protocol]
    rx = int(ecu.rx, 16)
    notes: list[str] = []
    select_ecu(elm, int(ecu.tx, 16), rx)
    _, opened = open_session(elm, rx, dialect, notes)
    if not opened:
        return f"session {dialect.session} not opened, nothing sent ({'; '.join(notes) or 'no answer'})"
    outcome = clear_outcome(elm.request(dialect.clear, clear=True), rx, 0x14)
    after = read_dtcs(elm, rx, dialect, notes)
    ask(elm, dialect.leave, rx, notes)
    return f"{outcome}; DTCs {len(ecu.dtcs or [])} -> {'?' if after is None else len(after)}"


def clear_obd(elm: Elm, targets: list[ObdResult]) -> list[str]:
    select_obd(elm)
    notes: list[str] = []
    reply = obd_request(elm, "04", notes, clear=True)
    after = read_obd_dtcs(elm, notes)
    lines = []
    for target in targets:
        rid = int(target.id, 16)
        before = sum(len(d) for d in target.dtcs.values())
        now = sum(len(d) for d in after[rid].values()) if rid in after else "?"
        lines.append(f"  OBD-II {target.id}: {clear_outcome(reply, rid, 0x04)}; DTCs {before} -> {now}")
    return lines


def clear_dtcs(elm: Elm, report: Report) -> None:
    ecus = [e for e in report.ecus if e.dtcs]
    obd = [o for o in report.obd if any(o.dtcs.values())]
    if not ecus and not obd:
        print("\n--clear: no DTCs found, nothing to clear.")
        return
    print("\n--clear will erase the DTCs of:")
    for e in ecus:
        print(f"  {e.name} {e.tx}->{e.rx}: {e.protocol} {DIALECTS[e.protocol].clear} ({len(e.dtcs)} DTCs)")
    if obd:
        print(f"  OBD-II: mode 04 to 7DF, every OBD-II ECU clears (DTCs listed by {', '.join(o.id for o in obd)})")
    try:
        typed = input("Type CLEAR to proceed: ")
    except EOFError:
        typed = ""
    if typed.strip() != "CLEAR":
        print("Aborted, nothing cleared.")
        return
    if report.guard is not None:
        elm.send("AT#GUARD CLEAR")
    try:
        for e in ecus:
            print(f"  {e.name} {e.tx}->{e.rx}: {clear_ecu(elm, e)}")
        if obd:
            print("\n".join(clear_obd(elm, obd)))
    finally:
        if report.guard is not None:
            elm.send("AT#GUARD READ")


def modules_answered(report: Report) -> int:
    renault_rx = {e.rx for e in report.ecus}
    return len(report.ecus) + sum(1 for o in report.obd if o.id not in renault_rx)


def dtc_total(report: Report) -> int:
    return (sum(len(e.dtcs or []) for e in report.ecus)
            + sum(len(d) for o in report.obd for d in o.dtcs.values()))


def ecu_dtc_text(protocol: str, dtc: Dtc) -> str:
    if protocol == "UDS":
        return f"{dtc.code}  DTC {dtc.raw} status 0x{dtc.status:02X} {dtc.summary}".rstrip()
    return dtc.code


def format_report(r: Report) -> str:
    guard = f" (arduino_can firmware, AT#GUARD {r.guard})" if r.guard else ""
    out = [f"Adapter: {r.adapter or 'unknown'} on {r.port}{guard}",
           f"Survey: {r.survey_frames} frames, {len(r.survey_ids)} distinct ids"
           + (": " + " ".join(r.survey_ids) if r.survey_ids else ""),
           "", "OBD-II (7DF):"]
    if r.obd_skipped:
        out.append(f"  skipped: {r.obd_skipped}")
    elif not r.obd:
        out.append("  no responder")
    for o in r.obd:
        mil = "?" if o.mil is None else ("on" if o.mil else "off")
        count = "?" if o.dtc_count is None else o.dtc_count
        out.append(f"  {o.id} {o.name}".rstrip() + f": MIL {mil}, {count} stored DTCs (0101), VIN {o.vin or '?'}")
        for _, label in OBD_DTC_MODES:
            dtcs = o.dtcs.get(label)
            if not dtcs:
                out.append(f"    {label:<9}  {'not read' if dtcs is None else 'none'}")
            for d in dtcs or []:
                out.append(f"    {label:<9}  {d.code} {d.description}".rstrip())

    out += ["", "Renault/Dacia modules:"]
    if r.renault_skipped:
        out.append(f"  skipped: {r.renault_skipped}")
    elif not r.ecus:
        out.append("  none answered")
    for e in r.ecus:
        out.append(f"  {e.name:<40} {e.tx}->{e.rx}  {e.protocol}")
        for key, value in e.ident.items():
            out.append(f"      {key}: {value}")
        if not e.dtcs:
            out.append(f"      DTCs: {'not read' if e.dtcs is None else 'none'}")
        for d in e.dtcs or []:
            out.append(f"      {ecu_dtc_text(e.protocol, d)}")
        for note in e.notes:
            out.append(f"      ({note})")
    if r.skipped_ecus:
        out.append("  not probed, request id seen as live traffic: " + ", ".join(r.skipped_ecus))

    out.append("")
    if r.statuses:
        out.append("Adapter status lines: " + ", ".join(f"{k} x{v}" for k, v in r.statuses.items()))
    out += [f"Note: {note}" for note in r.notes]
    out.append(f"{modules_answered(r)} modules answered, {dtc_total(r)} DTCs")
    return "\n".join(out)


def id_set(text: str) -> set[int]:
    return {int(part, 16) for part in text.split(",") if part.strip()}


def parse_args(argv: Optional[list[str]]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="exit status: 0 scan completed (with or without DTCs), 2 adapter does not answer, "
               "3 bus looks dead (no frames heard and no ECU answered)")
    parser.add_argument("port", metavar="PORT", help="adapter serial port, e.g. /dev/ttyACM0 or /dev/ttyUSB0")
    parser.add_argument("--json", metavar="FILE", help="also write the report as JSON to FILE")
    parser.add_argument("--clear", action="store_true",
                        help="after the report, clear the DTCs found (asks you to type CLEAR)")
    parser.add_argument("--no-obd", action="store_true", help="skip generic OBD-II")
    parser.add_argument("--no-renault", action="store_true", help="skip the Renault/Dacia module scan")
    parser.add_argument("--only", type=id_set, metavar="IDS",
                        help="probe only these Renault/Dacia request ids, e.g. 7E0,745")
    parser.add_argument("--timeout-ms", type=int, default=150, help="adapter response timeout, ATST (default %(default)s)")
    parser.add_argument("--listen-ms", type=int, default=1500,
                        help="passive survey length before anything is transmitted (default %(default)s)")
    parser.add_argument("--protocol", default="6",
                        help="ELM protocol for ATSP (default %(default)s = ISO 15765-4 CAN 11-bit 500 kbit)")
    parser.add_argument("--verbose", action="store_true", help="print every line sent/received on stderr")
    return parser.parse_args(argv)


def main(argv: Optional[list[str]] = None) -> int:
    args = parse_args(argv)
    try:
        port = serial.serial_for_url(args.port, baudrate=BAUD, timeout=0.05)
    except serial.SerialException as exc:
        print(f"canscan: cannot open {args.port}: {exc}", file=sys.stderr)
        return 2
    with port:
        elm = Elm(port, args.verbose)
        try:
            report = scan(elm, args)
            print(format_report(report))
            if args.json:
                with open(args.json, "w") as f:
                    json.dump(dataclasses.asdict(report), f, indent=2)
            if args.clear:
                clear_dtcs(elm, report)
        except (AdapterTimeout, serial.SerialException) as exc:
            print(f"canscan: {exc}", file=sys.stderr)
            return 2
    bus_dead = report.survey_frames == 0 and not report.obd and not report.ecus
    return 3 if bus_dead else 0


if __name__ == "__main__":
    sys.exit(main())

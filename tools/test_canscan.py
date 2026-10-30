import pytest

from canscan import (
    Elm,
    Reply,
    decode_2180,
    decode_kwp_dtcs,
    decode_obd_dtcs,
    decode_uds_dtcs,
    decode_vin,
    nrc_name,
    parse_frames,
    reassemble,
    sae_code,
    scan_ecu,
    status_lines,
    survey_ids,
)


class FakeAdapter:
    """The serial port of an ELM327 with echo off. A request answers `answers[request]` or
    NO DATA and is logged with the header it went out on; ATMA prints `traffic`."""

    def __init__(self, answers=None, traffic=()):
        self.answers = answers or {}
        self.traffic = list(traffic)
        self.header = "7DF"
        self.requests: list[tuple[str, str]] = []
        self._out = bytearray()

    @property
    def in_waiting(self) -> int:
        return len(self._out)

    def reset_input_buffer(self) -> None:
        self._out.clear()

    def read(self, size: int = 1) -> bytes:
        chunk = bytes(self._out[:size])
        del self._out[:size]
        return chunk

    def write(self, data: bytes) -> None:
        cmd = data.decode("ascii").strip().replace(" ", "").upper()
        if cmd == "ATMA":
            self._out += "".join(f"{line}\r" for line in self.traffic).encode()
            return
        self._out += ("\r".join(self._answer(cmd)) + "\r\r>").encode()

    def _answer(self, cmd: str) -> list[str]:
        if not cmd:
            return ["STOPPED"]
        if cmd.startswith("AT"):
            if cmd.startswith("ATSH"):
                self.header = cmd[4:]
            return {"ATZ": ["ELM327 v1.5"], "ATI": ["ELM327 v1.5"], "AT#GUARD": ["?"]}.get(cmd, ["OK"])
        self.requests.append((self.header, cmd))
        return self.answers.get(cmd, ["NO DATA"])


def elm_lines(can_id: str, payload: bytes) -> list[str]:
    """What an ELM327 prints in CAF1+H1 for an ISO-TP multi-frame message, padded to 8 bytes."""
    frames = [bytes([0x10 | len(payload) >> 8, len(payload) & 0xFF]) + payload[:6]]
    rest = payload[6:]
    for seq, start in enumerate(range(0, len(rest), 7), start=1):
        frames.append(bytes([0x20 | seq % 16]) + rest[start:start + 7])
    return [f"{can_id} " + frame.ljust(8, b"\xAA").hex(" ").upper() for frame in frames]


@pytest.mark.parametrize("b0, b1, code", [
    (0x00, 0x01, "P0001"),
    (0x3F, 0xFF, "P3FFF"),
    (0x40, 0x00, "C0000"),
    (0x7F, 0xFF, "C3FFF"),
    (0x80, 0x00, "B0000"),
    (0xBF, 0xFF, "B3FFF"),
    (0xC0, 0x01, "U0001"),
    (0xFF, 0xFF, "U3FFF"),
    (0x03, 0x35, "P0335"),
    (0x13, 0x35, "P1335"),
    (0x23, 0x35, "P2335"),
    (0x33, 0x35, "P3335"),
])
def test_sae_letter_and_digit(b0, b1, code):
    assert sae_code(b0, b1) == code


def test_obd_dtcs_follow_the_count_byte_and_skip_0000():
    stored = decode_obd_dtcs(bytes.fromhex("43 03 0335 0000 C100 0420"))
    assert [d.code for d in stored] == ["P0335", "U0100"]
    assert [d.code for d in decode_obd_dtcs(bytes.fromhex("47 01 0420"))] == ["P0420"]
    assert decode_obd_dtcs(bytes.fromhex("4A 00")) == []


def test_uds_records_with_status_summary():
    dtcs = decode_uds_dtcs(bytes.fromhex("59 02 FF  03 35 1C 2F  C1 00 87 08  01 00 00 50"))
    assert [(d.code, d.raw, d.status, d.summary) for d in dtcs] == [
        ("P0335-1C", "03351C", 0x2F, "active, stored, pending"),
        ("U0100-87", "C10087", 0x08, "stored"),
        ("P0100-00", "010000", 0x50, ""),
    ]


def test_kwp_records_bounded_by_count():
    dtcs = decode_kwp_dtcs(bytes.fromhex("57 02  03 35 24  C1 00 E0  01 02 03"))
    assert [d.code for d in dtcs] == [
        "DTC 0335 status 0x24 (P0335 if SAE)",
        "DTC C100 status 0xE0 (U0100 if SAE)",
    ]
    assert [(d.raw, d.status) for d in dtcs] == [("0335", 0x24), ("C100", 0xE0)]


def test_multiframe_vin():
    lines = [
        "7E8 10 14 49 02 01 31 44 34",
        "7E8 21 47 50 30 30 52 35 35",
        "7E8 22 42 31 32 33 34 35 36",
    ]
    (message,) = reassemble(parse_frames(lines)[0x7E8])
    assert decode_vin(message) == "1D4GP00R55B123456"


def test_multiframe_padding_is_trimmed():
    lines = [
        "765 10 1A 61 80 82 00 45 15",
        "765 21 05 08 32 31 33 21 11",
        "765 22 31 39 09 00 09 06 02",
        "765 23 05 01 0D 8D 39 00 AA",
    ]
    (message,) = reassemble(parse_frames(lines)[0x765])
    assert message == bytes.fromhex("6180820045150508323133211131390900090602 05010D8D3900")
    assert decode_2180(message) == {
        "diag version": "8",
        "supplier": "213",
        "part number": "2111313909",
        "soft": "0009",
        "version": "0602",
    }


def test_multiframe_sequence_wraps_after_f():
    payload = bytes.fromhex("5902FF") + b"".join(bytes([0x01, n, 0x00, 0x08]) for n in range(29))
    lines = elm_lines("765", payload)
    assert [line.split()[1] for line in lines[14:18]] == ["2E", "2F", "20", "21"]
    assert reassemble(parse_frames(lines)[0x765]) == [payload]
    assert len(decode_uds_dtcs(payload)) == 29


def test_multiframe_with_a_missing_frame_is_dropped():
    lines = elm_lines("765", bytes(range(30)))
    del lines[2]
    assert reassemble(parse_frames(lines)[0x765]) == []


def test_response_pending_then_real_answer():
    reply = Reply.parse([
        "765 03 7F 22 78 AA AA AA AA",
        "765 10 0D 62 F1 87 32 33 37",
        "765 21 31 30 31 32 33 34 52",
    ])
    assert reply.messages == {0x765: [b"\x62\xF1\x87237101234R"]}
    assert reply.answer(0x765, 0x22)[3:] == b"237101234R"


def test_a_module_answering_only_response_pending_is_listed_without_a_protocol():
    pending = ["772 03 7F 10 78 AA AA AA AA"]
    port = FakeAdapter(answers={"1003": pending, "10C0": pending})
    ecu = scan_ecu(Elm(port), 0x752, 0x772, "Airbag-SRS")
    assert ecu is not None and ecu.protocol == "?"
    assert scan_ecu(Elm(FakeAdapter()), 0x752, 0x772, "Airbag-SRS") is None


def test_interleaved_responders_are_keyed_by_id():
    reply = Reply.parse([
        "7E8 10 14 49 02 01 31 44 34",
        "7E9 10 14 49 02 01 56 46 31",
        "7E8 21 47 50 30 30 52 35 35",
        "7E9 21 42 52 30 42 30 48 31",
        "7E9 22 32 33 34 35 36 37 38",
        "7E8 22 42 31 32 33 34 35 36",
    ])
    vins = {rid: decode_vin(answer) for rid, answer in reply.answers(0x09).items()}
    assert vins == {0x7E8: "1D4GP00R55B123456", 0x7E9: "VF1BR0B0H12345678"}


def test_29bit_ids_flow_control_dropped_and_negative_answer():
    reply = Reply.parse([
        "18 DA F1 10 30 00 00 AA AA AA AA AA",
        "18 DA F1 10 03 7F 22 31 AA AA AA AA",
    ])
    assert reply.messages == {0x18DAF110: [b"\x7F\x22\x31"]}
    assert nrc_name(reply.answer(0x18DAF110, 0x22)[2]) == "requestOutOfRange"


def test_status_lines_are_surfaced_and_not_parsed_as_frames():
    lines = ["SEARCHING...", "7E8 06 41 00 BE 3F A8 13 00", "NO DATA", "BLOCKED", "CAN ERROR", "?", "STOPPED"]
    assert parse_frames(lines) == {0x7E8: [bytes.fromhex("064100BE3FA81300")]}
    assert status_lines(lines) == ["SEARCHING...", "NO DATA", "BLOCKED", "CAN ERROR", "?", "STOPPED"]
    blocked = Reply.parse(["BLOCKED"])
    assert blocked.messages == {} and blocked.statuses == ["BLOCKED"]


def test_survey_ids_from_monitor_output():
    lines = [
        "0C9 12 34 56 78 9A BC DE F0",
        "1F6 01 02 <DATA ERROR",
        "18 DA F1 10 02 50 03",
        "FC: 7E0 30 00 00",
        "0C9 12 34 56 78 9A BC DE F1 BUFFER FULL",
        "STOPPED",
    ]
    assert survey_ids(lines) == [0x0C9, 0x1F6, 0x18DAF110, 0x7E0, 0x0C9]

#!/usr/bin/env python3
# /// script
# requires-python = ">=3.9"
# dependencies = ["pyserial>=3.5"]
# ///
"""Client compatibility check for the ELM327 emulator.

Drives the real ELM classes of ddt4all and pyren (pinned upstream commits) and python-OBD
(PyPI), each in its own uv environment and subprocess, plus raw guard/bitrate exchanges and
our scanner, against the virtual Renault/Dacia car. The car is served either by the host
simulator (`--sim`, started and stopped here) or by the board running the loopback demo
firmware (`--port`).

Over TCP (`--sim ... --tcp`, or `--port socket://HOST:PORT` for the board's WiFi) every client
opens the adapter the way it opens a WiFi ELM327: python-OBD, canscan and the raw checks as
`socket://HOST:PORT`, ddt4all (WiFi adapter type) and pyren as `HOST:PORT`.

    uv run tools/compat/check_clients.py --sim .pio/build/sim/program
    uv run tools/compat/check_clients.py --sim .pio/build/sim/program --tcp
    uv run tools/compat/check_clients.py --port /dev/ttyACM0 --only guard,bitrate
    uv run tools/compat/check_clients.py --port socket://192.168.0.10:35000
    uv run tools/compat/check_clients.py --self-check     # fetch and import the clients only

Client sources are cached in ~/.cache/arduino_can_compat/, client output goes to its logs/
(overwritten each run), and a failing check prints where that client's output and wire log are.
"""
from __future__ import annotations

import argparse
import contextlib
import json
import os
import platform
import re
import select
import shutil
import signal
import socket
import subprocess
import sys
import tempfile
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Iterator, Optional

# Only the standard library above: this file also runs inside each client's own environment
# (`--client`), where pyserial may be absent or, for pyren, its bundled copy.

REPO = Path(__file__).resolve().parents[2]
CACHE = Path.home() / ".cache" / "arduino_can_compat"
LOG_DIR = CACHE / "logs"

SIM_READY_TIMEOUT = 10.0
CLIENT_TIMEOUT = 240.0   # includes uv resolving the client's environment on a first run
CANSCAN_TIMEOUT = 400.0
GIT_TIMEOUT = 300.0
RAW_TIMEOUT = 10.0
QUIET_TIME = 0.5

SOCKET = "socket://"

CHECKS = ("python-obd", "ddt4all", "pyren", "guard", "guard-tcp", "bitrate", "canscan")
TCP_ONLY = ("guard-tcp",)
CLIENTS = ("python-obd", "ddt4all", "pyren")


@dataclass(frozen=True)
class Source:
    name: str
    url: str
    sha: str

    @property
    def path(self) -> Path:
        return CACHE / f"{self.name}-{self.sha[:8]}"


SOURCES = {
    "ddt4all": Source("ddt4all", "https://github.com/cedricp/ddt4all.git",
                      "e7b582257e156f1cf2956831874c26305403c9b9"),
    "pyren": Source("pyren", "https://gitlab.com/py_ren/pyren.git",
                    "b0ae37376ac1cd33f22df0f72314f45fa0936b0f"),
}

# ddt4all pins from its pyproject.toml at the pinned commit; pyren3/ ships its own pyserial.
CLIENT_DEPS = {
    "python-obd": ["obd==0.7.3"],
    "ddt4all": ["pyserial==3.5", "platformdirs>=3.0.0,<4.0", "pyusb==1.2.1", "crcmod==1.7"],
    "pyren": [],
}


def payload(hex_text: str, ascii_tail: str = "") -> str:
    """A response payload as ddt4all/pyren return it: spaced upper-case hex."""
    return (bytes.fromhex(hex_text) + ascii_tail.encode("ascii")).hex(" ").upper()


VIN = "UU15SDAG551234567"
UDS_DEFAULT_SESSION = payload("50 01 00 32 01 F4")
UDS_EXTENDED_SESSION = payload("50 03 00 32 01 F4")
ECM_DTCS = payload("59 02 FF 03 35 1C 2F 01 15 64 08")
AIRBAG_DTCS = payload("59 02 FF 9A 01 11 08")
UCH_2180 = payload("61 80 82 00 44 66 27 44 32 31 33 82 00 38 71 38 00 A7 75 00 00 00 00 00 00 00")
ABS_2180 = payload("61 80 01 02 03 04 05 44 42 4F 53 01 02 03 04 05 00 12 34 56")


@dataclass(frozen=True)
class Negative:
    """How ddt4all and pyren return a negative response: `NR:<nrc>:<their description>`."""
    nrc: str

    def __repr__(self) -> str:
        return repr(f"NR:{self.nrc}:...")


class _Missing:
    def __repr__(self) -> str:
        return "nothing (the client stopped before this step)"


MISSING = _Missing()


def matches(expected: object, got: object) -> bool:
    if isinstance(expected, Negative):
        return isinstance(got, str) and got.startswith(f"NR:{expected.nrc}:")
    return expected == got


@dataclass(frozen=True)
class Read:
    method: str  # the client ELM method: cmd, request or start_session
    req: str
    expected: object
    note: str = ""

    @property
    def label(self) -> str:
        return f"{self.req} {self.note}" if self.note else self.req


@dataclass(frozen=True)
class Ecu:
    name: str
    addr: str  # Renault diagnostic address, the key of the clients' dnat/snat tables
    reads: tuple[Read, ...]
    brp: str = ""  # set_can_addr's ecu['brp'] (bus rates listed for the ECU)
    connected: object = None  # what set_can_addr reports, checked when not None

    def plan(self) -> dict:
        return {"name": self.name, "addr": self.addr, "ecu": {"brp": self.brp} if self.brp else {},
                "reads": [[r.label, r.method, r.req] for r in self.reads]}


DDT4ALL_ECUS = (
    Ecu("ECM", "7A", (
        Read("cmd", "1003", UDS_EXTENDED_SESSION),
        Read("request", "22F1A0", payload("62 F1 A0 44")),
        Read("request", "22F18A", payload("62 F1 8A", "CONTINENTAL")),
        Read("request", "22F194", payload("62 F1 94", "S180067A")),
        Read("request", "22F195", payload("62 F1 95", "0A04")),
        Read("request", "22F187", payload("62 F1 87", "237107631R")),
        Read("request", "22F190", payload("62 F1 90", VIN), "(VIN, multi-frame)"),
        Read("request", "1902AF", ECM_DTCS),
        Read("cmd", "1001", UDS_DEFAULT_SESSION),
    ), connected=["7E0", "7E8"]),
    Ecu("UCH", "26", (
        Read("cmd", "1003", Negative("12")),
        Read("cmd", "10C0", payload("50 C0")),
        Read("request", "2180", UCH_2180, "(multi-frame)"),
        Read("request", "17FF00", payload("57 01 93 23 E0")),
        Read("cmd", "3E01", payload("7E 01")),
        Read("cmd", "1081", payload("50 81")),
    ), connected=["745", "765"]),
    Ecu("ABS", "01", (
        Read("cmd", "1003", "", "(no answer)"),
        Read("cmd", "10C0", payload("50 C0")),
        Read("request", "2180", ABS_2180, "(multi-frame)"),
        Read("request", "17FF00", payload("57 00")),
        Read("cmd", "1081", payload("50 81")),
    ), connected=["740", "760"]),
    Ecu("airbag", "2C", (
        Read("cmd", "1003", UDS_EXTENDED_SESSION, "(after response pending)"),
        Read("request", "22F18A", payload("62 F1 8A", "TEMIC"), "(after response pending)"),
        Read("cmd", "22F187", Negative("31")),
        Read("request", "1902AF", AIRBAG_DTCS, "(after response pending)"),
        Read("cmd", "1001", UDS_DEFAULT_SESSION, "(after response pending)"),
    ), connected=["752", "772"]),
    Ecu("EPS", "04", (
        Read("cmd", "1003", "", "(absent)"),
        Read("cmd", "10C0", "", "(absent)"),
    ), connected=["742", "762"]),
)

# pyren repeats an ISO-TP read with its frame count appended (`0322F1903`) once it has
# seen the answer, so the second reads exercise the CAF0 response-count digit.
PYREN_ECUS = (
    Ecu("ECM", "7A", (
        Read("start_session", "1003", UDS_EXTENDED_SESSION),
        Read("request", "22F1A0", payload("62 F1 A0 44")),
        Read("request", "22F18A", payload("62 F1 8A", "CONTINENTAL")),
        Read("request", "22F194", payload("62 F1 94", "S180067A")),
        Read("request", "22F195", payload("62 F1 95", "0A04")),
        Read("request", "22F187", payload("62 F1 87", "237107631R")),
        Read("request", "22F190", payload("62 F1 90", VIN), "(VIN, multi-frame)"),
        Read("request", "22F190", payload("62 F1 90", VIN), "again (frame-count digit)"),
        Read("cmd", "1902AF", ECM_DTCS),
    )),
    # brp "01" makes pyren probe 250 kbit with 0210C0 and fall back to 500 kbit on CAN ERROR.
    Ecu("UCH", "26", (
        Read("start_session", "10C0", payload("50 C0")),
        Read("request", "2180", UCH_2180, "(multi-frame)"),
        Read("request", "2180", UCH_2180, "again (frame-count digit)"),
        Read("cmd", "17FF00", payload("57 01 93 23 E0")),
    ), brp="01", connected={"brp": "0"}),
)

PYTHON_OBD_EXPECTED = (
    ("status", "status", "Car Connected"),
    ("protocol", "protocol id after ATSP0 search", "6"),
    ("rpm", "RPM (plain, frame-count digit, bare-CR repeat)", [750.0, 750.0, 750.0]),
    ("coolant", "coolant temperature degC", 50.0),
    ("vin", "VIN", VIN),
    ("dtc", "GET_DTC", ["P0335", "P0115"]),
)


class Results:
    def __init__(self) -> None:
        self.passed = 0
        self.failed = 0

    def check(self, name: str, expected: object, got: object) -> bool:
        if matches(expected, got):
            self.ok(name)
            return True
        self.fail(name, expected, got)
        return False

    def ok(self, name: str) -> None:
        self.passed += 1
        print(f"PASS {name}", flush=True)

    def fail(self, name: str, expected: object, got: object) -> None:
        self.failed += 1
        print(f"FAIL {name}: expected {expected!r} got {got!r}", flush=True)

    def summary(self) -> int:
        print(f"\n{self.passed} passed, {self.failed} failed")
        return 1 if self.failed else 0


def note(text: str) -> None:
    print(f"  {text}", flush=True)


def child_env() -> dict[str, str]:
    env = dict(os.environ)
    # Under `uv run` this points at the harness's own environment; the clients get theirs.
    env.pop("VIRTUAL_ENV", None)
    env["PYTHONUNBUFFERED"] = "1"
    env["GIT_TERMINAL_PROMPT"] = "0"
    return env


def git_head(path: Path) -> Optional[str]:
    if not (path / ".git").is_dir():
        return None
    run = subprocess.run(["git", "-C", str(path), "rev-parse", "HEAD"], capture_output=True, text=True)
    return run.stdout.strip() if run.returncode == 0 else None


def fetch_source(source: Source) -> Path:
    """The pinned commit, checked out under CACHE (a shallow clone cannot select a sha)."""
    dest = source.path
    if git_head(dest) == source.sha:
        return dest
    dest.mkdir(parents=True, exist_ok=True)

    def git(*args: str) -> str:
        return subprocess.run(["git", "-C", str(dest), *args], check=True, capture_output=True,
                              text=True, timeout=GIT_TIMEOUT, env=child_env()).stdout

    if not (dest / ".git").is_dir():
        git("init", "-q")
    if "origin" not in git("remote").split():
        git("remote", "add", "origin", source.url)
    git("fetch", "-q", "--depth", "1", "origin", source.sha)
    git("-c", "advice.detachedHead=false", "checkout", "-q", "--force", "FETCH_HEAD")
    head = git_head(dest)
    if head != source.sha:
        raise RuntimeError(f"checkout ended at {head}")
    return dest


def fetch_sources(names: list[str], results: Results) -> dict[str, Path]:
    fetched = {}
    for name in names:
        source = SOURCES[name]
        try:
            fetched[name] = fetch_source(source)
        except (OSError, subprocess.SubprocessError, RuntimeError) as exc:
            detail = exc.stderr.strip() if isinstance(exc, subprocess.CalledProcessError) else str(exc)
            results.fail(f"{name} source", f"{source.url} at {source.sha}", detail)
    return fetched


def run_bounded(cmd: list[str], timeout: float, log_path: Path, cwd: Optional[Path] = None) -> Optional[int]:
    """Exit status of `cmd` (output to log_path); None if it had to be killed after `timeout` s."""
    log_path.parent.mkdir(parents=True, exist_ok=True)
    with open(log_path, "wb") as log:
        # Own process group: `uv run` does not take its python child down when it is killed.
        proc = subprocess.Popen(cmd, stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT,
                                cwd=cwd, env=child_env(), start_new_session=True)
        try:
            return proc.wait(timeout)
        except subprocess.TimeoutExpired:
            return None
        finally:
            if proc.poll() is None:
                os.killpg(proc.pid, signal.SIGKILL)
                proc.wait()


def uv() -> str:
    return os.environ.get("UV") or shutil.which("uv") or "uv"


def run_client(name: str, plan: dict) -> tuple[dict, str]:
    """What the client recorded (possibly partial), and why it ended badly ('' if it exited cleanly)."""
    cmd = [uv(), "run", "--no-project", "--isolated"]
    for dep in CLIENT_DEPS[name]:
        cmd += ["--with", dep]
    with tempfile.TemporaryDirectory(prefix="arduino-can-compat-") as tmp:
        plan_path, out_path = Path(tmp) / "plan.json", Path(tmp) / "recorded.json"
        plan_path.write_text(json.dumps(plan))
        cmd += ["python", str(Path(__file__).resolve()), "--client", name, str(plan_path), str(out_path)]
        # pyren writes its wire log to ./logs/ relative to the working directory.
        status = run_bounded(cmd, CLIENT_TIMEOUT, LOG_DIR / f"{name}.out", cwd=CACHE)
        recorded = json.loads(out_path.read_text()) if out_path.exists() else {}
    if status is None:
        return recorded, f"killed after {CLIENT_TIMEOUT:g} s"
    return recorded, f"exit status {status}" if status else ""


def client_logs(name: str, recorded: dict) -> None:
    note(f"{name} output: {LOG_DIR / (name + '.out')}")
    if recorded.get("wire log"):
        note(f"{name} wire log: {recorded['wire log']}")


def client_main(name: str, plan_path: str, out_path: str) -> int:
    plan = json.loads(Path(plan_path).read_text())
    recorded: dict = {"python": platform.python_version()}

    def record(key: str, value: object) -> None:
        recorded[key] = value
        tmp = out_path + ".tmp"
        Path(tmp).write_text(json.dumps(recorded))
        os.replace(tmp, out_path)

    {"python-obd": client_python_obd, "ddt4all": client_ddt4all, "pyren": client_pyren}[name](plan, record)
    return 0


def client_python_obd(plan: dict, record: Callable[[str, object], None]) -> None:
    import logging

    import obd

    record("version", f"python-OBD {obd.__version__}")
    if "port" not in plan:
        return
    obd.logger.setLevel(logging.DEBUG)

    def value(response: object) -> object:
        if response.is_null():
            return None
        return float(response.value.magnitude) if hasattr(response.value, "magnitude") else response.value

    connection = obd.OBD(plan["port"])
    record("status", connection.status())
    record("protocol", connection.protocol_id())
    record("rpm", [value(connection.query(obd.commands.RPM)) for _ in range(3)])
    record("coolant", value(connection.query(obd.commands.COOLANT_TEMP)))
    vin = value(connection.query(obd.commands.VIN, force=True))
    record("vin", None if vin is None else bytes(vin).decode("ascii", "replace"))
    dtcs = value(connection.query(obd.commands.GET_DTC))
    record("dtc", None if dtcs is None else [code for code, _ in dtcs])
    connection.close()


def client_ddt4all(plan: dict, record: Callable[[str, object], None]) -> None:
    src = Path(plan["source"]) / "src"
    sys.path.insert(0, str(src))
    import ddt4all.core.elm.elm as elm_module
    import ddt4all.options as options
    from ddt4all.core.elm.elm import ELM
    from ddt4all.file_manager import get_logs_dir
    from ddt4all.version import __version__

    record("version", f"ddt4all {__version__}")
    if "port" not in plan:
        return
    # The app loads its CAN address tables from projects.json at startup (main.load_this).
    tables = json.loads((src / "ddt4all" / "resources" / "projects.json").read_text(encoding="utf-8"))
    for key in ("dnat", "snat", "dnat_ext", "snat_ext"):
        setattr(elm_module, key, tables["projects"]["All"][key])
    options.log = "compat"
    for kind in ("elm", "ecu"):
        (Path(get_logs_dir()) / f"{kind}_compat.txt").unlink(missing_ok=True)
    record("wire log", str(Path(get_logs_dir()) / "elm_compat.txt"))

    elm = ELM(plan["port"], 38400, "STD_WIFI" if plan["tcp"] else "STD_USB")
    options.elm = elm
    record("connect", elm.connectionStatus)
    if not elm.connectionStatus:
        return
    for ecu in plan["ecus"]:
        elm.init_can()
        ids = elm.set_can_addr(ecu["addr"], {"ecuname": "SCAN", **ecu["ecu"]})
        record(f"{ecu['name']} set_can_addr", list(ids) if ids else ids)
        for label, method, req in ecu["reads"]:
            got = elm.request(req, cache=False) if method == "request" else elm.cmd(req)
            record(f"{ecu['name']} {label}", got.strip())


def client_pyren(plan: dict, record: Callable[[str, object], None]) -> None:
    sys.path.insert(0, str(Path(plan["source"]) / "pyren3"))
    import mod_elm
    import mod_globals

    version = re.search(r"version:\s*(\S+)", mod_elm.__doc__ or "")
    record("version", f"pyren mod_elm {version.group(1) if version else '?'}")
    if "port" not in plan:
        return
    mod_globals.opt_port = plan["port"]
    mod_globals.opt_speed = 38400
    mod_globals.opt_log = "pyren.txt"
    Path("logs").mkdir(exist_ok=True)
    for kind in ("elm", "ecu"):
        Path("logs", f"{kind}_pyren.txt").unlink(missing_ok=True)
    record("wire log", str(Path("logs", "elm_pyren.txt").resolve()))

    elm = mod_elm.ELM(mod_globals.opt_port, mod_globals.opt_speed, mod_globals.opt_log)
    record("connect", True)
    for ecu in plan["ecus"]:
        elm.init_can()
        ecu_args = dict(ecu["ecu"])
        elm.set_can_addr(ecu["addr"], ecu_args)
        record(f"{ecu['name']} set_can_addr", ecu_args)
        for label, method, req in ecu["reads"]:
            if method == "start_session":
                elm.start_session(req)
                got = elm.lastinitrsp
            elif method == "request":
                got = elm.request(req, cache=False)
            else:
                got = elm.cmd(req)
            record(f"{ecu['name']} {label}", got.strip())


class AdapterError(Exception):
    pass


class Adapter:
    """Raw ELM327 conversation: one command, the lines up to the `>` prompt, echo removed."""

    def __init__(self, path: str):
        import serial

        self.port = serial.serial_for_url(path, baudrate=38400, timeout=0.05)

    def __enter__(self) -> Adapter:
        return self

    def __exit__(self, *exc: object) -> None:
        self.port.close()

    def send(self, cmd: str, timeout: float = RAW_TIMEOUT) -> list[str]:
        self.port.reset_input_buffer()
        self.port.write(cmd.encode("ascii") + b"\r")
        buf = bytearray()
        deadline = time.monotonic() + timeout
        while b">" not in buf:
            if time.monotonic() > deadline:
                raise AdapterError(f"no '>' within {timeout:g} s after {cmd!r} (received {bytes(buf)!r})")
            buf += self.port.read(self.port.in_waiting or 1)
        text = buf[:buf.index(b">")].decode("ascii", "replace")
        lines = [line.strip() for line in re.split(r"[\r\n]", text) if line.strip()]
        if lines and lines[0].replace(" ", "").upper() == cmd.replace(" ", "").upper():
            lines = lines[1:]
        return lines

    def setup(self, *cmds: str) -> None:
        for cmd in cmds:
            lines = self.send(cmd)
            if lines != ["OK"]:
                raise AdapterError(f"{cmd} answered {lines!r}, expected ['OK']")

    def reset(self) -> None:
        """ATZ, then wait until the line is quiet: the client that closed the port just before
        wrote its own ATZ on the way out, and that answer may still be in flight."""
        self.send("ATZ")
        quiet_from = time.monotonic() + QUIET_TIME
        while time.monotonic() < quiet_from:
            if self.port.read(self.port.in_waiting or 1):
                quiet_from = time.monotonic() + QUIET_TIME


def reset_adapter(path: str) -> str:
    """Power-up settings, and no previous command left for a bare CR (pyren's first probe)
    to repeat. Returns the problem, '' if none."""
    try:
        with Adapter(path) as elm:
            elm.reset()
    except (AdapterError, OSError) as exc:
        return str(exc)
    return ""


def caf1_payload(lines: list[str]) -> bytes:
    """The ISO-TP payload of a CAF1 + H0 answer: one single-frame line, or `LLL` + `N:` lines."""
    hexits = ""
    for line in lines:
        if ":" in line:
            hexits += line.split(":", 1)[1]
        elif len(line.replace(" ", "")) > 3:
            hexits += line
    try:
        return bytes.fromhex(hexits)
    except ValueError:
        return b""


def vin_of(lines: list[str]) -> str:
    """The VIN in a CAF1 + H0 answer to 0902 (`49 02 01` + 17 characters), else the raw lines."""
    data = caf1_payload(lines)
    if data[:3] != b"\x49\x02\x01":
        return repr(lines)
    return data[3:].rstrip(b"\x00").decode("ascii", "replace")


def check_guard(port: str, results: Results) -> None:
    with Adapter(port) as elm:
        elm.setup("ATE0", "ATSP6")
        # This check raises the tier and sends DTC-clear requests: only ever to the virtual car.
        if not results.check("guard target is the virtual car (OBD VIN)", VIN, vin_of(elm.send("0902"))):
            note("guard: not sending clear requests to an unknown car")
            return
        results.check("guard AT#GUARD READ", ["OK"], elm.send("AT#GUARD READ"))
        results.check("guard AT#GUARD prints the tier", ["READ"], elm.send("AT#GUARD"))

        # Aimed at the UCH, which answers 14 FF 00 with 54 FF 00: any transmission would show.
        elm.setup("ATSH745", "ATCRA765", "ATCAF0")
        results.check("guard CAF0 0314FF00 at READ", ["BLOCKED"], elm.send("0314FF00"))
        elm.setup("ATCAF1")
        results.check("guard CAF1 14FF00 at READ", ["BLOCKED"], elm.send("14FF00"))
        try:
            results.check("guard AT#GUARD CLEAR", ["OK"], elm.send("AT#GUARD CLEAR"))
            results.check("guard AT#GUARD prints CLEAR", ["CLEAR"], elm.send("AT#GUARD"))
            elm.setup("ATSH7E0", "ATCRA7E8")
            results.check("guard ECM 14FFFFFF at CLEAR", ["54"], elm.send("14FFFFFF"))
        finally:
            elm.send("AT#GUARD READ")
        results.check("guard back at READ", ["READ"], elm.send("AT#GUARD"))


def check_guard_tcp(port: str, results: Results) -> None:
    with Adapter(port) as elm:
        elm.setup("ATE0", "AT#GUARD READ")
        try:
            results.check("guard WRITE refused over TCP: AT#GUARD WRITE", ["?"], elm.send("AT#GUARD WRITE"))
            results.check("guard WRITE refused over TCP: AT#GUARD still READ", ["READ"], elm.send("AT#GUARD"))
        finally:
            elm.send("AT#GUARD READ")


def check_bitrate(port: str, results: Results) -> None:
    with Adapter(port) as elm:
        elm.setup("ATE0", "ATS0", "ATCAF0", "ATSH745", "AT SP 8", "ATCRA765")
        results.check("bitrate AT SP 8 then 0210C0 on the 500k car", ["CAN ERROR"], elm.send("0210C0"))
        elm.setup("AT SP 6", "ATCRA765")
        results.check("bitrate AT SP 6 then 0210C0 (UCH answers)", ["0250C00000000000"], elm.send("0210C0"))


def raw_check(name: str, check: Callable[[str, Results], None]) -> Callable[[str, Results, dict], None]:
    def run(port: str, results: Results, sources: dict) -> None:
        try:
            check(port, results)
        except (AdapterError, OSError) as exc:
            results.fail(f"{name} exchange", "an answer to every command", str(exc))
    return run


def compare_ecus(client: str, ecus: tuple[Ecu, ...], recorded: dict, results: Results) -> bool:
    ok = results.check(f"{client} connects", True, recorded.get("connect", MISSING))
    for ecu in ecus:
        if ecu.connected is not None:
            ok &= results.check(f"{client} {ecu.name} set_can_addr", ecu.connected,
                                recorded.get(f"{ecu.name} set_can_addr", MISSING))
        for read in ecu.reads:
            key = f"{ecu.name} {read.label}"
            ok &= results.check(f"{client} {key}", read.expected, recorded.get(key, MISSING))
    return ok


def finish_client(name: str, recorded: dict, problem: str, ok: bool, results: Results) -> None:
    if problem:
        results.fail(f"{name} process", "a clean exit", problem)
    if problem or not ok:
        client_logs(name, recorded)


def tcp_address(port: str) -> Optional[str]:
    """`HOST:PORT` of a `socket://HOST:PORT` port, the form ddt4all and pyren take for WiFi; None for serial."""
    return port[len(SOCKET):] if port.startswith(SOCKET) else None


def check_ecu_client(name: str, ecus: tuple[Ecu, ...]) -> Callable[[str, Results, dict], None]:
    def run(port: str, results: Results, sources: dict) -> None:
        address = tcp_address(port)
        plan = {"port": address or port, "tcp": address is not None, "source": str(sources[name]),
                "ecus": [ecu.plan() for ecu in ecus]}
        recorded, problem = run_client(name, plan)
        finish_client(name, recorded, problem, compare_ecus(name, ecus, recorded, results), results)
    return run


def check_python_obd(port: str, results: Results, sources: dict) -> None:
    recorded, problem = run_client("python-obd", {"port": port})
    ok = True
    for key, label, expected in PYTHON_OBD_EXPECTED:
        ok &= results.check(f"python-obd {label}", expected, recorded.get(key, MISSING))
    finish_client("python-obd", recorded, problem, ok, results)


def check_canscan(port: str, results: Results, sources: dict) -> None:
    report_path = LOG_DIR / "canscan.json"
    report_path.unlink(missing_ok=True)
    status = run_bounded([uv(), "run", str(REPO / "tools" / "canscan.py"), port, "--json", str(report_path)],
                         CANSCAN_TIMEOUT, LOG_DIR / "canscan.out", cwd=REPO)
    ok = results.check("canscan exit status", 0, f"killed after {CANSCAN_TIMEOUT:g} s" if status is None else status)
    if not report_path.exists():
        results.fail("canscan report", f"JSON in {report_path}", "no file")
        note(f"canscan output: {LOG_DIR / 'canscan.out'}")
        return
    report = json.loads(report_path.read_text())

    ecus = {ecu["tx"]: ecu for ecu in report["ecus"]}
    ok &= results.check("canscan Renault modules found", ["740", "745", "752", "7E0"], sorted(ecus))
    for tx, name, protocol, dtcs in (("7E0", "ECM", "UDS", ["03351C", "011564"]),
                                     ("745", "UCH", "KWP", ["9323"]),
                                     ("740", "ABS", "KWP", []),
                                     ("752", "airbag", "UDS", ["9A0111"])):
        ecu = ecus.get(tx, {})
        found = ecu.get("dtcs")
        ok &= results.check(f"canscan {name} protocol", protocol, ecu.get("protocol"))
        ok &= results.check(f"canscan {name} DTCs", dtcs, None if found is None else [d["raw"] for d in found])
    ok &= results.check("canscan skips 622 (live traffic)", True,
                        any(entry.startswith("622->") for entry in report["skipped_ecus"]))

    obd = {entry["id"]: entry for entry in report["obd"]}
    engine = obd.get("7E8", {})
    ok &= results.check("canscan OBD responders", ["7E8"], sorted(obd))
    ok &= results.check("canscan OBD VIN", VIN, engine.get("vin"))
    ok &= results.check("canscan OBD stored DTCs", ["P0335", "P0115"],
                        [d["code"] for d in engine.get("dtcs", {}).get("stored", [])])
    ok &= results.check("canscan OBD MIL", True, engine.get("mil"))
    if not ok:
        note(f"canscan output: {LOG_DIR / 'canscan.out'}, report: {report_path}")


RUNNERS: dict[str, Callable[[str, Results, dict], None]] = {
    "python-obd": check_python_obd,
    "ddt4all": check_ecu_client("ddt4all", DDT4ALL_ECUS),
    "pyren": check_ecu_client("pyren", PYREN_ECUS),
    "guard": raw_check("guard", check_guard),
    "guard-tcp": raw_check("guard-tcp", check_guard_tcp),
    "bitrate": raw_check("bitrate", check_bitrate),
    "canscan": check_canscan,
}


def run_checks(port: str, selected: list[str], sources: dict, results: Results,
               problem: Callable[[], str]) -> None:
    for name in CHECKS:
        if name not in selected:
            continue
        if name in SOURCES and name not in sources:
            continue
        down = problem()
        if down:
            results.fail(f"{name} (adapter)", "a running simulator", down)
            return
        trouble = reset_adapter(port)
        if trouble:
            results.fail(f"{name} ATZ before the check", "a '>' prompt", trouble)
            continue
        RUNNERS[name](port, results, sources)


def self_check(results: Results) -> None:
    sources = fetch_sources(list(SOURCES), results)
    for name in CLIENTS:
        if name in SOURCES and name not in sources:
            continue
        plan = {"source": str(sources[name])} if name in sources else {}
        recorded, problem = run_client(name, plan)
        if "version" in recorded and not problem:
            where = f" at {SOURCES[name].sha[:12]}" if name in SOURCES else ""
            results.ok(f"{name} import: {recorded['version']}{where}, Python {recorded['python']}")
        else:
            results.fail(f"{name} import", "a version", problem or "no version recorded")
            client_logs(name, recorded)


class SimulatorError(Exception):
    pass


def free_tcp_port() -> int:
    with socket.socket() as probe:
        probe.bind(("127.0.0.1", 0))
        return probe.getsockname()[1]


def wait_ready(proc: subprocess.Popen, timeout: float) -> str:
    fd = proc.stdout.fileno()
    buf = b""
    deadline = time.monotonic() + timeout
    while b"\n" not in buf:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise SimulatorError(f"no 'ready PATH' line within {timeout:g} s (stdout {buf!r})")
        if select.select([fd], [], [], remaining)[0]:
            chunk = os.read(fd, 4096)
            if not chunk:
                raise SimulatorError(f"exited with status {proc.wait()} before 'ready PATH'")
            buf += chunk
    line = buf.split(b"\n", 1)[0].decode("utf-8", "replace").strip()
    if not line.startswith("ready "):
        raise SimulatorError(f"first line {line!r}")
    return line[len("ready "):].strip()


def ready_port(ready: str, tcp: bool) -> str:
    """The port to check from the ready line's `PATH`, or `PATH tcp HOST:PORT` with --tcp."""
    if not tcp:
        return ready
    _, sep, address = ready.rpartition(" tcp ")
    if not sep or not address.strip():
        raise SimulatorError(f"ready line {'ready ' + ready!r} has no 'tcp HOST:PORT'")
    return SOCKET + address.strip()


@contextlib.contextmanager
def simulator(binary: str, tcp: bool) -> Iterator[tuple[str, Callable[[], str]]]:
    link = Path(tempfile.gettempdir()) / f"elm327-compat-{os.getpid()}"
    cmd = [binary, "--link", str(link)]
    if tcp:
        cmd += ["--tcp", f"127.0.0.1:{free_tcp_port()}"]
    try:
        proc = subprocess.Popen(cmd, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE)
    except OSError as exc:
        raise SimulatorError(str(exc)) from exc
    try:
        port = ready_port(wait_ready(proc, SIM_READY_TIMEOUT), tcp)
        threading.Thread(target=shutil.copyfileobj, args=(proc.stdout, open(os.devnull, "wb")), daemon=True).start()

        def problem() -> str:
            status = proc.poll()
            return "" if status is None else f"simulator exited with status {status}"

        yield port, problem
    finally:
        if proc.poll() is None:
            proc.terminate()
            try:
                proc.wait(5)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait()


def check_list(text: str) -> list[str]:
    names = [part.strip() for part in text.split(",") if part.strip()]
    unknown = [name for name in names if name not in CHECKS]
    if unknown:
        raise argparse.ArgumentTypeError(f"unknown check(s) {', '.join(unknown)}; choose from {', '.join(CHECKS)}")
    return names


def parse_args(argv: Optional[list[str]]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter,
                                     epilog="exit status: 0 every check passed, 1 otherwise")
    where = parser.add_mutually_exclusive_group()
    where.add_argument("--port", help="serial port of a device serving the virtual car, e.g. /dev/ttyACM0, "
                                      "or its WiFi as socket://HOST:PORT, e.g. socket://192.168.0.10:35000")
    where.add_argument("--sim", metavar="PROGRAM", help="simulator binary to start, e.g. .pio/build/sim/program")
    where.add_argument("--self-check", action="store_true",
                       help="only fetch and import the three clients and print their versions")
    parser.add_argument("--tcp", action="store_true",
                        help="with --sim: also serve the car on a local TCP port and run every check over TCP")
    parser.add_argument("--only", type=check_list, metavar="CHECKS",
                        help=f"comma-separated subset of {','.join(CHECKS)} "
                             f"(default: all; {','.join(TCP_ONLY)} only over TCP)")
    parser.add_argument("--client", nargs=3, metavar=("NAME", "PLAN", "OUT"), help=argparse.SUPPRESS)
    args = parser.parse_args(argv)
    if not (args.client or args.self_check or args.port or args.sim):
        parser.error("one of --port, --sim or --self-check is required")
    if args.tcp and not args.sim:
        parser.error("--tcp needs --sim; for a board over WiFi use --port socket://HOST:PORT")
    tcp = args.tcp or (args.port or "").startswith(SOCKET)
    if args.only is None:
        args.only = [name for name in CHECKS if tcp or name not in TCP_ONLY]
    elif not tcp and set(TCP_ONLY) & set(args.only):
        parser.error(f"{','.join(TCP_ONLY)} runs only over TCP (--sim PROGRAM --tcp or --port socket://HOST:PORT)")
    return args


def main(argv: Optional[list[str]] = None) -> int:
    args = parse_args(argv)
    if args.client:
        return client_main(*args.client)
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    results = Results()
    if args.self_check:
        self_check(results)
        return results.summary()

    sources = fetch_sources([name for name in SOURCES if name in args.only], results)
    if args.port:
        run_checks(args.port, args.only, sources, results, lambda: "")
        return results.summary()
    try:
        with simulator(args.sim, args.tcp) as (port, problem):
            run_checks(port, args.only, sources, results, problem)
    except SimulatorError as exc:
        expected = ("'ready PATH tcp HOST:PORT' from {} --link ... --tcp ..." if args.tcp
                    else "'ready PATH' from {} --link ...")
        results.fail("simulator", expected.format(args.sim), str(exc))
    return results.summary()


if __name__ == "__main__":
    sys.exit(main())

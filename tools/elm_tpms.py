#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.10"
# dependencies = ["pyserial>=3.5"]
# ///
"""Hunt for the 4Runner's tire pressures over OBD through an ELM327.

The requests come from ScanGauge's "Toyota TPMS Method 1" X-Gauges (ISO 14230 /
KWP2000): header 82 2A F0 (TPMS ECU 0x2A, tester 0xF0), service 0x21
readDataByLocalIdentifier. Tire N is one byte at reply data offset N+1 (right
after the 61 <lid> echo). Every exchange is logged to captures/, which is
gitignored because replies may carry sensor IDs.

  tools/elm_tpms.py probe        adapter info, battery voltage
  tools/elm_tpms.py obd          sanity check: 0100 on the engine ECU, auto protocol
  tools/elm_tpms.py tpms         read pressure sets A/B/C and temperatures once
  tools/elm_tpms.py poll         re-read one pressure set every few seconds (--lid)
  tools/elm_tpms.py scan         try every local ID 0x00-0xFF on the TPMS ECU
  tools/elm_tpms.py repl         type raw AT/hex commands

The CAN modes look for the pressures in the modules on the OBD port's 500k bus
instead. They speak Toyota's KWP2000-over-CAN: physical modules answer on their
request ID + 8, and modules behind the 0x750 gateway take an extended address
as the first data byte and answer on 0x758. Module specs are "7C4" or "750/40".

  tools/elm_tpms.py can-find     find modules: tester present to 700-7F7 and 750/00-FF
  tools/elm_tpms.py can-sweep    try service 21 local IDs 00-FF on each module (--modules)
  tools/elm_tpms.py snap NAME    read every local ID can-sweep found into captures/snap-NAME.json
  tools/elm_tpms.py diff A B ... compare snapshots: snap twice, change one tire, snap again,
                                 then diff base1 base2 after (IDs that moved between the
                                 first two are reported as noise)

The cheap ELM327 v1.4 clone tested on 2026-10-27 needed workarounds that these
modes use for every adapter: frames are built by hand (ATCAF0), because it
accepts ATCEA but never sends the address; it rejects hand-built frames with
"?" until a normal request (0100) has gone out; and its CAN receive filters
(ATCF/ATCM, ATCRA with X) are broken, so only exact ATCRA IDs are used.
"""

import argparse
import datetime
import glob
import json
import pathlib
import sys
import time

import serial

ECU = 0x2A
TESTER = 0xF0
BAUDS = (38400, 115200, 9600, 500000)

# Local IDs and math from the ScanGauge page. MTH is (x * mul / div + add) in the
# ScanGauge's display units; the pressure sets only make sense if those are
# tenths of a PSI, which is unconfirmed until checked against a tire gauge.
# A 2017 4Runner owner reports set B (0x38) is the one that works there.
SETS = {
    0x38: ("pressure B", lambda x: (x * 29 / 8 - 145) / 10, "psi?"),
    0x62: ("pressure A", lambda x: (x * 29 / 4 - 145) / 10, "psi?"),
    0x8A: ("pressure C", lambda x: (x * 79 / 40) / 10, "psi?"),
    0x48: ("temperature", lambda x: x * 9 / 5 - 40, "F"),
}
TIRES = 5

# Modules that answered can-find on the 2018 4Runner. None of them changed a
# byte the way a pressure would when driver front dropped from 34.8 to 30 psi,
# so the TPMS ECU isn't among them (or doesn't expose pressure via service 21).
MODULES = ["7C4", "7C6", "750/40", "750/DC", "780", "784", "7A1", "7B0", "7C0", "7C2"]
GATEWAY = 0x750

REPO = pathlib.Path(__file__).resolve().parent.parent
LOG_DIR = REPO / "captures"
LIDS_FILE = LOG_DIR / "can-lids.json"


class Elm:
    def __init__(self, port, baud, log):
        self.log = log
        self.ser = serial.Serial()
        self.ser.port = port
        self.ser.baudrate = baud
        self.ser.timeout = 0.2
        self.ser.dtr = False
        self.ser.rts = False
        self.ser.open()

    def cmd(self, text, timeout=5.0):
        """Send one line, return the reply lines up to the '>' prompt."""
        self.ser.reset_input_buffer()
        self.ser.write((text + "\r").encode())
        buf = b""
        deadline = time.monotonic() + timeout
        while b">" not in buf and time.monotonic() < deadline:
            buf += self.ser.read(256)
        lines = [ln.strip() for ln in buf.decode(errors="replace").replace(">", "").splitlines()]
        lines = [ln for ln in lines if ln and ln != text]
        self.log(f"> {text}")
        for ln in lines:
            self.log(f"< {ln}")
        if b">" not in buf:
            self.log("! no prompt (timeout)")
        return lines

    def setup(self):
        self.cmd("ATZ", timeout=3)
        for c in ("ATE0", "ATL0", "ATS1", "ATH1", "ATAT1"):
            self.cmd(c)

    def kline(self, protocol):
        """Point the adapter at the TPMS ECU over KWP2000."""
        self.cmd(f"ATSP{protocol}")
        self.cmd(f"ATSH 82 {ECU:02X} {TESTER:02X}")
        if protocol == 4:
            # 5-baud init addresses the ECU directly.
            self.cmd(f"ATIIA {ECU:02X}")


class Can:
    """KWP2000 over ISO-TP on the 500k bus, with frames built by hand."""

    def __init__(self, elm):
        self.elm = elm
        self.module = None
        elm.cmd("ATSP6")
        elm.cmd("0100", timeout=8)
        for c in ("ATCAF0", "ATST 10", "ATAT0"):
            elm.cmd(c)

    def select(self, module):
        if module == self.module:
            return
        rid, ea = parse_module(module)
        self.elm.cmd(f"ATSH {rid:03X}")
        self.elm.cmd(f"ATCRA {rid + 8:03X}")
        self.module = module

    def request(self, module, payload, timeout=1.5):
        """Send one single-frame request, return the reassembled reply bytes or None."""
        self.select(module)
        _, ea = parse_module(module)
        pre = [] if ea is None else [ea]
        lines = self.elm.cmd(frame(pre + [len(payload)] + payload), timeout=timeout)
        rows = reply_rows(lines, len(pre))
        if not rows:
            return None
        if rows[0][0] >> 4 == 1 and len(rows) == 1:
            # First frame only: the adapter sent no flow control (it never does
            # for extended addresses), so send it by hand.
            lines = self.elm.cmd(frame(pre + [0x30, 0x00, 0x00]), timeout=timeout)
            rows += reply_rows(lines, len(pre))
        if rows[0][0] >> 4 == 1:
            n = (rows[0][0] & 0x0F) << 8 | rows[0][1]
            data = rows[0][2:]
            for row in rows[1:]:
                data += row[1:]
            return data[:n]
        return rows[0][1:1 + rows[0][0]]


def parse_module(spec):
    rid, _, ea = spec.partition("/")
    return int(rid, 16), int(ea, 16) if ea else None


def frame(data):
    return " ".join(f"{x:02X}" for x in data + [0] * (8 - len(data)))


def reply_rows(lines, skip):
    """CAN reply lines -> data bytes after the ID (and extended address), skipping junk."""
    rows = []
    for ln in lines:
        try:
            b = [int(t, 16) for t in ln.split()[1:]]
        except ValueError:
            continue
        if len(b) > skip:
            rows.append(b[skip:])
    return rows


def hexs(data):
    return " ".join(f"{x:02X}" for x in data)


def load_snap(name):
    path = LOG_DIR / f"snap-{name}.json"
    if not path.exists():
        sys.exit(f"no {path.relative_to(REPO)}; run snap {name} first")
    return json.loads(path.read_text())


def diff(names):
    if len(names) < 2:
        sys.exit("diff needs at least two snapshot names")
    snaps = [load_snap(n) for n in names]
    base, after = snaps[:-1], snaps[-1]
    noisy = {k for k in base[0] if any(s.get(k) != base[0][k] for s in base[1:])}
    moved = [k for k in base[0] if k not in noisy and after.get(k) != base[0][k]]
    print(f"== stable in {', '.join(names[:-1])}, changed in {names[-1]}:")
    for k in moved:
        print(f"{k}\n  before: {base[0][k]}\n  after:  {after.get(k)}")
    if noisy:
        print("== noisy (changed between baselines):")
        for k in sorted(noisy):
            print(f"{k}: " + " | ".join(str(s.get(k)) for s in snaps))


def find_port():
    ports = sorted(glob.glob("/dev/cu.usbserial*"))
    if not ports:
        sys.exit("no /dev/cu.usbserial* port; is the adapter plugged in?")
    return ports[0]


def open_elm(args, log):
    port = args.port or find_port()
    for baud in [args.baud] if args.baud else BAUDS:
        elm = Elm(port, baud, log)
        if any("ELM" in ln for ln in elm.cmd("ATI", timeout=2)):
            log(f"# {port} @ {baud}")
            elm.setup()
            return elm
        elm.ser.close()
    sys.exit(f"no ELM327 answer on {port} at {BAUDS}")


def parse_kwp(line, lid):
    """Pull the data bytes after '61 <lid>' out of one ATH1 reply line, or None."""
    try:
        b = [int(t, 16) for t in line.split()]
    except ValueError:
        return None
    if len(b) < 5 or b[1] != TESTER or b[2] != ECU:
        return None
    n = b[0] & 0x3F
    if n:
        data = b[3:3 + n]
    else:
        # Length didn't fit the format byte; it's the next byte.
        n = b[3]
        data = b[4:4 + n]
    if len(data) < 2 or data[0] != 0x61 or data[1] != lid:
        return None
    return data[2:]


def read_set(elm, lid):
    for line in elm.cmd(f"21 {lid:02X}", timeout=8):
        data = parse_kwp(line, lid)
        if data is not None:
            return data
    return None


def show(lid, data, log):
    name, conv, unit = SETS[lid]
    raw = " ".join(f"{x:02X}" for x in data)
    log(f"# 21 {lid:02X} {name}: {raw}")
    for i, x in enumerate(data[:TIRES]):
        log(f"#   tire {i + 1}: raw {x:3d} (0x{x:02X}) -> {conv(x):6.1f} {unit}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("mode", choices=["probe", "obd", "tpms", "poll", "scan", "repl",
                                     "can-find", "can-sweep", "snap", "diff"])
    ap.add_argument("names", nargs="*", help="snap: snapshot name; diff: snapshot names")
    ap.add_argument("-p", "--port", help="serial port (default: first /dev/cu.usbserial*)")
    ap.add_argument("-b", "--baud", type=int, help=f"baud (default: try {BAUDS})")
    ap.add_argument("--protocol", type=int, choices=[4, 5], default=5,
                    help="ELM protocol: 5 = KWP fast init (default), 4 = KWP 5-baud init")
    ap.add_argument("--interval", type=float, default=5.0, help="poll period, seconds")
    ap.add_argument("--lid", type=lambda v: int(v, 16), default=0x38, choices=list(SETS),
                    metavar="HEX", help="local ID for poll (default 38)")
    ap.add_argument("--modules", type=lambda v: v.upper().split(","), default=MODULES,
                    help="can-sweep modules, comma separated (default: the ones this truck answers on)")
    args = ap.parse_args()

    if args.mode == "diff":
        return diff(args.names)
    if args.mode == "snap" and len(args.names) != 1:
        sys.exit("snap needs one name, e.g. snap base1")

    LOG_DIR.mkdir(exist_ok=True)
    stamp = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
    log_path = LOG_DIR / f"elm-{args.mode}-{stamp}.log"
    log_file = log_path.open("w")

    def log(msg):
        line = f"{datetime.datetime.now():%H:%M:%S.%f}"[:-3] + " " + msg
        print(line, flush=True)
        log_file.write(line + "\n")
        log_file.flush()

    log(f"# logging to {log_path.relative_to(REPO)}")
    elm = open_elm(args, log)

    try:
        if args.mode == "probe":
            for c in ("ATI", "AT@1", "STI", "ATRV", "ATDP"):
                elm.cmd(c)
        elif args.mode == "obd":
            elm.cmd("ATSP0")
            elm.cmd("0100", timeout=15)
            elm.cmd("ATDP")
            elm.cmd("ATDPN")
        elif args.mode == "tpms":
            elm.kline(args.protocol)
            for lid in SETS:
                data = read_set(elm, lid)
                if data is None:
                    log(f"# 21 {lid:02X}: no usable reply")
                else:
                    show(lid, data, log)
        elif args.mode == "poll":
            elm.kline(args.protocol)
            while True:
                data = read_set(elm, args.lid)
                if data is None:
                    log(f"# 21 {args.lid:02X}: no usable reply")
                else:
                    show(args.lid, data, log)
                time.sleep(args.interval)
        elif args.mode == "scan":
            elm.kline(args.protocol)
            for lid in range(0x100):
                lines = elm.cmd(f"21 {lid:02X}", timeout=4)
                if any(parse_kwp(ln, lid) is not None for ln in lines):
                    log(f"# lid 0x{lid:02X} answered")
        elif args.mode == "can-find":
            can = Can(elm)
            for rid in range(0x700, 0x7F8):
                if rid & 0x0F >= 8:
                    continue
                if can.request(f"{rid:03X}", [0x3E, 0x00]) is not None:
                    log(f"# module {rid:03X} answered")
            for ea in range(0x100):
                # KWP tester present, no sub-function, as Toyota's gateway expects.
                if can.request(f"{GATEWAY:03X}/{ea:02X}", [0x3E]) is not None:
                    log(f"# module {GATEWAY:03X}/{ea:02X} answered")
        elif args.mode == "can-sweep":
            can = Can(elm)
            found = json.loads(LIDS_FILE.read_text()) if LIDS_FILE.exists() else {}
            for module in args.modules:
                lids = []
                for lid in range(0x100):
                    data = can.request(module, [0x21, lid])
                    if data and data[0] == 0x61:
                        lids.append(lid)
                        log(f"# {module} 21 {lid:02X}: {hexs(data)}")
                log(f"# {module}: {len(lids)} local IDs answered")
                found[module] = lids
                LIDS_FILE.write_text(json.dumps(found, indent=1))
            log(f"# local IDs saved to {LIDS_FILE.relative_to(REPO)}")
        elif args.mode == "snap":
            if not LIDS_FILE.exists():
                sys.exit(f"no {LIDS_FILE.relative_to(REPO)}; run can-sweep first")
            can = Can(elm)
            snap = {}
            for module, lids in json.loads(LIDS_FILE.read_text()).items():
                for lid in lids:
                    data = can.request(module, [0x21, lid])
                    if data:
                        snap[f"{module} 21 {lid:02X}"] = hexs(data)
            path = LOG_DIR / f"snap-{args.names[0]}.json"
            path.write_text(json.dumps(snap, indent=0))
            log(f"# {len(snap)} local IDs read -> {path.relative_to(REPO)}")
        elif args.mode == "repl":
            elm.kline(args.protocol)
            print("raw commands, blank line to quit")
            while line := input("elm> ").strip():
                elm.cmd(line, timeout=8)
    except KeyboardInterrupt:
        pass
    finally:
        elm.ser.close()
        log_file.close()


if __name__ == "__main__":
    main()

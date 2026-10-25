#!/usr/bin/env python3
"""Live viewer for the RTK GPS passed through the Nucleo-F303K8 (ST-Link VCP).

Parses NMEA (GGA/RMC/GSA/GSV/ZDA + $PHDANT antenna status) and counts any RTCM3
frames mixed into the stream. Bad-checksum sentences are counted, not shown.

RTK corrections: an NTRIP client (same caster as AMP_ASS_v8/parts/gps.py) pulls
RTCM3 from the InDOT network and writes it back down the serial port; the Nucleo
forwards it to the receiver. The receiver's GGA is sent to the caster every
second so *_NEAR / VRS mountpoints can pick a base station.

    python3 read_gps.py                    # live dashboard + RTK corrections
    python3 read_gps.py --approx 40.4237,-86.9212   # get corrections before the first fix
    python3 read_gps.py --no-ntrip         # receiver output only
    python3 read_gps.py --raw              # print every valid NMEA sentence
    python3 read_gps.py --log capture.bin  # also save the raw byte stream
"""
import argparse
import base64
import socket
import sys
import threading
import time
import urllib.parse
from collections import Counter

import serial

FIX_QUALITY = {
    0: "NO FIX", 1: "GPS (SPS)", 2: "DGPS", 3: "PPS", 4: "RTK FIXED",
    5: "RTK FLOAT", 6: "DEAD RECKONING", 7: "MANUAL", 8: "SIMULATION",
}
NTRIP_URL = "108.59.49.226:9000/MSM4_NEAR"  # InDOT; MSM4 = GPS+GLO+GAL+BDS
NTRIP_USER = NTRIP_PASSWORD = "automp1"
GSV_SYSTEM = {"GP": "GPS", "GL": "GLONASS", "GA": "Galileo", "GB": "BeiDou",
              "BD": "BeiDou", "GQ": "QZSS", "GI": "NavIC"}


def nmea_checksum_ok(sentence: bytes) -> bool:
    try:
        body, cs = sentence[1:].split(b"*", 1)
        calc = 0
        for b in body:
            calc ^= b
        return calc == int(cs[:2], 16)
    except ValueError:
        return False


def dm_to_deg(value: str, hemi: str):
    """NMEA ddmm.mmmm / dddmm.mmmm -> signed decimal degrees."""
    if not value:
        return None
    dot = value.index(".")
    deg = float(value[:dot - 2]) + float(value[dot - 2:]) / 60.0
    return -deg if hemi in ("S", "W") else deg


def make_gga(lat: float, lon: float, alt: float = 0.0) -> bytes:
    """Minimal GGA sentence for telling the caster roughly where we are."""
    def dm(v, width):
        a = abs(v)
        return f"{int(a):0{width}d}{(a - int(a)) * 60:07.4f}"
    body = (f"GPGGA,{time.strftime('%H%M%S', time.gmtime())}.00,"
            f"{dm(lat, 2)},{'N' if lat >= 0 else 'S'},{dm(lon, 3)},{'E' if lon >= 0 else 'W'},"
            f"1,10,1.0,{alt:.1f},M,0.0,M,,")
    cs = 0
    for b in body.encode():
        cs ^= b
    return f"${body}*{cs:02X}\r\n".encode()


def rtcm_type(frame: bytes) -> int:
    return (frame[3] << 4) | (frame[4] >> 4)


def crc24q(data: bytes) -> int:
    crc = 0
    for b in data:
        crc ^= b << 16
        for _ in range(8):
            crc <<= 1
            if crc & 0x1000000:
                crc ^= 0x1864CFB
    return crc & 0xFFFFFF


class StreamParser:
    """Splits a mixed NMEA / RTCM3 byte stream into frames."""

    def __init__(self):
        self.buf = bytearray()

    def feed(self, data: bytes):
        self.buf += data
        while self.buf:
            start = self.buf[0]
            if start == ord("$"):
                end = self.buf.find(b"\n")
                if end < 0:
                    if len(self.buf) > 512:  # runaway line, resync
                        del self.buf[0]
                        continue
                    return
                line = bytes(self.buf[:end]).rstrip(b"\r")
                del self.buf[:end + 1]
                yield ("nmea", line)
            elif start == 0xD3:  # RTCM3 preamble
                if len(self.buf) < 3:
                    return
                length = ((self.buf[1] & 0x03) << 8) | self.buf[2]
                if len(self.buf) < length + 6:
                    return
                frame = bytes(self.buf[:length + 6])
                if crc24q(frame[:-3]) == int.from_bytes(frame[-3:], "big"):
                    del self.buf[:length + 6]
                    yield ("rtcm", frame)
                else:
                    del self.buf[0]
            else:
                # skip to the next plausible frame start
                nxt = min((i for i in (self.buf.find(b"$", 1), self.buf.find(b"\xd3", 1)) if i > 0),
                          default=len(self.buf))
                yield ("junk", bytes(self.buf[:nxt]))
                del self.buf[:nxt]


class GpsState:
    def __init__(self):
        self.time = self.date = None
        self.lat = self.lon = self.alt = None
        self.quality = 0
        self.sats_used = 0
        self.hdop = self.pdop = self.vdop = None
        self.diff_age = self.diff_station = None
        self.speed_kn = self.course = None
        self.antenna = None
        self.gsv = {}          # (talker, signal_id) -> {prn: snr}
        self.gsv_pending = {}  # sats being accumulated across multi-part GSV
        self.counts = Counter()
        self.rtcm = Counter()
        self.bad_checksum = 0
        self.junk_bytes = 0
        self.last_gga = 0.0
        self.last_gga_sentence = None  # raw GGA, forwarded to the NTRIP caster

    def handle_nmea(self, line: bytes):
        if not nmea_checksum_ok(line):
            self.bad_checksum += 1
            return
        text = line[1:line.index(b"*")].decode("ascii", "replace")
        f = text.split(",")
        talker, kind = f[0][:2], f[0][2:]
        self.counts[f[0]] += 1

        if kind == "GGA":
            self.time = f[1]
            self.lat, self.lon = dm_to_deg(f[2], f[3]), dm_to_deg(f[4], f[5])
            self.quality = int(f[6] or 0)
            self.sats_used = int(f[7] or 0)
            self.hdop = float(f[8]) if f[8] else None
            self.alt = float(f[9]) if f[9] else None
            self.diff_age = f[13] or None
            self.diff_station = f[14] or None
            self.last_gga = time.time()
            self.last_gga_sentence = line
        elif kind == "RMC":
            self.speed_kn = float(f[7]) if f[7] else None
            self.course = float(f[8]) if f[8] else None
            self.date = f[9]
        elif kind == "GSA":
            self.pdop = float(f[15]) if f[15] else self.pdop
            self.vdop = float(f[17]) if len(f) > 17 and f[17] else self.vdop
        elif kind == "GSV":
            total, num = int(f[1]), int(f[2])
            sats = f[4:]
            signal = sats.pop() if len(sats) % 4 == 1 else ""  # NMEA 4.10+ signal ID
            key = (talker, signal)
            pending = self.gsv_pending.setdefault(key, {})
            if num == 1:
                pending.clear()
            for i in range(0, len(sats) - 3, 4):
                prn, snr = sats[i], sats[i + 3]
                if prn:
                    pending[prn] = int(snr) if snr else None
            if num == total:
                self.gsv[key] = dict(pending)
                pending.clear()
        elif f[0] == "PHDANT":
            # Proprietary antenna status, e.g. PHDANT,01,02,02,ANT_OPEN,D1,ANT_SAT,LNA_LOGAIN,...
            self.antenna = next((x for x in f if x.startswith("ANT_") and x != "ANT_SAT"), None)

    def handle_rtcm(self, frame: bytes):
        self.rtcm[rtcm_type(frame)] += 1


class NtripClient(threading.Thread):
    """Fetches RTCM3 corrections from an NTRIP caster and writes them to the receiver.

    Like pygnssutils' GNSSNTRIPClient in AMP_ASS_v8/parts/gps.py (ggamode=0), it
    uploads the receiver's own GGA every gga_interval seconds. Before the receiver
    has a fix there's no position to send, so --approx can stand in for it.
    """

    def __init__(self, url, user, password, ser, state, gga_interval=1.0,
                 approx=None, gps_baud=230400, verbose=False):
        super().__init__(daemon=True)
        u = urllib.parse.urlsplit(url if "://" in url else "http://" + url)
        self.host, self.port, self.mountpoint = u.hostname, u.port or 2101, u.path.lstrip("/")
        self.auth = base64.b64encode(f"{user}:{password}".encode()).decode()
        self.ser, self.state = ser, state
        self.gga_interval = gga_interval
        self.approx = approx
        # Never push corrections faster than the GPS UART can drain them through
        # the Nucleo (PC side runs at 2x the GPS baud), with some margin.
        self.max_rate = gps_baud / 10 * 0.7
        self.next_write = 0.0
        self.verbose = verbose
        self.stop_event = threading.Event()
        self.parser = StreamParser()

        self.status = "starting"
        self.gga_source = None
        self.bytes_rx = 0
        self.last_rx = None
        self.rtcm = Counter()

    @property
    def name_str(self):
        return f"{self.host}:{self.port}/{self.mountpoint}"

    def _set_status(self, status):
        if status != self.status and self.verbose:
            print(f"[NTRIP] {status}", file=sys.stderr, flush=True)
        self.status = status

    def _gga(self):
        st = self.state
        if st.quality > 0 and st.last_gga_sentence:
            self.gga_source = "receiver"
            return st.last_gga_sentence + b"\r\n"
        if self.approx:
            self.gga_source = "--approx"
            return make_gga(*self.approx)
        self.gga_source = None
        return None

    def run(self):
        while not self.stop_event.is_set():
            # Don't connect until there's a position to send: a connection left
            # idle with no GGA gets silently dropped (caster or NAT) and never streams.
            if self._gga() is None:
                self._set_status("waiting for receiver fix to send GGA (or use --approx)")
                self.stop_event.wait(1)
                continue
            try:
                self._session()
            except (OSError, ConnectionError, ValueError) as e:
                self._set_status(f"error: {e} (retrying in 5 s)")
            self.stop_event.wait(5)

    def _session(self):
        self._set_status("connecting")
        with socket.create_connection((self.host, self.port), timeout=10) as sock:
            sock.sendall((
                f"GET /{self.mountpoint} HTTP/1.1\r\n"
                f"Host: {self.host}:{self.port}\r\n"
                "Ntrip-Version: Ntrip/2.0\r\n"
                "User-Agent: NTRIP gps-tester/1.0\r\n"
                f"Authorization: Basic {self.auth}\r\n"
                "Connection: close\r\n\r\n").encode())

            resp = b""
            while b"\r\n\r\n" not in resp:
                chunk = sock.recv(1024)
                if not chunk:
                    raise ConnectionError("caster closed connection")
                resp += chunk
            header, _, body = resp.partition(b"\r\n\r\n")
            status_line = header.split(b"\r\n")[0].decode(errors="replace")
            if " 200" not in status_line:  # "HTTP/1.1 200 OK" (v2) or "ICY 200 OK" (v1)
                raise ConnectionError(status_line)
            if b"gnss/sourcetable" in header.lower():
                raise ConnectionError(f"mountpoint '{self.mountpoint}' not on caster")
            chunked = b"transfer-encoding: chunked" in header.lower()

            self._set_status("connected, waiting for GGA position")
            sock.settimeout(0.5)
            pending = bytearray()
            next_gga = 0.0
            first_gga_at = None
            connected_at = time.monotonic()
            data = body
            while not self.stop_event.is_set():
                if data:
                    if chunked:
                        pending += data
                        data = self._dechunk(pending)
                    self._forward(data)

                now = time.monotonic()
                if now >= next_gga:
                    gga = self._gga()
                    if gga:
                        sock.sendall(gga)
                        first_gga_at = first_gga_at or now
                    next_gga = now + self.gga_interval
                if self.last_rx and self.last_rx > connected_at:
                    self._set_status("streaming corrections")
                    if now - self.last_rx > 10:
                        raise ConnectionError("no corrections for 10 s")
                elif first_gga_at:
                    self._set_status(f"connected, sent GGA ({self.gga_source}), waiting for data")
                    if now - first_gga_at > 15:
                        raise ConnectionError("caster sent nothing 15 s after GGA")
                else:
                    self._set_status("connected, waiting for receiver fix to send GGA (or use --approx)")

                try:
                    data = sock.recv(4096)
                except socket.timeout:
                    data = b""
                    continue
                if not data:
                    raise ConnectionError("caster closed connection")

    @staticmethod
    def _dechunk(buf: bytearray) -> bytes:
        out = bytearray()
        while (eol := buf.find(b"\r\n")) >= 0:
            size = int(buf[:eol].split(b";")[0], 16)
            if size == 0:
                raise ConnectionError("caster ended stream")
            if len(buf) < eol + 2 + size + 2:
                break
            out += buf[eol + 2:eol + 2 + size]
            del buf[:eol + 2 + size + 2]
        return bytes(out)

    def _forward(self, data: bytes):
        self.bytes_rx += len(data)
        self.last_rx = time.monotonic()
        for kind, frame in self.parser.feed(data):
            if kind == "rtcm":
                self.rtcm[rtcm_type(frame)] += 1
        for i in range(0, len(data), 256):
            chunk = data[i:i + 256]
            delay = self.next_write - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            self.ser.write(chunk)
            self.next_write = max(self.next_write, time.monotonic()) + len(chunk) / self.max_rate


def fmt(v, spec="", none="--"):
    return none if v is None else format(v, spec)


def render(st: GpsState, port: str, baud: int, ntrip: NtripClient = None) -> str:
    q = FIX_QUALITY.get(st.quality, str(st.quality))
    colour = {4: "\033[92m", 5: "\033[93m", 0: "\033[91m"}.get(st.quality, "\033[96m")
    t = st.time or ""
    utc = f"{t[0:2]}:{t[2:4]}:{t[4:]}" if len(t) >= 6 else "--"
    stale = time.time() - st.last_gga > 2

    by_system = {}
    for (talker, _sig), sats in st.gsv.items():
        sysname = GSV_SYSTEM.get(talker, talker)
        for prn, snr in sats.items():
            prev = by_system.setdefault(sysname, {}).get(prn)
            by_system[sysname][prn] = max(filter(None, (prev, snr)), default=None)

    lines = [
        f"RTK GPS on {port} @ {baud}" + ("   \033[91m[no GGA for >2s]\033[0m" if stale else ""),
        "-" * 60,
        f"UTC        {utc}   date {st.date or '--'}",
        f"Fix        {colour}{q}\033[0m  (quality {st.quality})",
        f"Lat/Lon    {fmt(st.lat, '.8f')}, {fmt(st.lon, '.8f')}",
        f"Alt (MSL)  {fmt(st.alt, '.3f')} m",
        f"Speed      {fmt(st.speed_kn and st.speed_kn * 0.514444, '.2f')} m/s   course {fmt(st.course, '.1f')}",
        f"Sats used  {st.sats_used}   HDOP {fmt(st.hdop)}  PDOP {fmt(st.pdop)}  VDOP {fmt(st.vdop)}",
        f"Corrections age {st.diff_age or '--'} s   base station {st.diff_station or '--'}",
        f"Antenna    {st.antenna or '--'}",
        "",
        "Satellites in view (PRN:SNR dB-Hz):",
    ]
    if by_system:
        for sysname, sats in sorted(by_system.items()):
            sat_txt = " ".join(f"{p}:{s if s is not None else '-'}" for p, s in sorted(sats.items()))
            lines.append(f"  {sysname:8} {len(sats):2}  {sat_txt}")
    else:
        lines.append("  none")
    lines.append("")
    if ntrip:
        age = f"{time.monotonic() - ntrip.last_rx:.1f} s ago" if ntrip.last_rx else "never"
        lines += [
            f"NTRIP      {ntrip.name_str}",
            f"           {ntrip.status}",
            f"           {ntrip.bytes_rx} B received, last {age}",
            "           RTCM in: " + (" ".join(f"{k}={v}" for k, v in sorted(ntrip.rtcm.items())) or "none"),
        ]
    else:
        lines.append("NTRIP      disabled")
    lines += [
        "",
        "NMEA: " + " ".join(f"{k}={v}" for k, v in sorted(st.counts.items())),
        "RTCM from receiver: " + (" ".join(f"{k}={v}" for k, v in sorted(st.rtcm.items())) or "none"),
        f"Bad checksums: {st.bad_checksum}   unparsed bytes: {st.junk_bytes}",
        "",
        "Ctrl-C to quit",
    ]
    return "\n".join(lines)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-p", "--port", default="/dev/ttyACM0")
    ap.add_argument("-b", "--baud", type=int, default=460800, help="PC-side baud (PC_BAUD in firmware)")
    ap.add_argument("--raw", action="store_true", help="print each NMEA sentence instead of the dashboard")
    ap.add_argument("--log", help="write the raw byte stream to this file")
    ap.add_argument("--gps-baud", type=int, default=230400, help="receiver baud (GPS_BAUD in firmware)")
    ap.add_argument("--no-ntrip", action="store_true", help="don't fetch RTK corrections")
    ap.add_argument("--ntrip-url", default=NTRIP_URL, help="host:port/mountpoint")
    ap.add_argument("--ntrip-user", default=NTRIP_USER)
    ap.add_argument("--ntrip-password", default=NTRIP_PASSWORD)
    ap.add_argument("--gga-interval", type=float, default=1.0, help="seconds between GGA uploads")
    ap.add_argument("--approx", metavar="LAT,LON[,ALT]",
                    help="position to send the caster until the receiver has its own fix")
    args = ap.parse_args()
    approx = tuple(float(x) for x in args.approx.split(",")) if args.approx else None

    ser = serial.Serial(args.port, args.baud, timeout=0.1)
    ser.reset_input_buffer()
    log = open(args.log, "wb") if args.log else None
    parser, st = StreamParser(), GpsState()
    dashboard = not args.raw and sys.stdout.isatty()
    last_draw = 0.0

    ntrip = None
    if not args.no_ntrip:
        ntrip = NtripClient(args.ntrip_url, args.ntrip_user, args.ntrip_password, ser, st,
                            args.gga_interval, approx, args.gps_baud, verbose=not dashboard)
        ntrip.start()

    try:
        while True:
            data = ser.read(4096)
            if log and data:
                log.write(data)
            for kind, frame in parser.feed(data):
                if kind == "nmea":
                    st.handle_nmea(frame)
                    if not dashboard:
                        print(frame.decode("ascii", "replace"), flush=True)
                elif kind == "rtcm":
                    st.handle_rtcm(frame)
                    if not dashboard:
                        print(f"<RTCM3 type {rtcm_type(frame)}, {len(frame)} bytes>", flush=True)
                else:
                    st.junk_bytes += len(frame)
            if dashboard and time.time() - last_draw > 0.2:
                sys.stdout.write("\033[H\033[J" + render(st, args.port, args.baud, ntrip) + "\n")
                sys.stdout.flush()
                last_draw = time.time()
    except KeyboardInterrupt:
        pass
    finally:
        if ntrip:
            ntrip.stop_event.set()
            ntrip.join(timeout=2)
        ser.close()
        if log:
            log.close()


if __name__ == "__main__":
    main()

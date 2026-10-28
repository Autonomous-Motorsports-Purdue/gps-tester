#!/usr/bin/env python3
"""Feed live GEM1305 fixes (GGA only) into EKF_Localizer and plot the result.

Reuses the serial parsing + NTRIP client from read_gps.py, so RTK corrections
from the InDOT caster are forwarded to the receiver exactly as there. Positions
are converted to a local east/north frame (meters) centred on the first fix.
Every fix is plotted as a raw GPS point, but only RTK FIXED fixes go into the
filter: it starts on the first one, and float/single fixes are never fed in.

    python3 test_ekf_gps.py                       # /dev/ttyACM0, with RTK corrections
    python3 test_ekf_gps.py -p COM5 --no-ntrip
    python3 test_ekf_gps.py --approx 40.4237,-86.9212

There is no real IMU, so the predict step is fed fake readings at IMU_HZ: zero
accel and gyro (the kart is assumed stationary) plus Gaussian noise at the
datasheet ACCEL_STDEV / GYRO_STDEV. That lets the covariance grow between fixes
like it would on the kart. No drive encoder. The raw fix is plotted alongside so
you can see how far the estimate lags.
"""
import argparse
import math
import threading
import time
from collections import deque

import matplotlib.pyplot as plt
import numpy as np
import serial
from matplotlib.animation import FuncAnimation

from ekf_localizer_new import ACCEL_STDEV, DEFAULT_HZ, GEM_GPS_OFFSET_M, GYRO_STDEV, EKF_Localizer, rotate_2d_vector
from read_gps import FIX_QUALITY, NTRIP_PASSWORD, NTRIP_URL, NTRIP_USER, GpsState, NtripClient, StreamParser

EARTH_RADIUS_M = 6378137.0
START_HEADING = math.pi / 2  # assume the kart starts facing north
DIST_BETWEEN_GPS_M = 0.5
TRAIL_LEN = 2000
RTK_FIXED = 4  # GGA fix quality, see FIX_QUALITY
IMU_HZ = DEFAULT_HZ  # rate of the fake IMU readings that drive the predict step


class LocalFrame:
    """Equirectangular lat/lon -> east/north meters around an origin (fine over a few km)."""

    def __init__(self, lat0, lon0):
        self.lat0, self.lon0 = lat0, lon0
        self.cos_lat0 = math.cos(math.radians(lat0))

    def to_xy(self, lat, lon):
        x = math.radians(lon - self.lon0) * EARTH_RADIUS_M * self.cos_lat0
        y = math.radians(lat - self.lat0) * EARTH_RADIUS_M
        return x, y


class GpsEkfRunner(threading.Thread):
    """Reads the serial port; plots every new GGA with a fix, and runs one EKF update on each RTK FIXED one."""

    def __init__(self, ser):
        super().__init__(daemon=True)
        self.ser = ser
        self.parser, self.st = StreamParser(), GpsState()
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.frame = self.ekf = None
        self.last_fix_xy = None
        self.ekf_trail = deque(maxlen=TRAIL_LEN)
        self.gps_trail = deque(maxlen=TRAIL_LEN)
        self.rng = np.random.default_rng()
        self.next_imu = None  # monotonic time of the next fake IMU reading

    def run(self):
        while not self.stop_event.is_set():
            data = self.ser.read(4096)
            with self.lock:
                self._step_imu(time.monotonic())
                for kind, frame in self.parser.feed(data):
                    if kind == "nmea":
                        prev_gga = self.st.last_gga
                        self.st.handle_nmea(frame)
                        if self.st.last_gga != prev_gga:
                            self._on_gga()
                    elif kind == "rtcm":
                        self.st.handle_rtcm(frame)

    def _on_gga(self):
        st = self.st
        if st.quality == 0 or st.lat is None or st.lon is None:
            return
        # the origin is the first fix of any quality, so the raw position shows up before RTK FIXED
        if self.frame is None:
            self.frame = LocalFrame(st.lat, st.lon)
        gps_xy = self.frame.to_xy(st.lat, st.lon)
        self.last_fix_xy = gps_xy
        self.gps_trail.append(gps_xy)

        # only RTK FIXED fixes reach the filter; float/single are metres off
        if st.quality != RTK_FIXED:
            return
        # create the ekf on the first RTK FIXED fix, starting where that fix puts the kart
        if self.ekf is None:
            # the state tracks the kart centre, not the antenna
            cx, cy = np.array(gps_xy) - rotate_2d_vector(np.array(GEM_GPS_OFFSET_M), START_HEADING)
            self.ekf = EKF_Localizer(cx, cy, START_HEADING, DIST_BETWEEN_GPS_M)
            self.next_imu = time.monotonic()

        # feed gps measurement into ekf (no IMU here, so this is a pure measurement update)
        self.ekf.update(None, None, None, gps_xy, None)
        self.ekf_trail.append(tuple(self.ekf.get_position()))

    def _step_imu(self, now):
        """Run the predict step at IMU_HZ up to now, with fake readings of a stationary IMU: zero plus noise."""
        if self.ekf is None:
            return
        while self.next_imu <= now:
            accel = self.rng.normal(0, ACCEL_STDEV, 2)
            gyro = self.rng.normal(0, GYRO_STDEV)
            self.ekf.update(accel, gyro, None, None, None, cur_time=self.next_imu)
            self.next_imu += 1 / IMU_HZ


def view_limits(pts, min_span, box_aspect):
    """Equal-aspect x/y limits containing every point, shaped to an axes box of height/width box_aspect."""
    lo, hi = pts.min(axis=0), pts.max(axis=0)
    centre = (lo + hi) / 2
    # 20% padding so markers on the extremes aren't clipped
    half_w, half_h = np.maximum((hi - lo) * 1.2, min_span) / 2
    # grow the shorter side to fill the box; never shrink, so nothing falls outside
    if half_h / half_w > box_aspect:
        half_w = half_h / box_aspect
    else:
        half_h = half_w * box_aspect
    return (centre[0] - half_w, centre[0] + half_w), (centre[1] - half_h, centre[1] + half_h)


def age(t, now):
    return f"{now - t:.1f} s" if t else "never"


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-p", "--port", default="/dev/ttyACM0")
    ap.add_argument("-b", "--baud", type=int, default=460800, help="PC-side baud (PC_BAUD in firmware)")
    ap.add_argument("--gps-baud", type=int, default=230400, help="receiver baud (GPS_BAUD in firmware)")
    ap.add_argument("--no-ntrip", action="store_true", help="don't fetch RTK corrections")
    ap.add_argument("--ntrip-url", default=NTRIP_URL, help="host:port/mountpoint")
    ap.add_argument("--ntrip-user", default=NTRIP_USER)
    ap.add_argument("--ntrip-password", default=NTRIP_PASSWORD)
    ap.add_argument("--approx", metavar="LAT,LON[,ALT]",
                    help="position to send the caster until the receiver has its own fix")
    ap.add_argument("--min-span", type=float, default=1.0, help="smallest plot width in meters")
    args = ap.parse_args()
    approx = tuple(float(x) for x in args.approx.split(",")) if args.approx else None

    ser = serial.Serial(args.port, args.baud, timeout=0.1)
    ser.reset_input_buffer()
    runner = GpsEkfRunner(ser)
    runner.start()

    ntrip = None
    if not args.no_ntrip:
        ntrip = NtripClient(args.ntrip_url, args.ntrip_user, args.ntrip_password, ser, runner.st,
                            approx=approx, gps_baud=args.gps_baud, verbose=True)
        ntrip.start()

    fig, ax = plt.subplots(figsize=(9, 8))
    fig.canvas.manager.set_window_title("EKF GPS test")
    fig.subplots_adjust(top=0.74)  # room for the status text, including the EKF state lines
    ax.set_xlabel("east (m)")
    ax.set_ylabel("north (m)")
    # limits come from view_limits() already matching the box shape, so "box" never crops them
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, alpha=0.3)
    gps_trail_line, = ax.plot([], [], ".", color="tab:gray", ms=2, alpha=0.5, label="raw GPS fixes")
    ekf_trail_line, = ax.plot([], [], "-", color="tab:blue", lw=1, alpha=0.6, label="EKF trail")
    gps_dot, = ax.plot([], [], "o", color="tab:orange", ms=8, label="raw GPS (latest)")
    ekf_dot, = ax.plot([], [], "o", color="tab:blue", ms=12, label="EKF position")
    # fixed 0.6 in on screen whatever the zoom; hidden until the ekf exists. Screen angles ("uv") match
    # east/north because the aspect is equal; "xy" would need ax.dataLim, which set_data never updates
    heading_arrow = ax.quiver([0], [0], [1], [0], angles="uv", scale_units="inches", scale=1 / 0.6,
                              color="navy", width=0.006, zorder=3)
    heading_arrow.set_visible(False)
    ax.legend(loc="lower right", fontsize=8)
    info = fig.text(0.02, 0.98, "", va="top", ha="left", family="monospace", fontsize=10)

    def draw(_):
        now_wall, now_mono = time.time(), time.monotonic()
        with runner.lock:
            st = runner.st
            quality, lat, lon = st.quality, st.lat, st.lon
            last_gga, sats, hdop, diff_age = st.last_gga, st.sats_used, st.hdop, st.diff_age
            ekf_state = runner.ekf.x.copy() if runner.ekf else None
            ekf_stdev = np.sqrt(np.diag(runner.ekf.est_error_cov)) if runner.ekf else None
            ekf_compass = runner.ekf.get_compass_heading_deg() if runner.ekf else None
            ekf_xy = ekf_state[:2] if ekf_state is not None else None
            gps_xy = runner.last_fix_xy
            ekf_trail = np.array(runner.ekf_trail) if runner.ekf_trail else np.empty((0, 2))
            gps_trail = np.array(runner.gps_trail) if runner.gps_trail else np.empty((0, 2))

        mode = FIX_QUALITY.get(quality, str(quality))
        if ntrip:
            rtk_age = age(ntrip.last_rx, now_mono)
            ntrip_status = ntrip.status
        else:
            rtk_age, ntrip_status = "n/a", "disabled (--no-ntrip)"
        lines = [
            f"Mode        {mode}   (GGA quality {quality}, {sats} sats, HDOP {hdop if hdop is not None else '--'})",
            f"Lat / Lon   {lat:.8f}, {lon:.8f}" if lat is not None and lon is not None else "Lat / Lon   --",
            f"Since GPS   {age(last_gga, now_wall)}",
            f"Since RTK   {rtk_age}   (receiver diff age {diff_age or '--'} s)",
            f"NTRIP       {ntrip_status}",
        ]
        if ekf_state is not None:
            # every state with its 1-sigma from the covariance diagonal
            x, y, heading, fwd_vel, gyro_bias = ekf_state
            sx, sy, s_heading, s_vel, s_bias = ekf_stdev
            lines += [
                f"EKF x, y    {x:+.3f} ± {sx:.3f}, {y:+.3f} ± {sy:.3f} m",
                f"EKF heading {math.degrees(heading):+.1f} ± {math.degrees(s_heading):.1f} deg"
                f"   (compass {ekf_compass:.1f} deg)",
                f"EKF fwd vel {fwd_vel:+.3f} ± {s_vel:.3f} m/s",
                f"EKF bias    {math.degrees(gyro_bias):+.3f} ± {math.degrees(s_bias):.3f} deg/s (gyro)",
            ]
        else:
            lines.append("EKF         waiting for RTK FIXED")
        info.set_text("\n".join(lines))
        info.set_color("tab:red" if last_gga == 0 or now_wall - last_gga > 2 else "black")

        gps_trail_line.set_data(gps_trail[:, 0], gps_trail[:, 1])
        ekf_trail_line.set_data(ekf_trail[:, 0], ekf_trail[:, 1])
        if gps_xy is not None:
            gps_dot.set_data([gps_xy[0]], [gps_xy[1]])
        if ekf_xy is not None:
            ekf_dot.set_data([ekf_xy[0]], [ekf_xy[1]])
            heading_arrow.set_offsets([ekf_xy])
            heading_arrow.set_UVC(math.cos(ekf_state[2]), math.sin(ekf_state[2]))
            heading_arrow.set_visible(True)

        latest = np.array([p for p in (gps_xy, ekf_xy) if p is not None], dtype=float).reshape(-1, 2)
        pts = np.vstack([gps_trail, ekf_trail, latest])
        if len(pts):
            pos = ax.get_position(original=True)
            fig_w, fig_h = fig.get_size_inches()
            xlim, ylim = view_limits(pts, args.min_span, (pos.height * fig_h) / (pos.width * fig_w))
            ax.set_xlim(*xlim)
            ax.set_ylim(*ylim)
        return gps_trail_line, ekf_trail_line, gps_dot, ekf_dot, heading_arrow, info

    anim = FuncAnimation(fig, draw, interval=100, cache_frame_data=False)  # noqa: F841 (keep a reference)
    try:
        plt.show()
    finally:
        runner.stop_event.set()
        if ntrip:
            ntrip.stop_event.set()
            ntrip.join(timeout=2)
        runner.join(timeout=1)
        ser.close()


if __name__ == "__main__":
    main()

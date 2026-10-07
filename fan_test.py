#!/usr/bin/env python3
#
# Automated hardware test: runs a command sequence, records telemetry to CSV,
# prints per-step overshoot / settling time and saves a plot.
#
#   python3 fan_test.py --label run1 --extended

import argparse
import csv
import os
import re
import sys
import threading
import time
from datetime import datetime

import serial
import serial.tools.list_ports

USB_VID = 0x0483
USB_PID = 0x5740          # STM32 USB CDC (not the ST-LINK, which is 0x374B)
BAUD_RATE = 115200

TELEMETRY_RE = re.compile(
    r"Time:\s*([-\d.]+),\s*Set:\s*([-\d.naif]+),\s*Meas:\s*([-\d.naif]+),\s*PWM:\s*([-\d.naif]+)",
    re.IGNORECASE)

# (step name, command to send or None, duration in seconds)
BASE_SEQUENCE = [
    ("steady_1600",   "s1600", 10),
    ("step_1000",     "s1000", 20),
    ("step_2500",     "s2500", 20),
    ("step_1600",     "s1600", 20),
    ("manual_50",     "m50",   10),
    ("bumpless_1600", "s1600", 20),
    ("manual_100",    "m100",   8),
    ("manual_0",      "m0",     6),
    ("restart_1600",  "s1600", 20),
]

EXTENDED_SEQUENCE = [
    ("reject_pnan",   "pnan",   5),
    ("reject_snan",   "snan",   5),
    ("reject_sinf",   "sinf",   5),
    ("invalid_cmd",   "x1",     3),
    ("max_rpm_2000",  "r2000",  3),
    ("clamped_2500",  "s2500", 12),     # setpoint must be limited to 2000
    ("restore_max",   "r3100",  2),
    ("back_1600",     "s1600", 12),
    ("save_upper",    "SAVE",  10),
    ("after_save",    None,    10),
]

SETTLE_BAND = 0.02        # +-2 % of setpoint
TAIL_S = 3.0              # window at the end of a step used for steady-state values


def find_port():
    for p in serial.tools.list_ports.comports():
        if p.vid == USB_VID and p.pid == USB_PID:
            return p.device
    return None


class Recorder:
    """Reads telemetry in a background thread and survives board resets."""

    def __init__(self, port_override):
        self.port_override = port_override
        self.ser = None
        self.rows = []            # host_t, step, fw_time, set, meas, pwm
        self.events = []          # host_t, step, text
        self.step = "connect"
        self.t0 = time.monotonic()
        self.lock = threading.Lock()
        self.running = True
        self.disconnects = 0
        self.connected = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)

    def now(self):
        return time.monotonic() - self.t0

    def _open(self):
        while self.running:
            port = self.port_override or find_port()
            if port:
                try:
                    self.ser = serial.Serial(port, BAUD_RATE, timeout=0.5)
                    self.connected.set()
                    self.event(f"connected to {port}")
                    return True
                except serial.SerialException:
                    pass
            time.sleep(0.2)
        return False

    def _run(self):
        if not self._open():
            return
        while self.running:
            try:
                raw = self.ser.readline()
            except (serial.SerialException, OSError, TypeError):
                if not self.running:
                    return          # port closed by stop()
                self.connected.clear()
                self.disconnects += 1
                self.event("disconnected")
                try:
                    self.ser.close()
                except Exception:
                    pass
                if not self._open():
                    return
                continue
            if not raw:
                continue
            line = raw.decode("utf-8", errors="replace").strip()
            m = TELEMETRY_RE.search(line)
            with self.lock:
                if m:
                    self.rows.append([round(self.now(), 3), self.step,
                                      *(float(x) for x in m.groups())])
                    print(f"  [{self.step:>14}] {line}")
                elif line:
                    self.events.append([round(self.now(), 3), self.step, line])
                    print(f"  [{self.step:>14}] >>> {line}")

    def event(self, text):
        with self.lock:
            self.events.append([round(self.now(), 3), self.step, text])
        print(f"--- {text}")

    def send(self, cmd):
        self.event(f"send '{cmd}'")
        self.ser.write((cmd + "\n").encode())

    def start(self):
        self.thread.start()

    def stop(self):
        self.running = False
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass


def run_reset_test(rec, duration):
    rec.step = "pre_reset"
    before = rec.disconnects
    print("\n>>> Press the black RESET button on the board now (waiting up to 30 s)...")
    deadline = time.monotonic() + 30
    while rec.disconnects == before and time.monotonic() < deadline:
        time.sleep(0.1)
    if rec.disconnects == before:
        rec.event("no reset detected, skipping startup test")
        return False
    rec.step = "startup_reset"
    rec.connected.wait(timeout=15)
    rec.event("board back after reset, recording startup")
    time.sleep(duration)
    return True


def summarize(rows, sequence):
    print("\n" + "=" * 92)
    print(f"{'step':<15}{'cmd':<7}{'start':>8}{'final':>9}{'ss_err':>8}"
          f"{'overshoot':>11}{'settle_s':>10}{'pwm_max':>9}{'pwm_end':>9}")
    print("-" * 92)
    summary = []
    cmds = dict((n, c) for n, c, _ in sequence)
    cmds["startup_reset"] = "RESET"
    for step in dict.fromkeys(r[1] for r in rows):
        data = [r for r in rows if r[1] == step]
        if not data:
            continue
        t_start = data[0][0]
        t_end = data[-1][0]
        tail = [r for r in data if r[0] >= t_end - TAIL_S] or data
        sp = data[-1][3]
        start_meas = data[0][4]
        final = sum(r[4] for r in tail) / len(tail)
        pwm_max = max(r[5] for r in data)
        pwm_end = sum(r[5] for r in tail) / len(tail)
        ss_err = sp - final
        cmd = cmds.get(step) or ""
        overshoot = settle = None
        auto = cmd.lower().startswith("s") or step == "startup_reset"
        if auto and sp > 0:
            step_size = sp - start_meas
            if abs(step_size) > 50:
                if step_size > 0:
                    peak = max(r[4] for r in data)
                    overshoot = max(0.0, (peak - sp) / abs(step_size) * 100)
                else:
                    peak = min(r[4] for r in data)
                    overshoot = max(0.0, (sp - peak) / abs(step_size) * 100)
            band = SETTLE_BAND * sp
            last_out = None
            for r in data:
                if abs(r[4] - sp) > band:
                    last_out = r[0]
            if last_out is None:
                settle = 0.0
            elif last_out < t_end - 1.0:
                settle = last_out - t_start
        ss_str = f"{ss_err:.0f}" if auto else "-"
        os_str = f"{overshoot:.1f}%" if overshoot is not None else "-"
        settle_str = "-" if not auto else (f"{settle:.2f}" if settle is not None else "no")
        print(f"{step:<15}{cmd:<7}{start_meas:>8.0f}{final:>9.0f}{ss_str:>8}"
              f"{os_str:>11}{settle_str:>10}{pwm_max:>9.1f}{pwm_end:>9.1f}")
        summary.append(dict(step=step, cmd=cmd, setpoint=sp, start_meas=round(start_meas, 1),
                            final_meas=round(final, 1),
                            ss_error=round(ss_err, 1) if auto else "",
                            overshoot_pct=round(overshoot, 1) if overshoot is not None else "",
                            settle_s=round(settle, 2) if settle is not None else ("" if not auto else "not settled"),
                            pwm_max=round(pwm_max, 1), pwm_end=round(pwm_end, 1)))
    print("=" * 92)
    return summary


def plot(rows, events, path, title):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib not installed, skipping plot")
        return
    t = [r[0] for r in rows]
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(14, 7), sharex=True,
                                   gridspec_kw=dict(height_ratios=[2, 1]))
    ax1.step(t, [r[3] for r in rows], where="post", label="Setpoint", color="#888888", linestyle="--")
    ax1.plot(t, [r[4] for r in rows], label="Measured", color="#1f77b4")
    ax1.set_ylabel("RPM")
    ax1.legend(loc="upper right")
    ax1.grid(alpha=0.3)
    ax2.plot(t, [r[5] for r in rows], color="#d62728")
    ax2.set_ylabel("PWM (%)")
    ax2.set_xlabel("Time (s)")
    ax2.set_ylim(-5, 105)
    ax2.grid(alpha=0.3)
    # Step boundaries
    prev = None
    for r in rows:
        if r[1] != prev:
            for ax in (ax1, ax2):
                ax.axvline(r[0], color="black", alpha=0.15)
            ax1.annotate(r[1], (r[0], 1), xycoords=("data", "axes fraction"),
                         rotation=90, va="top", ha="right", fontsize=7, alpha=0.7)
            prev = r[1]
    ax1.set_title(title)
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    print(f"Plot:    {path}")


def main():
    ap = argparse.ArgumentParser(description="Automated fan controller test")
    ap.add_argument("--label", required=True, help="run name, e.g. old / new")
    ap.add_argument("--port", help="serial port (default: auto-detect STM32 CDC)")
    ap.add_argument("--no-reset", action="store_true", help="skip the RESET startup test")
    ap.add_argument("--extended", action="store_true",
                    help="add NaN/inf rejection and SAVE tests (fixed firmware only)")
    args = ap.parse_args()

    if args.extended and args.label.lower() == "old":
        print("Refusing --extended with label 'old': on the old firmware 'pnan' stops the fan")
        print("and SAVE can store NaN in flash permanently.")
        sys.exit(1)

    sequence = BASE_SEQUENCE + (EXTENDED_SEQUENCE if args.extended else [])

    rec = Recorder(args.port)
    rec.start()
    if not rec.connected.wait(timeout=10):
        print("Could not find/open the STM32 serial port. Is fan_controller.py still running?")
        sys.exit(1)

    total = sum(d for _, _, d in sequence) + (0 if args.no_reset else 15)
    print(f"Running {len(sequence)} steps, about {total // 60} min {total % 60} s. Ctrl+C aborts.")
    time.sleep(1.0)

    try:
        if not args.no_reset:
            run_reset_test(rec, 15)
        for name, cmd, duration in sequence:
            rec.step = name
            if cmd:
                rec.send(cmd)
            time.sleep(duration)
    except KeyboardInterrupt:
        print("\nAborted, saving what was recorded.")
    finally:
        try:
            rec.ser.write(b"s1600\n")       # leave the fan in a sane state
        except Exception:
            pass
        rec.stop()

    os.makedirs("results", exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    base = os.path.join("results", f"{args.label}_{stamp}")

    with rec.lock:
        rows = list(rec.rows)
        events = list(rec.events)

    with open(base + "_telemetry.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["host_time_s", "step", "fw_time_s", "setpoint_rpm", "measured_rpm", "pwm_pct"])
        w.writerows(rows)
    with open(base + "_events.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["host_time_s", "step", "event"])
        w.writerows(events)

    if not rows:
        print("No telemetry recorded.")
        return

    summary = summarize(rows, sequence)
    with open(base + "_summary.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(summary[0].keys()))
        w.writeheader()
        w.writerows(summary)

    print(f"Data:    {base}_telemetry.csv")
    print(f"Events:  {base}_events.csv")
    print(f"Summary: {base}_summary.csv")
    plot(rows, events, base + "_plot.png", f"Fan controller test: {args.label} firmware")


if __name__ == "__main__":
    main()

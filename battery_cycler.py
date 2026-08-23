#!/usr/bin/env python3
"""
Battery Cycler CLI — Sequence Edition
--------------------------------------
Interactive Python wrapper for the ESP32 battery cycler sequencer firmware.
Builds or loads a custom multi-step protocol (CC charge / CV charge /
CC discharge / rest, with an optional repeat loop), uploads it to the
firmware as raw current values (C-rate math happens here, not on the MCU),
then monitors, plots, and logs the run.

Usage:
    python battery_cycler.py
"""

import serial
import serial.tools.list_ports
import threading
import time
import csv
import os
import sys
import glob
import json
from datetime import datetime
from collections import deque

import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
from matplotlib.lines import Line2D

# ── Firmware step type codes (must match StepType enum in the .cpp) ─────────
STEP_TYPES = {
    "CC_CHARGE":    0,
    "CV_CHARGE":    1,
    "CC_DISCHARGE": 2,
    "REST":         3,
    "EIS":          4,
}
STEP_TYPE_NAMES = {v: k for k, v in STEP_TYPES.items()}

# ── EIS sweep step defaults / limits ────────────────────────────────────────
# Mirror the EIS live-window defaults and the firmware MAX_POINTS clamp. An EIS
# step runs a blocking impedance sweep inline with the cycling sequence; the
# cycler drive is off during the measurement (only the battery thermal guard
# stays active). Uploaded via the firmware 'E' command, not 'W'.
EIS_DEFAULT_POINTS   = 50
EIS_DEFAULT_START_HZ = 50000.0   # sweep high -> low: settles the fast points first
EIS_DEFAULT_STOP_HZ  = 1.0       # and defers the long low-f integrations to the end
EIS_MAX_POINTS       = 200      # firmware MAX_POINTS clamp

# ── Plot colours, keyed on the firmware's state STRING (not an int) ─────────
COLOR_CHARGE     = "#2196F3"   # blue
COLOR_DISCHARGE  = "#F44336"   # red
COLOR_REST       = "#9E9E9E"   # grey
COLOR_FAULT      = "#FF1744"
COLOR_TEMP       = "#FF5722"   # deep-orange
COLOR_TEMP2      = "#CE93D8"   # light purple

STATE_COLORS = {
    "CHARGING":    COLOR_CHARGE,
    "DISCHARGING": COLOR_DISCHARGE,
    "REST":        COLOR_REST,
    "IDLE":        COLOR_REST,
    "GITT_PULSE":  COLOR_CHARGE,
    "GITT_RELAX":  COLOR_REST,
    "EIS":         "#00BCD4",
    "SEQ_EIS":     "#00BCD4",
    "FAULT":       COLOR_FAULT,
}

ROLLING_WINDOW_S = 600   # 10-minute rolling window for the live plot
PROTOCOL_GLOB    = "protocol_*.json"

# ─────────────────────────────────────────────────────────────────────────────
# Serial port helpers
# ─────────────────────────────────────────────────────────────────────────────

def find_serial_port():
    """Auto-detect the ESP32 serial port."""
    ports = serial.tools.list_ports.comports()
    candidates = []
    for p in ports:
        desc = (p.description or "").lower()
        hwid = (p.hwid or "").lower()
        if any(k in desc or k in hwid for k in
               ("usb", "uart", "ch340", "cp210", "ftdi", "esp", "silicon")):
            candidates.append(p)

    for p in ports:
        if p.device.startswith(("/dev/ttyUSB", "/dev/ttyACM")):
            if p not in candidates:
                candidates.append(p)

    if not candidates:
        candidates = ports

    if len(candidates) == 1:
        print(f"  Auto-detected port: {candidates[0].device}  ({candidates[0].description})")
        return candidates[0].device

    print("\n  Available serial ports:")
    for i, p in enumerate(candidates):
        print(f"    [{i}] {p.device}  —  {p.description}")
    while True:
        choice = input("  Select port number: ").strip()
        if choice.isdigit() and 0 <= int(choice) < len(candidates):
            return candidates[int(choice)].device
        print("  Invalid choice, try again.")


def open_port(port):
    """Open serial port with DTR/RTS suppressed to avoid resetting the ESP32."""
    ser = serial.Serial()
    ser.port     = port
    ser.baudrate = 115200
    ser.timeout  = 2
    ser.dtr      = False
    ser.rts      = False
    ser.open()
    ser.reset_input_buffer()
    return ser


# ─────────────────────────────────────────────────────────────────────────────
# Prompt helpers
# ─────────────────────────────────────────────────────────────────────────────

def prompt_float(label, default, lo=None, hi=None):
    while True:
        raw = input(f"    {label} [{default}]: ").strip()
        if raw == "":
            return default
        try:
            val = float(raw)
            if lo is not None and val < lo:
                print(f"      x  Must be >= {lo}"); continue
            if hi is not None and val > hi:
                print(f"      x  Must be <= {hi}"); continue
            return val
        except ValueError:
            print("      x  Please enter a number.")

def prompt_int(label, default, lo=None, hi=None):
    while True:
        raw = input(f"    {label} [{default}]: ").strip()
        if raw == "":
            return default
        try:
            val = int(raw)
            if lo is not None and val < lo:
                print(f"      x  Must be >= {lo}"); continue
            if hi is not None and val > hi:
                print(f"      x  Must be <= {hi}"); continue
            return val
        except ValueError:
            print("      x  Please enter a whole number.")

def prompt_yesno(label, default_yes=True):
    suffix = "[Y/n]" if default_yes else "[y/N]"
    raw = input(f"    {label} {suffix}: ").strip().lower()
    if raw == "":
        return default_yes
    return raw.startswith("y")


# ─────────────────────────────────────────────────────────────────────────────
# Thermal fault limits — set once per session, not per step
# ─────────────────────────────────────────────────────────────────────────────

def configure_thermal_limits(ser):
    """Prompt once for T1 (NMOS) / T2 (battery) fault ceilings and push them
    to the firmware via the existing 'T'/'t' commands. Firmware defaults
    (40.0C / 30.0C) are used if the user just hits enter."""
    print("\n" + "-" * 56)
    print("  Thermal fault limits (set once for this session)")
    print("-" * 56)
    t1 = prompt_float("NMOS over-temp fault (C)",  40.0, lo=20.0, hi=80.0)
    t2 = prompt_float("Batt over-temp fault (C)",  30.0, lo=20.0, hi=60.0)

    send_confirmed(ser, f"T {t1:.1f}\n", "Set T1 Fault")
    send_confirmed(ser, f"t {t2:.1f}\n", "Set T2 Fault")


# ─────────────────────────────────────────────────────────────────────────────
# Interactive protocol builder
# ─────────────────────────────────────────────────────────────────────────────

def build_step(idx):
    print(f"\n  --- Step {idx} ---")
    print("    [1] CC charge   [2] CV charge (taper)   [3] CC discharge   [4] Rest   [5] EIS sweep")
    while True:
        choice = input("    Step type: ").strip()
        if choice in ("1", "2", "3", "4", "5"):
            break
        print("      x  Enter 1-5.")

    step = {"sample_ms": 0, "duration_min": 0.0}

    if choice == "1":
        step["type"]           = "CC_CHARGE"
        step["c_rate"]         = prompt_float("Charge C-rate (e.g. 0.5 = C/2)", 0.5, lo=0.01, hi=5.0)
        step["voltage_limit"]  = prompt_float("Vmax cutoff (V)", 3.60, lo=2.0, hi=4.3)
        step["duration_min"]   = prompt_float("Time cap in minutes (0 = none, stop on Vmax only)", 0.0, lo=0.0)
    elif choice == "2":
        step["type"]           = "CV_CHARGE"
        # This is the software PI loop's current CEILING (cvIMax in firmware),
        # not a taper target — it must be > 0 or beginChargeCV() clamps the
        # bumpless-transfer seed to 0 A and the CC->CV handoff collapses the
        # charge current instantly instead of tapering. Default to matching a
        # typical prior CC step so the common case "just works".
        step["c_rate"]         = prompt_float("Current ceiling C-rate (CV loop cap, e.g. same as prior CC step)", 0.5, lo=0.01, hi=5.0)
        step["voltage_limit"]  = prompt_float("Hold voltage Vmax (V)", 3.60, lo=2.0, hi=4.3)
        step["cutoff_c_rate"]  = prompt_float("Taper cutoff C-rate (e.g. 0.0333 = C/30)", 0.0333, lo=0.001, hi=1.0)
        step["duration_min"]   = prompt_float("Time cap in minutes (0 = none, stop on taper only)", 0.0, lo=0.0)
    elif choice == "3":
        step["type"]           = "CC_DISCHARGE"
        step["c_rate"]         = prompt_float("Discharge C-rate (e.g. 1.0 = 1C)", 1.0, lo=0.01, hi=5.0)
        step["voltage_limit"]  = prompt_float("Vmin cutoff (V)", 2.50, lo=1.5, hi=3.8)
        step["duration_min"]   = prompt_float("Time cap in minutes (0 = none, stop on Vmin only)", 0.0, lo=0.0)
    elif choice == "4":
        step["type"]           = "REST"
        step["duration_min"]   = prompt_float("Rest duration in minutes", 60.0, lo=0.001)
    else:
        # EIS sweep. Runs a blocking RCAL + battery impedance sweep in place; the
        # cycler power paths are off for the duration (battery thermal guard stays
        # live in firmware). No current / duration / sample-interval applies, so we
        # return early before those prompts.
        step["type"]         = "EIS"
        step["eis_points"]   = prompt_int("Sweep points",          EIS_DEFAULT_POINTS,   lo=1,    hi=EIS_MAX_POINTS)
        step["eis_start_hz"] = prompt_float("Start frequency (Hz)", EIS_DEFAULT_START_HZ, lo=0.01)
        step["eis_stop_hz"]  = prompt_float("Stop frequency (Hz)",  EIS_DEFAULT_STOP_HZ,  lo=0.02)
        print("    Note: each EIS step adds ~30s of settle time plus the per-point"
              " measurement; the run plot pauses (no telemetry) while it sweeps.")
        return step

    if prompt_yesno("Use a faster sample interval for this step? (e.g. for short pulses)", default_yes=False):
        step["sample_ms"] = prompt_int("Sample interval (ms)", 50, lo=10, hi=5000)

    return step


def describe_step(i, step):
    t = step["type"]
    parts = [f"[{i}] {t}"]
    if t == "EIS":
        parts.append(f"{step.get('eis_points', EIS_DEFAULT_POINTS)}pts")
        parts.append(f"{step.get('eis_start_hz', EIS_DEFAULT_START_HZ):g}-"
                     f"{step.get('eis_stop_hz', EIS_DEFAULT_STOP_HZ):g}Hz")
        return "  ".join(parts)
    if t in ("CC_CHARGE", "CC_DISCHARGE"):
        parts.append(f"{step['c_rate']}C")
    if t in ("CC_CHARGE", "CV_CHARGE"):
        parts.append(f"Vmax={step['voltage_limit']}V")
    if t == "CV_CHARGE":
        parts.append(f"ceiling={step.get('c_rate', 0.0)}C")
        parts.append(f"cutoff={step['cutoff_c_rate']}C")
    if t == "CC_DISCHARGE":
        parts.append(f"Vmin={step['voltage_limit']}V")
    if step.get("duration_min", 0) > 0:
        parts.append(f"{step['duration_min']}min cap")
    if step.get("sample_ms", 0) > 0:
        parts.append(f"sample={step['sample_ms']}ms")
    return "  ".join(parts)


def build_protocol():
    print("\n" + "=" * 56)
    print("  New Protocol Builder")
    print("=" * 56)

    name        = input("  Protocol name (used for filename): ").strip() or "unnamed"
    capacity_mah = prompt_float("  Cell rated capacity (mAh)", 3000.0, lo=1.0)

    steps = []
    idx = 0
    while True:
        steps.append(build_step(idx))
        idx += 1
        print("\n  Current protocol:")
        for i, s in enumerate(steps):
            print("    " + describe_step(i, s))
        if not prompt_yesno("\n  Add another step?", default_yes=True):
            break

    loop = None
    if prompt_yesno("\n  Define a repeat loop over a range of steps?", default_yes=False):
        print("\n  Steps available for the loop range:")
        for i, s in enumerate(steps):
            print("    " + describe_step(i, s))
        start_idx = prompt_int("  Loop start step index", 0, lo=0, hi=len(steps) - 1)
        end_idx   = prompt_int("  Loop end step index", len(steps) - 1, lo=start_idx, hi=len(steps) - 1)
        guard_v   = prompt_float("  Guard voltage — abort loop if crossed at ANY step inside it (V)", 2.50, lo=0.5, hi=4.3)
        loop = {"start_idx": start_idx, "end_idx": end_idx, "guard_voltage": guard_v}
        print("  Note: firmware caps this at 50 iterations regardless of guard voltage.")

    protocol = {"name": name, "capacity_mah": capacity_mah, "steps": steps, "loop": loop}

    safe_name = "".join(c if c.isalnum() or c in "-_" else "_" for c in name)
    filename = f"protocol_{safe_name}.json"
    with open(filename, "w") as f:
        json.dump(protocol, f, indent=2)
    print(f"\n  Saved to {os.path.abspath(filename)}")

    return protocol


def load_protocol():
    files = sorted(glob.glob(PROTOCOL_GLOB))
    if not files:
        print("  No saved protocol files found in this directory.")
        return None
    print("\n  Saved protocols:")
    for i, f in enumerate(files):
        print(f"    [{i}] {f}")
    while True:
        choice = input("  Select protocol number: ").strip()
        if choice.isdigit() and 0 <= int(choice) < len(files):
            with open(files[int(choice)]) as fh:
                return json.load(fh)
        print("  Invalid choice, try again.")


def print_protocol_summary(protocol):
    print("\n  +-- Protocol: " + protocol["name"] + " " + "-" * 10)
    print(f"  |  Capacity: {protocol['capacity_mah']} mAh")
    for i, s in enumerate(protocol["steps"]):
        marker = ""
        if protocol.get("loop"):
            if protocol["loop"]["start_idx"] <= i <= protocol["loop"]["end_idx"]:
                marker = "  <- loop"
        print("  |  " + describe_step(i, s) + marker)
    if protocol.get("loop"):
        l = protocol["loop"]
        print(f"  |  Loop: repeat steps {l['start_idx']}-{l['end_idx']} until Vmin={l['guard_voltage']}V (cap 50 iter)")
    print("  +" + "-" * 30)


# ─────────────────────────────────────────────────────────────────────────────
# Upload protocol to firmware
# ─────────────────────────────────────────────────────────────────────────────

def send_confirmed(ser, cmd_str, expected_substr, timeout=2.0):
    ser.write(cmd_str.encode())
    deadline = time.time() + timeout
    while time.time() < deadline:
        raw = ser.readline().decode("utf-8", errors="replace").strip()
        if not raw:
            continue
        if raw.startswith("MSG,"):
            print(f"    [FW] {raw[4:]}")
            if expected_substr in raw:
                return True
    print(f"    [WARN] No confirmation for: {cmd_str.strip()!r}")
    return False


def upload_protocol(ser, protocol):
    print("\n  Uploading protocol to firmware...")
    cap_A = protocol["capacity_mah"] / 1000.0

    send_confirmed(ser, "C\n", "cleared")

    for i, step in enumerate(protocol["steps"]):
        if step["type"] == "EIS":
            # EIS steps carry no cycler current fields; the 'E' command takes
            # points / start / stop instead. Firmware stores them on the same
            # ProtocolStep and runs a blocking sweep when the sequence reaches it.
            points = int(step.get("eis_points",   EIS_DEFAULT_POINTS))
            start  = float(step.get("eis_start_hz", EIS_DEFAULT_START_HZ))
            stop   = float(step.get("eis_stop_hz",  EIS_DEFAULT_STOP_HZ))
            cmd = f"E,{points},{start:.3f},{stop:.3f}\n"
            send_confirmed(ser, cmd, f"Step {i} added")
            continue

        t = STEP_TYPES[step["type"]]
        current_A      = step.get("c_rate", 0.0) * cap_A
        voltage_limit  = step.get("voltage_limit", 0.0)
        current_cutoff = step.get("cutoff_c_rate", 0.0) * cap_A if step["type"] == "CV_CHARGE" else 0.0
        duration_ms    = int(step.get("duration_min", 0.0) * 60000)
        sample_ms      = int(step.get("sample_ms", 0))

        cmd = f"W,{t},{current_A:.4f},{voltage_limit:.3f},{current_cutoff:.4f},{duration_ms},{sample_ms}\n"
        send_confirmed(ser, cmd, f"Step {i} added")

    if protocol.get("loop"):
        l = protocol["loop"]
        cmd = f"L,{l['start_idx']},{l['end_idx']},{l['guard_voltage']:.3f}\n"
        send_confirmed(ser, cmd, "Loop set")

    print("\n  Starting sequence...")
    send_confirmed(ser, "G\n", "Starting sequence", timeout=3.0)


# ─────────────────────────────────────────────────────────────────────────────
# Shared data store (written by serial thread, read by plot thread)
# ─────────────────────────────────────────────────────────────────────────────

class RunData:
    def __init__(self):
        self.lock = threading.Lock()
        maxlen = int(ROLLING_WINDOW_S / 0.050) + 200  # sized for fastest sample rate (50ms)

        self.t       = deque(maxlen=maxlen)
        self.v       = deque(maxlen=maxlen)
        self.i_ch    = deque(maxlen=maxlen)
        self.i_dis   = deque(maxlen=maxlen)
        self.temp    = deque(maxlen=maxlen)
        self.temp2   = deque(maxlen=maxlen)
        self.state   = deque(maxlen=maxlen)   # strings now, not ints
        self.step_idx  = deque(maxlen=maxlen)
        self.loop_iter = deque(maxlen=maxlen)

        # Full history for capacity integration
        self.all_t     = []
        self.all_i_ch   = []
        self.all_i_dis  = []
        self.all_state  = []

        self.run_done  = False
        self.fault_msg = ""
        self.t0        = None

        # Interleaved-EIS results captured during the run (one dict per sweep:
        # step_idx, loop_iter, v_before, v_after, freq/real/imag/mag/phase lists,
        # saved_path). Populated by serial_reader as EIS steps complete.
        self.eis_sweeps = []

        # The sweep currently streaming in, updated point-by-point so a live
        # plot can draw it as it fills. Same dict object serial_reader is
        # accumulating into; mutated and read only under self.lock. None between
        # sweeps. Cleared (set to None) when the sweep is finalized.
        self.eis_live = None


# ─────────────────────────────────────────────────────────────────────────────
# Serial reader thread
# ─────────────────────────────────────────────────────────────────────────────

def save_eis_sweep(run_csv_path, sweep):
    """Write one interleaved EIS sweep to its own CSV, named next to the run log
    (…_eis_step<idx>_iter<n>.csv). Returns the path, or None on failure."""
    base = os.path.splitext(run_csv_path)[0] if run_csv_path else "run"
    step = sweep.get("step_idx", "x")
    it   = sweep.get("loop_iter", 0)
    path = f"{base}_eis_step{step}_iter{it}.csv"
    try:
        with open(path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow([f"# Interleaved EIS sweep — step {step}, loop_iter {it}"])
            w.writerow([f"# v_before={sweep.get('v_before')}",
                        f"v_after={sweep.get('v_after')}",
                        f"points={len(sweep.get('freq', []))}"])
            w.writerow(["freq_hz", "real_ohm", "imag_ohm", "mag_ohm", "phase_deg"])
            for fr, re_, im_, mg, ph in zip(sweep["freq"], sweep["real"],
                                            sweep["imag"], sweep["mag"], sweep["phase"]):
                w.writerow([f"{fr:.4f}", f"{re_:.6f}", f"{im_:.6f}",
                            f"{mg:.6f}", f"{ph:.4f}"])
        return path
    except OSError as e:
        print(f"\n  [!] Could not save EIS sweep: {e}")
        return None


def serial_reader(ser, data: RunData, csv_writer, csv_file, stop_event: threading.Event):
    run_csv_path = getattr(csv_file, "name", None)
    eis_cur = None   # sweep dict currently being accumulated (None between sweeps)

    def _finalize_eis(v_after=None):
        nonlocal eis_cur
        if eis_cur is None:
            return
        if v_after is not None:
            eis_cur["v_after"] = v_after
        path = save_eis_sweep(run_csv_path, eis_cur)
        eis_cur["saved_path"] = path
        with data.lock:
            data.eis_sweeps.append(eis_cur)
            data.eis_live = None    # sweep is complete; stop live-plotting it
            # Note the sweep in the main run CSV for correlation (leading '#').
        try:
            csv_writer.writerow([f"# EIS step {eis_cur.get('step_idx')} "
                                 f"iter {eis_cur.get('loop_iter')} "
                                 f"v_before={eis_cur.get('v_before')} "
                                 f"v_after={eis_cur.get('v_after')} "
                                 f"file={os.path.basename(path) if path else '(unsaved)'}"])
            csv_file.flush()
        except Exception:
            pass
        n = len(eis_cur.get("freq", []))
        print(f"\r  [EIS] step {eis_cur.get('step_idx')} sweep complete: {n} pts, "
              f"V {eis_cur.get('v_before')}->{eis_cur.get('v_after')}  "
              f"{'saved ' + os.path.basename(path) if path else '(unsaved)'}        ")
        eis_cur = None

    while not stop_event.is_set():
        try:
            raw = ser.readline()
        except serial.SerialException:
            print("\n  [!] Serial connection lost.")
            stop_event.set()
            break

        if not raw:
            continue
        line = raw.decode("utf-8", errors="replace").strip()

        if line.startswith("MSG,"):
            msg = line[4:]
            print(f"\r  [FW] {msg}                    ")
            if "FAULT" in msg.upper():
                _finalize_eis()          # flush any partial EIS sweep before stopping
                with data.lock:
                    data.fault_msg = msg
                    data.run_done  = True
                stop_event.set()
            elif "Sequence complete" in msg or "Loop guard" in msg:
                _finalize_eis()
                with data.lock:
                    data.run_done = True
                # Firmware follows a loop-guard trip with its own "Sequence
                # complete/ended." message and returns to IDLE on its own —
                # no 's' needs to be sent back.
            continue

        # --- Interleaved EIS output (tagged so it never collides with DATA,) ---
        #   EISV,BEFORE,<step>,<v>   EISV,AFTER,<step>,<v>
        #   EISHDR,<columns>         EIS,<f>,<real>,<imag>,<mag>,<phase>   EISDONE,<step>
        if line.startswith("EISV,"):
            parts = line.split(",")
            if len(parts) >= 4:
                which = parts[1].strip().upper()
                try:
                    step_i = int(parts[2]); volt = float(parts[3])
                except ValueError:
                    continue
                if which == "BEFORE":
                    _finalize_eis()   # flush any straggler before starting fresh
                    with data.lock:
                        loop_it = data.loop_iter[-1] if data.loop_iter else 0
                    eis_cur = {"step_idx": step_i, "loop_iter": loop_it,
                               "v_before": volt, "v_after": None,
                               "freq": [], "real": [], "imag": [],
                               "mag": [], "phase": []}
                    with data.lock:
                        data.eis_live = eis_cur   # expose the (empty) live sweep
                    print(f"\r  [EIS] step {step_i} start — V_before={volt:.4f} V           ")
                elif which == "AFTER":
                    _finalize_eis(v_after=volt)
            continue

        if line.startswith("EISHDR"):
            if eis_cur is None:   # header without a preceding BEFORE — start bare
                eis_cur = {"step_idx": -1, "loop_iter": 0, "v_before": None,
                           "v_after": None, "freq": [], "real": [], "imag": [],
                           "mag": [], "phase": []}
            continue

        if line.startswith("EISDONE,"):
            # Sweep data complete; the file is written on the following EISV,AFTER
            # (so V_after lands in the header). Nothing to do here.
            continue

        if line.startswith("EIS,"):
            parts = line.split(",")
            if len(parts) == 6 and eis_cur is not None:
                try:
                    fr, re_, im_, mg, ph = (float(x) for x in parts[1:6])
                except ValueError:
                    continue  # e.g. a TIMEOUT row — skip the point
                # Append under the lock: a live plot copies these lists under the
                # same lock, so the mutation and the copy never interleave.
                with data.lock:
                    eis_cur["freq"].append(fr);  eis_cur["real"].append(re_)
                    eis_cur["imag"].append(im_); eis_cur["mag"].append(mg)
                    eis_cur["phase"].append(ph)
                # Live echo of the impedance point as it's measured.
                n = len(eis_cur["freq"])
                print(f"  [EIS] {n:>3}  {fr:>10.2f} Hz   "
                      f"Z'={re_:>9.5f}  Z''={im_:>9.5f}  "
                      f"|Z|={mg:>9.5f} Ohm  {ph:>7.2f} deg")
            continue

        if line.startswith("HEADER,"):
            continue
        if not line.startswith("DATA,"):
            continue

        parts = line.split(",")
        if len(parts) != 10:
            continue  # firmware now emits 10 fields (added step_idx, loop_iter)

        try:
            fw_ms    = int(parts[1])
            state    = parts[2]                # STRING now, e.g. "CHARGING"
            v_batt   = float(parts[3])
            i_charge = float(parts[4])
            i_disc   = float(parts[5])
            temp_c   = float(parts[6])
            temp2_c  = float(parts[7])
            step_idx = int(parts[8])
            loop_it  = int(parts[9])
        except ValueError:
            continue

        with data.lock:
            if data.t0 is None:
                data.t0 = fw_ms / 1000.0
            elapsed = fw_ms / 1000.0 - data.t0

            data.all_t.append(elapsed)
            data.all_i_ch.append(i_charge)
            data.all_i_dis.append(i_disc)
            data.all_state.append(state)

            data.t.append(elapsed)
            data.v.append(v_batt)
            data.i_ch.append(i_charge)
            data.i_dis.append(i_disc)
            data.temp.append(temp_c)
            data.temp2.append(temp2_c)
            data.state.append(state)
            data.step_idx.append(step_idx)
            data.loop_iter.append(loop_it)

        csv_writer.writerow([f"{elapsed:.3f}", state, f"{v_batt:.4f}",
                              f"{i_charge:.3f}", f"{i_disc:.3f}",
                              f"{temp_c:.1f}", f"{temp2_c:.1f}",
                              step_idx, loop_it])
        csv_file.flush()


# ─────────────────────────────────────────────────────────────────────────────
# CSV logger
# ─────────────────────────────────────────────────────────────────────────────

def open_csv(protocol):
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    fn = f"run_{protocol['name']}_{ts}.csv"
    fh = open(fn, "w", newline="")
    wr = csv.writer(fh)
    wr.writerow(["# Battery Cycler sequence run — " + ts])
    wr.writerow(["# protocol=" + protocol["name"],
                 "capacity_mah=" + str(protocol["capacity_mah"]),
                 "num_steps=" + str(len(protocol["steps"]))])
    wr.writerow(["elapsed_s", "state", "v_batt", "i_charge", "i_discharge",
                 "temp_nmos_c", "temp_batt_c", "step_idx", "loop_iter"])
    fh.flush()
    print(f"  Logging to: {os.path.abspath(fn)}")
    return fh, wr


# ─────────────────────────────────────────────────────────────────────────────
# Live plot
# ─────────────────────────────────────────────────────────────────────────────

def build_segments(x, y, states):
    if len(x) < 2:
        return []
    segments = []
    seg_x, seg_y = [x[0]], [y[0]]
    seg_col = STATE_COLORS.get(states[0], COLOR_CHARGE)
    for i in range(1, len(x)):
        c = STATE_COLORS.get(states[i], COLOR_CHARGE)
        if c != seg_col:
            seg_x.append(x[i]); seg_y.append(y[i])
            segments.append((seg_x, seg_y, seg_col))
            seg_x, seg_y, seg_col = [x[i]], [y[i]], c
        else:
            seg_x.append(x[i]); seg_y.append(y[i])
    segments.append((seg_x, seg_y, seg_col))
    return segments


def run_plot(data: RunData, protocol: dict, stop_event: threading.Event):
    matplotlib.use("TkAgg")

    fig = plt.figure(figsize=(13, 8))
    fig.patch.set_facecolor("#1e1e2e")

    ax_v = fig.add_subplot(2, 1, 1)
    ax_i = fig.add_subplot(2, 2, 3)
    ax_t = fig.add_subplot(2, 2, 4)

    def style_ax(ax):
        ax.set_facecolor("#2a2a3e")
        ax.tick_params(colors="#cccccc", labelsize=8)
        for spine in ax.spines.values():
            spine.set_edgecolor("#555577")

    for ax in (ax_v, ax_i, ax_t):
        style_ax(ax)

    fig.suptitle(f"Battery Cycler — {protocol['name']}", color="#eeeeee", fontsize=13)
    ax_v.set_ylabel("Voltage (V)",      color="#cccccc")
    ax_i.set_ylabel("Current (A)",      color="#cccccc")
    ax_i.set_xlabel("Elapsed time (s)", color="#cccccc")
    ax_t.set_ylabel("Temperature (°C)", color="#cccccc")
    ax_t.set_xlabel("Elapsed time (s)", color="#cccccc")

    legend_v = [
        Line2D([0], [0], color=COLOR_CHARGE,    lw=2, label="Charging"),
        Line2D([0], [0], color=COLOR_DISCHARGE, lw=2, label="Discharging"),
        Line2D([0], [0], color=COLOR_REST,      lw=2, label="Rest / Idle"),
        Line2D([0], [0], color=COLOR_FAULT,     lw=2, label="Fault"),
    ]
    legend_t = [
        Line2D([0], [0], color=COLOR_TEMP,  lw=2, label="T NMOS"),
        Line2D([0], [0], color=COLOR_TEMP2, lw=2, label="T Batt", linestyle="--"),
    ]

    def legend_kw():
        return dict(facecolor="#2a2a3e", edgecolor="#555577", labelcolor="#cccccc", fontsize=7)

    fig.tight_layout(rect=[0, 0, 1, 0.95], h_pad=3.0, w_pad=2.0)
    plt.ion(); plt.show()

    num_steps = len(protocol["steps"])

    while not stop_event.is_set():
        with data.lock:
            if len(data.t) < 2:
                time.sleep(0.5)
                continue
            t      = np.array(data.t)
            v      = np.array(data.v)
            i_ch   = np.array(data.i_ch)
            i_dis  = np.array(data.i_dis)
            temp   = np.array(data.temp)
            temp2  = np.array(data.temp2)
            states = list(data.state)
            cur_step = data.step_idx[-1] if data.step_idx else -1
            cur_loop = data.loop_iter[-1] if data.loop_iter else 0
            cur_st   = states[-1] if states else "IDLE"

        t_max = t[-1]
        window_s = ROLLING_WINDOW_S
        t_min = max(0.0, t_max - window_s)
        mask = t >= t_min
        t_w, v_w   = t[mask], v[mask]
        i_ch_w, i_dis_w = i_ch[mask], i_dis[mask]
        temp_w, temp2_w = temp[mask], temp2[mask]
        st_w = [s for s, m in zip(states, mask) if m]

        ax_v.cla(); style_ax(ax_v)
        ax_v.set_ylabel("Voltage (V)", color="#cccccc")
        ax_v.legend(handles=legend_v, loc="upper left", **legend_kw())
        for sx, sy, sc in build_segments(t_w.tolist(), v_w.tolist(), st_w):
            ax_v.plot(sx, sy, color=sc, lw=1.8)
        if len(t_w):
            ax_v.set_xlim(t_min, t_min + window_s)

        step_lbl = f"Step {cur_step}/{num_steps - 1}" if cur_step >= 0 else ""
        loop_lbl = f"  Loop iter {cur_loop}" if protocol.get("loop") and cur_loop > 0 else ""
        ax_v.text(0.99, 0.06, f"{cur_st}  {step_lbl}{loop_lbl}",
                   transform=ax_v.transAxes, ha="right", va="bottom",
                   color="#eeeeee", fontsize=9,
                   bbox=dict(boxstyle="round,pad=0.3", facecolor="#2a2a3e", edgecolor="#555577"))
        ax_v.set_xlabel("Elapsed time (s)", color="#cccccc")

        ax_i.cla(); style_ax(ax_i)
        ax_i.set_ylabel("Current (A)", color="#cccccc")
        ax_i.set_xlabel("Elapsed time (s)", color="#cccccc")
        ax_i.plot(t_w, i_ch_w,  color=COLOR_CHARGE,    lw=1.5, label="I charge")
        ax_i.plot(t_w, i_dis_w, color=COLOR_DISCHARGE, lw=1.5, label="I discharge")
        ax_i.legend(loc="upper left", **legend_kw())
        if len(t_w):
            ax_i.set_xlim(t_min, t_min + window_s)
            ax_i.xaxis.set_major_formatter(ticker.FuncFormatter(lambda x, _: f"{int(x)}"))

        ax_t.cla(); style_ax(ax_t)
        ax_t.set_ylabel("Temperature (°C)", color="#cccccc")
        ax_t.set_xlabel("Elapsed time (s)", color="#cccccc")
        ax_t.plot(t_w, temp_w,  color=COLOR_TEMP,  lw=1.5)
        ax_t.plot(t_w, temp2_w, color=COLOR_TEMP2, lw=1.5, linestyle="--")
        ax_t.legend(handles=legend_t, loc="upper left", **legend_kw())
        if len(t_w):
            ax_t.set_xlim(t_min, t_min + window_s)
            ax_t.xaxis.set_major_formatter(ticker.FuncFormatter(lambda x, _: f"{int(x)}"))

        try:
            fig.canvas.draw_idle()
            fig.canvas.flush_events()
        except Exception:
            break

        time.sleep(1.0)

    plt.ioff()
    plt.close("all")


def live_monitor(ser, duration=8.0):
    """Stream live sensor readings for `duration` s so the user can verify
    sensors before committing to a run."""
    print("\n" + "-" * 56)
    print("  Live sensor monitor  (Ctrl+C to skip)")
    print("-" * 56)
    deadline = time.time() + duration
    t0 = time.time()
    prev_print = 0.0
    try:
        while time.time() < deadline:
            raw = ser.readline().decode("utf-8", errors="replace").strip()
            if not raw.startswith("DATA,"):
                continue
            parts = raw.split(",")
            if len(parts) != 10:
                continue
            try:
                state  = parts[2]
                v      = float(parts[3])
                i_ch   = float(parts[4])
                t1     = float(parts[6])
                t2     = float(parts[7])
            except ValueError:
                continue
            now = time.time()
            if now - prev_print >= 1.0:
                prev_print = now
                print(f"  [{now - t0:5.1f}s]  {state:<12}  V={v:.4f}V  "
                      f"I_ch={i_ch:.3f}A  T_nmos={t1:.1f}C  T_batt={t2:.1f}C")
    except KeyboardInterrupt:
        print("  (monitor skipped)")
    print("-" * 56)
    ser.reset_input_buffer()


# ─────────────────────────────────────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────────────────────────────────────

def main():
    print("\n" + "=" * 56)
    print("  Battery Cycler CLI — Sequence Edition")
    print("=" * 56)

    print("\n  Scanning for serial ports...")
    port = find_serial_port()
    print(f"  Connecting to {port} @ 115200 baud...")
    try:
        ser = open_port(port)
    except serial.SerialException as e:
        print(f"  [ERROR] Could not open port: {e}")
        sys.exit(1)

    print("  Waiting for live data (up to 12s)...")
    deadline = time.time() + 12.0
    got_data = False
    while time.time() < deadline:
        raw = ser.readline().decode("utf-8", errors="replace").strip()
        if "READY" in raw:
            print("  Firmware ready.")
        if raw.startswith("DATA,"):
            got_data = True
            break
    if not got_data:
        print("  [ERROR] No data received from board. Is it running / is the")
        print("  Arduino Serial Monitor closed?")
        ser.close()
        sys.exit(1)

    print("  Board responding.")
    ser.reset_input_buffer()

    print("\n  Streaming live readings for ~8s to verify sensors.")
    live_monitor(ser, duration=8.0)

    configure_thermal_limits(ser)

    print("\n  Build a new protocol, or load a saved one?")
    print("    [n] New   [l] Load")
    choice = input("  Select [n/l]: ").strip().lower()
    if choice == "l":
        protocol = load_protocol()
        if protocol is None:
            protocol = build_protocol()
    else:
        protocol = build_protocol()

    print_protocol_summary(protocol)
    if not prompt_yesno("\n  Upload and start this protocol?", default_yes=True):
        print("  Aborted.")
        ser.close()
        sys.exit(0)

    ser.reset_input_buffer()
    upload_protocol(ser, protocol)

    csv_fh, csv_wr = open_csv(protocol)

    data = RunData()
    stop_event = threading.Event()
    reader_thread = threading.Thread(
        target=serial_reader, args=(ser, data, csv_wr, csv_fh, stop_event), daemon=True)
    reader_thread.start()

    print("\n  Run started. Close the plot window or Ctrl+C to stop early.\n")
    try:
        run_plot(data, protocol, stop_event)
    except KeyboardInterrupt:
        print("\n  Interrupted by user.")
    finally:
        stop_event.set()
        try:
            ser.write(b's\n')   # safe even if the sequence already finished
        except Exception:
            pass
        reader_thread.join(timeout=3)
        csv_fh.close()
        ser.close()

    with data.lock:
        fault = data.fault_msg
        done  = data.run_done
        n_eis = len(data.eis_sweeps)

    print("\n" + "=" * 56)
    if fault:
        print(f"  [FAULT] {fault}")
    elif done:
        print("  Run complete.")
    else:
        print("  Run stopped early.")
    print("  CSV saved.")
    if n_eis:
        print(f"  {n_eis} interleaved EIS sweep(s) saved next to the run CSV.")
    print("=" * 56 + "\n")


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
EIS_GUI.py — Two-window EIS control panel with live ECM fitting.

Window 1: Launcher  — all configuration collected before measurement begins.
Window 2: EISApp    — live Nyquist + Bode plots, serial log, fit overlay.

Requires:
    pip install pyserial matplotlib impedance scipy
"""

import csv
import glob
import os
import queue
import re
import subprocess
import sys
import threading
import time
import tkinter as tk
import tkinter.filedialog as filedialog
import tkinter.font as tkfont
import warnings
from datetime import datetime
from tkinter import ttk

import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import numpy as np
from scipy.signal import medfilt

try:
    from impedance.models.circuits import CustomCircuit
    IMPEDANCE_AVAILABLE = True
except ImportError:
    IMPEDANCE_AVAILABLE = False

warnings.filterwarnings("ignore")

# ── defaults ──────────────────────────────────────────────────────────────────
DEFAULT_BAUD        = 115200
DEFAULT_POINTS      = 50
DEFAULT_START       = 65000.0
DEFAULT_STOP        = 1.0
DEFAULT_SWEEPS      = 1
DEFAULT_FQBN        = "esp32:esp32:esp32"
DEFAULT_OUTPUT_DIR  = "data"
DEFAULT_TIMEOUT     = 120

DEFAULT_CIRCUIT     = "L0-R0-p(R1,CPE1)-p(R2,CPE2)-W1"
DEFAULT_INIT_GUESS  = "1e-7, 0.050, 0.015, 0.025, 0.70, 0.030, 0.022, 0.82, 0.060"

SWEEP_COLORS = [
    "#89b4fa", "#a6e3a1", "#f38ba8",
    "#fab387", "#cba6f7", "#f9e2af", "#94e2d5",
]

# ── colour palette (Catppuccin-inspired dark) ─────────────────────────────────
BG        = "#1e1e2e"
BG_ENTRY  = "#2a2a3e"
BG_PANEL  = "#181825"
FG        = "#cdd6f4"
FG_DIM    = "#6c7086"
ACCENT    = "#89b4fa"
ACCENT2   = "#cba6f7"
BTN_GO    = "#a6e3a1"
BTN_STOP  = "#f38ba8"
BTN_SAVE  = "#89b4fa"
BTN_FG    = "#1e1e2e"
BORDER    = "#313244"
SUCCESS   = "#a6e3a1"
WARNING   = "#f9e2af"
ERROR_COL = "#f38ba8"


# ── serial / parsing helpers ──────────────────────────────────────────────────
def strip_timestamp(raw_line):
    return re.sub(r"^\d{2}:\d{2}:\d{2}\.\d+ -> ", "", raw_line.strip())

def parse_data_line(raw_line):
    line  = strip_timestamp(raw_line)
    parts = line.split(",")
    if len(parts) != 5:
        return None
    try:
        return tuple(float(p) for p in parts)
    except ValueError:
        return None

def is_header_line(raw_line):
    return strip_timestamp(raw_line).startswith("Frequency(Hz)")

def is_sweep_complete(raw_line):
    return "SWEEP COMPLETE" in raw_line or "sweep complete" in raw_line.lower()

def _empty_sweep():
    return dict(freq=[], real=[], imag=[], mag=[], phase=[])


# ── port helpers ──────────────────────────────────────────────────────────────
def list_serial_ports():
    try:
        import serial.tools.list_ports
        return [p.device for p in serial.tools.list_ports.comports()]
    except Exception:
        return []

def find_best_port():
    try:
        import serial.tools.list_ports
        for p in serial.tools.list_ports.comports():
            desc = (p.description or "").lower()
            mfr  = (p.manufacturer  or "").lower()
            if any(k in desc or k in mfr
                   for k in ("cp210", "ch340", "ftdi", "esp", "uart", "usb serial")):
                return p.device
        ports = serial.tools.list_ports.comports()
        return ports[0].device if ports else ""
    except Exception:
        return ""


# ── .ino patching ─────────────────────────────────────────────────────────────
def find_ino(hint=None):
    if hint and os.path.isfile(hint):
        return hint
    candidates = glob.glob("*.ino")
    return candidates[0] if len(candidates) == 1 else None

def patch_ino(ino_path, num_points, start_freq, stop_freq):
    with open(ino_path) as f:
        original = f.read()
    patched = original
    patched = re.sub(r"(#define\s+NUM_POINTS\s+)\d+",
                     rf"\g<1>{num_points}", patched)
    patched = re.sub(r"(float\s+startFreq\s*=\s*)[\d.eE+\-f]+\s*;",
                     rf"\g<1>{stop_freq:.1f}f;", patched)
    patched = re.sub(r"(float\s+stopFreq\s*=\s*)[\d.eE+\-f]+\s*;",
                     rf"\g<1>{start_freq:.1f}f;", patched)
    return patched, original


# ── CSV helpers ───────────────────────────────────────────────────────────────
def make_output_dir(path):
    try:
        os.makedirs(path, exist_ok=True)
        return path
    except OSError:
        return "."

def save_sweep_csv(sweep_data, sweep_idx, cfg, output_dir, run_ts, note=""):
    filename = f"EIS_{run_ts}_sweep{sweep_idx + 1:02d}.csv"
    filepath = os.path.join(output_dir, filename)
    n        = len(sweep_data["freq"])
    order    = sorted(range(n), key=lambda k: sweep_data["freq"][k])
    with open(filepath, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow([f"# EIS sweep  {datetime.now().isoformat()}"])
        w.writerow([f"# Sweep index: {sweep_idx + 1}"])
        w.writerow([f"# Port: {cfg['port']}  Baud: {cfg['baud']}"])
        w.writerow([f"# NUM_POINTS: {cfg['points']}  "
                    f"Start: {cfg['start']} Hz  Stop: {cfg['stop']} Hz"])
        w.writerow([f"# Points recorded: {n}"])
        if note.strip():
            w.writerow([f"# Note: {note.strip()}"])
        w.writerow(["Frequency(Hz)", "Real(Ohm)", "Imaginary(Ohm)",
                    "Magnitude(Ohm)", "Phase(Deg)"])
        for k in order:
            w.writerow([
                f"{sweep_data['freq'][k]:.6g}",
                f"{sweep_data['real'][k]:.6g}",
                f"{sweep_data['imag'][k]:.6g}",
                f"{sweep_data['mag'][k]:.6g}",
                f"{sweep_data['phase'][k]:.4f}",
            ])
    return filepath

def append_fit_to_csv(filepath, circuit_str, param_names, param_values, nrmse):
    """Append a # FIT: comment block to an existing EIS CSV.

    Written as plain text lines so '#' is never CSV-quoted.
    pandas read_csv(comment='#') skips all of these lines — fully
    backward-compatible with existing batch analysis scripts.
    """
    with open(filepath, "a", newline="\n") as f:
        f.write("# --- ECM FIT ---\n")
        f.write(f"# Circuit: {circuit_str}\n")
        f.write(f"# NRMSE: {nrmse:.5f}\n")
        f.write("# " + ",".join(param_names) + "\n")
        f.write("# " + ",".join(f"{v:.6g}" for v in param_values) + "\n")


# ── IQR outlier filter ────────────────────────────────────────────────────────
def remove_outliers(freq, Z, window=5, iqr_factor=3.0, abs_floor=1e-4, trim_n=1):
    """
    Complex-plane IQR outlier filter.  On clean data MAD → 0 causes MAD-based
    filters to remove everything; IQR is derived from residual spread and stays
    stable even on near-perfect sweeps.
    """
    k         = window if window % 2 == 1 else window + 1
    real_med  = medfilt(Z.real, kernel_size=k)
    imag_med  = medfilt(Z.imag, kernel_size=k)
    residuals = np.abs(Z - (real_med + 1j * imag_med))

    q75, q25  = np.percentile(residuals, [75, 25])
    threshold = max(iqr_factor * (q75 - q25), abs_floor)

    outlier_mask = residuals > threshold
    if trim_n > 0:
        outlier_mask[:trim_n]  = True
        outlier_mask[-trim_n:] = True

    return freq[~outlier_mask], Z[~outlier_mask], outlier_mask


# ── ECM fitting helper ────────────────────────────────────────────────────────
def run_ecm_fit(freq, Z, circuit_str, initial_guess):
    """
    Run impedance.py fit.  Returns (param_names, param_values, nrmse, Z_model_dense)
    or raises on failure.
    """
    if not IMPEDANCE_AVAILABLE:
        raise RuntimeError("impedance package not installed")

    circuit = CustomCircuit(circuit_str, initial_guess=list(initial_guess))
    circuit.fit(freq, Z, weight_by_modulus=True)

    Z_pred = circuit.predict(freq)
    nrmse  = (np.sqrt(np.mean(np.abs(Z - Z_pred) ** 2))
              / np.abs(Z).mean())

    param_names, _ = circuit.get_param_names()

    # Dense prediction for a smooth model curve on the Nyquist plot
    f_dense  = np.logspace(np.log10(freq.min()), np.log10(freq.max()), 400)
    Z_dense  = circuit.predict(f_dense)

    return param_names, circuit.parameters_, nrmse, Z_dense


# ── background serial worker ──────────────────────────────────────────────────
class SerialWorker(threading.Thread):
    """
    Daemon thread.  Reads serial lines and posts messages to gui_queue.

    Messages:
        ("log",        str)
        ("point",      freq, r, i, m, p)
        ("sweep_done", sweep_idx)
        ("status",     str)
        ("error",      str)
    """
    def __init__(self, cfg, gui_queue, stop_event):
        super().__init__(daemon=True)
        self.cfg        = cfg
        self.q          = gui_queue
        self.stop_event = stop_event

    def run(self):
        try:
            import serial
        except ImportError:
            self.q.put(("error", "pyserial not installed."))
            return

        cfg = self.cfg
        try:
            ser = serial.Serial(cfg["port"], cfg["baud"], timeout=1)
        except serial.SerialException as e:
            self.q.put(("error", f"Cannot open {cfg['port']}: {e}"))
            return

        self.q.put(("status", f"Connected: {cfg['port']} @ {cfg['baud']} baud"))
        time.sleep(2)
        ser.reset_input_buffer()

        num_sweeps      = cfg["sweeps"]
        timeout_s       = cfg["timeout"]
        sweep_idx       = 0
        in_data_section = False
        header_seen     = False
        last_data_time  = time.time()

        def send_start():
            nonlocal in_data_section, header_seen, last_data_time
            in_data_section = False
            header_seen     = False
            last_data_time  = time.time()
            
            # Dynamically format the command with current GUI parameters
            cmd_str = f"START {cfg['points']} {cfg['start']} {cfg['stop']}\n"
            
            self.q.put(("log", f"-- Sending {cmd_str.strip()} (sweep {sweep_idx+1}/{num_sweeps}) --"))
            ser.write(cmd_str.encode("utf-8"))
            ser.flush()

        send_start()

        try:
            while sweep_idx < num_sweeps and not self.stop_event.is_set():

                if time.time() - last_data_time > timeout_s:
                    self.q.put(("status", f"Timeout after {timeout_s}s — aborted."))
                    self.q.put(("log",   f"WARNING: No data for {timeout_s}s, aborting."))
                    break

                raw = ser.readline()
                if not raw:
                    continue

                last_data_time = time.time()

                try:
                    line = raw.decode("utf-8", errors="replace").rstrip()
                except Exception:
                    continue

                self.q.put(("log", strip_timestamp(line)))

                if not header_seen and is_header_line(line):
                    header_seen     = True
                    in_data_section = True
                    continue

                if in_data_section:
                    parsed = parse_data_line(line)
                    if parsed:
                        freq, real, imag, mag, phase = parsed
                        self.q.put(("point", freq, real, imag, mag, phase))

                    if is_sweep_complete(line):
                        self.q.put(("sweep_done", sweep_idx))
                        sweep_idx += 1
                        if sweep_idx < num_sweeps and not self.stop_event.is_set():
                            time.sleep(0.5)
                            send_start()

        finally:
            ser.close()
            self.q.put(("status", "Idle — serial port closed."))

    def send_stop(self, port, baud):
        try:
            import serial
            s = serial.Serial(port, baud, timeout=1)
            s.write(b"STOP\n")
            s.flush()
            time.sleep(0.1)
            s.close()
        except Exception:
            pass


# ═════════════════════════════════════════════════════════════════════════════
# WINDOW 1 — LAUNCHER
# ═════════════════════════════════════════════════════════════════════════════
class LauncherWindow(tk.Tk):
    """
    Pre-flight configuration window.  Collect all settings, then launch
    the main EIS plot window.
    """

    def __init__(self):
        super().__init__()
        self.title("EIS — Launch Configuration")
        self.configure(bg=BG)
        self.resizable(False, False)

        self._build_styles()
        self._build_ui()
        self._refresh_ports()

        # Update initial-guess labels when model string changes
        self.var_circuit.trace_add("write", lambda *_: self._on_circuit_change())

    # ── styles ────────────────────────────────────────────────────────────────
    def _build_styles(self):
        s = ttk.Style(self)
        s.theme_use("clam")
        s.configure(".",
                    background=BG, foreground=FG,
                    fieldbackground=BG_ENTRY,
                    bordercolor=BORDER, relief="flat")
        for w in ("TLabel", "TFrame", "TLabelframe"):
            s.configure(w, background=BG, foreground=FG)
        s.configure("TLabelframe.Label", background=BG, foreground=ACCENT,
                    font=("Helvetica", 9, "bold"))
        s.configure("TEntry",    fieldbackground=BG_ENTRY, foreground=FG,
                    insertcolor=FG, bordercolor=BORDER)
        s.configure("TCombobox", fieldbackground=BG_ENTRY, foreground=FG,
                    selectbackground=BG_ENTRY, arrowcolor=FG)
        s.map("TCombobox",
              fieldbackground=[("readonly", BG_ENTRY)],
              foreground     =[("readonly", FG)])
        s.configure("TCheckbutton", background=BG, foreground=FG,
                    indicatorcolor=BG_ENTRY)
        s.map("TCheckbutton", background=[("active", BG)])
        s.configure("Dim.TLabel", background=BG, foreground=FG_DIM,
                    font=("Helvetica", 8))
        s.configure("Head.TLabel", background=BG, foreground=ACCENT,
                    font=("Helvetica", 9, "bold"))
        s.configure("TSeparator", background=BORDER)

    # ── main UI ───────────────────────────────────────────────────────────────
    def _build_ui(self):
        outer = ttk.Frame(self, padding=16)
        outer.grid(row=0, column=0, sticky="nsew")
        outer.columnconfigure(0, weight=1)

        # Title bar
        title_frame = tk.Frame(outer, bg=BG_PANEL,
                               highlightbackground=BORDER,
                               highlightthickness=1)
        title_frame.grid(row=0, column=0, sticky="ew", pady=(0, 12))
        tk.Label(title_frame, text="EIS Control Panel",
                 bg=BG_PANEL, fg=ACCENT,
                 font=("Helvetica", 14, "bold"),
                 padx=14, pady=8).pack(side="left")
        tk.Label(title_frame, text="Launch Configuration",
                 bg=BG_PANEL, fg=FG_DIM,
                 font=("Helvetica", 10),
                 padx=4, pady=8).pack(side="left")

        # Two-column body
        body = ttk.Frame(outer)
        body.grid(row=1, column=0, sticky="nsew")
        body.columnconfigure(0, weight=1)
        body.columnconfigure(1, weight=1)

        self._build_left_col(body)
        self._build_right_col(body)

        # Bottom bar
        self._build_bottom_bar(outer)

    def _build_left_col(self, parent):
        col = ttk.Frame(parent, padding=(0, 0, 10, 0))
        col.grid(row=0, column=0, sticky="nsew")
        col.columnconfigure(1, weight=1)
        r = 0

        # ── Connection ──
        self._sec(col, "⬡  Connection", r);  r += 1
        ttk.Label(col, text="Port").grid(row=r, column=0, sticky="w", pady=2)
        pf = ttk.Frame(col)
        pf.grid(row=r, column=1, sticky="ew", pady=2)
        pf.columnconfigure(0, weight=1)
        self.var_port = tk.StringVar(value=find_best_port())
        self.combo_port = ttk.Combobox(pf, textvariable=self.var_port, width=14)
        self.combo_port.grid(row=0, column=0, sticky="ew")
        ttk.Button(pf, text="⟳", width=2,
                   command=self._refresh_ports).grid(row=0, column=1, padx=(4, 0))
        r += 1

        ttk.Label(col, text="Baud").grid(row=r, column=0, sticky="w", pady=2)
        self.var_baud = tk.StringVar(value=str(DEFAULT_BAUD))
        ttk.Entry(col, textvariable=self.var_baud, width=10).grid(
            row=r, column=1, sticky="w", pady=2)
        r += 1

        self._sep(col, r);  r += 1

        # ── Sweep Parameters ──
        self._sec(col, "⬡  Sweep Parameters", r);  r += 1
        for label, varname, default in [
            ("Points",     "var_points", str(DEFAULT_POINTS)),
            ("Start (Hz)", "var_start",  str(int(DEFAULT_START))),
            ("Stop (Hz)",  "var_stop",   str(int(DEFAULT_STOP))),
            ("Sweeps",     "var_sweeps", str(DEFAULT_SWEEPS)),
            ("Timeout (s)","var_timeout",str(DEFAULT_TIMEOUT)),
        ]:
            ttk.Label(col, text=label).grid(row=r, column=0, sticky="w", pady=2)
            var = tk.StringVar(value=default)
            setattr(self, varname, var)
            ttk.Entry(col, textvariable=var, width=10).grid(
                row=r, column=1, sticky="w", pady=2)
            r += 1

        self._sep(col, r);  r += 1

        # ── Output ──
        self._sec(col, "⬡  Output", r);  r += 1
        ttk.Label(col, text="Save dir").grid(row=r, column=0, sticky="w", pady=2)
        df = ttk.Frame(col); df.grid(row=r, column=1, sticky="ew", pady=2)
        df.columnconfigure(0, weight=1)
        self.var_outdir = tk.StringVar(value=DEFAULT_OUTPUT_DIR)
        ttk.Entry(df, textvariable=self.var_outdir).grid(row=0, column=0, sticky="ew")
        ttk.Button(df, text="…", width=2,
                   command=self._browse_dir).grid(row=0, column=1, padx=(4,0))
        r += 1

        ttk.Label(col, text="Note").grid(row=r, column=0, sticky="w", pady=2)
        self.var_note = tk.StringVar()
        ttk.Entry(col, textvariable=self.var_note).grid(
            row=r, column=1, sticky="ew", pady=2)
        r += 1
        ttk.Label(col, text="appended to CSV header",
                  style="Dim.TLabel").grid(row=r, column=1, sticky="w")
        r += 1

        self.var_save  = tk.BooleanVar(value=True)
        self.var_equal = tk.BooleanVar(value=False)
        ttk.Checkbutton(col, text="Auto-save sweep CSV",
                        variable=self.var_save).grid(
            row=r, column=0, columnspan=2, sticky="w", pady=1); r += 1
        ttk.Checkbutton(col, text="Equal aspect on Nyquist",
                        variable=self.var_equal).grid(
            row=r, column=0, columnspan=2, sticky="w", pady=1); r += 1

        self._sep(col, r); r += 1

        # ── Advanced / Flash (collapsible) ──
        self._build_advanced(col, r)

    def _build_right_col(self, parent):
        col = ttk.Frame(parent, padding=(10, 0, 0, 0))
        col.grid(row=0, column=1, sticky="nsew")
        col.columnconfigure(1, weight=1)
        r = 0

        # ── ECM Fitting ──
        self._sec(col, "⬡  Equivalent Circuit Model (ECM)", r); r += 1

        if not IMPEDANCE_AVAILABLE:
            tk.Label(col, text="⚠  impedance package not found.\n"
                               "  pip install impedance\n"
                               "  Fitting will be disabled.",
                     bg=BG, fg=WARNING, justify="left",
                     font=("Courier", 9)).grid(
                row=r, column=0, columnspan=2, sticky="w", pady=4)
            r += 1

        # Circuit string
        ttk.Label(col, text="Circuit").grid(row=r, column=0, sticky="nw", pady=2)
        self.var_circuit = tk.StringVar(value=DEFAULT_CIRCUIT)
        circ_entry = ttk.Entry(col, textvariable=self.var_circuit, width=32)
        circ_entry.grid(row=r, column=1, sticky="ew", pady=2); r += 1

        ttk.Label(col, text="",
                  style="Dim.TLabel").grid(row=r, column=1, sticky="w")
        self._circuit_hint = ttk.Label(col,
                                       text="e.g.  R0-p(R1,CPE1)-p(R2,C2)-W1",
                                       style="Dim.TLabel")
        self._circuit_hint.grid(row=r, column=1, sticky="w"); r += 1

        # Initial guesses
        ttk.Label(col, text="Initial\nguesses",
                  justify="right").grid(row=r, column=0, sticky="nw", pady=2)
        self.var_guess = tk.StringVar(value=DEFAULT_INIT_GUESS)
        guess_entry = ttk.Entry(col, textvariable=self.var_guess, width=32)
        guess_entry.grid(row=r, column=1, sticky="ew", pady=2); r += 1

        ttk.Label(col, text="comma-separated, one per parameter",
                  style="Dim.TLabel").grid(row=r, column=1, sticky="w"); r += 1

        # Preview button + param name display
        preview_btn = tk.Button(col, text="Preview params",
                                bg=BG_ENTRY, fg=ACCENT,
                                font=("Helvetica", 9),
                                relief="flat", padx=6, pady=3,
                                command=self._preview_params)
        preview_btn.grid(row=r, column=1, sticky="w", pady=(2, 0)); r += 1

        self._param_preview = tk.Text(col, bg=BG_PANEL, fg=FG_DIM,
                                      font=("Courier", 8),
                                      relief="flat", height=6, width=36,
                                      state="disabled", wrap="none")
        self._param_preview.grid(row=r, column=0, columnspan=2,
                                 sticky="ew", pady=(4, 0)); r += 1

        self._sep(col, r); r += 1

        # ── Filter settings ──
        self._sec(col, "⬡  Outlier Filter", r); r += 1

        ttk.Label(col, text="IQR factor").grid(row=r, column=0, sticky="w", pady=2)
        self.var_iqr    = tk.StringVar(value="3.0")
        ttk.Entry(col, textvariable=self.var_iqr, width=8).grid(
            row=r, column=1, sticky="w", pady=2); r += 1

        ttk.Label(col, text="Abs floor (Ω)").grid(row=r, column=0, sticky="w", pady=2)
        self.var_floor  = tk.StringVar(value="1e-4")
        ttk.Entry(col, textvariable=self.var_floor, width=8).grid(
            row=r, column=1, sticky="w", pady=2); r += 1

        ttk.Label(col, text="Trim endpoints").grid(row=r, column=0, sticky="w", pady=2)
        self.var_trim   = tk.StringVar(value="1")
        ttk.Entry(col, textvariable=self.var_trim, width=4).grid(
            row=r, column=1, sticky="w", pady=2); r += 1

        ttk.Label(col,
                  text="Points removed from each end of sweep regardless of residual.",
                  style="Dim.TLabel",
                  wraplength=260).grid(
            row=r, column=0, columnspan=2, sticky="w"); r += 1

        self._sep(col, r); r += 1

        # ── Fit options ──
        self._sec(col, "⬡  Fit Options", r); r += 1

        self.var_fit_enable = tk.BooleanVar(value=IMPEDANCE_AVAILABLE)
        ttk.Checkbutton(col, text="Run ECM fit after each sweep",
                        variable=self.var_fit_enable).grid(
            row=r, column=0, columnspan=2, sticky="w", pady=1); r += 1

        self.var_fit_autosave = tk.BooleanVar(value=True)
        ttk.Checkbutton(col, text="Append fit result to CSV automatically",
                        variable=self.var_fit_autosave).grid(
            row=r, column=0, columnspan=2, sticky="w", pady=1); r += 1

        ttk.Label(col, text="Fit is saved as  # FIT:  comment block at end of CSV,\n"
                             "transparent to pandas  read_csv(comment='#').",
                  style="Dim.TLabel",
                  wraplength=260).grid(
            row=r, column=0, columnspan=2, sticky="w", pady=(0, 4)); r += 1

    def _build_advanced(self, parent, start_row):
        self._adv_visible = tk.BooleanVar(value=False)
        ttk.Checkbutton(parent, text="▶  Advanced / Flash firmware",
                        variable=self._adv_visible,
                        command=self._toggle_advanced).grid(
            row=start_row, column=0, columnspan=2, sticky="w", pady=(4, 0))

        self._adv_frame = ttk.Frame(parent)
        adv = self._adv_frame
        adv.columnconfigure(1, weight=1)

        ttk.Label(adv, text=".ino path").grid(row=0, column=0, sticky="w", pady=2)
        ino_f = ttk.Frame(adv); ino_f.grid(row=0, column=1, sticky="ew", pady=2)
        ino_f.columnconfigure(0, weight=1)
        self.var_ino = tk.StringVar(value=find_ino() or "")
        ttk.Entry(ino_f, textvariable=self.var_ino).grid(row=0, column=0, sticky="ew")
        ttk.Button(ino_f, text="…", width=2,
                   command=self._browse_ino).grid(row=0, column=1, padx=(4, 0))

        ttk.Label(adv, text="FQBN").grid(row=1, column=0, sticky="w", pady=2)
        self.var_fqbn = tk.StringVar(value=DEFAULT_FQBN)
        ttk.Entry(adv, textvariable=self.var_fqbn).grid(
            row=1, column=1, sticky="ew", pady=2)

        self.var_flash = tk.BooleanVar(value=False)
        ttk.Checkbutton(adv, text="Patch + compile + upload before sweep",
                        variable=self.var_flash).grid(
            row=2, column=0, columnspan=2, sticky="w", pady=2)

    def _toggle_advanced(self):
        if self._adv_visible.get():
            self._adv_frame.grid(row=99, column=0, columnspan=2,
                                 sticky="ew", padx=(8, 0), pady=(0, 4))
        else:
            self._adv_frame.grid_remove()

    def _build_bottom_bar(self, parent):
        bar = ttk.Frame(parent, padding=(0, 12, 0, 0))
        bar.grid(row=2, column=0, sticky="ew")

        self._launch_status = tk.StringVar(value="")
        ttk.Label(bar, textvariable=self._launch_status,
                  style="Dim.TLabel", wraplength=340).pack(side="left")

        tk.Button(bar, text="  LAUNCH  ",
                  bg=BTN_GO, fg=BTN_FG,
                  font=("Helvetica", 12, "bold"),
                  relief="flat", padx=10, pady=6,
                  command=self._on_launch).pack(side="right")

    # ── helpers ───────────────────────────────────────────────────────────────
    def _sec(self, parent, text, row):
        ttk.Label(parent, text=text, style="Head.TLabel",
                  background=BG).grid(
            row=row, column=0, columnspan=2, sticky="w", pady=(10, 2))

    def _sep(self, parent, row):
        ttk.Separator(parent, orient="horizontal").grid(
            row=row, column=0, columnspan=2, sticky="ew", pady=4)

    def _refresh_ports(self):
        ports = list_serial_ports()
        self.combo_port["values"] = ports
        if not self.var_port.get() and ports:
            self.var_port.set(ports[0])

    def _browse_dir(self):
        d = filedialog.askdirectory(initialdir=self.var_outdir.get() or ".")
        if d:
            self.var_outdir.set(d)

    def _browse_ino(self):
        f = filedialog.askopenfilename(
            filetypes=[("Arduino sketch", "*.ino"), ("All files", "*.*")])
        if f:
            self.var_ino.set(f)

    def _on_circuit_change(self):
        """Clear preview text when circuit string is edited."""
        self._set_preview_text("(press  Preview params  to validate)", FG_DIM)

    def _set_preview_text(self, text, colour=FG_DIM):
        self._param_preview.configure(state="normal")
        self._param_preview.delete("1.0", "end")
        self._param_preview.insert("end", text)
        self._param_preview.configure(fg=colour, state="disabled")

    def _preview_params(self):
        """Validate circuit string and show parameter names vs guess values."""
        if not IMPEDANCE_AVAILABLE:
            self._set_preview_text("impedance package not available.", ERROR_COL)
            return
        try:
           guesses = [float(x.strip()) for x in self.var_guess.get().split(",")]
        except ValueError:
            self._set_preview_text("Initial guesses must be comma-separated numbers.",
                           ERROR_COL)
            return

        try:
            c = CustomCircuit(self.var_circuit.get().strip(),
                      initial_guess=guesses)
        except Exception as e:
            self._set_preview_text(f"Circuit parse error:\n{e}", ERROR_COL)
            return
        
        names, _ = c.get_param_names()
        n_params  = len(names)

        if len(guesses) != n_params:
            self._set_preview_text(
                f"⚠  Circuit needs {n_params} values, "
                f"you provided {len(guesses)}.\n\n"
                f"Parameters:  {', '.join(names)}",
                WARNING)
            return

        lines = [f"{'Parameter':<14}  {'Initial guess':>14}",
                 "─" * 30]
        for nm, gv in zip(names, guesses):
            lines.append(f"  {nm:<12}  {gv:>14g}")
        lines.append(f"\n✓  {n_params} parameter(s) — OK")
        self._set_preview_text("\n".join(lines), SUCCESS)

    # ── validation & launch ───────────────────────────────────────────────────
    def _collect_cfg(self):
        """Validate all fields and return a config dict, or None on error."""
        def _float(varname, label):
            try:
                return float(getattr(self, varname).get())
            except ValueError:
                raise ValueError(f"{label} must be a number.")

        def _int(varname, label):
            try:
                return int(getattr(self, varname).get())
            except ValueError:
                raise ValueError(f"{label} must be an integer.")

        try:
            cfg = {
                "port":         self.var_port.get(),
                "baud":         _int("var_baud",    "Baud rate"),
                "points":       _int("var_points",  "Points"),
                "start":        _float("var_start", "Start Hz"),
                "stop":         _float("var_stop",  "Stop Hz"),
                "sweeps":       _int("var_sweeps",  "Sweeps"),
                "timeout":      _int("var_timeout", "Timeout"),
                "save":         self.var_save.get(),
                "outdir":       self.var_outdir.get(),
                "note":         self.var_note.get(),
                "equal":        self.var_equal.get(),
                "flash":        self.var_flash.get(),
                "ino":          self.var_ino.get(),
                "fqbn":         self.var_fqbn.get(),
                "fit_enable":   self.var_fit_enable.get() and IMPEDANCE_AVAILABLE,
                "fit_autosave": self.var_fit_autosave.get(),
                "circuit_str":  self.var_circuit.get().strip(),
                "iqr_factor":   _float("var_iqr",   "IQR factor"),
                "abs_floor":    _float("var_floor",  "Abs floor"),
                "trim_n":       _int("var_trim",     "Trim endpoints"),
            }
        except ValueError as e:
            self._launch_status.set(f"⚠  {e}")
            return None

        # Parse initial guesses
        if cfg["fit_enable"]:
            try:
                guesses = [float(x.strip())
                           for x in self.var_guess.get().split(",")]
            except ValueError:
                self._launch_status.set("⚠  Initial guesses must be comma-separated numbers.")
                return None

            # Validate count against circuit
            try:
                c = CustomCircuit(cfg["circuit_str"], initial_guess=guesses)
                names, _ = c.get_param_names()
            except Exception as e:
                    self._launch_status.set(f"⚠  Circuit error: {e}")
                    return None

            cfg["initial_guess"] = guesses
        else:
            cfg["initial_guess"] = []

        if not cfg["port"]:
            self._launch_status.set("⚠  No serial port selected.")
            return None

        return cfg

    def _on_launch(self):
        cfg = self._collect_cfg()
        if cfg is None:
            return

        # Flash firmware if requested
        if cfg["flash"]:
            ino = cfg["ino"] or find_ino()
            if not ino or not os.path.isfile(ino):
                self._launch_status.set("⚠  .ino file not found.")
                return
            self._launch_status.set("Compiling & uploading firmware…")
            self.update()
            try:
                patched, _ = patch_ino(ino, cfg["points"],
                                       cfg["start"], cfg["stop"])
                with open(ino, "w") as f:
                    f.write(patched)
                r = subprocess.run(
                    ["arduino-cli", "compile", "--fqbn", cfg["fqbn"],
                     os.path.dirname(os.path.abspath(ino))],
                    capture_output=True, text=True)
                if r.returncode != 0:
                    self._launch_status.set("⚠  Compile failed — see terminal.")
                    print(r.stderr)
                    return
                r = subprocess.run(
                    ["arduino-cli", "upload", "--fqbn", cfg["fqbn"],
                     "--port", cfg["port"],
                     os.path.dirname(os.path.abspath(ino))],
                    capture_output=True, text=True)
                if r.returncode != 0:
                    self._launch_status.set("⚠  Upload failed — see terminal.")
                    print(r.stderr)
                    return
            except FileNotFoundError:
                self._launch_status.set("⚠  arduino-cli not in PATH.")
                return

        # Hand off to main plot window
        self.withdraw()
        app = EISApp(cfg, on_close=self._on_app_close)
        app.mainloop()

    def _on_app_close(self):
        """Called when EISApp is closed — return to launcher or exit."""
        self.deiconify()


# ═════════════════════════════════════════════════════════════════════════════
# WINDOW 2 — MAIN PLOT WINDOW
# ═════════════════════════════════════════════════════════════════════════════
class EISApp(tk.Toplevel):

    def __init__(self, cfg, on_close=None):
        super().__init__()
        self.title("EIS — Live Measurement")
        self.configure(bg=BG)
        self.resizable(True, True)
        self.minsize(1100, 680)

        self._cfg           = cfg
        self._on_close_cb   = on_close

        self._worker        = None
        self._stop_event    = threading.Event()
        self._gui_queue     = queue.Queue()
        self._run_ts        = ""
        self._sweep_idx     = 0
        self._current       = _empty_sweep()
        self._completed     = []
        self._output_dir    = None
        self._saved_paths   = []        # one path per completed sweep

        # Fit state
        self._last_fit       = None     # dict with fit results for latest sweep
        self._fit_thread     = None

        self._build_styles()
        self._build_layout()

        self.protocol("WM_DELETE_WINDOW", self._on_close)

        # Start automatically
        self.after(200, self._on_start)

        # Poll queue every 50 ms
        self.after(50, self._poll_queue)

    # ── styles ────────────────────────────────────────────────────────────────
    def _build_styles(self):
        s = ttk.Style(self)
        s.theme_use("clam")
        s.configure(".",
                    background=BG, foreground=FG,
                    fieldbackground=BG_ENTRY,
                    bordercolor=BORDER, relief="flat")
        for w in ("TLabel", "TFrame", "TLabelframe"):
            s.configure(w, background=BG, foreground=FG)
        s.configure("TLabelframe.Label", background=BG, foreground=ACCENT,
                    font=("Helvetica", 9, "bold"))
        s.configure("Dim.TLabel", background=BG, foreground=FG_DIM,
                    font=("Helvetica", 8))
        s.configure("TSeparator", background=BORDER)
        s.configure("TCheckbutton", background=BG, foreground=FG,
                    indicatorcolor=BG_ENTRY)
        s.map("TCheckbutton", background=[("active", BG)])

    # ── layout ────────────────────────────────────────────────────────────────
    def _build_layout(self):
        self.columnconfigure(1, weight=1)
        self.rowconfigure(0, weight=1)

        left  = ttk.Frame(self, padding=10)
        right = ttk.Frame(self, padding=(0, 10, 10, 10))
        left.grid(row=0, column=0, sticky="nsew")
        right.grid(row=0, column=1, sticky="nsew")
        right.columnconfigure(0, weight=1)
        right.rowconfigure(0, weight=3)
        right.rowconfigure(1, weight=1)

        self._build_control_panel(left)
        self._build_plot_panel(right)
        self._build_log_panel(right)

    # ── left: control panel ───────────────────────────────────────────────────
    def _build_control_panel(self, parent):
        parent.columnconfigure(1, weight=1)
        r = 0

        # ── Summary header ──
        cfg = self._cfg
        self._sec(parent, "⬡  Session", r); r += 1
        for label, value in [
            ("Port",    f"{cfg['port']} @ {cfg['baud']}"),
            ("Freq",    f"{cfg['start']} – {cfg['stop']} Hz"),
            ("Points",  str(cfg['points'])),
            ("Sweeps",  str(cfg['sweeps'])),
        ]:
            ttk.Label(parent, text=label, style="Dim.TLabel").grid(
                row=r, column=0, sticky="w", pady=1)
            ttk.Label(parent, text=value).grid(
                row=r, column=1, sticky="w", pady=1)
            r += 1

        # ECM info
        if cfg["fit_enable"]:
            self._sec(parent, "⬡  ECM Model", r); r += 1
            ttk.Label(parent, text="Circuit", style="Dim.TLabel").grid(
                row=r, column=0, sticky="nw", pady=1)
            tk.Label(parent, text=cfg["circuit_str"],
                     bg=BG, fg=ACCENT2,
                     font=("Courier", 8),
                     wraplength=150,
                     justify="left").grid(
                row=r, column=1, sticky="w", pady=1)
            r += 1

        self._sep(parent, r); r += 1

        # ── Plot options ──
        self._sec(parent, "⬡  Plot Options", r); r += 1

        self.var_equal = tk.BooleanVar(value=cfg.get("equal", False))
        ttk.Checkbutton(parent, text="Equal aspect (Nyquist)",
                        variable=self.var_equal,
                        command=self._on_equal_aspect).grid(
            row=r, column=0, columnspan=2, sticky="w", pady=1); r += 1

        self.var_show_fit = tk.BooleanVar(value=True)
        ttk.Checkbutton(parent, text="Show fit overlay",
                        variable=self.var_show_fit,
                        command=lambda: self._redraw_plot()).grid(
            row=r, column=0, columnspan=2, sticky="w", pady=1); r += 1

        self._sep(parent, r); r += 1

        # ── Buttons ──
        self._sec(parent, "⬡  Control", r); r += 1

        btn_frame = ttk.Frame(parent)
        btn_frame.grid(row=r, column=0, columnspan=2, pady=(4, 0)); r += 1

        self.btn_stop = tk.Button(
            btn_frame, text="  STOP  ",
            bg=BTN_STOP, fg=BTN_FG,
            font=("Helvetica", 11, "bold"),
            relief="flat", padx=6, pady=4,
            state="normal",
            command=self._on_stop)
        self.btn_stop.pack(side="left", padx=(0, 6))

        self.btn_rerun = tk.Button(
            btn_frame, text="  RE-RUN  ",
            bg=BTN_GO, fg=BTN_FG,
            font=("Helvetica", 11, "bold"),
            relief="flat", padx=6, pady=4,
            state="disabled",
            command=self._on_start)
        self.btn_rerun.pack(side="left")

        self._sep(parent, r); r += 1

        # ── Save controls ──
        self._sec(parent, "⬡  Save", r); r += 1

        self.btn_save_fit = tk.Button(
            parent, text="Save Fit to CSV",
            bg=BTN_SAVE, fg=BTN_FG,
            font=("Helvetica", 9, "bold"),
            relief="flat", padx=6, pady=3,
            state="disabled",
            command=self._save_fit_manual)
        self.btn_save_fit.grid(row=r, column=0, columnspan=2,
                               sticky="ew", pady=(2, 1)); r += 1

        ttk.Label(parent,
                  text="Appends # FIT: block to the most recent sweep CSV.",
                  style="Dim.TLabel", wraplength=190).grid(
            row=r, column=0, columnspan=2, sticky="w"); r += 1

        self._sep(parent, r); r += 1

        # ── Status ──
        self.var_status = tk.StringVar(value="Starting…")
        ttk.Label(parent, textvariable=self.var_status,
                  style="Dim.TLabel", wraplength=200).grid(
            row=r, column=0, columnspan=2, sticky="w", pady=(6, 0))

    # ── right: plot ───────────────────────────────────────────────────────────
    def _build_plot_panel(self, parent):
        frame = ttk.LabelFrame(parent, text="Live Plot", padding=4)
        frame.grid(row=0, column=0, sticky="nsew", pady=(0, 6))
        frame.columnconfigure(0, weight=1)
        frame.rowconfigure(0, weight=1)

        self._fig    = Figure(figsize=(9, 4.5), dpi=96,
                              facecolor=BG_PANEL, tight_layout=True)
        self._ax_nyq   = self._fig.add_subplot(121)
        self._ax_bode  = self._fig.add_subplot(122)
        self._ax_phase = self._ax_bode.twinx()

        self._style_axes()

        self._canvas = FigureCanvasTkAgg(self._fig, master=frame)
        self._canvas.get_tk_widget().grid(row=0, column=0, sticky="nsew")

    def _style_axes(self):
        for ax in (self._ax_nyq, self._ax_bode, self._ax_phase):
            ax.set_facecolor(BG_PANEL)
            ax.tick_params(colors=FG_DIM, labelsize=7)
            for spine in ax.spines.values():
                spine.set_edgecolor(BORDER)

        self._ax_nyq.set_xlabel("Z' (Ω)",    color=FG_DIM, fontsize=8)
        self._ax_nyq.set_ylabel("−Z'' (Ω)", color=FG_DIM, fontsize=8)
        self._ax_nyq.set_title("Nyquist",    color=FG,     fontsize=9)

        self._ax_bode.set_xlabel("Freq (Hz)", color=FG_DIM, fontsize=8)
        self._ax_bode.set_ylabel("|Z| (Ω)",   color=FG_DIM, fontsize=8)
        self._ax_bode.set_title("Bode",       color=FG,     fontsize=9)
        self._ax_bode.set_xscale("log")

        self._ax_phase.set_ylabel("Phase (°)", color=FG_DIM, fontsize=8)
        self._ax_phase.yaxis.set_label_position("right")
        self._ax_phase.tick_params(axis="y", colors=FG_DIM, labelsize=7)
        self._fig.patch.set_facecolor(BG_PANEL)

    # ── right: log ────────────────────────────────────────────────────────────
    def _build_log_panel(self, parent):
        frame = ttk.LabelFrame(parent, text="Serial Log / Fit Results", padding=4)
        frame.grid(row=1, column=0, sticky="nsew")
        frame.columnconfigure(0, weight=1)
        frame.rowconfigure(0, weight=1)

        self._log = tk.Text(
            frame, bg=BG_PANEL, fg=FG_DIM,
            font=("Courier", 8), relief="flat",
            state="disabled", wrap="none",
            height=9)
        scroll = ttk.Scrollbar(frame, command=self._log.yview)
        self._log.configure(yscrollcommand=scroll.set)
        self._log.grid(row=0, column=0, sticky="nsew")
        scroll.grid(row=0, column=1, sticky="ns")

        # Colour tags
        self._log.tag_configure("fit",     foreground=ACCENT2)
        self._log.tag_configure("ok",      foreground=SUCCESS)
        self._log.tag_configure("warn",    foreground=WARNING)
        self._log.tag_configure("err",     foreground=ERROR_COL)
        self._log.tag_configure("section", foreground=ACCENT)

    # ── helpers ───────────────────────────────────────────────────────────────
    def _sec(self, parent, text, row):
        ttk.Label(parent, text=text,
                  foreground=ACCENT,
                  font=("Helvetica", 9, "bold"),
                  background=BG).grid(
            row=row, column=0, columnspan=2, sticky="w", pady=(8, 2))

    def _sep(self, parent, row):
        ttk.Separator(parent, orient="horizontal").grid(
            row=row, column=0, columnspan=2, sticky="ew", pady=4)

    def _log_write(self, text, tag=None):
        self._log.configure(state="normal")
        if tag:
            self._log.insert("end", text + "\n", tag)
        else:
            self._log.insert("end", text + "\n")
        self._log.see("end")
        self._log.configure(state="disabled")

    def _on_equal_aspect(self):
        if self.var_equal.get():
            self._ax_nyq.set_aspect("equal", adjustable="datalim")
        else:
            self._ax_nyq.set_aspect("auto")
        self._canvas.draw_idle()

    # ── start / stop ──────────────────────────────────────────────────────────
    def _on_start(self):
        self._run_ts    = datetime.now().strftime("%Y%m%d_%H%M%S")
        self._sweep_idx = 0
        self._current   = _empty_sweep()
        self._completed = []
        self._saved_paths = []
        self._last_fit  = None
        self._clear_plot()

        cfg = self._cfg
        if cfg.get("save"):
            self._output_dir = make_output_dir(cfg["outdir"])
        else:
            self._output_dir = None

        self._stop_event.clear()
        self._worker = SerialWorker(cfg, self._gui_queue, self._stop_event)
        self._worker.start()

        self.btn_stop.config(state="normal")
        self.btn_rerun.config(state="disabled")
        self.btn_save_fit.config(state="disabled")
        self.var_status.set(f"Running sweep 1/{cfg['sweeps']}…")

    def _on_stop(self):
        self._stop_event.set()
        cfg = self._cfg
        if cfg.get("port"):
            threading.Thread(
                target=self._worker.send_stop,
                args=(cfg["port"], cfg["baud"]),
                daemon=True).start()
        self.var_status.set("Stop requested…")

    def _on_close(self):
        self._stop_event.set()
        self.destroy()
        if self._on_close_cb:
            self._on_close_cb()

    # ── queue polling ─────────────────────────────────────────────────────────
    def _poll_queue(self):
        try:
            while True:
                msg = self._gui_queue.get_nowait()
                self._handle_msg(msg)
        except queue.Empty:
            pass
        self.after(50, self._poll_queue)

    def _handle_msg(self, msg):
        kind = msg[0]

        if kind == "log":
            self._log_write(msg[1])

        elif kind == "point":
            _, freq, real, imag, mag, phase = msg
            self._current["freq"].append(freq)
            self._current["real"].append(real)
            self._current["imag"].append(imag)
            self._current["mag"].append(mag)
            self._current["phase"].append(phase)
            self._redraw_plot()

        elif kind == "sweep_done":
            sweep_idx = msg[1]
            if self._current["freq"]:
                self._completed.append(
                    {k: list(v) for k, v in self._current.items()})
            self._current   = _empty_sweep()
            self._sweep_idx = sweep_idx + 1
            self._redraw_plot()

            saved_path = None
            if self._output_dir and self._cfg.get("save"):
                saved_path = save_sweep_csv(
                    self._completed[-1], sweep_idx,
                    self._cfg, self._output_dir, self._run_ts,
                    note=self._cfg.get("note", ""))
                self._saved_paths.append(saved_path)
                self._log_write(f"Saved: {saved_path}", "ok")
            else:
                self._saved_paths.append(None)

            # Trigger ECM fit in background
            if self._cfg.get("fit_enable") and len(self._completed) > 0:
                self._run_fit_async(self._completed[-1], sweep_idx, saved_path)

            total = self._cfg.get("sweeps", 1)
            if self._sweep_idx < total:
                self.var_status.set(
                    f"Running sweep {self._sweep_idx + 1}/{total}…")
            else:
                self.var_status.set(f"Complete — {total} sweep(s) done.")
                self.btn_stop.config(state="disabled")
                self.btn_rerun.config(state="normal")

        elif kind == "status":
            self.var_status.set(msg[1])

        elif kind == "error":
            self._log_write("ERROR: " + msg[1], "err")
            self.var_status.set("Error — see log.")
            self.btn_stop.config(state="disabled")
            self.btn_rerun.config(state="normal")

        elif kind == "fit_result":
            self._handle_fit_result(msg[1])

        elif kind == "fit_error":
            self._log_write(f"ECM fit failed: {msg[1]}", "warn")

    # ── ECM fitting ───────────────────────────────────────────────────────────
    def _run_fit_async(self, sweep_data, sweep_idx, saved_path):
        """Kick off ECM fit in a daemon thread so the GUI stays responsive."""
        cfg = self._cfg

        def _worker():
            try:
                freq  = np.array(sweep_data["freq"])
                Z     = (np.array(sweep_data["real"])
                         + 1j * np.array(sweep_data["imag"]))
                order = np.argsort(freq)
                freq, Z = freq[order], Z[order]

                freq_c, Z_c, _ = remove_outliers(
                    freq, Z,
                    iqr_factor=cfg.get("iqr_factor", 3.0),
                    abs_floor=cfg.get("abs_floor",   1e-4),
                    trim_n=cfg.get("trim_n",          1))

                names, params, nrmse, Z_dense = run_ecm_fit(
                    freq_c, Z_c,
                    cfg["circuit_str"],
                    cfg["initial_guess"])

                self._gui_queue.put(("fit_result", {
                    "sweep_idx":   sweep_idx,
                    "saved_path":  saved_path,
                    "names":       names,
                    "params":      params,
                    "nrmse":       nrmse,
                    "Z_dense":     Z_dense,
                    "freq_c":      freq_c,
                    "Z_c":         Z_c,
                }))
            except Exception as e:
                self._gui_queue.put(("fit_error", str(e)))

        t = threading.Thread(target=_worker, daemon=True)
        t.start()
        self._fit_thread = t

    def _handle_fit_result(self, result):
        self._last_fit = result

        # ── Print to log ──
        names  = result["names"]
        params = result["params"]
        nrmse  = result["nrmse"]
        sidx   = result["sweep_idx"] + 1

        self._log_write(
            f"\n{'─'*44}", "section")
        self._log_write(
            f"  ECM Fit — Sweep {sidx}   "
            f"circuit: {self._cfg['circuit_str']}", "section")
        self._log_write(
            f"  NRMSE = {nrmse:.5f}", "fit")

        col_w = max(len(n) for n in names) + 2
        self._log_write(
            f"  {'Parameter':{col_w}}  {'Value':>14}", "fit")
        self._log_write(
            f"  {'─'*(col_w+18)}", "fit")
        for n, v in zip(names, params):
            self._log_write(f"  {n:{col_w}}  {v:>14.6g}", "fit")
        self._log_write(f"{'─'*44}\n", "section")

        # Auto-save fit to CSV if enabled
        if self._cfg.get("fit_autosave") and result.get("saved_path"):
            try:
                append_fit_to_csv(
                    result["saved_path"],
                    self._cfg["circuit_str"],
                    names, params, nrmse)
                self._log_write(
                    f"  Fit appended to: {result['saved_path']}", "ok")
            except Exception as e:
                self._log_write(f"  Could not append fit: {e}", "warn")
        elif result.get("saved_path") and not self._cfg.get("fit_autosave"):
            # Enable manual save button
            self.btn_save_fit.config(state="normal")

        # Always enable manual save button after a fit
        self.btn_save_fit.config(state="normal")

        # Redraw to show fit overlay
        self._redraw_plot()

    def _save_fit_manual(self):
        """Save fit result to the most recently saved sweep CSV."""
        if self._last_fit is None:
            self._log_write("No fit result available yet.", "warn")
            return

        # If a path was saved for the latest sweep, use it;
        # otherwise ask the user where to save
        saved_path = self._last_fit.get("saved_path")
        if not saved_path:
            saved_path = filedialog.asksaveasfilename(
                defaultextension=".csv",
                filetypes=[("CSV files", "*.csv"), ("All files", "*.*")],
                title="Append fit to which CSV?")
            if not saved_path:
                return

        try:
            append_fit_to_csv(
                saved_path,
                self._cfg["circuit_str"],
                self._last_fit["names"],
                self._last_fit["params"],
                self._last_fit["nrmse"])
            self._log_write(f"Fit saved to: {saved_path}", "ok")
            self.btn_save_fit.config(state="disabled")
        except Exception as e:
            self._log_write(f"Save failed: {e}", "err")

    # ── plotting ──────────────────────────────────────────────────────────────
    def _clear_plot(self):
        self._ax_nyq.cla()
        self._ax_bode.cla()
        self._ax_phase.cla()
        self._style_axes()
        self._canvas.draw_idle()

    def _redraw_plot(self):
        self._ax_nyq.cla()
        self._ax_bode.cla()
        self._ax_phase.cla()
        self._style_axes()

        all_sweeps = self._completed + (
            [self._current] if self._current["freq"] else [])

        for i, sweep in enumerate(all_sweeps):
            if not sweep["freq"]:
                continue
            color  = SWEEP_COLORS[i % len(SWEEP_COLORS)]
            label  = f"Sweep {i + 1}"
            n      = len(sweep["freq"])
            order  = sorted(range(n), key=lambda k: sweep["freq"][k])

            freq_s  = [sweep["freq"][k]  for k in order]
            real_s  = [sweep["real"][k]  for k in order]
            nimag_s = [-sweep["imag"][k] for k in order]
            mag_s   = [sweep["mag"][k]   for k in order]
            phase_s = [sweep["phase"][k] for k in order]

            self._ax_nyq.scatter(real_s, nimag_s,
                                 color=color, s=18, alpha=0.85,
                                 label=label, zorder=3)
            self._ax_bode.plot(freq_s, mag_s,
                               color=color, lw=1.5,
                               marker="o", ms=3, label=label)
            self._ax_phase.plot(freq_s, phase_s,
                                color=color, lw=1.5,
                                ls="--", marker="^", ms=3)

        # ── ECM fit overlay on Nyquist ──
        fit = self._last_fit
        if (fit is not None
                and self.var_show_fit.get()
                and self._cfg.get("fit_enable")):

            Z_d = fit["Z_dense"]
            self._ax_nyq.plot(
                Z_d.real, -Z_d.imag,
                color="white", lw=2, alpha=0.7,
                linestyle="--", zorder=4,
                label=f"ECM fit  (NRMSE={fit['nrmse']:.4f})")

            # Also show the cleaned points used for fitting
            Z_c = fit["Z_c"]
            self._ax_nyq.scatter(
                Z_c.real, -Z_c.imag,
                color="white", s=8, alpha=0.3,
                zorder=2)

        # ── Legend & formatting ──
        if any(s["freq"] for s in all_sweeps):
            self._ax_nyq.legend(fontsize=7, loc="best",
                                facecolor=BORDER, labelcolor=FG,
                                edgecolor=BORDER)
            self._ax_bode.legend(fontsize=7, loc="best",
                                 facecolor=BORDER, labelcolor=FG,
                                 edgecolor=BORDER)
            if self.var_equal.get():
                self._ax_nyq.set_aspect("equal", adjustable="datalim")

        self._canvas.draw_idle()


# ── entry point ───────────────────────────────────────────────────────────────
def main():
    launcher = LauncherWindow()
    launcher.mainloop()

if __name__ == "__main__":
    main()

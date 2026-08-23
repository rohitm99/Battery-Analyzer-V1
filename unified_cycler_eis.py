#!/usr/bin/env python3
"""
unified_cycler_eis.py  —  Iteration 1 wrapper for the combined Cycler + EIS board
=================================================================================

The combined ESP32-S3 (N16R8 Dev Module) firmware exposes ONE serial port that
runs TWO protocols, switched by a firmware-side mode flag:

  * CYCLER mode (default on boot) — line-prefixed protocol:
        DATA,time_ms,state,v_batt,i_charge,i_discharge,temp_t1,temp_t2,step_idx,loop_iter
        MSG,...            status / log lines
        single-char commands (c/d/g/G/s, W/L/C sequence builder, parameter setters)

  * EIS mode — entered by sending 'e' from cycler IDLE.  Raw V9 line protocol:
        START <points> <startHz> <stopHz>   RCAL pre-sweep + battery sweep
        STOP / EXIT
    Sweep emits  "Frequency(Hz),Real(Ohm),Imaginary(Ohm),Magnitude(Ohm),Phase(Deg)"
    then  freq,real,imag,mag,phase  rows, then  "SWEEP COMPLETE".
    No DATA telemetry is emitted while in EIS mode, so the two streams never mix.

The hardware is physically EITHER cycling OR doing EIS, so this wrapper is modal:
a terminal menu runs one session at a time over a single shared serial connection.

  Cycling -> reuses your cycler CLI's control logic (build/load, upload, serial
             reader, CSV) verbatim. The live plot is re-expressed here so the
             temperature axis keeps a sensible minimum span, and telemetry is
             echoed to the terminal about once a minute for long runs.
  EIS     -> 'e' -> banner handshake, then a live window with a large Nyquist
             over a smaller Bode (|Z| + phase), plus optional impedance.py ECM
             fitting. On close it sends 'EXIT' to hand the board back to IDLE.

Requires:  pyserial, matplotlib, numpy      (cycling + EIS live plots)
Optional:  impedance, scipy                 (ECM fitting; auto-disabled if absent)
           Your cycler CLI file must sit next to this script (see CYCLER_FILE).

Usage:     python unified_cycler_eis.py
"""

import csv
import glob
import importlib.util
import os
import queue
import re
import sys
import threading
import time
import warnings
import tkinter as tk
from datetime import datetime

import matplotlib
matplotlib.use("TkAgg")   # set before the cycler module imports pyplot
import matplotlib.ticker as ticker
from matplotlib.lines import Line2D
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import numpy as np

import serial  # used directly here and via the cycler module

warnings.filterwarnings("ignore")   # quiet impedance.py / scipy fit chatter

# ── Optional stacks (graceful degradation if missing) ─────────────────────────
# scipy.medfilt powers the outlier filter; impedance.py powers ECM fitting.
try:
    from scipy.signal import medfilt
    SCIPY_AVAILABLE = True
except ImportError:
    SCIPY_AVAILABLE = False

try:
    from impedance.models.circuits import CustomCircuit, calculateCircuitLength
    IMPEDANCE_AVAILABLE = True
except ImportError:
    IMPEDANCE_AVAILABLE = False


# ─────────────────────────────────────────────────────────────────────────────
# Tunables
# ─────────────────────────────────────────────────────────────────────────────
CYCLER_TELEMETRY_INTERVAL_S = 60.0   # how often to echo telemetry to the terminal
CYCLER_MIN_TEMP_SPAN_C      = 10.0   # min °C window on the temp plot (de-noises)
_COOLDOWN_WAIT_S            = 6.0    # must exceed firmware COOLDOWN_MS (5 s)

# Cycler plot styling
CYCLER_LW_VOLTAGE     = 2.6   # voltage trace line width
CYCLER_LW_CURRENT     = 2.4   # current trace line width
CYCLER_LABEL_FONTSIZE = 12    # axis label size (slightly larger than default)
CYCLER_TICK_FONTSIZE  = 9     # tick label size

# Over-current fault (firmware MAX_CURRENT_FAULT, set via the 'F' command).
# The firmware boots this at 0.500 A and applies it as a SINGLE global ceiling to
# both charge and discharge (i >= limit -> fault), so any protocol step above the
# default trips instantly unless we raise it. We size one limit for the whole run
# from the protocol's peak step current, plus a margin.
OVERCURRENT_MARGIN = 0.10    # +10% headroom above the run's peak commanded current
OVERCURRENT_PROMPT = True    # True: confirm/override each run; False: apply silently
OVERCURRENT_FLOOR  = 0.100   # never set the limit below this (A); guards all-rest runs

# Candidate keys for a step's current magnitude in the protocol dict. Auto-detect
# is only trusted when EVERY step yields a current via one of these; otherwise the
# wrapper falls back to asking for the peak current (never guesses a partial value).
_CURRENT_KEYS = ("current_A", "current_a", "current", "curr_A", "amps",
                 "current_amps", "i_set", "i")

DEFAULT_CIRCUIT    = "L0-R0-p(R1,CPE1)-p(R2,CPE2)-W1"
DEFAULT_INIT_GUESS = "1e-7, 0.050, 0.015, 0.025, 0.70, 0.030, 0.022, 0.82, 0.060"


# ─────────────────────────────────────────────────────────────────────────────
# Load your existing cycler CLI as a module (control logic stays authoritative)
# ─────────────────────────────────────────────────────────────────────────────
CYCLER_FILE_CANDIDATES = [
    "battery_cycler.py",
    "battery_cycler_1_.py",
    "battery_cycler_1.py",
]


def _find_cycler_file():
    here = os.path.dirname(os.path.abspath(__file__))
    for name in CYCLER_FILE_CANDIDATES:
        p = os.path.join(here, name)
        if os.path.isfile(p):
            return p
    for p in sorted(glob.glob(os.path.join(here, "*cycler*.py"))):
        if os.path.abspath(p) == os.path.abspath(__file__):
            continue
        try:
            with open(p, "r", errors="ignore") as fh:
                src = fh.read()
            if "def build_protocol" in src and "def serial_reader" in src:
                return p
        except OSError:
            continue
    return None


def _load_cycler_module():
    path = _find_cycler_file()
    if not path:
        print("\n  [FATAL] Could not find your cycler CLI file.")
        print("  Place it next to this script (e.g. 'battery_cycler.py'), or edit")
        print("  CYCLER_FILE_CANDIDATES at the top of this file.\n")
        sys.exit(1)
    spec = importlib.util.spec_from_file_location("cycler_cli", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    print(f"  Cycler CLI loaded from: {path}")
    return mod


cyc = _load_cycler_module()


# ─────────────────────────────────────────────────────────────────────────────
# EIS parsing helpers (small, inlined so this wrapper has no dep on the EIS GUI)
# ─────────────────────────────────────────────────────────────────────────────
def strip_timestamp(raw_line):
    return re.sub(r"^\d{2}:\d{2}:\d{2}\.\d+ -> ", "", raw_line.strip())


def parse_eis_data_line(raw_line):
    """A sweep row is exactly 5 comma floats: f,real,imag,mag,phase.
    Cycler DATA (10 fields) and MSG (non-float) lines return None."""
    parts = strip_timestamp(raw_line).split(",")
    if len(parts) != 5:
        return None
    try:
        return tuple(float(p) for p in parts)
    except ValueError:
        return None


def is_eis_header(raw_line):
    return strip_timestamp(raw_line).startswith("Frequency(Hz)")


def is_sweep_complete(raw_line):
    return "sweep complete" in raw_line.lower()


def _empty_sweep():
    return dict(freq=[], real=[], imag=[], mag=[], phase=[])


def _sorted_view(sweep):
    """Return a sweep's fields as lists sorted by ascending frequency."""
    n = len(sweep["freq"])
    order = sorted(range(n), key=lambda k: sweep["freq"][k])
    return {k: [sweep[k][j] for j in order]
            for k in ("freq", "real", "imag", "mag", "phase")}


# ─────────────────────────────────────────────────────────────────────────────
# ECM fitting helpers (ported from EIS_GUI.py)
# ─────────────────────────────────────────────────────────────────────────────
def _circuit_param_count(circuit_str):
    n_params = calculateCircuitLength(circuit_str)
    dummy    = [0.1] * n_params
    c        = CustomCircuit(circuit_str, initial_guess=dummy)
    names, _ = c.get_param_names()
    return names, n_params


def remove_outliers(freq, Z, window=5, iqr_factor=3.0, abs_floor=1e-4, trim_n=1):
    """Complex-plane IQR outlier filter (stable on near-perfect sweeps)."""
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


def run_ecm_fit(freq, Z, circuit_str, initial_guess):
    """impedance.py fit. Returns (names, values, nrmse, Z_model_dense)."""
    if not IMPEDANCE_AVAILABLE:
        raise RuntimeError("impedance package not installed")
    circuit = CustomCircuit(circuit_str, initial_guess=list(initial_guess))
    circuit.fit(freq, Z, weight_by_modulus=True)
    Z_pred = circuit.predict(freq)
    nrmse  = np.sqrt(np.mean(np.abs(Z - Z_pred) ** 2)) / np.abs(Z).mean()
    names, _ = circuit.get_param_names()
    f_dense = np.logspace(np.log10(freq.min()), np.log10(freq.max()), 400)
    Z_dense = circuit.predict(f_dense)
    return names, circuit.parameters_, nrmse, Z_dense


def append_fit_to_csv(filepath, circuit_str, param_names, param_values, nrmse):
    with open(filepath, "a", newline="\n") as f:
        f.write("# --- ECM FIT ---\n")
        f.write(f"# Circuit: {circuit_str}\n")
        f.write(f"# NRMSE: {nrmse:.5f}\n")
        f.write("# " + ",".join(param_names) + "\n")
        f.write("# " + ",".join(f"{v:.6g}" for v in param_values) + "\n")


# ─────────────────────────────────────────────────────────────────────────────
# EIS colour palette (mirrors your EIS GUI)
# ─────────────────────────────────────────────────────────────────────────────
BG        = "#1e1e2e"
BG_PANEL  = "#181825"
BG_ENTRY  = "#2a2a3e"
FG        = "#cdd6f4"
FG_DIM    = "#6c7086"
ACCENT    = "#89b4fa"
BORDER    = "#313244"
BTN_GO    = "#a6e3a1"
BTN_STOP  = "#f38ba8"
BTN_EXIT  = "#fab387"
SWEEP_COLORS = ["#89b4fa", "#a6e3a1", "#f38ba8",
                "#fab387", "#cba6f7", "#f9e2af", "#94e2d5"]


# ─────────────────────────────────────────────────────────────────────────────
# EIS mode handshake  (cycler IDLE  ->  'e'  ->  EIS banner)
# ─────────────────────────────────────────────────────────────────────────────
def enter_eis_mode(ser):
    """Drive the board from cycler IDLE into EIS mode.
    Returns True once the board is in EIS idle, False otherwise."""
    print("\n  Handing the battery over to the EIS front-end...")
    ser.reset_input_buffer()

    ser.write(b"s\n")
    ser.flush()
    print(f"  Waiting {_COOLDOWN_WAIT_S:.0f}s for hardware cooldown...")
    time.sleep(_COOLDOWN_WAIT_S)
    ser.reset_input_buffer()

    ser.write(b"e\n")
    ser.flush()

    deadline = time.time() + 20.0
    while time.time() < deadline:
        try:
            line = ser.readline().decode("utf-8", errors="replace").strip()
        except serial.SerialException:
            print("  [!] Serial connection lost during EIS entry.")
            return False
        if not line:
            continue
        if "EIS MODE (AD5941)" in line:
            print("  EIS mode active.")
            return True
        low = line.lower()
        if "aborted" in low or "battery not detected" in low or "over-voltage" in low:
            print(f"  [EIS] Entry failed: {strip_timestamp(line)}")
            return False
        if "[WAIT]" in line:
            print("  Cooldown not elapsed — retrying...")
            time.sleep(_COOLDOWN_WAIT_S)
            ser.reset_input_buffer()
            ser.write(b"e\n")
            ser.flush()
            deadline = time.time() + 20.0

    print("  [EIS] Timed out waiting for EIS banner.")
    return False


# ─────────────────────────────────────────────────────────────────────────────
# Compact EIS live window  (big Nyquist over a smaller Bode; optional ECM fit)
# ─────────────────────────────────────────────────────────────────────────────
class EISLiveWindow:
    """
    Live EIS panel fed by the SHARED serial handle (owns no port of its own).
    On close it sends EXIT so the board returns to the cycler (IDLE).
    Reader thread -> queue -> Tk main thread (poll) -> plot/log update.
    """

    def __init__(self, ser):
        self.ser         = ser
        self.q           = queue.Queue()
        self.stop_event  = threading.Event()
        self.reader      = None

        self.completed   = []
        self.current     = _empty_sweep()
        self.clean       = []       # per-sweep outlier-filtered view (or None), aligned to completed
        self.fits        = {}       # sweep_idx -> {"Z_dense", "nrmse"}
        self.saved_paths = []
        self.last_fit    = None
        self.num_sweeps  = 1
        self.run_ts      = None
        self.save        = True
        self.outdir      = "data"
        self.fit_enable  = False
        self.filter_enable = SCIPY_AVAILABLE
        self.circuit_str = DEFAULT_CIRCUIT
        self.init_guess  = []

        self._build_gui()

    # ── GUI construction ──────────────────────────────────────────────────────
    def _build_gui(self):
        self.root = tk.Tk()
        self.root.title("EIS — Live (combined board)")
        self.root.configure(bg=BG)
        self.root.geometry("1200x860")
        self.root.protocol("WM_DELETE_WINDOW", self._on_exit)

        left = tk.Frame(self.root, bg=BG_PANEL, width=310)
        left.pack(side="left", fill="y")
        left.pack_propagate(False)

        tk.Label(left, text="EIS MEASUREMENT", bg=BG_PANEL, fg=ACCENT,
                 font=("TkDefaultFont", 11, "bold")).pack(pady=(12, 8))

        self.vars = {}
        for label, key, default in (
            ("Points",      "points",  "50"),
            ("Start (Hz)",  "start",   "50000"),
            ("Stop (Hz)",   "stop",    "1"),
            ("Sweeps",      "sweeps",  "1"),
            ("Timeout (s)", "timeout", "60"),
        ):
            row = tk.Frame(left, bg=BG_PANEL)
            row.pack(fill="x", padx=16, pady=3)
            tk.Label(row, text=label, bg=BG_PANEL, fg=FG, width=11,
                     anchor="w").pack(side="left")
            v = tk.StringVar(value=default)
            self.vars[key] = v
            tk.Entry(row, textvariable=v, width=14, bg=BG_ENTRY, fg=FG,
                     insertbackground=FG, relief="flat").pack(side="left")

        self.var_save = tk.BooleanVar(value=True)
        tk.Checkbutton(left, text="Save each sweep to CSV", variable=self.var_save,
                       bg=BG_PANEL, fg=FG, selectcolor=BG_ENTRY,
                       activebackground=BG_PANEL, activeforeground=FG).pack(
                           anchor="w", padx=16, pady=(6, 2))

        row = tk.Frame(left, bg=BG_PANEL)
        row.pack(fill="x", padx=16, pady=3)
        tk.Label(row, text="Out dir", bg=BG_PANEL, fg=FG, width=11,
                 anchor="w").pack(side="left")
        self.var_outdir = tk.StringVar(value="data")
        tk.Entry(row, textvariable=self.var_outdir, width=14, bg=BG_ENTRY, fg=FG,
                 insertbackground=FG, relief="flat").pack(side="left")

        # ── ECM fit controls ──
        self._sep(left)
        tk.Label(left, text="ECM FIT (impedance.py)", bg=BG_PANEL, fg=ACCENT,
                 font=("TkDefaultFont", 9, "bold")).pack(anchor="w", padx=16)

        self.var_fit_enable   = tk.BooleanVar(value=IMPEDANCE_AVAILABLE)
        self.var_show_fit     = tk.BooleanVar(value=True)
        self.var_fit_autosave = tk.BooleanVar(value=True)
        self.var_filter       = tk.BooleanVar(value=SCIPY_AVAILABLE)

        fcb = tk.Checkbutton(left, text="Filter outliers (IQR) on completed sweeps",
                             variable=self.var_filter, bg=BG_PANEL, fg=FG,
                             selectcolor=BG_ENTRY, activebackground=BG_PANEL,
                             activeforeground=FG, command=self._redraw)
        fcb.pack(anchor="w", padx=16, pady=(4, 0))
        if not SCIPY_AVAILABLE:
            fcb.config(state="disabled")
            tk.Label(left, text="(install 'scipy' to enable outlier filtering)",
                     bg=BG_PANEL, fg=FG_DIM).pack(anchor="w", padx=16)

        cb = tk.Checkbutton(left, text="Fit each completed sweep",
                            variable=self.var_fit_enable, bg=BG_PANEL, fg=FG,
                            selectcolor=BG_ENTRY, activebackground=BG_PANEL,
                            activeforeground=FG)
        cb.pack(anchor="w", padx=16, pady=(4, 0))
        if not IMPEDANCE_AVAILABLE:
            cb.config(state="disabled")
            tk.Label(left, text="(install 'impedance' + 'scipy' to enable)",
                     bg=BG_PANEL, fg=FG_DIM).pack(anchor="w", padx=16)

        row = tk.Frame(left, bg=BG_PANEL)
        row.pack(fill="x", padx=16, pady=3)
        tk.Label(row, text="Circuit", bg=BG_PANEL, fg=FG, width=11,
                 anchor="w").pack(side="left")
        self.var_circuit = tk.StringVar(value=DEFAULT_CIRCUIT)
        tk.Entry(row, textvariable=self.var_circuit, width=14, bg=BG_ENTRY, fg=FG,
                 insertbackground=FG, relief="flat").pack(side="left")

        tk.Label(left, text="Initial guess (comma-separated)", bg=BG_PANEL,
                 fg=FG_DIM).pack(anchor="w", padx=16)
        self.var_guess = tk.StringVar(value=DEFAULT_INIT_GUESS)
        tk.Entry(left, textvariable=self.var_guess, bg=BG_ENTRY, fg=FG,
                 insertbackground=FG, relief="flat").pack(fill="x", padx=16, pady=(0, 4))

        tk.Checkbutton(left, text="Show fit overlay on Nyquist",
                       variable=self.var_show_fit, bg=BG_PANEL, fg=FG,
                       selectcolor=BG_ENTRY, activebackground=BG_PANEL,
                       activeforeground=FG,
                       command=self._redraw).pack(anchor="w", padx=16)
        tk.Checkbutton(left, text="Append fit to sweep CSV",
                       variable=self.var_fit_autosave, bg=BG_PANEL, fg=FG,
                       selectcolor=BG_ENTRY, activebackground=BG_PANEL,
                       activeforeground=FG).pack(anchor="w", padx=16, pady=(0, 4))

        # ── action buttons ──
        self._sep(left)
        btns = tk.Frame(left, bg=BG_PANEL)
        btns.pack(fill="x", padx=16, pady=(6, 6))
        self.btn_start = tk.Button(btns, text="Start Sweep", command=self._on_start,
                                   bg=BTN_GO, fg=BG, relief="flat", width=12)
        self.btn_start.pack(side="left", padx=(0, 6))
        self.btn_stop = tk.Button(btns, text="Stop", command=self._on_stop,
                                  bg=BTN_STOP, fg=BG, relief="flat", width=8,
                                  state="disabled")
        self.btn_stop.pack(side="left")

        self.btn_exit = tk.Button(left, text="Exit EIS  →  Cycler", command=self._on_exit,
                                  bg=BTN_EXIT, fg=BG, relief="flat")
        self.btn_exit.pack(fill="x", padx=16, pady=(2, 10))

        self.var_status = tk.StringVar(value="EIS idle. Set parameters and Start.")
        tk.Label(left, textvariable=self.var_status, bg=BG_PANEL, fg=FG_DIM,
                 wraplength=280, justify="left").pack(fill="x", padx=16, pady=(0, 6))

        tk.Label(left, text="Serial log / fit", bg=BG_PANEL, fg=ACCENT).pack(
            anchor="w", padx=16)
        self.log = tk.Text(left, height=12, bg="#11111b", fg=FG, relief="flat",
                           wrap="word", font=("TkFixedFont", 8))
        self.log.pack(fill="both", expand=True, padx=16, pady=(2, 12))

        # ── plots: big Nyquist (top), smaller Bode (bottom, twin-axis) ──
        self.fig = Figure(figsize=(8.6, 8.4), facecolor=BG)
        gs = self.fig.add_gridspec(3, 1, height_ratios=[3.0, 3.0, 1.6], hspace=0.45)
        self.ax_nyq   = self.fig.add_subplot(gs[0:2, 0])   # top ~2/3
        self.ax_mag   = self.fig.add_subplot(gs[2, 0])     # bottom ~1/3
        self.ax_phase = self.ax_mag.twinx()                # phase on the right axis
        self._style_axes()
        self.fig.tight_layout(pad=2.0)

        self.canvas = FigureCanvasTkAgg(self.fig, master=self.root)
        self.canvas.get_tk_widget().pack(side="right", fill="both", expand=True)
        self.canvas.draw()

    def _sep(self, parent):
        tk.Frame(parent, bg=BORDER, height=1).pack(fill="x", padx=12, pady=8)

    def _style_axes(self):
        self.ax_nyq.set_facecolor(BG_PANEL)
        self.ax_mag.set_facecolor(BG_PANEL)
        for ax in (self.ax_nyq, self.ax_mag, self.ax_phase):
            ax.tick_params(colors=FG_DIM, labelsize=8)
            for sp in ax.spines.values():
                sp.set_edgecolor(BORDER)
        self.ax_nyq.grid(True, color=BORDER, alpha=0.4, lw=0.5)
        self.ax_mag.grid(True, color=BORDER, alpha=0.4, lw=0.5)

        self.ax_nyq.set_title("Nyquist", color=FG, fontsize=11)
        self.ax_nyq.set_xlabel("Z'  (Ω)", color=FG_DIM)
        self.ax_nyq.set_ylabel("-Z''  (Ω)", color=FG_DIM)
        # Equal unit scaling so semicircles read true (not stretched to the box).
        self.ax_nyq.set_aspect("equal", adjustable="datalim")

        self.ax_mag.set_title("Bode", color=FG, fontsize=10)
        self.ax_mag.set_xlabel("frequency (Hz)", color=FG_DIM)
        self.ax_mag.set_ylabel("|Z| (Ω)", color=ACCENT)
        self.ax_mag.set_xscale("log")
        self.ax_mag.set_yscale("log")
        self.ax_mag.tick_params(axis="y", colors=ACCENT)
        self.ax_phase.set_ylabel("phase (°)", color="#fab387")
        self.ax_phase.tick_params(axis="y", colors="#fab387")

    def _log_write(self, text):
        self.log.insert("end", text + "\n")
        self.log.see("end")

    # ── controls ──────────────────────────────────────────────────────────────
    def _on_start(self):
        if self.reader and self.reader.is_alive():
            return
        try:
            cfg = {
                "points":  int(self.vars["points"].get()),
                "start":   float(self.vars["start"].get()),
                "stop":    float(self.vars["stop"].get()),
                "sweeps":  int(self.vars["sweeps"].get()),
                "timeout": float(self.vars["timeout"].get()),
            }
        except ValueError:
            self.var_status.set("⚠  Points/Sweeps must be ints; freqs/timeout numbers.")
            return

        # Resolve fit settings for this run.
        self.filter_enable = bool(self.var_filter.get()) and SCIPY_AVAILABLE
        self.fit_enable  = bool(self.var_fit_enable.get()) and IMPEDANCE_AVAILABLE
        self.circuit_str = self.var_circuit.get().strip()
        if self.fit_enable:
            try:
                guesses = [float(x.strip()) for x in self.var_guess.get().split(",")]
                names, _ = _circuit_param_count(self.circuit_str)
                if len(guesses) != len(names):
                    self._log_write(f"Fit disabled: circuit needs {len(names)} values "
                                    f"({', '.join(names)}), got {len(guesses)}.")
                    self.fit_enable = False
                else:
                    self.init_guess = guesses
            except Exception as e:
                self._log_write(f"Fit disabled: {e}")
                self.fit_enable = False

        self.num_sweeps = max(1, cfg["sweeps"])
        self.save       = self.var_save.get()
        self.outdir     = self.var_outdir.get().strip() or "data"
        self.run_ts     = datetime.now().strftime("%Y%m%d_%H%M%S")

        self.completed   = []
        self.current     = _empty_sweep()
        self.clean       = []
        self.fits        = {}
        self.saved_paths = []
        self.last_fit    = None
        self._clear_plot()

        self.stop_event.clear()
        self.reader = threading.Thread(target=self._reader_run, args=(cfg,), daemon=True)
        self.reader.start()

        self.btn_start.config(state="disabled")
        self.btn_stop.config(state="normal")
        self.var_status.set(f"Running sweep 1/{self.num_sweeps}…")

    def _on_stop(self):
        self.stop_event.set()
        try:
            self.ser.write(b"STOP\n"); self.ser.flush()
        except Exception:
            pass
        self.var_status.set("Stop requested (takes effect at the next sweep boundary).")
        self.btn_stop.config(state="disabled")

    def _on_exit(self):
        self.stop_event.set()
        try:
            self.ser.write(b"STOP\n"); self.ser.flush(); time.sleep(0.2)
            self.ser.write(b"EXIT\n"); self.ser.flush(); time.sleep(0.2)
            self.ser.write(b"x\n");    self.ser.flush(); time.sleep(0.1)  # clear any fault latch
        except Exception:
            pass
        try:
            self.root.quit()
            self.root.destroy()
        except Exception:
            pass

    # ── serial reader (EIS line protocol; sends START WITH ARGS) ──────────────
    def _reader_run(self, cfg):
        ser = self.ser
        num_sweeps = self.num_sweeps
        timeout_s  = cfg["timeout"]
        sweep_idx  = 0
        in_data     = False
        header_seen = False
        last_line_t = time.time()

        def send_start():
            nonlocal in_data, header_seen, last_line_t
            in_data = False
            header_seen = False
            last_line_t = time.time()
            cmd = f"START {cfg['points']} {cfg['start']:.3f} {cfg['stop']:.3f}\n"
            self.q.put(("log", f"-- Sending {cmd.strip()} (sweep {sweep_idx+1}/{num_sweeps}) --"))
            ser.write(cmd.encode())
            ser.flush()

        send_start()
        try:
            while sweep_idx < num_sweeps and not self.stop_event.is_set():
                # Firmware sweeps have two blocking 15 s settle gaps (RCAL +
                # battery) with no output — keep timeout above ~20 s.
                if time.time() - last_line_t > timeout_s:
                    self.q.put(("status", f"Timeout after {timeout_s:.0f}s — aborted."))
                    self.q.put(("log", f"WARNING: no data for {timeout_s:.0f}s, aborting."))
                    break
                try:
                    raw = ser.readline()
                except serial.SerialException:
                    self.q.put(("error", "Serial connection lost."))
                    break
                if not raw:
                    continue
                last_line_t = time.time()
                line = raw.decode("utf-8", errors="replace").rstrip()
                if not line:
                    continue
                self.q.put(("log", strip_timestamp(line)))

                if "FAULT" in line.upper():
                    self.q.put(("error", "Firmware FAULT during EIS — board left EIS mode."))
                    break

                if not header_seen and is_eis_header(line):
                    header_seen = True
                    in_data = True
                    continue

                if in_data:
                    pt = parse_eis_data_line(line)
                    if pt:
                        self.q.put(("point", *pt))
                    if is_sweep_complete(line):
                        self.q.put(("sweep_done", sweep_idx))
                        sweep_idx += 1
                        if sweep_idx < num_sweeps and not self.stop_event.is_set():
                            time.sleep(0.5)
                            send_start()
        finally:
            self.q.put(("all_done", sweep_idx))

    # ── queue polling ─────────────────────────────────────────────────────────
    def _poll_queue(self):
        try:
            while True:
                self._handle(self.q.get_nowait())
        except queue.Empty:
            pass
        self.root.after(50, self._poll_queue)

    def _handle(self, msg):
        kind = msg[0]
        if kind == "log":
            self._log_write(msg[1])
        elif kind == "status":
            self.var_status.set(msg[1])
        elif kind == "point":
            _, f, r, im, m, ph = msg
            c = self.current
            c["freq"].append(f); c["real"].append(r); c["imag"].append(im)
            c["mag"].append(m);  c["phase"].append(ph)
            self._redraw()
        elif kind == "sweep_done":
            if self.current["freq"]:
                sw = {k: list(v) for k, v in self.current.items()}
                idx = len(self.completed)
                self.completed.append(sw)
                path = self._save_sweep(sw, idx) if self.save else None
                self.saved_paths.append(path)

                clean = self._filter_sweep(sw)
                self.clean.append(clean)
                if clean is not None:
                    nrem = len(clean["rem_freq"])
                    if nrem:
                        rem = ", ".join(f"{x:.1f}" for x in clean["rem_freq"])
                        self._log_write(f"  Sweep {idx+1}: filtered {nrem} outlier(s) @ {rem} Hz")
                    else:
                        self._log_write(f"  Sweep {idx+1}: no outliers (endpoints trimmed).")

                if self.fit_enable:
                    src = clean if clean is not None else _sorted_view(sw)
                    freq = np.array(src["freq"], float)
                    Z = np.array(src["real"], float) + 1j * np.array(src["imag"], float)
                    self._run_fit_async(freq, Z, idx, path)
            self.current = _empty_sweep()
            self._redraw()
        elif kind == "fit_result":
            self._handle_fit_result(msg[1])
        elif kind == "fit_error":
            self._log_write(f"ECM fit failed: {msg[1]}")
        elif kind == "error":
            self._log_write("ERROR: " + msg[1])
            self.var_status.set("Error — see log.")
            self.btn_start.config(state="normal")
            self.btn_stop.config(state="disabled")
        elif kind == "all_done":
            done = msg[1]
            self.var_status.set(f"Complete — {self.num_sweeps} sweep(s) done."
                                if done >= self.num_sweeps else "Stopped.")
            self.btn_start.config(state="normal")
            self.btn_stop.config(state="disabled")

    # ── outlier filter (IQR in the complex plane) ─────────────────────────────
    def _filter_sweep(self, sweep):
        """Return an outlier-filtered, freq-sorted view of a sweep, plus the
        removed points, or None if filtering is off/unavailable/too short."""
        if not (SCIPY_AVAILABLE and self.filter_enable):
            return None
        try:
            v = _sorted_view(sweep)
            freq  = np.array(v["freq"], float)
            real  = np.array(v["real"], float)
            imag  = np.array(v["imag"], float)
            mag   = np.array(v["mag"], float)
            phase = np.array(v["phase"], float)
            if freq.size < 5:      # medfilt needs at least the kernel width
                return None
            Z = real + 1j * imag
            _, _, mask = remove_outliers(freq, Z)   # True = removed
            keep = ~mask
            return {
                "freq":  freq[keep].tolist(),  "real": real[keep].tolist(),
                "imag":  imag[keep].tolist(),  "mag":  mag[keep].tolist(),
                "phase": phase[keep].tolist(),
                "rem_freq": freq[mask].tolist(),
                "rem_real": real[mask].tolist(),
                "rem_imag": imag[mask].tolist(),
            }
        except Exception as e:
            self._log_write(f"Outlier filter skipped: {e}")
            return None

    # ── ECM fit (background thread; receives already-cleaned arrays) ──────────
    def _run_fit_async(self, freq, Z, sweep_idx, saved_path):
        circuit = self.circuit_str
        guess   = list(self.init_guess)

        def worker():
            try:
                names, params, nrmse, Z_dense = run_ecm_fit(freq, Z, circuit, guess)
                self.q.put(("fit_result", {
                    "sweep_idx": sweep_idx, "saved_path": saved_path,
                    "names": names, "params": params, "nrmse": nrmse,
                    "Z_dense": Z_dense,
                }))
            except Exception as e:
                self.q.put(("fit_error", str(e)))
        threading.Thread(target=worker, daemon=True).start()

    def _handle_fit_result(self, res):
        self.last_fit = res
        self.fits[res["sweep_idx"]] = {"Z_dense": res["Z_dense"], "nrmse": res["nrmse"]}
        names, params, nrmse = res["names"], res["params"], res["nrmse"]
        self._log_write("─" * 34)
        self._log_write(f"  ECM Fit — Sweep {res['sweep_idx']+1}   {self.circuit_str}")
        self._log_write(f"  NRMSE = {nrmse:.5f}")
        w = max(len(n) for n in names) + 2
        for n, val in zip(names, params):
            self._log_write(f"  {n:{w}}  {val:>14.6g}")
        self._log_write("─" * 34)

        if self.var_fit_autosave.get() and res.get("saved_path"):
            try:
                append_fit_to_csv(res["saved_path"], self.circuit_str,
                                  names, params, nrmse)
                self._log_write(f"  Fit appended to: {res['saved_path']}")
            except Exception as e:
                self._log_write(f"  Could not append fit: {e}")
        self._redraw()

    # ── plotting ──────────────────────────────────────────────────────────────
    def _clear_plot(self):
        for ax in (self.ax_nyq, self.ax_mag, self.ax_phase):
            ax.cla()
        self._style_axes()
        self.canvas.draw_idle()

    def _redraw(self):
        for ax in (self.ax_nyq, self.ax_mag, self.ax_phase):
            ax.cla()
        self._style_axes()

        any_data = False
        show_fit = self.var_show_fit.get()

        # Completed sweeps: prefer the outlier-filtered view for display + Bode.
        for i, raw in enumerate(self.completed):
            color = SWEEP_COLORS[i % len(SWEEP_COLORS)]
            clean = self.clean[i] if i < len(self.clean) else None
            disp  = clean if clean is not None else _sorted_view(raw)
            if disp["freq"]:
                any_data = True
                ni = [-x for x in disp["imag"]]
                self.ax_nyq.scatter(disp["real"], ni, color=color, s=18, alpha=0.9,
                                    label=f"Sweep {i+1}", zorder=3)
                self.ax_mag.plot(disp["freq"], disp["mag"], color=color, lw=1.4,
                                 marker="o", ms=3)
                self.ax_phase.plot(disp["freq"], disp["phase"], color=color, lw=1.2,
                                   ls="--", marker="^", ms=3)
            # Removed outliers shown faintly (kept, not deleted from view entirely).
            if clean is not None and clean["rem_freq"]:
                self.ax_nyq.scatter(clean["rem_real"], [-x for x in clean["rem_imag"]],
                                    marker="x", color=FG_DIM, s=28, alpha=0.6, zorder=2,
                                    label="filtered" if i == 0 else None)
            # Per-sweep ECM fit overlay, in the sweep's colour.
            fit = self.fits.get(i)
            if fit is not None and show_fit:
                Zd = fit["Z_dense"]
                self.ax_nyq.plot(Zd.real, -Zd.imag, color=color, lw=2.0, alpha=0.95,
                                 zorder=4, label=f"Fit {i+1} (NRMSE={fit['nrmse']:.4f})")

        # Live (in-progress) sweep: raw points, de-emphasised.
        if self.current["freq"]:
            any_data = True
            cv = _sorted_view(self.current)
            color = SWEEP_COLORS[len(self.completed) % len(SWEEP_COLORS)]
            self.ax_nyq.scatter(cv["real"], [-x for x in cv["imag"]], color=color,
                                s=14, alpha=0.55, label="acquiring…", zorder=3)
            self.ax_mag.plot(cv["freq"], cv["mag"], color=color, lw=1.1, marker="o",
                             ms=3, alpha=0.55)
            self.ax_phase.plot(cv["freq"], cv["phase"], color=color, lw=1.0, ls="--",
                               marker="^", ms=3, alpha=0.55)

        if any_data:
            self.ax_nyq.legend(fontsize=7, loc="best", facecolor=BORDER,
                               labelcolor=FG, edgecolor=BORDER)
        self.canvas.draw_idle()

    def _save_sweep(self, sweep, idx):
        try:
            os.makedirs(self.outdir, exist_ok=True)
            path = os.path.join(self.outdir, f"EIS_{self.run_ts}_sweep{idx+1:02d}.csv")
            n = len(sweep["freq"])
            order = sorted(range(n), key=lambda k: sweep["freq"][k])
            with open(path, "w", newline="") as f:
                w = csv.writer(f)
                w.writerow([f"# EIS sweep {datetime.now().isoformat()}"])
                w.writerow([f"# Sweep index: {idx+1}"])
                w.writerow(["Frequency(Hz)", "Real(Ohm)", "Imaginary(Ohm)",
                            "Magnitude(Ohm)", "Phase(Deg)"])
                for k in order:
                    w.writerow([f"{sweep['freq'][k]:.6g}", f"{sweep['real'][k]:.6g}",
                                f"{sweep['imag'][k]:.6g}", f"{sweep['mag'][k]:.6g}",
                                f"{sweep['phase'][k]:.4f}"])
            self._log_write(f"Saved: {path}")
            return path
        except OSError as e:
            self._log_write(f"Could not save sweep: {e}")
            return None

    def run(self):
        self.root.after(50, self._poll_queue)
        self.root.mainloop()


# ─────────────────────────────────────────────────────────────────────────────
# Cycling: terminal telemetry printer + custom live plot (temp min-span)
# ─────────────────────────────────────────────────────────────────────────────
def telemetry_printer(data, stop_event, interval=CYCLER_TELEMETRY_INTERVAL_S):
    """Echo the latest cycling telemetry to the terminal about once per minute."""
    printed_first = False
    last = 0.0
    while not stop_event.is_set():
        time.sleep(1.0)
        now = time.time()
        if printed_first and (now - last) < interval:
            continue
        with data.lock:
            if not data.t:
                continue
            state   = data.state[-1] if data.state else "?"
            # During a blocking EIS step the firmware emits no DATA, lines, so the
            # arrays are frozen at the last cycler sample from BEFORE the handoff.
            # Reprinting it every minute is stale and misleading (the voltage is a
            # leftover ADS reading, not a live one — the sense MUX is on the EIS
            # board). Skip it; the [FW]/[EIS] sweep lines carry the live output.
            if state in ("SEQ_EIS", "EIS"):
                continue
            elapsed = data.t[-1]
            v       = data.v[-1]
            i_ch    = data.i_ch[-1]
            i_dis   = data.i_dis[-1]
            t1      = data.temp[-1]
            t2      = data.temp2[-1]
            step    = data.step_idx[-1]  if data.step_idx  else -1
            loop    = data.loop_iter[-1] if data.loop_iter else 0
        printed_first = True
        last = now
        mm, ss = divmod(int(elapsed), 60)
        hh, mm = divmod(mm, 60)
        print(f"\n  [TELEM {hh:02d}:{mm:02d}:{ss:02d}] {state:<11} "
              f"V={v:.4f}  I_ch={i_ch:.3f}  I_dis={i_dis:.3f}  "
              f"T_nmos={t1:.1f}C  T_batt={t2:.1f}C  step={step} loop={loop}")


def _style_eis_axes(ax_nyq, ax_mag, ax_phase):
    """Dark styling for the interleaved-EIS figure, matching the discrete window."""
    for ax in (ax_nyq, ax_mag, ax_phase):
        ax.tick_params(colors=FG_DIM, labelsize=8)
        for sp in ax.spines.values():
            sp.set_edgecolor(BORDER)
    ax_nyq.set_facecolor(BG_PANEL)
    ax_mag.set_facecolor(BG_PANEL)
    ax_nyq.grid(True, color=BORDER, alpha=0.4, lw=0.5)
    ax_mag.grid(True, color=BORDER, alpha=0.4, lw=0.5)
    ax_nyq.set_title("Interleaved EIS — Nyquist", color=FG, fontsize=11)
    ax_nyq.set_xlabel("Z'  (Ω)", color=FG_DIM)
    ax_nyq.set_ylabel("-Z''  (Ω)", color=FG_DIM)
    # Equal unit scaling so semicircles read true (not stretched to the box).
    ax_nyq.set_aspect("equal", adjustable="datalim")
    ax_mag.set_title("Bode", color=FG, fontsize=10)
    ax_mag.set_xlabel("frequency (Hz)", color=FG_DIM)
    ax_mag.set_ylabel("|Z|  (Ω)", color=FG_DIM)
    ax_mag.set_xscale("log")
    ax_phase.set_ylabel("phase (°)", color=FG_DIM)


def _eis_sweep_label(sw):
    step = sw.get("step_idx")
    it   = sw.get("loop_iter", 0)
    if step is None or step < 0:
        return "EIS"
    return f"step {step}" + (f"·it{it}" if it else "")


def _draw_interleaved_eis(fig, ax_nyq, ax_mag, ax_phase, completed, live):
    """Redraw the interleaved-EIS Nyquist + Bode from completed sweeps (solid,
    coloured) plus the in-progress sweep (accent, live). Frequency-sorted so the
    traces connect in order. Operates only on the passed-in snapshot copies."""
    ax_nyq.cla(); ax_mag.cla(); ax_phase.cla()
    _style_eis_axes(ax_nyq, ax_mag, ax_phase)

    def _plot_one(sw, color, is_live=False):
        f = np.asarray(sw.get("freq", []), float)
        if f.size == 0:
            return
        r  = np.asarray(sw["real"],  float)
        im = np.asarray(sw["imag"],  float)
        mg = np.asarray(sw["mag"],   float)
        ph = np.asarray(sw["phase"], float)
        order = np.argsort(f)
        f, r, im, mg, ph = f[order], r[order], im[order], mg[order], ph[order]
        lbl = _eis_sweep_label(sw) + (" (live)" if is_live else "")
        ms  = 5 if is_live else 4
        lw  = 1.3 if is_live else 1.6
        a   = 1.0 if is_live else 0.9
        ax_nyq.plot(r, -im, color=color, lw=lw, marker="o", ms=ms, alpha=a, label=lbl)
        ax_mag.plot(f, mg,  color=color, lw=lw, marker="o", ms=max(ms - 1, 3), alpha=a)
        ax_phase.plot(f, ph, color=color, lw=lw * 0.8, ls="--", alpha=a * 0.8)

    for j, sw in enumerate(completed):
        _plot_one(sw, SWEEP_COLORS[j % len(SWEEP_COLORS)])
    if live is not None:
        _plot_one(live, ACCENT, is_live=True)

    any_pts = bool(completed) or (live is not None and len(live.get("freq", [])) > 0)
    if any_pts:
        ax_nyq.legend(loc="upper left", fontsize=7, facecolor=BG_PANEL,
                      edgecolor=BORDER, labelcolor=FG)
    else:
        ax_nyq.text(0.5, 0.5, "waiting for an EIS step…", transform=ax_nyq.transAxes,
                    ha="center", va="center", color=FG_DIM, fontsize=10)
    try:
        fig.tight_layout(pad=1.6)
    except Exception:
        pass


def run_cycler_plot(data, protocol, stop_event):
    """Live plot mirroring the cycler CLI, but the temperature axis keeps a
    minimum span (CYCLER_MIN_TEMP_SPAN_C) so stable readings don't fill the
    axis with ADC noise. Reuses the cycler module's colours/segment helper.

    If the protocol contains any EIS step, a second window shows the interleaved
    Nyquist/Bode, updating point-by-point as each sweep streams in (and while the
    cycler plot is frozen during the firmware's blocking sweep)."""
    import matplotlib.pyplot as plt

    C_CHARGE  = cyc.COLOR_CHARGE
    C_DISCH   = cyc.COLOR_DISCHARGE
    C_REST    = cyc.COLOR_REST
    C_FAULT   = cyc.COLOR_FAULT
    C_TEMP    = cyc.COLOR_TEMP
    C_TEMP2   = cyc.COLOR_TEMP2
    WINDOW_S  = cyc.ROLLING_WINDOW_S
    seg       = cyc.build_segments

    fig = plt.figure(figsize=(13, 8))
    fig.patch.set_facecolor("#1e1e2e")
    ax_v = fig.add_subplot(2, 1, 1)
    ax_i = fig.add_subplot(2, 2, 3)
    ax_t = fig.add_subplot(2, 2, 4)

    def style_ax(ax):
        ax.set_facecolor("#2a2a3e")
        ax.tick_params(colors="#cccccc", labelsize=CYCLER_TICK_FONTSIZE)
        for sp in ax.spines.values():
            sp.set_edgecolor("#555577")

    for ax in (ax_v, ax_i, ax_t):
        style_ax(ax)

    fig.suptitle(f"Battery Cycler — {protocol['name']}", color="#eeeeee", fontsize=13)
    legend_v = [Line2D([0], [0], color=C_CHARGE, lw=2, label="Charging"),
                Line2D([0], [0], color=C_DISCH,  lw=2, label="Discharging"),
                Line2D([0], [0], color=C_REST,   lw=2, label="Rest / Idle"),
                Line2D([0], [0], color=C_FAULT,  lw=2, label="Fault")]
    legend_t = [Line2D([0], [0], color=C_TEMP,  lw=2, label="T NMOS"),
                Line2D([0], [0], color=C_TEMP2, lw=2, label="T Batt", linestyle="--")]

    def legend_kw():
        return dict(facecolor="#2a2a3e", edgecolor="#555577",
                    labelcolor="#cccccc", fontsize=7)

    fig.tight_layout(rect=[0, 0, 1, 0.95], h_pad=3.0, w_pad=2.0)

    # ── Second window: interleaved EIS (created only if the run has an EIS step) ──
    has_eis = any(isinstance(s, dict) and str(s.get("type", "")).upper() == "EIS"
                  for s in protocol.get("steps", []))
    fig_eis = ax_e_nyq = ax_e_mag = ax_e_phase = None
    eis_win_open = False
    last_eis_sig = None
    if has_eis:
        fig_eis = plt.figure(figsize=(7, 8))
        fig_eis.patch.set_facecolor(BG)
        try:
            fig_eis.canvas.manager.set_window_title("Interleaved EIS — live")
        except Exception:
            pass
        gse = fig_eis.add_gridspec(2, 1, height_ratios=[3.0, 1.6], hspace=0.45)
        ax_e_nyq   = fig_eis.add_subplot(gse[0, 0])
        ax_e_mag   = fig_eis.add_subplot(gse[1, 0])
        ax_e_phase = ax_e_mag.twinx()
        _style_eis_axes(ax_e_nyq, ax_e_mag, ax_e_phase)
        eis_win_open = True

        def _on_eis_close(_evt):
            nonlocal eis_win_open
            eis_win_open = False
        fig_eis.canvas.mpl_connect("close_event", _on_eis_close)

    plt.ion(); plt.show()

    num_steps = len(protocol["steps"])

    while not stop_event.is_set():
        with data.lock:
            if len(data.t) < 2:
                time.sleep(0.5)
                continue
            t     = np.array(data.t)
            v     = np.array(data.v)
            i_ch  = np.array(data.i_ch)
            i_dis = np.array(data.i_dis)
            temp  = np.array(data.temp)
            temp2 = np.array(data.temp2)
            states = list(data.state)
            cur_step = data.step_idx[-1]  if data.step_idx  else -1
            cur_loop = data.loop_iter[-1] if data.loop_iter else 0
            cur_st   = states[-1] if states else "IDLE"

        t_max = t[-1]
        t_min = max(0.0, t_max - WINDOW_S)
        mask = t >= t_min
        t_w, v_w = t[mask], v[mask]
        i_ch_w, i_dis_w = i_ch[mask], i_dis[mask]
        temp_w, temp2_w = temp[mask], temp2[mask]
        st_w = [s for s, m in zip(states, mask) if m]

        # Voltage
        ax_v.cla(); style_ax(ax_v)
        ax_v.set_ylabel("Voltage (V)", color="#cccccc", fontsize=CYCLER_LABEL_FONTSIZE)
        ax_v.legend(handles=legend_v, loc="upper left", **legend_kw())
        for sx, sy, sc in seg(t_w.tolist(), v_w.tolist(), st_w):
            ax_v.plot(sx, sy, color=sc, lw=CYCLER_LW_VOLTAGE)
        if len(t_w):
            ax_v.set_xlim(t_min, t_min + WINDOW_S)
        step_lbl = f"Step {cur_step}/{num_steps-1}" if cur_step >= 0 else ""
        loop_lbl = f"  Loop iter {cur_loop}" if protocol.get("loop") and cur_loop > 0 else ""
        ax_v.text(0.99, 0.06, f"{cur_st}  {step_lbl}{loop_lbl}",
                  transform=ax_v.transAxes, ha="right", va="bottom",
                  color="#eeeeee", fontsize=10,
                  bbox=dict(boxstyle="round,pad=0.3", facecolor="#2a2a3e",
                            edgecolor="#555577"))
        ax_v.set_xlabel("Elapsed time (s)", color="#cccccc", fontsize=CYCLER_LABEL_FONTSIZE)

        # Current
        ax_i.cla(); style_ax(ax_i)
        ax_i.set_ylabel("Current (A)", color="#cccccc", fontsize=CYCLER_LABEL_FONTSIZE)
        ax_i.set_xlabel("Elapsed time (s)", color="#cccccc", fontsize=CYCLER_LABEL_FONTSIZE)
        ax_i.plot(t_w, i_ch_w,  color=C_CHARGE, lw=CYCLER_LW_CURRENT, label="I charge")
        ax_i.plot(t_w, i_dis_w, color=C_DISCH,  lw=CYCLER_LW_CURRENT, label="I discharge")
        ax_i.legend(loc="upper left", **legend_kw())
        if len(t_w):
            ax_i.set_xlim(t_min, t_min + WINDOW_S)
            ax_i.xaxis.set_major_formatter(ticker.FuncFormatter(lambda x, _: f"{int(x)}"))

        # Temperature — with a minimum y-span so noise doesn't fill the axis
        ax_t.cla(); style_ax(ax_t)
        ax_t.set_ylabel("Temperature (°C)", color="#cccccc", fontsize=CYCLER_LABEL_FONTSIZE)
        ax_t.set_xlabel("Elapsed time (s)", color="#cccccc", fontsize=CYCLER_LABEL_FONTSIZE)
        ax_t.plot(t_w, temp_w,  color=C_TEMP,  lw=1.8)
        ax_t.plot(t_w, temp2_w, color=C_TEMP2, lw=1.8, linestyle="--")
        ax_t.legend(handles=legend_t, loc="upper left", **legend_kw())
        if len(t_w):
            ax_t.set_xlim(t_min, t_min + WINDOW_S)
            ax_t.xaxis.set_major_formatter(ticker.FuncFormatter(lambda x, _: f"{int(x)}"))
        vals = np.concatenate([temp_w, temp2_w])
        vals = vals[np.isfinite(vals)]
        if vals.size:
            lo, hi = float(vals.min()), float(vals.max())
            center = 0.5 * (lo + hi)
            half = max(hi - lo, CYCLER_MIN_TEMP_SPAN_C) * 0.5 * 1.15
            ax_t.set_ylim(center - half, center + half)

        try:
            fig.canvas.draw_idle()
            fig.canvas.flush_events()
        except Exception:
            break

        # --- Interleaved EIS window: redraw only when the data changed ---------
        if has_eis and fig_eis is not None and eis_win_open:
            with data.lock:
                n_done = len(data.eis_sweeps)
                n_live = len(data.eis_live["freq"]) if data.eis_live is not None else -1
            if (n_done, n_live) != last_eis_sig:
                last_eis_sig = (n_done, n_live)
                with data.lock:
                    completed = [{
                        "freq":  list(sw["freq"]),  "real":  list(sw["real"]),
                        "imag":  list(sw["imag"]),  "mag":   list(sw["mag"]),
                        "phase": list(sw["phase"]),
                        "step_idx":  sw.get("step_idx"),
                        "loop_iter": sw.get("loop_iter"),
                    } for sw in data.eis_sweeps]
                    live = None
                    if data.eis_live is not None:
                        lv = data.eis_live
                        live = {
                            "freq":  list(lv.get("freq",  [])),
                            "real":  list(lv.get("real",  [])),
                            "imag":  list(lv.get("imag",  [])),
                            "mag":   list(lv.get("mag",   [])),
                            "phase": list(lv.get("phase", [])),
                            "step_idx":  lv.get("step_idx"),
                            "loop_iter": lv.get("loop_iter"),
                        }
                try:
                    _draw_interleaved_eis(fig_eis, ax_e_nyq, ax_e_mag, ax_e_phase,
                                          completed, live)
                except Exception:
                    pass
            try:
                fig_eis.canvas.draw_idle()
                fig_eis.canvas.flush_events()
            except Exception:
                eis_win_open = False

        time.sleep(1.0)

    plt.ioff()
    plt.close("all")


# ─────────────────────────────────────────────────────────────────────────────
# Session drivers
# ─────────────────────────────────────────────────────────────────────────────
def wait_for_board(ser, timeout=12.0):
    print("  Waiting for live data (up to 12s)...")
    deadline = time.time() + timeout
    while time.time() < deadline:
        raw = ser.readline().decode("utf-8", errors="replace").strip()
        if "READY" in raw:
            print("  Firmware ready.")
        if raw.startswith("DATA,"):
            return True
    return False


def _step_current(step):
    """Best-effort magnitude (A) of a step's current, or None if not found."""
    if not isinstance(step, dict):
        return None
    for k in _CURRENT_KEYS:
        val = step.get(k)
        if isinstance(val, (int, float)) and not isinstance(val, bool):
            return abs(float(val))
    return None


def _peak_protocol_current(protocol):
    """Return (peak_A_or_None, n_steps, n_currents_found)."""
    steps = protocol.get("steps", []) if isinstance(protocol, dict) else []
    found = [c for c in (_step_current(s) for s in steps) if c is not None]
    peak = max(found) if found else None
    return peak, len(steps), len(found)


def _read_fault_ack(ser, timeout=1.5):
    """Drain the firmware's 'MSG,Set Fault I:' acknowledgement, if it arrives."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        try:
            line = ser.readline().decode("utf-8", errors="replace").strip()
        except Exception:
            return None
        if not line:
            continue
        if "Set Fault I" in line:
            return line
    return None


def configure_overcurrent(ser, protocol):
    """Raise the firmware over-current ceiling to match this run.

    The firmware enforces ONE global limit (MAX_CURRENT_FAULT) for the whole
    sequence, so we can't set a different limit per step during an autonomous
    run. We derive a single limit = peak commanded current * (1 + margin),
    confirm it, and send it via 'F' before the sequence is loaded/started.
    """
    peak, n_steps, n_found = _peak_protocol_current(protocol)

    if peak is not None and n_steps > 0 and n_found == n_steps:
        source = f"peak step current {peak:.3f} A"
    else:
        # Couldn't confidently read every step — ask rather than guess.
        if n_steps > 0 and n_found < n_steps:
            print(f"\n  Note: could only read current from {n_found}/{n_steps} steps "
                  f"of this protocol automatically.")
        raw = input("  Enter the highest charge/discharge current in this run "
                    "(A), or blank to skip: ").strip()
        if not raw:
            print("  Skipping over-current adjustment — firmware default "
                  "(0.500 A) stays in effect.")
            return
        try:
            peak = abs(float(raw))
        except ValueError:
            print("  Unparseable — skipping; firmware default (0.500 A) stays.")
            return
        source = f"entered current {peak:.3f} A"

    fault_A = max(peak * (1.0 + OVERCURRENT_MARGIN), OVERCURRENT_FLOOR)

    if OVERCURRENT_PROMPT:
        pct = int(round(OVERCURRENT_MARGIN * 100))
        print(f"\n  Over-current fault limit  —  {source}  +{pct}%.")
        raw = input(f"  Fault current (A) [default {fault_A:.3f}]: ").strip()
        if raw:
            try:
                fault_A = abs(float(raw))
            except ValueError:
                print(f"  Unparseable — using {fault_A:.3f} A.")

    if fault_A <= peak:
        print(f"  ⚠  Limit {fault_A:.3f} A is not above the run's peak "
              f"{peak:.3f} A; the run may fault. Continuing anyway.")

    ser.reset_input_buffer()
    ser.write(f"F{fault_A:.3f}\n".encode())
    ser.flush()
    ack = _read_fault_ack(ser)
    if ack:
        print(f"  Over-current fault limit set to {fault_A:.3f} A.")
    else:
        print(f"  Sent F{fault_A:.3f} (no ack seen — verify in the run log).")


def run_cycler_session(ser):
    """Reuse the cycler CLI's control logic on the shared port, with terminal
    telemetry and the min-span temperature plot."""
    import matplotlib.pyplot as plt

    ser.reset_input_buffer()

    print("\n  Streaming live readings for ~8s to verify sensors.")
    cyc.live_monitor(ser, duration=8.0)
    cyc.configure_thermal_limits(ser)

    print("\n  Build a new protocol, or load a saved one?")
    print("    [n] New   [l] Load")
    choice = input("  Select [n/l]: ").strip().lower()
    protocol = (cyc.load_protocol() or cyc.build_protocol()) if choice == "l" \
        else cyc.build_protocol()

    cyc.print_protocol_summary(protocol)
    if not cyc.prompt_yesno("\n  Upload and start this protocol?", default_yes=True):
        print("  Aborted — returning to menu.")
        return

    # Raise the global over-current ceiling to fit this run BEFORE loading/starting
    # ('F' is a persistent global setter; the firmware default 0.5 A would otherwise
    #  fault instantly on any higher-current step).
    configure_overcurrent(ser, protocol)

    ser.reset_input_buffer()
    cyc.upload_protocol(ser, protocol)

    csv_fh, csv_wr = cyc.open_csv(protocol)
    data = cyc.RunData()
    stop_event = threading.Event()

    reader = threading.Thread(
        target=cyc.serial_reader, args=(ser, data, csv_wr, csv_fh, stop_event),
        daemon=True)
    reader.start()

    telem = threading.Thread(
        target=telemetry_printer, args=(data, stop_event), daemon=True)
    telem.start()

    print(f"\n  Run started. Telemetry prints ~every {int(CYCLER_TELEMETRY_INTERVAL_S)}s.")
    print("  Close the plot window or Ctrl+C to stop early.\n")
    try:
        run_cycler_plot(data, protocol, stop_event)
    except KeyboardInterrupt:
        print("\n  Interrupted by user.")
    finally:
        stop_event.set()
        try:
            ser.write(b"s\n"); ser.flush()   # halt run; safe if already finished
        except Exception:
            pass
        reader.join(timeout=3)
        telem.join(timeout=2)
        csv_fh.close()

    with data.lock:
        fault, done = data.fault_msg, data.run_done
        n_eis = len(getattr(data, "eis_sweeps", []))
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
    print("=" * 56)

    plt.close("all")


def run_eis_session(ser):
    if not enter_eis_mode(ser):
        print("  Could not enter EIS mode — staying on the cycler side.")
        ser.reset_input_buffer()
        return
    print("  Opening EIS live window... (close it to return to the menu)")
    EISLiveWindow(ser).run()   # blocks until closed; sends EXIT on the way out
    time.sleep(0.5)
    ser.reset_input_buffer()
    print("  Returned to cycler mode (IDLE).")


# ─────────────────────────────────────────────────────────────────────────────
# Main menu
# ─────────────────────────────────────────────────────────────────────────────
def main():
    print("\n" + "=" * 60)
    print("  Unified Battery Cycler + EIS  —  iteration 1")
    print("=" * 60)
    if not IMPEDANCE_AVAILABLE:
        print("  (ECM fitting off: 'impedance' / 'scipy' not installed.)")

    print("\n  Scanning for serial ports...")
    port = cyc.find_serial_port()
    print(f"  Connecting to {port} @ 115200 baud...")
    try:
        ser = cyc.open_port(port)        # DTR/RTS low → no board reset on open
    except serial.SerialException as e:
        print(f"  [ERROR] Could not open port: {e}")
        sys.exit(1)
    ser.timeout = 1                      # snappier readline for both modes

    if not wait_for_board(ser):
        print("  [ERROR] No data from board. Is it powered and the Arduino Serial")
        print("  Monitor closed?")
        ser.close()
        sys.exit(1)
    print("  Board responding.")

    try:
        while True:
            print("\n" + "-" * 60)
            print("  Main menu")
            print("    [1] Auto-cycling & EIS  (build/load protocol, run + live plots)")
            print("    [2] EIS measurement  (live Nyquist/Bode sweep)")
            print("    [q] Quit")
            choice = input("  Select [1/2/q]: ").strip().lower()
            if choice == "1":
                run_cycler_session(ser)
            elif choice == "2":
                run_eis_session(ser)
            elif choice == "q":
                break
            else:
                print("  Enter 1, 2, or q.")
    except KeyboardInterrupt:
        print("\n  Interrupted.")
    finally:
        try:
            ser.write(b"s\n"); ser.flush()
        except Exception:
            pass
        print(f"\n  Waiting {_COOLDOWN_WAIT_S:.0f}s for fan run-on to expire...")
        time.sleep(_COOLDOWN_WAIT_S)
        ser.close()
        print("  Serial closed. Board left in IDLE. Bye.")


if __name__ == "__main__":
    main()


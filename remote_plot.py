#!/usr/bin/env python3
"""
remote_plot.py  —  PC-side live plot of a cycler / idle-log run on the Pi
==========================================================================

All plotting happens HERE (the PC). The Pi only runs one `tail -F` on the
run CSV, under `nice -n 19`, through ssh. The CSV is already flushed on every
row, so nothing in the cycler process changes and a plotter crash or a dropped
connection can never affect the run.

    PC:  remote_plot.py  --ssh-->  Pi:  nice tail -F run_xxx.csv

Picks the newest run_*.csv / idle_*.csv in --dir (EIS sweep files excluded)
unless --file is given. Reconnects automatically if the link drops (Wi-Fi,
Tailscale, laptop sleep), re-reads the window, and switches to a newer run file
if a new test started in the meantime.

Requires (PC only): matplotlib, numpy.  Needs key-based ssh to the Pi
(BatchMode — it never prompts for a password).

Usage:
    python remote_plot.py pihit@PiBatt                    # newest run, last 30 min
    python remote_plot.py pihit@pibatt --window 120       # last 2 h (Tailscale name)
    python remote_plot.py pihit@PiBatt --window 0         # whole run
    python remote_plot.py pihit@PiBatt --file run_x.csv   # a specific file
    python remote_plot.py pihit@PiBatt --once --save run.png   # snapshot, exit
    python remote_plot.py --local ~/battery-cycler        # no ssh (same machine)
"""

import argparse
import csv
import io
import shlex
import subprocess
import sys
import threading
import time

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
from matplotlib.lines import Line2D

# ── Defaults ────────────────────────────────────────────────────────────────
DEFAULT_DIR      = "~/battery-cycler"   # where the wrapper is launched on the Pi
DEFAULT_WINDOW   = 30.0     # minutes shown (0 = whole run)
REFRESH_S        = 1.0      # redraw cadence
STALE_S          = 10.0     # "no new data" warning threshold
MAX_PLOT_POINTS  = 6000     # display decimation per trace (data kept in full)
MIN_TEMP_SPAN_C  = 10.0     # same de-noising span as the wrapper's temp plot
FASTEST_ROW_S    = 0.05     # fastest firmware sample period (sizes the backfill)
RECONNECT_MAX_S  = 30.0

# ── Styling (matches battery_cycler.py / unified wrapper) ────────────────────
BG, BG_AX, EDGE, FG = "#1e1e2e", "#2a2a3e", "#555577", "#cccccc"
C_CHARGE, C_DISCH, C_REST = "#2196F3", "#F44336", "#9E9E9E"
C_FAULT, C_EIS = "#FF1744", "#00BCD4"
C_TEMP, C_TEMP2 = "#FF5722", "#CE93D8"
STATE_GROUPS = {   # plotted series -> firmware state strings
    "Charging":    (C_CHARGE, ("CHARGING", "GITT_PULSE")),
    "Discharging": (C_DISCH,  ("DISCHARGING",)),
    "Rest / Idle": (C_REST,   ("REST", "IDLE", "GITT_RELAX")),
    "EIS":         (C_EIS,    ("SEQ_EIS", "EIS")),
    "Fault":       (C_FAULT,  ("FAULT",)),
}

RUN_COLS  = ["elapsed_s", "state", "v_batt", "i_charge", "i_discharge",
             "temp_nmos_c", "temp_batt_c", "step_idx", "loop_iter"]
IDLE_COLS = ["wallclock_iso", "unix_s", "elapsed_s", "v_batt", "v_min", "v_max",
             "temp_batt_c", "temp_board_c", "n_samples"]


# ─────────────────────────────────────────────────────────────────────────────
# Remote side: one small sh script, run over ssh (or locally with --local)
# ─────────────────────────────────────────────────────────────────────────────
def _sh_path(p):
    """Quote a path for the remote shell, keeping a leading ~ expandable."""
    if p == "~":
        return '"$HOME"'
    if p.startswith("~/"):
        return '"$HOME"/' + shlex.quote(p[2:])
    return shlex.quote(p)


def remote_script(directory, filename, backfill_lines):
    pick = (f"f={shlex.quote(filename)}" if filename else
            "f=$(ls -t run_*.csv idle_*.csv 2>/dev/null | grep -v '_eis_step' | head -n 1)")
    start = "+1" if backfill_lines is None else str(int(backfill_lines))
    return (f"cd {_sh_path(directory)} || exit 3; {pick}; "
            '[ -n "$f" ] && [ -f "$f" ] || exit 4; '
            'echo "@@FILE $f"; head -n 8 -- "$f"; echo "@@TAIL"; '
            f'exec nice -n 19 tail -n {start} -F -- "$f"')


# ─────────────────────────────────────────────────────────────────────────────
# Shared store (reader thread writes, plot loop reads)
# ─────────────────────────────────────────────────────────────────────────────
class Store:
    def __init__(self, window_s):
        self.lock      = threading.Lock()
        self.window_s  = window_s
        self.reset(None, None)
        self.status    = "connecting..."

    def reset(self, filename, fmt):
        self.filename = filename
        self.fmt      = fmt            # "run" | "idle" | None
        self.rows     = []             # list of tuples, per format
        self.eis_marks = []            # elapsed_s of interleaved EIS sweeps
        self.last_el  = -1.0
        self.last_rx  = None           # local monotonic time of last new row
        self.version  = 0              # bumps on reset (forces a figure rebuild)

    def trim(self):
        if self.window_s and len(self.rows) > 4096:
            cut = self.last_el - 1.5 * self.window_s
            i = 0
            while i < len(self.rows) and self.rows[i][0] < cut:
                i += 1
            if i > len(self.rows) // 4:
                del self.rows[:i]
                self.eis_marks = [m for m in self.eis_marks if m >= cut]


def _detect_format(header_line):
    h = header_line.strip().lstrip('"')
    if h.startswith("elapsed_s,state"):
        return "run"
    if h.startswith("wallclock_iso"):
        return "idle"
    return None


def _parse_row(fmt, line):
    try:
        f = next(csv.reader(io.StringIO(line)))
    except StopIteration:
        return None
    try:
        if fmt == "run" and len(f) >= 9:
            # (elapsed, state, v, i_ch, i_dis, t_nmos, t_batt, step, loop)
            return (float(f[0]), f[1], float(f[2]), float(f[3]), float(f[4]),
                    float(f[5]), float(f[6]), int(f[7]), int(f[8]))
        if fmt == "idle" and len(f) >= 9:
            # (elapsed, v, v_min, v_max, t_batt, t_board, wallclock)
            return (float(f[2]), float(f[3]), float(f[4]), float(f[5]),
                    float(f[6]), float(f[7]), f[0])
    except ValueError:
        return None
    return None


def reader(args, store, stop):
    """Run the remote tail, parse lines into `store`, reconnect with backoff."""
    backfill = None
    if args.window > 0:
        backfill = int(args.window * 60 / FASTEST_ROW_S * 1.2) + 50
    script = remote_script(args.dir, args.file, backfill)
    if args.local is not None:
        cmd = ["sh", "-c", script]
    else:
        cmd = (["ssh", "-T", "-o", "BatchMode=yes", "-o", "ServerAliveInterval=15",
                "-o", "ServerAliveCountMax=3", "-o", "ConnectTimeout=10"]
               + args.ssh_opt + [args.host, script])

    delay = 2.0
    while not stop.is_set():
        try:
            proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                    stdin=subprocess.DEVNULL, text=True, bufsize=1,
                                    errors="replace")
        except OSError as e:
            with store.lock:
                store.status = f"cannot start ssh: {e}"
            return
        stop_watch = threading.Thread(target=lambda: (stop.wait(), proc.terminate()),
                                      daemon=True)
        stop_watch.start()

        fmt, in_tail, got_any = None, False, False
        stream_el = -1.0   # elapsed of the last row seen in THIS stream (incl. replays)
        for raw in proc.stdout:
            line = raw.rstrip("\n")
            if line.startswith("@@FILE "):
                fname = line[7:]
                with store.lock:
                    if fname != store.filename:     # first connect or a new run
                        store.reset(fname, None)
                        store.version += 1
                    store.status = "streaming"
                continue
            if line == "@@TAIL":
                in_tail = True
                continue
            if not in_tail:
                f2 = _detect_format(line)
                if f2:
                    fmt = f2
                    with store.lock:
                        if store.fmt != fmt:
                            store.fmt = fmt
                            store.version += 1
                continue
            if fmt is None:
                continue
            s = line.lstrip('"')
            if s.startswith("#"):
                # Anchor to the row preceding the comment in this stream, so a
                # replay after a reconnect lands the marker where it really was.
                if fmt == "run" and s.startswith("# EIS step") and stream_el >= 0:
                    with store.lock:
                        if stream_el not in store.eis_marks:
                            store.eis_marks.append(stream_el)
                continue
            if _detect_format(line):
                continue
            row = _parse_row(fmt, line)
            if row is None:
                continue
            stream_el = row[0]
            with store.lock:
                if row[0] <= store.last_el:        # duplicate after a reconnect
                    continue
                store.rows.append(row)
                store.last_el = row[0]
                store.last_rx = time.monotonic()
                store.trim()
            got_any = True

        proc.wait()
        if stop.is_set():
            return
        err = (proc.stderr.read() or "").strip().splitlines()
        msg = {3: f"directory not found: {args.dir}",
               4: "no run_*.csv / idle_*.csv found yet"}.get(proc.returncode,
               (err[-1] if err else f"connection closed (rc={proc.returncode})"))
        if got_any:
            delay = 2.0
        with store.lock:
            store.status = f"{msg} — retrying in {delay:.0f}s"
        if args.once:
            return
        stop.wait(delay)
        delay = min(delay * 2, RECONNECT_MAX_S)


# ─────────────────────────────────────────────────────────────────────────────
# Plotting (PC)
# ─────────────────────────────────────────────────────────────────────────────
def _style(ax):
    ax.set_facecolor(BG_AX)
    ax.tick_params(colors=FG, labelsize=9)
    for sp in ax.spines.values():
        sp.set_edgecolor(EDGE)
    ax.grid(True, color=EDGE, alpha=0.35, lw=0.6)


def _legend(ax, **kw):
    return ax.legend(facecolor=BG_AX, edgecolor=EDGE, labelcolor=FG, fontsize=8, **kw)


def _decimate(*arrs):
    n = len(arrs[0])
    if n <= MAX_PLOT_POINTS:
        return arrs
    step = int(np.ceil(n / MAX_PLOT_POINTS))
    return tuple(a[::step] for a in arrs)


def _time_axis(span_s):
    """Pick minutes or hours for the x axis."""
    return (3600.0, "h") if span_s > 3 * 3600 else (60.0, "min")


class Plotter:
    def __init__(self, args, store):
        self.args, self.store = args, store
        self.fig = plt.figure(figsize=(13, 8))
        self.fig.patch.set_facecolor(BG)
        try:
            self.fig.canvas.manager.set_window_title(f"Cycler — {args.host or 'local'}")
        except Exception:
            pass
        self.version = None
        self.fmt = None
        self.closed = False
        self.fig.canvas.mpl_connect("close_event", lambda _e: setattr(self, "closed", True))
        self._build(None)

    # --- figure layout, rebuilt when the file / format changes ---------------
    def _build(self, fmt):
        self.fig.clf()
        self.fmt = fmt
        self.eis_lines = []
        self.band = None
        if fmt == "idle":
            self.ax_v = self.fig.add_subplot(2, 1, 1)
            self.ax_t = self.fig.add_subplot(2, 1, 2, sharex=self.ax_v)
            self.ax_i = None
        else:
            self.ax_v = self.fig.add_subplot(2, 1, 1)
            self.ax_i = self.fig.add_subplot(2, 2, 3)
            self.ax_t = self.fig.add_subplot(2, 2, 4, sharex=self.ax_i)
        axes = [a for a in (self.ax_v, self.ax_i, self.ax_t) if a is not None]
        for a in axes:
            _style(a)
        self.ax_v.set_ylabel("Voltage (V)", color=FG, fontsize=11)
        self.ax_t.set_ylabel("Temperature (°C)", color=FG, fontsize=11)
        for a in ((self.ax_t,) if fmt == "idle" else (self.ax_v, self.ax_i, self.ax_t)):
            a.set_xlabel("Elapsed (min)", color=FG, fontsize=10)   # reserves layout space

        if fmt == "idle":
            (self.l_v,) = self.ax_v.plot([], [], color=C_REST, lw=1.8, label="V (mean)")
            _legend(self.ax_v, loc="upper left")
            self.l_t1, = self.ax_t.plot([], [], color=C_TEMP2, lw=1.6, ls="--", label="T batt")
            self.l_t2, = self.ax_t.plot([], [], color=C_TEMP, lw=1.6, label="T board")
        else:
            self.l_state = {}
            for name, (col, _) in STATE_GROUPS.items():
                self.l_state[name], = self.ax_v.plot([], [], color=col, lw=2.2)
            _legend(self.ax_v, loc="upper left",
                    handles=[Line2D([0], [0], color=c, lw=2, label=n)
                             for n, (c, _) in STATE_GROUPS.items()])
            self.l_ich, = self.ax_i.plot([], [], color=C_CHARGE, lw=1.8, label="I charge")
            self.l_idis, = self.ax_i.plot([], [], color=C_DISCH, lw=1.8, label="I discharge")
            self.ax_i.set_ylabel("Current (A)", color=FG, fontsize=11)
            _legend(self.ax_i, loc="upper left")
            self.l_t1, = self.ax_t.plot([], [], color=C_TEMP, lw=1.6, label="T NMOS")
            self.l_t2, = self.ax_t.plot([], [], color=C_TEMP2, lw=1.6, ls="--", label="T batt")
        _legend(self.ax_t, loc="upper left")

        self.title = self.fig.suptitle("", color="#eeeeee", fontsize=13, y=0.985)
        self.status = self.fig.text(0.5, 0.935, "", ha="center", va="center",
                                    color="#eeeeee", fontsize=10, family="monospace")
        self.fig.tight_layout(rect=[0, 0, 1, 0.91], h_pad=2.5, w_pad=2.0)

    # --- one refresh ------------------------------------------------------------
    def update(self):
        st = self.store
        with st.lock:
            if st.version != self.version or st.fmt != self.fmt:
                self.version = st.version
                fmt = st.fmt
                rebuild = True
            else:
                fmt, rebuild = self.fmt, False
            rows = list(st.rows)
            marks = list(st.eis_marks)
            fname, status, last_rx = st.filename, st.status, st.last_rx
        if rebuild:
            self._build(fmt)

        name = fname or "(waiting for file)"
        self.title.set_text(f"{self.args.host or 'local'}  ·  {name}")
        if not rows or fmt is None:
            self.status.set_text(status)
            return

        el = np.fromiter((r[0] for r in rows), float, len(rows))
        t_end = el[-1]
        win = self.args.window * 60.0
        t_start = max(0.0, t_end - win) if win > 0 else 0.0
        m = el >= t_start
        idx = np.nonzero(m)[0]
        # Decimate rows BEFORE colour-masking so state transitions stay bridged;
        # always keep the newest row.
        step = max(1, int(np.ceil(len(idx) / MAX_PLOT_POINTS)))
        if step > 1:
            idx = np.unique(np.append(idx[::step], idx[-1]))
        rows_w = [rows[i] for i in idx]
        el_w = el[idx]
        scale, unit = _time_axis(max(t_end - t_start, win))
        x = el_w / scale

        if fmt == "run":
            v    = np.array([r[2] for r in rows_w])
            ich  = np.array([r[3] for r in rows_w])
            idis = np.array([r[4] for r in rows_w])
            t1   = np.array([r[5] for r in rows_w])
            t2   = np.array([r[6] for r in rows_w])
            states = np.array([r[1] for r in rows_w])
            for gname, (_, members) in STATE_GROUPS.items():
                mask = np.isin(states, members)
                mask[1:] |= mask[:-1]          # bridge the transition sample
                vv = np.where(mask, v, np.nan)
                self.l_state[gname].set_data(*_decimate(x, vv))
            self.l_ich.set_data(*_decimate(x, ich))
            self.l_idis.set_data(*_decimate(x, idis))
            last = rows_w[-1]
            info = f"{last[1]}  step {last[7]}  loop {last[8]}  V={last[2]:.4f}"
        else:
            v    = np.array([r[1] for r in rows_w])
            vmin = np.array([r[2] for r in rows_w])
            vmax = np.array([r[3] for r in rows_w])
            t1   = np.array([r[4] for r in rows_w])
            t2   = np.array([r[5] for r in rows_w])
            self.l_v.set_data(*_decimate(x, v))
            if self.band is not None:
                self.band.remove()
            xd, lo, hi = _decimate(x, vmin, vmax)
            self.band = self.ax_v.fill_between(xd, lo, hi, color=C_REST, alpha=0.25, lw=0)
            last = rows_w[-1]
            info = f"IDLE LOG  V={last[1]:.4f}  T_batt={last[4]:.1f}C  {last[6][11:19]}"

        self.l_t1.set_data(*_decimate(x, t1))
        self.l_t2.set_data(*_decimate(x, t2))

        # Interleaved EIS sweeps (run files): dashed cyan markers
        for ln in self.eis_lines:
            ln.remove()
        self.eis_lines = []
        for mk in marks:
            if mk >= t_start:
                for ax in (self.ax_v, self.ax_i, self.ax_t):
                    if ax is not None:
                        self.eis_lines.append(ax.axvline(mk / scale, color=C_EIS,
                                                          ls="--", lw=1.0, alpha=0.8))

        x0 = t_start / scale
        x1 = (t_start + win) / scale if win > 0 else max(t_end / scale, x0 + 1e-6)
        for ax in (self.ax_v, self.ax_i, self.ax_t):
            if ax is None:
                continue
            ax.set_xlim(x0, x1)
            ax.relim(visible_only=True)
            ax.autoscale_view(scalex=False)
            ax.xaxis.set_major_formatter(ticker.FormatStrFormatter("%.1f" if unit == "h" else "%.0f"))
        for ax in ((self.ax_v, self.ax_t) if fmt == "idle" else (self.ax_v, self.ax_i, self.ax_t)):
            ax.set_xlabel(f"Elapsed ({unit})", color=FG, fontsize=10)
        if fmt == "idle":
            self.ax_v.set_xlabel("")

        # Temperature: minimum span so a stable reading isn't all ADC noise
        tv = np.concatenate([t1, t2])
        tv = tv[np.isfinite(tv)]
        if tv.size:
            lo, hi = float(tv.min()), float(tv.max())
            c, half = 0.5 * (lo + hi), max(hi - lo, MIN_TEMP_SPAN_C) * 0.5 * 1.15
            self.ax_t.set_ylim(c - half, c + half)

        age = (time.monotonic() - last_rx) if last_rx else 0.0
        stale = age >= STALE_S
        if self.args.once:
            tail = "snapshot"
        elif stale or status != "streaming":
            tail = f"NO DATA {age:.0f}s  ({status})"
        else:
            tail = f"live, {age:.0f}s ago"
        self.status.set_text(f"{info}   ·   {tail}")
        self.status.set_color("#f38ba8" if stale else "#eeeeee")


# ─────────────────────────────────────────────────────────────────────────────
def main():
    ap = argparse.ArgumentParser(description="Live PC-side plot of a run on the Pi.")
    ap.add_argument("host", nargs="?", help="ssh target, e.g. pihit@PiBatt or a Tailscale name")
    ap.add_argument("--dir", default=DEFAULT_DIR,
                    help=f"run directory on the Pi (default {DEFAULT_DIR})")
    ap.add_argument("--file", help="plot this CSV instead of the newest one")
    ap.add_argument("--window", type=float, default=DEFAULT_WINDOW,
                    help="minutes to show (0 = whole run; default %(default)g)")
    ap.add_argument("--refresh", type=float, default=REFRESH_S, help="redraw period, s")
    ap.add_argument("--ssh-opt", action="append", default=[],
                    help="extra ssh option, repeatable (e.g. --ssh-opt=-p2222)")
    ap.add_argument("--local", metavar="DIR", nargs="?", const=".",
                    help="read a local directory instead of ssh (testing / same machine)")
    ap.add_argument("--save", metavar="PNG", help="also write the figure to this PNG each refresh")
    ap.add_argument("--once", action="store_true",
                    help="read what's there, draw once (to --save if given), exit")
    args = ap.parse_args()
    if args.local is not None:
        args.dir, args.host = args.local, None
    elif not args.host:
        ap.error("host is required (or use --local DIR)")

    store = Store(args.window * 60.0)
    stop = threading.Event()
    th = threading.Thread(target=reader, args=(args, store, stop), daemon=True)
    th.start()

    if args.once:
        # Backfill only: wait until the stream goes quiet, then draw once.
        t_end, last_n = time.monotonic() + 30, -1
        while time.monotonic() < t_end:
            time.sleep(1.0)
            with store.lock:
                n = len(store.rows)
            if n and n == last_n:
                break
            last_n = n
        stop.set()

    p = Plotter(args, store)
    if args.once:
        p.update()
        if args.save:
            p.fig.savefig(args.save, dpi=110, facecolor=BG)
            print(f"saved {args.save}")
        else:
            plt.show()
        return

    plt.ion()
    plt.show(block=False)
    try:
        while not p.closed:
            p.update()
            if args.save:
                p.fig.savefig(args.save, dpi=110, facecolor=BG)
            p.fig.canvas.draw_idle()
            plt.pause(args.refresh)
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        th.join(timeout=3)


if __name__ == "__main__":
    main()

# EIS-Cycler

Combined battery cycling + Electrochemical Impedance Spectroscopy (EIS)
instrument. ESP32-S3 firmware plus a Python host stack that builds cycling
protocols, streams live telemetry, and plots interleaved EIS sweeps.

---

## 1. Dependencies

### 1.1 Firmware build + flash toolchain (maintainer machine)

| Tool | Purpose | Arch install |
|------|---------|--------------|
| `arduino-cli` | Compile the `.ino` and export binaries | `pacman -S arduino-cli` (or AUR) |
| ESP32 board core | `esp32:esp32` platform for arduino-cli | see §2.1 |
| `esptool` | Flash the binary over USB | AUR: `esptool` |

> **Note on `esptool`:** The old `esptool.py write_flash` invocation is
> deprecated. On Arch (AUR `esptool` package) the current CLI uses hyphenated
> subcommands: `esptool write-flash`, `esptool flash-id`, `esptool chip-id`.

### 1.2 CLI flashing + serial monitoring (end user machine)

The end user does **not** need arduino-cli or the board core — only:

| Tool | Purpose | Install |
|------|---------|---------|
| `esptool` | Flash the distributed `.bin` | Arch: AUR `esptool` · pip: `pip install esptool` |
| Serial monitor | Read the board's voltage/temp stream | `arduino-cli monitor`, or any terminal (see §3.3) |

### 1.3 Python host stack

Requires **Python 3.9+**.

**Required** (core UI + telemetry + plotting):
```bash
pip install pyserial numpy matplotlib
```

**Optional** (enables extra features, auto-disabled if absent):
```bash
pip install scipy impedance
```
- `scipy` → IQR / median-filter outlier rejection on EIS sweeps.
- `impedance` → per-sweep ECM (equivalent-circuit) fit overlays.

**`tkinter`** (the GUI toolkit) ships with CPython on most platforms but is a
**separate package on Arch**:
```bash
sudo pacman -S tk
```

Quick check that everything imports:
```bash
python -c "import serial, numpy, matplotlib, tkinter; print('core OK')"
python -c "import scipy, impedance; print('optional OK')"   # ok to fail
```

---

## 2. Build & export the firmware binary

### 2.1 One-time: install the ESP32 board core

```bash
arduino-cli config init
arduino-cli config add board_manager.additional_urls \
  https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
arduino-cli core update-index
arduino-cli core install esp32:esp32
```

Confirm it's present:
```bash
arduino-cli core list          # expect esp32:esp32 in the list
```

### 2.2 The FQBN (board + option string)

The full FQBN must match the settings the board was validated with. The two
options that most commonly cause a "flashes fine but no serial output" failure
are `USBMode` and `CDCOnBoot` — omitting them lets arduino-cli substitute
defaults that route `Serial` to the *other* USB connector.

Validated string for the ESP32-S3 N16R8 (16 MB flash / 8 MB PSRAM):

```
esp32:esp32:esp32s3:USBMode=hwcdc,CDCOnBoot=cdc,FlashMode=qio,FlashSize=16M,PSRAM=opi,PartitionScheme=default
```

> **Before trusting the above,** verify it against what actually compiled by
> reading the `fqbn` field of the build folder's `build.options.json`, or the
> Arduino IDE Tools menu ("USB Mode" = *Hardware CDC and JTAG*, "USB CDC On
> Boot" = *Enabled*). If your IDE used different values, edit the string to
> match — the binary you ship bakes these in.

### 2.3 Compile with binary export

```bash
arduino-cli compile \
  --fqbn "esp32:esp32:esp32s3:USBMode=hwcdc,CDCOnBoot=cdc,FlashMode=qio,FlashSize=16M,PSRAM=opi,PartitionScheme=default" \
  --export-binaries \
  /home/rohit/Arduino/EIS-Cycler-Firmware_02-B2
```

The path argument is the **sketch folder (input)** — the directory containing
`EIS-Cycler-Firmware_02-B2.ino`. Arduino requires the folder basename and the
`.ino` filename to match exactly. Binaries land in a `build/` subfolder of the
sketch (add `--output-dir <DIR>` to redirect them elsewhere).

### 2.4 What gets produced

```bash
ls firmware/.../build/esp32.esp32.esp32s3/
```

The recent ESP32 core auto-merges, so the file you distribute is:

```
EIS-Cycler-Firmware_02-B2.ino.merged.bin      <-- single flashable image, offset 0x0
```

(The separate `.bootloader.bin`, `.partitions.bin`, and `.ino.bin` are the
components it merged; you don't need them for distribution.) Confirm the flash
offset/mode with:
```bash
cat firmware/.../build/esp32.esp32.esp32s3/flash_args
```

For a release, rename the merged image to a versioned name, tag the commit it
was built from, and attach the `.bin` to a GitHub Release — do **not** commit
the binary into the repo:
```bash
cp .../EIS-Cycler-Firmware_02-B2.ino.merged.bin EIS-Cycler_fw-v0.1.0.bin
git tag -a fw-v0.1.0 -m "release notes here"
git push origin fw-v0.1.0
# then upload EIS-Cycler_fw-v0.1.0.bin as a Release asset
```

---

## 3. Flash the board via CLI

### 3.1 Find the port

The ESP32-S3's native USB usually enumerates as `ttyACM*`; a board with a
separate UART bridge chip appears as `ttyUSB*`. Check both:
```bash
ls /dev/ttyACM* /dev/ttyUSB*
esptool --port /dev/ttyACM0 flash-id      # confirms the board is detected
```

### 3.2 Write the flash

```bash
esptool --chip esp32s3 --port /dev/ttyACM0 write-flash 0x0 EIS-Cycler_fw-v0.1.0.bin
```

A successful run prints `Hash of data verified.` and `Hard resetting via RTS
pin...`. **Note:** a verified write only proves the bytes are on the flash — it
does *not* prove the firmware is running or that you're watching the right
serial interface (see §3.3).

### 3.3 Read the serial monitor (verify it's actually running)

After flashing, the board should stream continuous voltage/temperature
readouts. Watch it at **115200 baud**:
```bash
arduino-cli monitor -p /dev/ttyACM0 -c baudrate=115200
```
Any generic terminal works too — e.g. `screen /dev/ttyACM0 115200` (exit with
`Ctrl-A` then `K`), or `python -m serial.tools.miniterm /dev/ttyACM0 115200`.

> **If the write verified but the monitor is silent:** you're almost certainly
> on the wrong port *or* the firmware's `Serial` was compiled to the other USB
> interface. Try the other `tty*` device, and confirm `CDCOnBoot`/`USBMode` in
> the FQBN (§2.2) match the board's known-good settings.

### 3.4 Windows differences

The workflow is the same; only the surface details change:

- **Ports are `COM` names**, not `/dev/tty*`. Find the number in Device Manager
  under "Ports (COM & LPT)", then:
  ```
  esptool --chip esp32s3 --port COM7 write-flash 0x0 EIS-Cycler_fw-v0.1.0.bin
  ```
- **Serial monitor:** `arduino-cli monitor -p COM7 -c baudrate=115200`, or PuTTY
  / the Arduino IDE monitor at 115200.
- **Python launcher:** use `py -m pip install ...` and `py unified_cycler_eis.py`.
- **USB drivers:** if the board never appears as a COM port, install the CP210x
  or CH34x VCP driver (for boards with a UART bridge). Native-USB S3 boards
  usually enumerate without a driver on Windows 10/11.
- `esptool` itself is identical (`pip install esptool` provides the same CLI).

---

## 4. Python host wrapper (`unified_cycler_eis.py`)

This is the single entry point. It auto-detects the serial port, opens it
**without resetting the board** (DTR/RTS held low), verifies the board is
streaming, then presents an interactive menu — there are no command-line flags.

```bash
python unified_cycler_eis.py
```

> Close the Arduino IDE Serial Monitor (or any other program holding the port)
> first — only one process can own the serial device at a time.

### 4.1 Main menu

| Key | Mode | What it does |
|-----|------|--------------|
| `1` | **Auto-cycling & EIS** | Build or load a protocol, upload it, run it with live telemetry + plots (Nyquist/Bode windows open automatically if the protocol contains an EIS step). |
| `2` | **EIS measurement** | Opens the live Nyquist/Bode sweep window for a standalone impedance measurement (no cycling). |
| `q` | **Quit** | Sends the board to IDLE and closes the port cleanly. |

On exit (menu quit, `Ctrl+C`, or closing a plot window) the wrapper always
sends the stop command so the board is left safely in **IDLE**.

### 4.2 Cycling session flow (menu option `1`)

1. **Sensor check** — streams ~8 s of live readings so you can confirm voltage
   and temperature look sane before committing.
2. **Thermal limits** — prompts for temperature cutoffs.
3. **Protocol** — choose `[n]` new (opens the builder, §4.4) or `[l]` load a
   saved `protocol_*.json`.
4. **Summary + confirm** — prints the protocol; `[y]` uploads and starts.
5. **Over-current ceiling** — before starting, the wrapper raises the firmware's
   global over-current limit to fit the highest-current step in the protocol
   (the firmware default would otherwise fault instantly on a higher-current
   step).
6. **Run** — a CSV is opened next to the script; a background reader logs every
   data row; telemetry prints to the terminal roughly once a minute; a live
   voltage/current/temperature plot updates in real time. Interleaved EIS sweeps
   are saved as separate CSVs alongside the run CSV.
7. **Stop** — close the plot window or press `Ctrl+C` to end early; the run ends
   on its own when the sequence (and any loop) completes or a fault trips.

### 4.3 EIS session flow (menu option `2`)

Enters EIS mode and opens the live window (Nyquist plus |Z|/phase Bode). Adjust
sweep points and start/stop frequency in the window; if `scipy`/`impedance` are
installed you also get outlier filtering and a live ECM fit overlay. Close the
window to return to IDLE and the main menu.

### 4.4 Protocol builder & the repeat-loop feature

`build_protocol()` walks you through naming the protocol, entering the cell's
rated capacity (mAh), then adding steps one at a time. Step types:

| # | Step | Key parameters |
|---|------|----------------|
| 1 | **CC charge** | C-rate, Vmax cutoff, optional time cap |
| 2 | **CV charge (taper)** | Hold voltage, taper cutoff C-rate, optional time cap |
| 3 | **CC discharge** | C-rate, Vmin cutoff, optional time cap |
| 4 | **Rest** | Duration (minutes) |
| 5 | **EIS sweep** | Sweep points, start Hz, stop Hz (blocking; power paths off during the sweep) |

Any charge/discharge/rest step can optionally take a faster sample interval
(useful for short pulses). A time cap of `0` means "no time limit — stop on the
voltage cutoff only".

**The repeat loop.** After you finish adding steps, the builder asks whether to
define a repeat loop over a *range* of steps:

- **Start / end step index** — the inclusive span of steps to repeat (e.g.
  steps 1–3 to loop a charge → rest → discharge block while leaving a leading
  formation step and a trailing EIS step outside the loop).
- **Guard voltage** — a safety floor checked at *every* step inside the loop; if
  the cell voltage crosses it at any point, the loop aborts and the sequence
  moves on. This is what makes the loop terminate naturally on capacity fade
  rather than running forever.
- **Iteration cap** — the firmware hard-caps the loop at **50 iterations**
  regardless of the guard voltage, as a backstop.

The protocol (steps + loop) is saved as `protocol_<name>.json` in the working
directory and can be reloaded later with the `[l]` option. The loop range and
guard are shown in the protocol summary, and the live plot annotates the current
loop iteration while the run is in the looped region.

> **Building a cycle-life test:** put your formation/conditioning steps first,
> then loop the charge→rest→discharge (optionally with an EIS step inside the
> loop to track impedance growth per cycle), with the guard voltage set to the
> end-of-life threshold you want to stop at.

---

## 5. Typical end-to-end flow

**Maintainer** (new firmware release):
```bash
arduino-cli compile --fqbn "<full FQBN from §2.2>" --export-binaries <sketch dir>
cp .../*.merged.bin EIS-Cycler_fw-vX.Y.Z.bin
git tag -a fw-vX.Y.Z -m "..." && git push origin fw-vX.Y.Z
# attach EIS-Cycler_fw-vX.Y.Z.bin to the GitHub Release
```

**End user** (flash a release):
```bash
pip install esptool pyserial numpy matplotlib       # + optional scipy impedance, + tk on Arch
esptool --chip esp32s3 --port <PORT> write-flash 0x0 EIS-Cycler_fw-vX.Y.Z.bin
arduino-cli monitor -p <PORT> -c baudrate=115200    # confirm voltage/temp stream
python unified_cycler_eis.py                          # run cycling / EIS
```

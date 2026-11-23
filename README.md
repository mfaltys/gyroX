# esp32-drift — open-source RC car gyro

A tiny steering gyro for RC cars built around a **Seeed XIAO ESP32C3** and an **MPU-6050**.
It sits **between your ELRS PWM receiver and the steering servo**, reads the receiver PWM,
adds gyro-based yaw correction, and drives the servo.

Settings are configured over a self-hosted Wi-Fi AP + web UI. The AP is off by default:
**hold the XIAO boot button for ~3 s after power-up** to bring it up, and it is gone
again after a power cycle.

Firmware targets **ESP-IDF 6.0.1** (ESP-IDF 5.x works too).

```
Receiver (PWM) ──┐
                 ├─ XIAO ESP32C3 ──► Servo
MPU-6050 (I2C) ─┘        (gyro mixing)
```

## Features

- PWM input decode (ELRS steering channel) with signal-lost failsafe → steering centers
- MPU-6050 gyro: yaw-rate reading, zero-rate calibration stored in NVS
- Two mixing modes:
  - **Rate:** steering correction proportional to yaw rate (deg/s)
  - **Heading hold:** holds a heading (target captured with "Reset heading-hold target")
- Adjustable gain, axis reversing, deadband, trim, servo min/center/max (µs)
- Web UI at `http://192.168.4.1/` while in AP mode; captive portal DNS included
- All settings saved in NVS and survive reboots
- 100 Hz control loop

## Wiring (suggested defaults)

The XIAO ESP32C3 silkscreen labels (D0–D10) map to raw GPIO numbers like this:

| XIAO silk | D0 | D1 | D2 | D3 | D4 | D5 | D6 | D7 | D8 | D9 | D10 |
|-----------|----|----|----|----|----|----|----|----|----|----|-----|
| **GPIO**  | 2  | 3  | 4  | 5  | 6  | 7  | 21 | 20 | 8  | 9  | 10  |
| **Note**  |    |    |    |    | SCL| SDA| TX | RX | SCK| BOOT| MOSI|

Wire by the **silkscreen name**:

| XIAO pin (silk) | GPIO | Function | Connects to |
|-----------------|------|----------|-------------|
| **D5** | GPIO 7 | I2C SDA | MPU-6050 SDA |
| **D4** | GPIO 6 | I2C SCL | MPU-6050 SCL |
| **D8** | GPIO 8 | Servo signal | Servo signal wire (usually white/orange) |
| **D10** | GPIO 10 | PWM input | ELRS receiver steering-channel signal |
| GND | — | Common ground | MPU-6050 GND, servo GND, receiver GND |
| 5V / 3V3 | — | Power | MPU-6050 VCC (3V3 recommended) |

> **Do not use D9 for the servo** — D9 is GPIO9, the XIAO's boot-button pin. The button
> shorts that pin to ground, which fights the servo PWM output and glitches the servo
> every time it's pressed. D8 (GPIO8) is fine — it's a boot strapping pin, but the servo
> input is high-impedance and never drives it, so boot mode is unaffected.

The **boot button (GPIO 9 / D9)** is on the board itself — no wiring needed.

> **Why not hold the button at power-up?** GPIO 9 is a strapping pin on the ESP32-C3:
> held low at reset it puts the chip into the ROM download mode (that's the usual
> flash gesture). The firmware therefore only checks the button *after* boot — hold it
> for ~3 s once the car is powered. If you accidentally end up in download mode
> (no app logs on the serial monitor, board appears dead), just power-cycle without
> touching the button.

**Power:** run the servo from its own supply/BEC, not the XIAO. All grounds must be common.
Power the MPU-6050 module from **3V3** — most breakout boards (GY-521) tie their I2C
pull-ups to VCC, so feeding them 5 V puts 5 V on the ESP32-C3 I2C pins, which are not
5 V tolerant. Most modules include I2C pull-ups; if yours doesn't, the firmware's
internal pull-ups usually suffice at 100 kHz.

> GPIO pins are compile-time defaults in `main/config.h` — change them there and rebuild
> if you want a different layout. All *tuning* values (gain, mode, trim, servo limits…)
> are stored in NVS and editable over the web UI. The boot button (GPIO 9) is not
> remappable.

## How it works

1. **Drive mode (default):** power the car **without** holding the boot button.
   - No Wi-Fi is started.
   - The mixer runs at 100 Hz: `output = clamp(rx + gyro_correction, -100, 100)`.
   - On first boot (no saved gyro offset) the firmware auto-calibrates the MPU-6050
     zero-rate offset — **keep the car still for ~1.5 s after power-up**.

2. **Web UI mode:** power up normally, then **press and hold the boot button for
   ~3 seconds** (any time after power-up).
   - The XIAO starts an open AP: SSID `ESP32-DRIFT`, address `192.168.4.1`.
   - Connect your phone/laptop, open `http://192.168.4.1/`.
   - Adjust gyro gain/mode/reversing, steering deadband/trim, servo limits, calibrate
     the gyro, reset the heading-hold target, and click **Save** (writes NVS).
   - The mixer keeps running the whole time — you can tune the car live.
   - **Power cycle the car** → AP comes back up only if you hold the boot button again.

## Config parameters (web UI / NVS)

| Key | Meaning | Default |
|-----|---------|---------|
| gyro_mode | 0 = rate, 1 = heading hold | 0 |
| gyro_gain | Correction strength, ×0.1 % per °/s | 80 (8 %) |
| gyro_reverse | Invert gyro correction axis | 0 |
| steer_reverse | Invert steering input | 0 |
| steer_deadband | RX deadband around center, % | 8 |
| steer_trim | Steering offset, % | 0 |
| servo_min_us | Servo pulse at -100 % | 500 |
| servo_center_us | Servo pulse at 0 % | 1500 |
| servo_max_us | Servo pulse at +100 % | 2500 |
| hold_p | Heading-hold P, ×0.1 (°/s per °) | 40 |
| gyro_z_offset | Stored gyro zero-rate offset (°/s) | calibrated |
| web_ap_ssid | AP SSID in web UI mode | ESP32-DRIFT |

## Build & flash

Requirements: ESP-IDF 6.0.1 (unpacked at `C:\esp\v6.0.1\esp-idf` here) and the IDF
Python dependencies installed in an IDF environment (e.g. the `idf6.0.1_env` venv
created under `C:\Users\mfaltys\.espressif\python_env\`).

```cmd
:: from the project root
set IDF_PATH=C:\esp\v6.0.1\esp-idf

:: pick the target (sdkconfig.defaults already selects esp32c3)
idf.py set-target esp32c3

:: build
idf.py build

:: flash (adjust -p to your COM port)
idf.py -p COM3 flash

:: watch logs
idf.py -p COM3 monitor
```

The XIAO ESP32C3 uses its native USB-Serial-JTAG for flashing — Windows should present
a COM port automatically.

## Project layout

```
esp32-drift/
├── CMakeLists.txt
├── sdkconfig.defaults      # ESP-IDF sdkconfig overrides (esp32c3, 4 MB, custom partitions)
├── partitions.csv          # NVS + factory app partition table
├── README.md
├── main/
│   ├── CMakeLists.txt
│   ├── app_main.c          # entry point: boot-button monitor (3 s hold), gyro cal, mixer task
│   ├── config.h / config.c # NVS settings with first-boot defaults
│   ├── gyro.h / gyro.c     # MPU-6050 driver + heading integration
│   ├── pwm_in.h / pwm_in.c # ELRS PWM input decode (GPIO ISR)
│   ├── servo.h / servo.c   # LEDC servo output
│   ├── mixing.h / mixing.c # rx + gyro mixing + failsafe
│   ├── web.h / web.c       # softAP + captive DNS + HTTP settings API
│   ├── web/index.html      # web UI (embedded at build time)
│   ├── web/style.css
│   └── cjson.c / cjson.h   # vendored cJSON (MIT, Dave Gamble et al.)
```

## Notes & limitations

- The gyro integrates yaw rate into a heading; it assumes the car stays roughly flat
  (no pitch/roll compensation). For drift cars this is exactly the axis you want.
- Heading hold captures its target when you click **Reset heading-hold target** — do
  that with the car pointing straight.
- AP mode is deliberately temporary: 3 s boot-button hold after power-up → AP on;
  power cycle → AP off.
- ELRS PWM update rates vary (50–500 Hz typical); the decoder edge-times every pulse
  and low-pass filters the result, so it works across rates.

## License

Open source. `main/cjson.{c,h}` is vendored cJSON, MIT licensed (see the file headers).
Everything else in this repo is MIT licensed — add a `LICENSE` file before publishing.

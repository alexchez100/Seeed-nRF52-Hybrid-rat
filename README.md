# Seeed nRF52 Hybrid Rat

Firmware for the **Seeed XIAO nRF52840 Sense Plus** board designed for hybrid rat neuroscience experiments. Provides real-time 6-axis IMU streaming over USB and independent PWM control of two vibration motors via serial commands.

## Hardware

| Component | Pin | Description |
|---|---|---|
| Left vibration motor | D0 (P0.02) | PWM @ 1 kHz |
| Right vibration motor | D1 (P0.03) | PWM @ 1 kHz |
| IMU (LSM6DS3TR-C) | I2C0 | 6-axis accelerometer + gyroscope |
| Red LED | P0.26 | Heartbeat indicator |
| Green LED | P0.30 | Left motor active indicator |
| Blue LED | P0.06 | Right motor active indicator |
| USB CDC ACM | — | Virtual COM port for commands and data |

## IMU Output

Accelerometer (m/s²) and gyroscope (rad/s) data is streamed continuously at **100 Hz** over the USB serial port:

```
ACC 1.176072 9.451649 3.220739 | GYR 0.001221 -0.042913 0.002138
ACC 1.168893 9.310472 3.144168 | GYR 0.007024 -0.048258 0.005650
```

When calibration is active, a `[CAL]` tag is appended. During calibration collection, a countdown `[CAL 8s]` is shown.

## Commands

All commands are **case-insensitive** and sent via USB serial terminal (CR or LF as line ending).

### Motor Control

| Command | Description |
|---|---|
| `L on [0-100]` | Left motor ON at given duty cycle % (default 100) |
| `L off` | Left motor OFF |
| `R on [0-100]` | Right motor ON at given duty cycle % (default 100) |
| `R off` | Right motor OFF |
| `on [0-100]` | Both motors ON |
| `off` | Both motors OFF |

### IMU Calibration

| Command | Description |
|---|---|
| `cal` | Start 10-second calibration (keep device still) |
| `cal off` | Disable calibration, revert to raw values |

Calibration computes gyroscope zero-rate bias (subtracted from readings) and accelerometer scale correction (magnitude normalized to 9.80665 m/s²). Can be re-run at any time.

### Status & LEDs

| Command | Description |
|---|---|
| `status` | Show motor duty %, calibration state, LED indicator state |
| `led on` | Enable green/blue motor indicator LEDs |
| `led off` | Disable indicator LEDs (motors keep running) |
| `help` | Print command list |

## LED Behavior

- **Red** — heartbeat: 100 ms on / 1400 ms off
- **Green** — lit while left motor duty > 0 %
- **Blue** — lit while right motor duty > 0 %

Green and blue indicators can be toggled with `led on` / `led off`.

## Building

Requires **nRF Connect SDK v3.2.3** (Zephyr RTOS).

### Command line

```bash
# Configure
cmake -B build -GNinja -DBOARD=xiao_ble/nrf52840/sense .

# Build
ninja -C build
```

### nRF Connect for VS Code

1. Open this folder as an application
2. Select board: **xiao_ble** with qualifier **nrf52840/sense**
3. Build

## Flashing

1. Double-tap the **Reset** button on the XIAO board — a USB drive `XIAO-SENSE` appears
2. Copy the UF2 file:

```bash
cp -X build/zephyr/zephyr.uf2 /Volumes/XIAO-SENSE/
```

The board reboots automatically after flashing.

## Project Structure

```
├── CMakeLists.txt    # Zephyr CMake project
├── prj.conf          # Kconfig: GPIO, PWM, I2C, sensor, USB, FPU
├── app.overlay       # Device tree: PWM on D0/D1 via pwm0
└── src/
    └── main.c        # Application logic
```

## License

Apache-2.0

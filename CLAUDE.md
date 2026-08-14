# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project overview

A small 4WD robot built to uproot weeds from a gravel driveway. Built on an A4WD1 robot kit chassis.

## Hardware

- **Chassis:** A4WD1 4-wheel-drive robot kit
- **Wheels:** 4x 120mm
- **Motors:** 4x 12V gearmotors, 30:1 ratio, each with a quadrature encoder
- **Motor driver:** Sabertooth Dual 12A 12V motor controller (Microcontroller / Independent mode; also powers the electronics)
- **Battery:** 12V 2800mAh NiMH rechargeable
- **Microcontroller:** Arduino Uno
- **Communication:** HC-05 Bluetooth module (wireless telemetry/tuning and RC control)
- **Sensing:** HC-SR04 ultrasonic sensor (front bumper/obstacle detection; hard-stops the drivetrain below 10cm while driving forward, see `OBSTACLE_STOP_MM` in `src/main.cpp`)
- **End effector (in progress):** trailing tine rake, towed behind the chassis on a parallel-arm rigid-rod linkage (pivots at both the chassis and the strip, so rake angle stays constant as it rides over gravel). Tines are individually-sprung coil tines (~1.2mm wire, e.g. Bosch/Atco/Qualcast scarifier-cassette replacement tines, F016T47920-compatible) mounted along a rod spanning the chassis track width, spaced ~25–30mm apart. Not yet built/validated — currently at the test-strip prototype stage.

## Commands

Build/upload the main sketch:
```
pio run -e uno -t upload
```

Build/upload a staged bring-up sketch (standalone, isolated hardware tests in `src/stages/`):
```
pio run -e stage1_encoders_only -t upload
pio run -e stage2_open_loop_drive -t upload
pio run -e stage3_single_side_pid -t upload
pio run -e stage4_full_dual_pid -t upload
```

Open the serial monitor (same environment flag as the build):
```
pio device monitor
```

There is no `pio test` suite (`test/` is an empty PlatformIO placeholder) and no linter configured — verification is bench/hardware testing via serial monitor, not automated tests.

## Power notes

- Sabertooth unit powers both motors and onboard electronics directly from the 12V NiMH pack — no separate regulator/BEC needed for the Arduino/HC-05 unless current draw requires it.
- Sabertooth 2x12 mode: **Microcontroller / Independent mode** — DIP switches: 1, 4, 6 DOWN | 2, 3, 5 UP

## Wiring / pin mapping

```
Left  FRONT A -> pin 2   (INT0, hardware interrupt)
Left  FRONT B -> pin 4   (plain digital input)
Right FRONT A -> pin 3   (INT1, hardware interrupt)
Right FRONT B -> pin 5   (plain digital input)
Left  REAR  A -> pin 11  (PCINT, pin-change interrupt group)
Left  REAR  B -> pin 6   (plain digital input)
Right REAR  A -> pin 12  (PCINT, pin-change interrupt group)
Right REAR  B -> pin 7   (plain digital input)
Sabertooth S2 (left side)  -> pin 10 (PWM out from Arduino)
Sabertooth S1 (right side) -> pin 9  (PWM out from Arduino)
Arduino GND -> Sabertooth 0V
```

HC-05 Bluetooth (wireless monitoring/tuning from a paired PC or phone):
```
HC-05 VCC -> Uno 5V
HC-05 GND -> Uno GND
HC-05 TX  -> Uno pin 0 (RX) (direct - 3.3V out is fine into Uno RX)
HC-05 RX  -> Uno pin 1 (TX) (THROUGH a 1k/2k voltage divider - Uno's 5V
                              TX would over-drive the HC-05's 3.3V RX pin)
```

HC-SR04 ultrasonic (front bumper/obstacle sensor):
```
HC-SR04 VCC  -> Uno 5V
HC-SR04 GND  -> Uno GND
HC-SR04 TRIG -> Uno pin 8  (output, 10us trigger pulse)
HC-SR04 ECHO -> Uno pin A0 (input, pulseIn() measures echo width)
```
Sampled independently of the 20Hz motor control loop (see `SENSOR_PERIOD_MS`) — `pulseIn()` blocks for up to `ECHO_TIMEOUT_US`, and running it inside the 50ms control cycle every time would eat a meaningful chunk of that budget.

## Architecture

`src/main.cpp` is a single-file control loop; there's no multi-module structure to navigate. The essential things to know before changing it:

- Encoders use a mix of true hardware interrupts (pins 2/3) and manually-coded `ISR(PCINT0_vect)` pin-change interrupts (pins 11/12), since the Uno only has 2 hardware interrupt pins.
- **Serial (pins 0/1) is shared** between USB (uploads/monitor) and the HC-05 Bluetooth module, wired to hardware `Serial` (not `SoftwareSerial`, which conflicts with the rear-encoder PCINT ISR; not `AltSoftSerial`, which conflicts with the Servo library's Timer1 use). Only one of USB-monitor or Bluetooth can be connected at a time, and the HC-05 RX line should be disconnected during USB uploads.
- **`loop()` runs two independent cadences**: a 20 Hz (`CONTROL_PERIOD_MS`) PID control loop and a 10 Hz (`SENSOR_PERIOD_MS`) ultrasonic sample, kept apart because `pulseIn()` blocking would eat into the control budget.
- **Serial input is multiplexed** between three protocols read from the same stream: `P<value>`/`K<value>` for live Kp/Ki tuning, single-character RC commands (`handleRcChar()`, matching the "Bluetooth RC Controller" app's F/B/L/R/G/H/I/J/S/U/D protocol), and a fallback typed `"speed [turn]"` numeric format.
- **Why front+rear are read separately per side:** left-side motors (front+rear) are paralleled onto Sabertooth M1, right-side onto M2 — same PWM command per side. But the two motors on a side are mechanically independent: if one wheel loses traction it spins faster than its paired motor while the other stays grounded. Reading all 4 encoders lets the firmware (1) detect that speed mismatch (wheelspin/skid) per side, and (2) use the AVERAGE of front+rear speed on a side as PID feedback — more robust than either encoder alone.
- **PWM output is slew-rate limited** (`slewLimit()`) and has a deadband/dead-zone floor (`MIN_RELIABLE_PWM_OFFSET`, applied in `computePID()`) to work around this drivetrain's static-friction and rate-limited-actuator instability — see **Gotchas / stability fixes** below for why, and read the inline comments on `slewLimit()`/`computePID()` before changing tuning constants or control flow: several plausible-looking "improvements" (EMA filtering, jumping straight to target PWM on stop, looser slew limits) were tried and reverted for specific, documented reasons.
- **Traction control (`applySlipCorrection()`) is present but disabled by default** (`slipResponseEnabled = false`, `slipCorrectionGain = 0.0`) — it's a no-op until deliberately enabled after confirming slip thresholds via telemetry.
- **`src/stages/*.cpp`** are standalone, incrementally more complex sketches (encoders-only → open-loop drive → single-side PID → full dual PID) used for isolated hardware bring-up/debugging; each has its own `platformio.ini` environment and is not compiled into the main build (`build_src_filter` excludes `stages/` in `env:uno`). Consult them for the historical debugging process behind constants tuned in `main.cpp`.

## Gotchas / stability fixes

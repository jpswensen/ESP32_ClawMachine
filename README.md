# ESP32 Claw Machine Controller

An ESP32-based controller for a 3-axis arcade claw machine, built with PlatformIO and the Arduino framework. It drives three stepper motors (X/Y/Z), a servo-based claw, and reads inputs from a joystick plus an action button. The firmware handles homing, manual jogging, automatic grab-and-drop sequences, and an interactive calibration/debug mode with persistent settings stored in LittleFS.

## Features

- 3-axis motion control using `AccelStepper` (A4988-style drivers)
- Servo-based claw control using `ESP32Servo`
- Automatic homing sequence for X, Y, and Z axes
- Manual joystick control for positioning the claw
- Single action button to start an automated drop/grab/drop-off sequence
- Long-press action button to enter calibration/debug mode
- Safe "drop-zone keep-out" region where grabs are disabled
- Persistent configuration of:
  - Z drop depth (in steps)
  - Claw closed position (servo angle)
- Configuration stored in LittleFS and automatically created/validated on boot
- Cooperative multitasking using two FreeRTOS tasks (motion + control)

## Hardware Overview

This project targets an ESP32 DevKit board: `esp32doit-devkit-v1`.

Per `src/main.cpp`, the pinout is:

- **Stepper drivers (A4988-style, DRIVER mode)**
  - `X_STEP` = GPIO 33
  - `X_DIR`  = GPIO 32
  - `Y_STEP` = GPIO 13
  - `Y_DIR`  = GPIO 14
  - `Z_STEP` = GPIO 15 *(bootstrapping pin: ensure external circuitry does **not** pull it high at boot)*
  - `Z_DIR`  = GPIO 16

- **Limit switches** (active LOW, using internal pull-ups)
  - `LIM_X_MIN` = GPIO 27
  - `LIM_Y_MIN` = GPIO 26
  - `LIM_Z_MAX` = GPIO 25

- **Servo**
  - `SERVO_PIN` = GPIO 17 (claw open/close)

- **Joystick** (active LOW, digital direction switches)
  - `JS_UP`    = GPIO 19
  - `JS_DOWN`  = GPIO 23
  - `JS_LEFT`  = GPIO 21
  - `JS_RIGHT` = GPIO 22

- **Action button** (active LOW)
  - `BTN_ACTION` = GPIO 18

Double-check wiring and power requirements for your particular steppers, drivers, and servo.

## Firmware Behavior

### Boot & Homing

1. `setup()` initializes serial, I/O pins, the servo, and LittleFS.
2. Persistent configuration is loaded from `/claw_config.bin`:
   - If the file is missing or invalid, defaults are created and written.
3. A full homing cycle runs:
   - Z axis up to `LIM_Z_MAX`
   - X axis toward `LIM_X_MIN`
   - Y axis toward `LIM_Y_MIN`
4. After homing, axis positions are set to their respective home positions:
   - `X_HOME_POSITION`
   - `Y_HOME_POSITION`
   - `Z_HOME_POSITION`
5. Motion limits and accelerations are restored and the system enters **ManualControl**.

During motion, limit switches are polled frequently. When a limit is struck while moving toward it, the corresponding axis position is synced to its configured home position.

### Manual Control (Joystick)

In `ClawState::ManualControl`:

- Joystick inputs are read each control cycle.
- X/Y axes are commanded toward their min/max positions depending on direction:
  - Up/Down moves Y toward min/max.
  - Left/Right moves X toward min/max.
- When the joystick is released in an axis, that axis is held at its current position.
- A **short press** of the action button starts the automatic drop sequence **unless** the claw is within a keep-out radius of the drop zone.

### Automatic Drop Sequence

When a drop is started, the controller walks through `SequenceStage` states:

1. **PrepareDrop**
   - Open the claw.
   - Wait `SERVO_OPEN_BEFORE_DROP_MS`.
2. **MoveDown**
   - Move Z down to the configured `dropSteps` (from persisted config).
3. **CloseClaw**
   - Close the claw to the configured close angle.
   - Wait `SERVO_GRAB_SETTLE_MS`.
4. **Retract**
   - Move Z up past home by `Z_RETRACT_OVERDRIVE_STEPS`.
   - When `LIM_Z_MAX` is triggered, sync Z to `Z_HOME_POSITION`.
5. **MoveToDropZone**
   - Move X/Y to the configured drop zone coordinates (`DROP_X_STEPS`, `DROP_Y_STEPS`).
6. **ReleasePayload**
   - Open the claw and wait `SERVO_RELEASE_SETTLE_MS`.
   - Return to manual or debug mode, depending on `gDebugModeEnabled`.

### Debug / Calibration Mode

A **long press** of the action button enters `ClawState::DebugMode`. In this mode you can interactively calibrate:

- Z drop depth (`dropSteps`)
- Claw closed servo position (`clawClosePosition`)

Workflow:

1. On entry, the claw opens and the controller moves X/Y to the center of the playfield.
2. Once centered, it moves Z to the currently persisted drop depth and sets the servo to the current close angle.
3. In the `Calibrating` stage:
   - Joystick **Up/Down** adjusts Z drop depth in small step increments.
   - Joystick **Left/Right** adjusts the servo close angle by 1° steps within `[CLAW_OPEN_POSITION, CLAW_CLOSED_POSITION]`.
4. A short or long press of the action button while in debug mode:
   - Saves the new calibration values to `/claw_config.bin`.
   - Logs them to serial.
   - Returns the machine to home position.
   - Exits back to normal manual control.

The main `loop()` is suspended permanently; runtime logic is handled by two FreeRTOS tasks:

- `motionTask` (core 0): runs the steppers and monitors limits.
- `controlTask` (core 1): handles button/joystick input and state machines.

## Configuration & Constants

Key config values are in `Cfg` in `src/main.cpp`:

- Max speed and acceleration per axis (`X_MAX_SPEED`, `X_ACCEL`, etc.)
- Home and workspace bounds (`X_HOME_POSITION`, `Y_HOME_POSITION`, `Z_HOME_POSITION`, min/max)
- Drop zone coordinates and keep-out radius (`DROP_X_STEPS`, `DROP_Y_STEPS`, `DROP_KEEP_OUT_RADIUS_STEPS`)
- Timing constants for button debounce and long-press detection
- Timing for servo open/close settle delays

Servo positions are defined at the top of `main.cpp`:

- `CLAW_OPEN_POSITION` – open angle (in degrees)
- `CLAW_CLOSED_POSITION` – maximum closed angle

You can tune these to your hardware, but for normal operation you should use the debug mode to adjust and persist `dropSteps` and `clawClosePosition` instead of editing code.

## Dependencies

Managed through `platformio.ini`:

- `framework = arduino`
- `lib_deps`:
  - `ESP32Servo`
  - `AccelStepper`

The ESP32 core and LittleFS support are brought in by PlatformIO via `platform = espressif32`.

## Building and Uploading

This project is set up for PlatformIO. From the project root:

```bash
# Build
pio run

# Upload to the board
pio run --target upload

# Open serial monitor (115200 baud)
pio device monitor
```

Or use the PlatformIO extension in VS Code and run the default `env:esp32doit-devkit-v1` environment tasks.

## File System (LittleFS)

- Filesystem type: `littlefs`
- Partition table: `min_spiffs.csv` (configured in `platformio.ini`)
- Config file: `/claw_config.bin`
- On first boot (or if the file is corrupt), defaults are written.

Formatting is triggered automatically if the initial `LittleFS.begin(false)` mount fails.

## Debug Logging

`src/debug.h` provides simple debug macros gated by `DEBUG`:

- Set `#define DEBUG 1` to enable `D_print`, `D_println`, `D_printf` etc.
- Set it to `0` to compile out debug traces.

The core firmware also logs key state transitions and calibration values directly to `Serial`.

## Safety and Notes

- **GPIO15 (Z_STEP)** is a bootstrapping pin; ensure attached circuitry does not drive it HIGH on reset.
- Use appropriate power supplies for stepper motors and servo.
- Ensure limit switches are wired correctly and tested before enabling full-speed motion.
- The keep-out zone prevents accidental grabs near the prize drop chute; adjust `DROP_KEEP_OUT_RADIUS_STEPS` if needed.

## Repository Layout

- `platformio.ini` – PlatformIO project configuration, board/env settings, library deps.
- `src/main.cpp` – Main firmware, motion control, game logic, debug mode, persistence.
- `src/debug.h` – Debug print macros.
- `include/` – For additional headers (currently minimal).
- `lib/` – For custom libraries (currently empty placeholder).
- `test/` – Placeholder for PlatformIO unit/integration tests.

## License

See `LICENSE` for licensing details.

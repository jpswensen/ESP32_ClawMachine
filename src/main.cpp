#include <Arduino.h>
#include <AccelStepper.h>
#include <ESP32Servo.h>
#include <LittleFS.h>

/* ======================== SERVO CONFIG (your exact bounds) ======================== */
#define CLAW_OPEN_POSITION    (5 * 4)    // 20°
#define CLAW_CLOSED_POSITION  (5 * 23)   // 115°
static Servo servo1; // single gripper servo
/* ================================================================================= */

/* ======================== PIN MAP (servo on GPIO4) ================================ */
namespace Pins {
  // A4988 STEP/DIR
  // NOTE: GPIO15 is a boot strapping pin. Ensure external circuitry does NOT pull it HIGH at boot.
  constexpr int X_STEP = 33;
  constexpr int X_DIR  = 32;
  constexpr int Y_STEP = 13;
  constexpr int Y_DIR  = 14;
  constexpr int Z_STEP = 15; // strap pin: keep low at boot
  constexpr int Z_DIR  = 16;

  // Limit switches
  constexpr int LIM_X_MIN = 27;
  constexpr int LIM_Y_MIN = 26;
  constexpr int LIM_Z_MAX = 25;

  // Servo
  constexpr int SERVO_PIN = 17;

  // Joystick (active LOW)
  constexpr int JS_UP    = 19;
  constexpr int JS_DOWN  = 23;
  constexpr int JS_LEFT  = 21;
  constexpr int JS_RIGHT = 22;

  // Action button (active LOW)
  constexpr int BTN_ACTION = 18;
}
/* ================================================================================= */

/* ===================================== CONFIG ==================================== */
namespace Cfg {
  // Motion
  constexpr float HOMING_SCALE_FACTOR = 0.5f;
  constexpr float X_MAX_SPEED = 2.0f * 4800.0f;
  constexpr float X_ACCEL = 1.5f * 9600.0f;
  constexpr float Y_MAX_SPEED = 2.0f * 4800.0f;
  constexpr float Y_ACCEL = 1.5f * 9600.0f;
  constexpr float Z_MAX_SPEED = 32.0f * 75.0f;
  constexpr float Z_ACCEL = 16.0f * 375.0f;

  // Workspace (steps)
  constexpr long X_HOME_POSITION = 52500L;
  constexpr long Y_HOME_POSITION = 45000L;
  constexpr long Z_HOME_POSITION = 16 * 500L;

  constexpr long X_MIN_POSITION = 0L;
  constexpr long X_MAX_POSITION = X_HOME_POSITION;
  constexpr long Y_MIN_POSITION = 0L;
  constexpr long Y_MAX_POSITION = Y_HOME_POSITION;
  constexpr long Z_MIN_POSITION = 0L;
  constexpr long Z_MAX_POSITION = Z_HOME_POSITION;
  constexpr long Z_RETRACT_OVERDRIVE_STEPS = 2000L;
  constexpr long Z_HOME_OVERRUN_POSITION = Z_HOME_POSITION + Z_RETRACT_OVERDRIVE_STEPS;

  // Game sequencing
  constexpr long Z_DROP_POSITION = Z_MIN_POSITION; // fully lowered
  constexpr uint32_t SERVO_GRAB_SETTLE_MS = 450;
  constexpr uint32_t SERVO_RELEASE_SETTLE_MS = 350;
  constexpr uint32_t SERVO_OPEN_BEFORE_DROP_MS = 250;
  constexpr uint32_t BUTTON_DEBOUNCE_MS = 30;
  constexpr uint32_t BUTTON_LONG_PRESS_MS = 1500;
  constexpr uint32_t LIMIT_POLL_INTERVAL_MS = 10;

  // Drop zone
  constexpr long DROP_X_STEPS = X_HOME_POSITION;
  constexpr long DROP_Y_STEPS = Y_HOME_POSITION;
  constexpr long DROP_KEEP_OUT_RADIUS_STEPS = 17500L;
  constexpr long DROP_KEEP_OUT_RADIUS_STEPS_SQ = DROP_KEEP_OUT_RADIUS_STEPS * DROP_KEEP_OUT_RADIUS_STEPS;
}
/* ================================================================================= */

namespace PersistentCfg {
  constexpr const char* FILE_PATH = "/claw_config.bin";

  struct Data {
    long dropSteps;
    int clawClosePosition;
  };

  inline Data makeDefaults() {
    return Data{Cfg::Z_DROP_POSITION, CLAW_CLOSED_POSITION};
  }

  static Data current = makeDefaults();

  void clamp(Data& data) {
    data.dropSteps = constrain(data.dropSteps, Cfg::Z_MIN_POSITION, Cfg::Z_MAX_POSITION);
    data.clawClosePosition = constrain(data.clawClosePosition, 0, 180);
  }

  bool writeToFile(const Data& data) {
    File f = LittleFS.open(FILE_PATH, FILE_WRITE);
    if (!f) {
      return false;
    }
    size_t written = f.write(reinterpret_cast<const uint8_t*>(&data), sizeof(Data));
    f.close();
    return written == sizeof(Data);
  }

  bool readFromFile(Data& data) {
    File f = LittleFS.open(FILE_PATH, FILE_READ);
    if (!f) {
      return false;
    }
    if (f.size() != sizeof(Data)) {
      f.close();
      return false;
    }
    size_t readBytes = f.readBytes(reinterpret_cast<char*>(&data), sizeof(Data));
    f.close();
    return readBytes == sizeof(Data);
  }

  void ensureFileExists() {
    if (!LittleFS.exists(FILE_PATH)) {
      Data defaults = makeDefaults();
      clamp(defaults);
      writeToFile(defaults);
    }
  }

  void init() {
    if (!LittleFS.begin(false)) {
      Serial.println(F("LittleFS mount failed; formatting..."));
      if (!LittleFS.begin(true)) {
        Serial.println(F("LittleFS unavailable, using defaults in RAM."));
        current = makeDefaults();
        return;
      }
    }

    ensureFileExists();

    Data loaded;
    if (readFromFile(loaded)) {
      Serial.printf("Loaded from file: dropSteps=%ld, clawClosePos=%d\n", loaded.dropSteps, loaded.clawClosePosition);
      clamp(loaded);
      current = loaded;
      Serial.println(F("Config loaded successfully."));
    } else {
      Serial.println(F("LittleFS config invalid; rewriting defaults."));
      Data defaults = makeDefaults();
      Serial.printf("Creating defaults: dropSteps=%ld, clawClosePos=%d\n", defaults.dropSteps, defaults.clawClosePosition);
      clamp(defaults);
      if (writeToFile(defaults)) {
        current = defaults;
      }
    }
  }

  long dropSteps() {
    return current.dropSteps;
  }

  int clawClosePosition() {
    return current.clawClosePosition;
  }

  void logCurrent() {
    Serial.printf("Persisted drop steps: %ld\n", static_cast<long>(current.dropSteps));
    Serial.printf("Persisted claw close pos: %d\n", current.clawClosePosition);
  }

  bool saveCalibration(long dropSteps, int clawClosePos) {
    Data newData;
    newData.dropSteps = dropSteps;
    newData.clawClosePosition = clawClosePos;
    Serial.printf("Saving calibration: dropSteps=%ld, clawClosePos=%d\n", dropSteps, clawClosePos);
    clamp(newData);
    Serial.printf("After clamp: dropSteps=%ld, clawClosePos=%d\n", newData.dropSteps, newData.clawClosePosition);
    if (writeToFile(newData)) {
      current = newData;
      Serial.println(F("Calibration saved to flash."));
      return true;
    }
    Serial.println(F("Failed to save calibration."));
    return false;
  }
}

AccelStepper stepX(AccelStepper::DRIVER, Pins::X_STEP, Pins::X_DIR);
AccelStepper stepY(AccelStepper::DRIVER, Pins::Y_STEP, Pins::Y_DIR);
AccelStepper stepZ(AccelStepper::DRIVER, Pins::Z_STEP, Pins::Z_DIR);

inline bool isLow(int pin) { return digitalRead(pin) == LOW; }

inline bool limitActive(int pin) {
  uint8_t lowCount = 0;
  constexpr int numSamples = 25;
  for (int i = 0; i < numSamples; ++i) {
    lowCount += (digitalRead(pin) == LOW);
  }
  return lowCount >= (numSamples / 2);
}

/* =========================== Runtime coordination structures ====================== */
enum class ClawState : uint8_t {
  Homing,
  ManualControl,
  AutoSequence,
  DebugMode,
};

enum class SequenceStage : uint8_t {
  Idle,
  PrepareDrop,
  MoveDown,
  CloseClaw,
  Retract,
  MoveToDropZone,
  ReleasePayload,
};

enum class DebugStage : uint8_t {
  MoveToCenter,
  Calibrating,
};

enum class ButtonEvent : uint8_t {
  None,
  ShortPress,
  LongPress,
};

struct SequenceContext {
  SequenceStage stage = SequenceStage::Idle;
  uint32_t stageStartMs = 0;
  bool stagePrimed = false;
};

struct JoystickInput {
  bool up = false;
  bool down = false;
  bool left = false;
  bool right = false;
};

static portMUX_TYPE gStepperMux = portMUX_INITIALIZER_UNLOCKED;
static TaskHandle_t gMotionTaskHandle = nullptr;
static TaskHandle_t gControlTaskHandle = nullptr;
static SequenceContext gSequence;
static volatile ClawState gState = ClawState::Homing;
static bool gDebugModeEnabled = false;

// Debug calibration state
static DebugStage gDebugStage = DebugStage::MoveToCenter;
static long gDebugDropSteps = 0;
static int gDebugClawClosePos = 0;

// Button tracking
static uint32_t gButtonPressStartMs = 0;
static bool gButtonHeld = false;
static bool gLongPressSent = false;

// Function declarations
void home_axes();
void motionTask(void* args);
void controlTask(void* args);
void handleManualJog(const JoystickInput& js);
JoystickInput readJoystick();
ButtonEvent pollActionButton();
void startDropSequence();
void processDropSequence();
void processManualControls(ButtonEvent evt);
void processDebugControls(ButtonEvent evt);
void enterDebugMode();
void exitDebugMode();
bool axisAtTarget(AccelStepper& axis, long tolerance = 10L);
void monitorLimitSwitches();
void setSequenceStage(SequenceStage stage);
void commandAxis(AccelStepper& axis, long target, float maxSpeed, long minPos, long maxPos);
void holdAxis(AccelStepper& axis);
void syncAxisAtLimit(AccelStepper& axis, long homePosition);
void openClaw();
void closeClaw();
bool isWithinDropKeepOut();

/* ================================ Helpers =================================== */
void openClaw() { servo1.write(CLAW_OPEN_POSITION); }

void closeClaw() { servo1.write(PersistentCfg::clawClosePosition()); }

void commandAxis(AccelStepper& axis, long target, float maxSpeed, long minPos, long maxPos) {
  const long bounded = constrain(target, minPos, maxPos);
  taskENTER_CRITICAL(&gStepperMux);
  axis.setMaxSpeed(maxSpeed);
  axis.moveTo(bounded);
  taskEXIT_CRITICAL(&gStepperMux);
}

void holdAxis(AccelStepper& axis) {
  taskENTER_CRITICAL(&gStepperMux);
  const long pos = axis.currentPosition();
  axis.moveTo(pos);
  taskEXIT_CRITICAL(&gStepperMux);
}

bool axisAtTarget(AccelStepper& axis, long tolerance) {
  taskENTER_CRITICAL(&gStepperMux);
  const long distance = axis.distanceToGo();
  taskEXIT_CRITICAL(&gStepperMux);
  return llabs(distance) <= tolerance;
}

bool isWithinDropKeepOut() {
  long xPos;
  long yPos;
  taskENTER_CRITICAL(&gStepperMux);
  xPos = stepX.currentPosition();
  yPos = stepY.currentPosition();
  taskEXIT_CRITICAL(&gStepperMux);

  const long dx = xPos - Cfg::DROP_X_STEPS;
  const long dy = yPos - Cfg::DROP_Y_STEPS;
  const long long distSq = static_cast<long long>(dx) * static_cast<long long>(dx) +
                           static_cast<long long>(dy) * static_cast<long long>(dy);
  return distSq <= static_cast<long long>(Cfg::DROP_KEEP_OUT_RADIUS_STEPS_SQ);
}

JoystickInput readJoystick() {
  JoystickInput js;
  js.up = isLow(Pins::JS_UP);
  js.down = isLow(Pins::JS_DOWN);
  js.left = isLow(Pins::JS_LEFT);
  js.right = isLow(Pins::JS_RIGHT);
  return js;
}

ButtonEvent pollActionButton() {
  const bool levelLow = isLow(Pins::BTN_ACTION);
  const uint32_t now = millis();

  if (levelLow) {
    if (!gButtonHeld) {
      gButtonHeld = true;
      gButtonPressStartMs = now;
      gLongPressSent = false;
    } else if (!gLongPressSent && (now - gButtonPressStartMs) >= Cfg::BUTTON_LONG_PRESS_MS) {
      gLongPressSent = true;
      return ButtonEvent::LongPress;
    }
  } else if (gButtonHeld) {
    ButtonEvent evt = ButtonEvent::None;
    const uint32_t heldMs = now - gButtonPressStartMs;
    if (!gLongPressSent && heldMs >= Cfg::BUTTON_DEBOUNCE_MS) {
      evt = ButtonEvent::ShortPress;
    }
    gButtonHeld = false;
    gLongPressSent = false;
    return evt;
  }

  return ButtonEvent::None;
}

void setSequenceStage(SequenceStage stage) {
  gSequence.stage = stage;
  gSequence.stageStartMs = millis();
  gSequence.stagePrimed = false;
}

void handleManualJog(const JoystickInput& js) {
  if (js.up && !js.down) {
    commandAxis(stepY, Cfg::Y_MIN_POSITION, Cfg::Y_MAX_SPEED, Cfg::Y_MIN_POSITION, Cfg::Y_MAX_POSITION);
  } else if (js.down && !js.up) {
    commandAxis(stepY, Cfg::Y_MAX_POSITION, Cfg::Y_MAX_SPEED, Cfg::Y_MIN_POSITION, Cfg::Y_MAX_POSITION);
  } else {
    holdAxis(stepY);
  }

  if (js.left && !js.right) {
    commandAxis(stepX, Cfg::X_MIN_POSITION, Cfg::X_MAX_SPEED, Cfg::X_MIN_POSITION, Cfg::X_MAX_POSITION);
  } else if (js.right && !js.left) {
    commandAxis(stepX, Cfg::X_MAX_POSITION, Cfg::X_MAX_SPEED, Cfg::X_MIN_POSITION, Cfg::X_MAX_POSITION);
  } else {
    holdAxis(stepX);
  }
}

void syncAxisAtLimit(AccelStepper& axis, long homePosition) {
  taskENTER_CRITICAL(&gStepperMux);
  axis.setCurrentPosition(homePosition);
  // Don't modify target - let it continue to its intended destination
  taskEXIT_CRITICAL(&gStepperMux);
}

void monitorLimitSwitches() {
  static uint32_t lastPoll = 0;
  static bool zLimitWasActive = false;
  static bool xLimitWasActive = false;
  static bool yLimitWasActive = false;
  
  const uint32_t now = millis();
  if ((now - lastPoll) < Cfg::LIMIT_POLL_INTERVAL_MS) {
    return;
  }
  lastPoll = now;

  bool zLimitActive = limitActive(Pins::LIM_Z_MAX);
  bool xLimitActive = limitActive(Pins::LIM_X_MIN);
  bool yLimitActive = limitActive(Pins::LIM_Y_MIN);

  long distZ;
  long distX;
  long distY;
  taskENTER_CRITICAL(&gStepperMux);
  distZ = stepZ.distanceToGo();
  distX = stepX.distanceToGo();
  distY = stepY.distanceToGo();
  taskEXIT_CRITICAL(&gStepperMux);

  // Only sync on rising edge (when limit becomes active) and moving toward it
  if (zLimitActive && !zLimitWasActive && distZ > 0) {
    syncAxisAtLimit(stepZ, Cfg::Z_HOME_POSITION);
  }
  if (xLimitActive && !xLimitWasActive && distX < 0) {
    syncAxisAtLimit(stepX, Cfg::X_HOME_POSITION);
  }
  if (yLimitActive && !yLimitWasActive && distY < 0) {
    syncAxisAtLimit(stepY, Cfg::Y_HOME_POSITION);
  }

  zLimitWasActive = zLimitActive;
  xLimitWasActive = xLimitActive;
  yLimitWasActive = yLimitActive;
}

void startDropSequence() {
  if (gSequence.stage != SequenceStage::Idle) {
    return; // already running
  }
  Serial.println(F("Starting drop sequence."));
  setSequenceStage(SequenceStage::PrepareDrop);
  gState = ClawState::AutoSequence;
}

void processDropSequence() {
  switch (gSequence.stage) {
    case SequenceStage::PrepareDrop:
      if (!gSequence.stagePrimed) {
        openClaw();
        gSequence.stagePrimed = true;
      }
      if ((millis() - gSequence.stageStartMs) >= Cfg::SERVO_OPEN_BEFORE_DROP_MS) {
        setSequenceStage(SequenceStage::MoveDown);
      }
      break;

    case SequenceStage::MoveDown:
      if (!gSequence.stagePrimed) {
        openClaw();
        commandAxis(stepZ, PersistentCfg::dropSteps(), Cfg::Z_MAX_SPEED, Cfg::Z_MIN_POSITION, Cfg::Z_MAX_POSITION);
        gSequence.stagePrimed = true;
      }
      if (axisAtTarget(stepZ)) {
        setSequenceStage(SequenceStage::CloseClaw);
      }
      break;

    case SequenceStage::CloseClaw:
      if (!gSequence.stagePrimed) {
        closeClaw();
        gSequence.stagePrimed = true;
      }
      if ((millis() - gSequence.stageStartMs) >= Cfg::SERVO_GRAB_SETTLE_MS) {
        setSequenceStage(SequenceStage::Retract);
      }
      break;

    case SequenceStage::Retract:
      if (!gSequence.stagePrimed) {
        const long retractTarget = Cfg::Z_HOME_POSITION + Cfg::Z_RETRACT_OVERDRIVE_STEPS;
        commandAxis(stepZ, retractTarget, Cfg::Z_MAX_SPEED, Cfg::Z_MIN_POSITION, Cfg::Z_HOME_OVERRUN_POSITION);
        gSequence.stagePrimed = true;
      }
      if (limitActive(Pins::LIM_Z_MAX)) {
        syncAxisAtLimit(stepZ, Cfg::Z_HOME_POSITION);
        setSequenceStage(SequenceStage::MoveToDropZone);
      }
      break;

    case SequenceStage::MoveToDropZone:
      if (!gSequence.stagePrimed) {
        commandAxis(stepX, Cfg::DROP_X_STEPS, Cfg::X_MAX_SPEED, Cfg::X_MIN_POSITION, Cfg::X_MAX_POSITION);
        commandAxis(stepY, Cfg::DROP_Y_STEPS, Cfg::Y_MAX_SPEED, Cfg::Y_MIN_POSITION, Cfg::Y_MAX_POSITION);
        gSequence.stagePrimed = true;
      }
      if (axisAtTarget(stepX) && axisAtTarget(stepY)) {
        setSequenceStage(SequenceStage::ReleasePayload);
      }
      break;

    case SequenceStage::ReleasePayload:
      if (!gSequence.stagePrimed) {
        openClaw();
        gSequence.stagePrimed = true;
      }
      if ((millis() - gSequence.stageStartMs) >= Cfg::SERVO_RELEASE_SETTLE_MS) {
        Serial.println(F("Sequence complete; returning to manual control."));
        setSequenceStage(SequenceStage::Idle);
        gState = gDebugModeEnabled ? ClawState::DebugMode : ClawState::ManualControl;
      }
      break;

    case SequenceStage::Idle:
    default:
      gState = gDebugModeEnabled ? ClawState::DebugMode : ClawState::ManualControl;
      break;
  }
}

void processManualControls(ButtonEvent evt) {
  handleManualJog(readJoystick());
  if (evt == ButtonEvent::ShortPress) {
    if (isWithinDropKeepOut()) {
      Serial.println(F("Action button disabled near drop zone."));
      int currentX = stepX.currentPosition();
      int currentY = stepY.currentPosition();
      // Serial.printf("Current position X: %d, Y: %d\n", currentX, currentY);
      // Serial.printf("Drop zone X: %d, Y: %d, Keep-out radius (steps): %d\n",
      //               Cfg::DROP_X_STEPS, Cfg::DROP_Y_STEPS, Cfg::DROP_KEEP_OUT_RADIUS_STEPS);
      // int dx = currentX - Cfg::DROP_X_STEPS;
      // int dy = currentY - Cfg::DROP_Y_STEPS;
      // long long distSq = static_cast<long long>(dx) * static_cast<long long>(dx) +
      //                    static_cast<long long>(dy) * static_cast<long long>(dy);
      // Serial.printf("Distance squared to drop zone center: %lld (keep-out radius squared: %lld)\n",
      //               distSq, static_cast<long long>(Cfg::DROP_KEEP_OUT_RADIUS_STEPS_SQ));
    } else {
      startDropSequence();
    }
  }
}

void enterDebugMode() {
  Serial.println(F("Entering debug calibration mode..."));
  gDebugStage = DebugStage::MoveToCenter;
  gDebugDropSteps = PersistentCfg::dropSteps();
  gDebugClawClosePos = PersistentCfg::clawClosePosition();
  openClaw();
  const long centerX = Cfg::X_HOME_POSITION / 2;
  const long centerY = Cfg::Y_HOME_POSITION / 2;
  
  long currentX, currentY;
  taskENTER_CRITICAL(&gStepperMux);
  currentX = stepX.currentPosition();
  currentY = stepY.currentPosition();
  taskEXIT_CRITICAL(&gStepperMux);
  
  Serial.printf("Current position: X=%ld, Y=%ld\n", currentX, currentY);
  Serial.printf("Moving to center: X=%ld, Y=%ld\n", centerX, centerY);
  commandAxis(stepX, centerX, Cfg::X_MAX_SPEED, Cfg::X_MIN_POSITION, Cfg::X_MAX_POSITION);
  commandAxis(stepY, centerY, Cfg::Y_MAX_SPEED, Cfg::Y_MIN_POSITION, Cfg::Y_MAX_POSITION);
}

void exitDebugMode() {
  Serial.println(F("Exiting debug mode; saving calibration..."));
  Serial.printf("Debug values: dropSteps=%ld, clawClosePos=%d\n", gDebugDropSteps, gDebugClawClosePos);
  PersistentCfg::saveCalibration(gDebugDropSteps, gDebugClawClosePos);
  PersistentCfg::logCurrent();
  openClaw();
  
  // Return to home position
  Serial.println(F("Returning to home position..."));
  commandAxis(stepX, Cfg::X_HOME_POSITION, Cfg::X_MAX_SPEED, Cfg::X_MIN_POSITION, Cfg::X_MAX_POSITION);
  commandAxis(stepY, Cfg::Y_HOME_POSITION, Cfg::Y_MAX_SPEED, Cfg::Y_MIN_POSITION, Cfg::Y_MAX_POSITION);
  commandAxis(stepZ, Cfg::Z_HOME_POSITION, Cfg::Z_MAX_SPEED, Cfg::Z_MIN_POSITION, Cfg::Z_MAX_POSITION);
  
  gDebugModeEnabled = false;
  gState = ClawState::ManualControl;
}

void processDebugControls(ButtonEvent evt) {
  if (evt == ButtonEvent::ShortPress || evt == ButtonEvent::LongPress) {
    exitDebugMode();
    return;
  }

  switch (gDebugStage) {
    case DebugStage::MoveToCenter:
      if (axisAtTarget(stepX, 50) && axisAtTarget(stepY, 50)) {
        Serial.println(F("Centered. Entering calibration stage."));
        commandAxis(stepZ, gDebugDropSteps, Cfg::Z_MAX_SPEED, Cfg::Z_MIN_POSITION, Cfg::Z_MAX_POSITION);
        servo1.write(gDebugClawClosePos);
        gDebugStage = DebugStage::Calibrating;
      }
      break;

    case DebugStage::Calibrating: {
      static uint32_t lastServoUpdate = 0;
      const uint32_t servoUpdateInterval = 50; // ms between servo updates
      
      JoystickInput js = readJoystick();
      uint32_t now = millis();
      
      // Up/down jogs Z axis position
      if (js.up && !js.down) {
        long currentZ;
        taskENTER_CRITICAL(&gStepperMux);
        currentZ = stepZ.currentPosition();
        taskEXIT_CRITICAL(&gStepperMux);
        long newZ = constrain(currentZ - 10, Cfg::Z_MIN_POSITION, Cfg::Z_MAX_POSITION);
        commandAxis(stepZ, newZ, Cfg::Z_MAX_SPEED, Cfg::Z_MIN_POSITION, Cfg::Z_MAX_POSITION);
        gDebugDropSteps = newZ;
      } else if (js.down && !js.up) {
        long currentZ;
        taskENTER_CRITICAL(&gStepperMux);
        currentZ = stepZ.currentPosition();
        taskEXIT_CRITICAL(&gStepperMux);
        long newZ = constrain(currentZ + 10, Cfg::Z_MIN_POSITION, Cfg::Z_MAX_POSITION);
        commandAxis(stepZ, newZ, Cfg::Z_MAX_SPEED, Cfg::Z_MIN_POSITION, Cfg::Z_MAX_POSITION);
        gDebugDropSteps = newZ;
      }

      // Left/right jogs claw open/close position (throttled for smooth movement)
      if ((now - lastServoUpdate) >= servoUpdateInterval) {
        if (js.left && !js.right) {
          gDebugClawClosePos = constrain(gDebugClawClosePos - 1, CLAW_OPEN_POSITION, CLAW_CLOSED_POSITION);
          servo1.write(gDebugClawClosePos);
          lastServoUpdate = now;
        } else if (js.right && !js.left) {
          gDebugClawClosePos = constrain(gDebugClawClosePos + 1, CLAW_OPEN_POSITION, CLAW_CLOSED_POSITION);
          servo1.write(gDebugClawClosePos);
          lastServoUpdate = now;
        }
      }
      break;
    }
  }
}

/* ================================ Homing =================================== */
void home_axes() {
  Serial.println(F("Starting Z Homing..."));
  stepZ.setMaxSpeed(Cfg::Z_MAX_SPEED * Cfg::HOMING_SCALE_FACTOR);
  stepZ.setAcceleration(Cfg::Z_ACCEL);
  stepZ.moveTo(stepZ.currentPosition() + 1 * 200000000L);
  while (!limitActive(Pins::LIM_Z_MAX)) {
    stepZ.run();
    taskYIELD();
  }
  Serial.println(F("Z Homing complete."));
  stepZ.setCurrentPosition(Cfg::Z_HOME_POSITION);
  stepZ.moveTo(stepZ.currentPosition());

  Serial.println(F("Starting X Homing..."));
  stepX.setMaxSpeed(Cfg::X_MAX_SPEED * Cfg::HOMING_SCALE_FACTOR);
  stepX.setAcceleration(Cfg::X_ACCEL);
  stepX.moveTo(stepX.currentPosition() + 1 * 200000000L);
  while (!limitActive(Pins::LIM_X_MIN)) {
    stepX.run();
    taskYIELD();
  }
  Serial.println(F("X Homing complete."));
  stepX.setCurrentPosition(Cfg::X_HOME_POSITION);
  stepX.moveTo(stepX.currentPosition());

  Serial.println(F("Starting Y Homing..."));
  stepY.setMaxSpeed(Cfg::Y_MAX_SPEED * Cfg::HOMING_SCALE_FACTOR);
  stepY.setAcceleration(Cfg::Y_ACCEL);
  stepY.moveTo(stepY.currentPosition() + 1 * 200000000L);
  while (!limitActive(Pins::LIM_Y_MIN)) {
    stepY.run();
    taskYIELD();
  }
  Serial.println(F("Y Homing complete."));
  stepY.setCurrentPosition(Cfg::Y_HOME_POSITION);
  stepY.moveTo(stepY.currentPosition());
}

/* ================================ Tasks =================================== */
void motionTask(void* args) {
  (void)args;
  disableCore0WDT();  // Disable watchdog for motion task
  for (;;) {
    taskENTER_CRITICAL(&gStepperMux);
    stepX.run();
    stepY.run();
    stepZ.run();
    taskEXIT_CRITICAL(&gStepperMux);
    monitorLimitSwitches();
    taskYIELD();  // Yield to other tasks without delay
  }
}

void controlTask(void* args) {
  (void)args;
  for (;;) {
    ButtonEvent evt = pollActionButton();
    if (evt == ButtonEvent::LongPress && !gDebugModeEnabled) {
      gDebugModeEnabled = true;
      gState = ClawState::DebugMode;
      enterDebugMode();
      Serial.println("Debug mode enabled");
      evt = ButtonEvent::None;
    }

    switch (gState) {
      case ClawState::ManualControl:
        processManualControls(evt);
        break;
      case ClawState::AutoSequence:
        processDropSequence();
        break;
      case ClawState::DebugMode:
        processDebugControls(evt);
        break;
      case ClawState::Homing:
      default:
        break;
    }

    taskYIELD();  // Yield to other tasks without delay
  }
}

void setup() {
  Serial.begin(115200);

  stepY.setPinsInverted(false, false, false); // DIR is inverted

  pinMode(Pins::LIM_Z_MAX, INPUT_PULLUP);
  pinMode(Pins::LIM_X_MIN, INPUT_PULLUP);
  pinMode(Pins::LIM_Y_MIN, INPUT_PULLUP);

  pinMode(Pins::JS_UP, INPUT_PULLUP);
  pinMode(Pins::JS_DOWN, INPUT_PULLUP);
  pinMode(Pins::JS_LEFT, INPUT_PULLUP);
  pinMode(Pins::JS_RIGHT, INPUT_PULLUP);
  pinMode(Pins::BTN_ACTION, INPUT_PULLUP);

  ESP32PWM::allocateTimer(3);
  servo1.setPeriodHertz(50);
  servo1.attach(Pins::SERVO_PIN, 500, 2400);
  openClaw();

  PersistentCfg::init();
  PersistentCfg::logCurrent();

  home_axes();

  // Restore run-time motion limits
  stepX.setMaxSpeed(Cfg::X_MAX_SPEED);
  stepX.setAcceleration(Cfg::X_ACCEL);
  stepY.setMaxSpeed(Cfg::Y_MAX_SPEED);
  stepY.setAcceleration(Cfg::Y_ACCEL);
  stepZ.setMaxSpeed(Cfg::Z_MAX_SPEED);
  stepZ.setAcceleration(Cfg::Z_ACCEL);

  gState = ClawState::ManualControl;

  xTaskCreatePinnedToCore(motionTask, "MotionTask", 4096, nullptr, 3, &gMotionTaskHandle, 0);
  xTaskCreatePinnedToCore(controlTask, "ControlTask", 6144, nullptr, 2, &gControlTaskHandle, 1);
}

void loop() {
  vTaskSuspend(NULL);  // Suspend loop task permanently
}

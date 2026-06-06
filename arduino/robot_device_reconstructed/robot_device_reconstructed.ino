// =============================================================
//  robot_device_reconstructed.ino
//
//  Reconstructed firmware for the plant-imaging camera gantry.
//
//  WHY THIS FILE EXISTS:
//  The firmware actually running on the robot was lost (it was
//  never committed, and the dev machine died). This is a rebuild
//  from the closest surviving ancestor -- robot_device_testing.ino
//  -- with the camera / day-night / interval features layered back
//  on, inferred from the live serial output:
//
//    Camera gantry starting...
//    Day/night cycle: 15 hours day, 9 hours night
//    Cycle interval: 15 minutes
//    Will photograph 3 shelves with 8 photos each
//
//  CONFIDENCE KEY in the comments below:
//    [KNOWN]   recovered from the scaffold, serial output, or user
//    [TUNE]    grounded in the scaffold but MUST be checked on the rig
//    [CONFIRM] a genuine unknown -- verify against the wiring first
//
//  DO NOT trust an unattended run until the [TUNE]/[CONFIRM] items
//  have been checked on the hardware.
// =============================================================

// ---------------- Run configuration --------------------------
const int numShelves     = 3;     // [KNOWN] from serial: "3 shelves"
const int photosPerShelf = 8;     // [KNOWN] from serial: "8 photos each"
const int dayHours       = 15;    // [KNOWN] from serial: "15 hours day"
const int nightHours     = 24 - dayHours;            // 9 hours night
const unsigned long cycleIntervalMs = 15UL * 60UL * 1000UL; // [KNOWN] 15 min

// ---------------- Pin map (from robot_device_testing.ino) -----
// [KNOWN] These match the scaffold the live firmware descends from.
const int horizSensor = 3;   // D3 photointerrupter
const int vertSensor  = 2;   // D2 photointerrupter

const int stepPinX = 5;      // horizontal motor
const int dirPinX  = 7;
const int enblPinX = 6;

const int stepPinY = 8;      // vertical motor
const int dirPinY  = 10;
const int enblPinY = 9;

const int cameraPin = 12;    // [KNOWN] user-confirmed: camera trigger on GND + D12

// [CONFIRM] Shelf-lighting relay pins. These are the OLD robot_device.ino
// values (Arduino Mega). If this board is an Uno, or the relays were
// rewired, THESE ARE WRONG. Index 0 = shelf 1. Active-LOW assumed.
// Set ENABLE_SHELF_LIGHTS = false to disable lighting entirely until
// you have confirmed these pins.
const bool ENABLE_SHELF_LIGHTS = true;
const int  shelfRelayPins[]    = {23, 25, 27, 29, 31, 33};
const int  RELAY_ON  = LOW;    // [CONFIRM] LOW = light on (active-low relay board)
const int  RELAY_OFF = HIGH;

// ---------------- Direction constants (from scaffold) --------
const boolean right = LOW;    // [KNOWN]
const boolean left  = HIGH;
const boolean up    = LOW;
const boolean down  = HIGH;

// ---------------- Timing -------------------------------------
const unsigned long STEP_DELAY_US    = 100;   // [KNOWN] step half-period (scaffold)
const unsigned long CAMERA_PULSE_US  = 100;   // [CONFIRM] trigger pulse width
const unsigned long LIGHT_SETTLE_MS  = 3000;  // [TUNE] dwell after light-on before shot
const unsigned long POST_SHOT_MS     = 500;   // [TUNE] dwell after shot (camera save)

// Travel between stops. Derived from the scaffold: it swept a shelf in
// ~10 s, so 10000ms / (8 photos - 1 gap) ~= 1400ms per box. Vertical
// matches the scaffold's 8000ms shelf-to-shelf move. [TUNE] so the
// camera actually lands on each box.
const unsigned long MOVE_PER_BOX_MS   = 1400; // [TUNE]
const unsigned long MOVE_PER_SHELF_MS = 8000; // [TUNE]

// After homing, the camera sits on the photointerrupters, which may not
// be box #1. [CONFIRM] offsets to jog to the first box. 0 = no offset.
const unsigned long START_OFFSET_X_MS = 0;    // [CONFIRM]
const unsigned long START_OFFSET_Y_MS = 0;    // [CONFIRM]

// ---------------- State --------------------------------------
bool  calibrated      = false;
float dayElapsedHours = 0;    // position within the 24h light cycle

void setup() {
  Serial.begin(9600);

  // Motors
  pinMode(stepPinX, OUTPUT); pinMode(dirPinX, OUTPUT); pinMode(enblPinX, OUTPUT);
  pinMode(stepPinY, OUTPUT); pinMode(dirPinY, OUTPUT); pinMode(enblPinY, OUTPUT);
  digitalWrite(stepPinX, LOW);   digitalWrite(stepPinY, LOW);
  digitalWrite(dirPinX, right);  digitalWrite(dirPinY, up);
  digitalWrite(enblPinX, LOW);   digitalWrite(enblPinY, LOW);  // LOW = enabled

  // Sensors
  pinMode(horizSensor, INPUT_PULLUP);
  pinMode(vertSensor,  INPUT_PULLUP);

  // Camera
  pinMode(cameraPin, OUTPUT);
  digitalWrite(cameraPin, LOW);

  // Shelf relays
  if (ENABLE_SHELF_LIGHTS) {
    for (int i = 0; i < numShelves; i++) {
      pinMode(shelfRelayPins[i], OUTPUT);
      digitalWrite(shelfRelayPins[i], RELAY_OFF);
    }
  }

  // Banner -- reproduces the observed serial output
  Serial.println("Camera gantry starting...");
  Serial.print("Day/night cycle: "); Serial.print(dayHours);
  Serial.print(" hours day, ");      Serial.print(nightHours);
  Serial.println(" hours night");
  Serial.print("Cycle interval: ");  Serial.print(cycleIntervalMs / 60000UL);
  Serial.println(" minutes");
  Serial.print("Will photograph ");  Serial.print(numShelves);
  Serial.print(" shelves with ");    Serial.print(photosPerShelf);
  Serial.println(" photos each");
}

void loop() {
  unsigned long cycleStart = millis();

  if (!calibrated) {
    Serial.println("Starting calibration sequence...");
    calibrate();
    calibrated = true;
    Serial.println("Calibration complete. Starting photography sequence...");
  }

  bool dayTime = (dayElapsedHours < dayHours);

  // Snake: first shelf sweeps right, direction flips each shelf.
  boolean dir = right;
  for (int shelf = 1; shelf <= numShelves; shelf++) {
    Serial.print("Photographing shelf "); Serial.println(shelf);
    photographShelf(shelf, dir, dayTime);

    if (shelf < numShelves) {
      Serial.println("Moving up to next shelf...");
      moveVertical(up, MOVE_PER_SHELF_MS);
    }
    dir = !dir;
  }

  Serial.println("Photography sequence complete! Returning home...");
  returnHome();

  // Advance the light-cycle clock by one interval, then wrap at 24h.
  dayElapsedHours += (cycleIntervalMs / 60000.0) / 60.0;  // minutes -> hours
  if (dayElapsedHours >= 24) dayElapsedHours = 0;

  // Pad the cycle so the whole loop takes exactly one interval.
  Serial.println("Cycle complete. Waiting for next interval...");
  while (millis() - cycleStart < cycleIntervalMs) {
    delay(200);
  }
}

// Take photosPerShelf pictures across one shelf in snake direction `dir`.
void photographShelf(int shelf, boolean dir, bool dayTime) {
  digitalWrite(dirPinX, dir);
  for (int box = 0; box < photosPerShelf; box++) {
    takePhoto(shelf, dayTime);
    if (box < photosPerShelf - 1) {
      moveHorizontal(dir, MOVE_PER_BOX_MS);
    }
  }
}

// Light the shelf (if enabled), pulse the camera trigger, then -- at
// night only -- switch the light back off.
void takePhoto(int shelf, bool dayTime) {
  int relay = shelfRelayPins[shelf - 1];

  if (ENABLE_SHELF_LIGHTS) {
    digitalWrite(relay, RELAY_ON);
    delay(LIGHT_SETTLE_MS);
  }

  digitalWrite(cameraPin, HIGH);
  delayMicroseconds(CAMERA_PULSE_US);
  digitalWrite(cameraPin, LOW);
  delay(POST_SHOT_MS);

  if (ENABLE_SHELF_LIGHTS && !dayTime) {
    digitalWrite(relay, RELAY_OFF);
  }
}

void moveHorizontal(boolean direction, unsigned long durationMs) {
  digitalWrite(dirPinX, direction);
  unsigned long t0 = millis();
  while (millis() - t0 < durationMs) {
    digitalWrite(stepPinX, HIGH);
    delayMicroseconds(STEP_DELAY_US);
    digitalWrite(stepPinX, LOW);
    delayMicroseconds(STEP_DELAY_US);
  }
}

void moveVertical(boolean direction, unsigned long durationMs) {
  digitalWrite(dirPinY, direction);
  unsigned long t0 = millis();
  while (millis() - t0 < durationMs) {
    digitalWrite(stepPinY, HIGH);
    delayMicroseconds(STEP_DELAY_US);
    digitalWrite(stepPinY, LOW);
    delayMicroseconds(STEP_DELAY_US);
  }
}

// Calibration: jog up+right clear of the flags, then drive down+left
// onto both photointerrupters. The wording here is verbatim from the
// scaffold, which the live serial output confirms is still in use.
void calibrate() {
  Serial.println("Calibration step 1: Moving right for 5 seconds...");
  moveHorizontal(right, 5000);
  Serial.println("Calibration step 2: Moving up for 5 seconds...");
  moveVertical(up, 5000);

  Serial.println("Calibration step 3: Finding photointerrupters...");
  homeToSensors();

  // Jog to the first box if the home position isn't box #1.
  if (START_OFFSET_X_MS) moveHorizontal(right, START_OFFSET_X_MS);
  if (START_OFFSET_Y_MS) moveVertical(up, START_OFFSET_Y_MS);

  Serial.println("Calibration complete - at home position");
}

void returnHome() {
  Serial.println("Returning to home position using photointerrupters...");
  homeToSensors();
}

// Drive down + left until BOTH photointerrupters trigger (read LOW).
// Each axis stops stepping as soon as its own sensor trips.
void homeToSensors() {
  digitalWrite(dirPinY, down);
  digitalWrite(dirPinX, left);

  bool vTrig = false, hTrig = false;
  while (!vTrig || !hTrig) {
    if (digitalRead(vertSensor)  == LOW) vTrig = true;
    if (digitalRead(horizSensor) == LOW) hTrig = true;

    if (!vTrig) digitalWrite(stepPinY, HIGH);
    if (!hTrig) digitalWrite(stepPinX, HIGH);
    delayMicroseconds(STEP_DELAY_US);
    if (!vTrig) digitalWrite(stepPinY, LOW);
    if (!hTrig) digitalWrite(stepPinX, LOW);
    delayMicroseconds(STEP_DELAY_US);
  }

  // Leave directions set for the next upward sweep.
  digitalWrite(dirPinX, right);
  digitalWrite(dirPinY, up);
}

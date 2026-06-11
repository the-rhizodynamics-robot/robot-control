// =============================================================
//  robot_device_serial.ino
//
//  Host-integrated firmware for the plant-imaging camera gantry.
//
//  Same proven motion / camera / lighting as robot_device.ino, but
//  driven by a host program over USB serial:
//    - startup HANDSHAKE: the host sends the run config, which the
//      firmware reads (and echoes back for verification) instead of
//      using hardcoded constants;
//    - "home" REPORT: the firmware prints "home" once per completed
//      cycle so the host watchdog knows it is alive; 
//    - KILL: at any time the host may send KILLCODE (2048) to stop
//      the robot immediately.
//
//  robot_device.ino (standalone, hardcoded config) is kept as the
//  known-good fallback. Flash that if this protocol build misbehaves.
//
//  Protocol contract (host sends these as ASCII text, in this order;
//  firmware echoes each back):
//    1. num_shelves
//    2. photos_per_shelf
//    3. cycle_interval_min
//    4. day_hours        (light hours per 24h; 24 = constant light)
//    5. start_hour       (hour into the current day cycle)
//    6. start signal     (any non-kill value, e.g. 1)
//  Link: 9600 baud, Serial.setTimeout(2).
// =============================================================

const int KILLCODE = 2048;   // must match the host program

// ---------------- Run configuration (set by handshake) -------
// Defaults below are only placeholders; the host overwrites them.
int numShelves      = 3;
int photosPerShelf  = 8;
int cycleIntervalMin = 15;
int dayHours        = 24;     // 24 = constant light
int startHour       = 0;

unsigned long cycleIntervalMs = 15UL * 60UL * 1000UL; // derived after handshake

// ---------------- Pin map (same as robot_device.ino) ---------
const int horizSensor = 3;   // D3 photointerrupter
const int vertSensor  = 2;   // D2 photointerrupter

const int stepPinX = 5;      // horizontal motor
const int dirPinX  = 7;
const int enblPinX = 6;

const int stepPinY = 8;      // vertical motor
const int dirPinY  = 10;
const int enblPinY = 9;

const int cameraPin = 12;    // camera trigger on GND + D12

// Shelf-lighting relays (Mega, active-LOW). Index 0 = shelf 1.
const bool ENABLE_SHELF_LIGHTS = true;
const int  shelfRelayPins[]    = {23, 25, 27, 29, 31, 33};
const int  MAX_SHELVES         = 6;   // size of shelfRelayPins
const int  RELAY_ON  = LOW;
const int  RELAY_OFF = HIGH;

// ---------------- Direction constants ------------------------
const boolean right = LOW;
const boolean left  = HIGH;
const boolean up    = LOW;
const boolean down  = HIGH;

// ---------------- Timing (same as robot_device.ino) ----------
const unsigned long STEP_DELAY_US    = 100;
const unsigned long CAMERA_PULSE_MS  = 50;    // a shorter pulse is ignored by the camera
const unsigned long LIGHT_SETTLE_MS  = 3000;
const unsigned long POST_SHOT_MS     = 500;
const unsigned long MOVE_PER_BOX_MS   = 1400; // [TUNE] land on each box
const unsigned long MOVE_PER_SHELF_MS = 8000; // [TUNE]
const unsigned long START_OFFSET_X_MS = 0;    // [CONFIRM]
const unsigned long START_OFFSET_Y_MS = 0;    // [CONFIRM]

// ---------------- State --------------------------------------
bool  calibrated      = false;
float dayElapsedHours = 0;    // position within the 24h light cycle

void setup() {
  Serial.begin(9600);
  Serial.setTimeout(2);  // needed for readString().toInt() of multi-digit numbers

  // ---- HANDSHAKE: receive run config from the host ----
  // Order must match the host (see protocol contract above).
  numShelves       = waitForValue();
  photosPerShelf   = waitForValue();
  cycleIntervalMin = waitForValue();
  dayHours         = waitForValue();
  startHour        = waitForValue();
  waitForValue();  // start signal (any non-kill value)

  // Derived values
  if (numShelves > MAX_SHELVES) numShelves = MAX_SHELVES;
  cycleIntervalMs = (unsigned long)cycleIntervalMin * 60000UL;
  dayElapsedHours = startHour;

  // ---- Pin setup ----
  pinMode(stepPinX, OUTPUT); pinMode(dirPinX, OUTPUT); pinMode(enblPinX, OUTPUT);
  pinMode(stepPinY, OUTPUT); pinMode(dirPinY, OUTPUT); pinMode(enblPinY, OUTPUT);
  digitalWrite(stepPinX, LOW);   digitalWrite(stepPinY, LOW);
  digitalWrite(dirPinX, right);  digitalWrite(dirPinY, up);
  digitalWrite(enblPinX, LOW);   digitalWrite(enblPinY, LOW);  // LOW = enabled

  pinMode(horizSensor, INPUT_PULLUP);
  pinMode(vertSensor,  INPUT_PULLUP);

  pinMode(cameraPin, OUTPUT);
  digitalWrite(cameraPin, LOW);

  if (ENABLE_SHELF_LIGHTS) {
    for (int i = 0; i < numShelves; i++) {
      pinMode(shelfRelayPins[i], OUTPUT);
      digitalWrite(shelfRelayPins[i], RELAY_OFF);
    }
  }

  // ---- Banner ----
  Serial.println("Camera gantry starting...");
  Serial.print("Day/night cycle: "); Serial.print(dayHours);
  Serial.print(" hours day, ");      Serial.print(24 - dayHours);
  Serial.println(" hours night");
  Serial.print("Cycle interval: ");  Serial.print(cycleIntervalMin);
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
    checkKillSignal();
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

  // ---- HOME REPORT: tell the host the cycle finished ----
  Serial.println("home");
  Serial.flush();

  // Advance the light-cycle clock by one interval, then wrap at 24h.
  dayElapsedHours += (float)cycleIntervalMin / 60.0;
  if (dayElapsedHours >= 24) dayElapsedHours = 0;

  // Pad the cycle to one interval, watching for a kill the whole time.
  Serial.println("Cycle complete. Waiting for next interval...");
  while (millis() - cycleStart < cycleIntervalMs) {
    checkKillSignal();
    delay(200);
  }
}

// Take photosPerShelf pictures across one shelf in snake direction `dir`.
void photographShelf(int shelf, boolean dir, bool dayTime) {
  digitalWrite(dirPinX, dir);
  for (int box = 0; box < photosPerShelf; box++) {
    checkKillSignal();
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
  delay(CAMERA_PULSE_MS);
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
// onto both photointerrupters.
void calibrate() {
  Serial.println("Calibration step 1: Moving right for 5 seconds...");
  moveHorizontal(right, 5000);
  Serial.println("Calibration step 2: Moving up for 5 seconds...");
  moveVertical(up, 5000);

  Serial.println("Calibration step 3: Finding photointerrupters...");
  homeToSensors();

  if (START_OFFSET_X_MS) moveHorizontal(right, START_OFFSET_X_MS);
  if (START_OFFSET_Y_MS) moveVertical(up, START_OFFSET_Y_MS);

  Serial.println("Calibration complete - at home position");
}

void returnHome() {
  Serial.println("Returning to home position using photointerrupters...");
  homeToSensors();
}

// Drive down + left until BOTH photointerrupters trigger (read LOW).
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

  digitalWrite(dirPinX, right);
  digitalWrite(dirPinY, up);
}

// =============================================================
//  Serial protocol helpers (ported from the original firmware)
// =============================================================

// Block until the host sends an int. Echo it back (newline-terminated)
// so the host can verify the handshake. A KILLCODE here halts the robot.
int waitForValue() {
  while (!Serial.available());
  int val = Serial.readString().toInt();
  if (val == KILLCODE) haltOnKill();
  Serial.println(val);   // echo for host verification
  Serial.flush();
  return val;
}

// Non-blocking: if the host has sent the killcode, stop immediately.
void checkKillSignal() {
  if (Serial.available() > 0) {
    int val = Serial.readString().toInt();
    if (val == KILLCODE) haltOnKill();
    // any other stray input is ignored
  }
}

// Emergency stop: lights off, motors disabled, halt forever.
void haltOnKill() {
  if (ENABLE_SHELF_LIGHTS) {
    for (int i = 0; i < numShelves; i++) {
      digitalWrite(shelfRelayPins[i], RELAY_OFF);
    }
  }
  digitalWrite(enblPinX, HIGH);  // HIGH = disabled
  digitalWrite(enblPinY, HIGH);
  Serial.println("KILL received - motors disabled, halting.");
  Serial.flush();
  while (1);
}

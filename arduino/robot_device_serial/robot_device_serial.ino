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

const int stepPinY = 8;      // vertical (both Y drivers ganged; bench-tested wiring)
const int dirPinY  = 9;
const int enblPinY = 10;

const int cameraPin = 12;    // -> 1k -> BFS GPIO Line 0 (OPTO_IN, black); GND -> blue (OPTO_GND)

// Single LED relay -- all lights on one channel (D11), active-LOW. Bench wiring
// for THIS robot (not the per-shelf relay bank of the previous build).
const bool ENABLE_LIGHTS = true;
const int  ledRelayPin   = 11;
const int  MAX_SHELVES   = 8;   // sanity clamp on host-sent numShelves
const int  RELAY_ON  = LOW;
const int  RELAY_OFF = HIGH;

// ---------------- Direction constants ------------------------
const boolean right = HIGH;   // bench-tested on this robot
const boolean left  = LOW;
const boolean up    = LOW;    // bench-tested
const boolean down  = HIGH;

// ---------------- Timing (same as robot_device.ino) ----------
const unsigned long STEP_DELAY_US    = 100;
// The camera's opto-isolated trigger input IGNORES very short pulses. Hold the
// line HIGH for ~50ms; a microsecond blip produces motion and lights but NO
// images. This has bitten us before -- do not lower this without testing.
const unsigned long CAMERA_PULSE_MS  = 50;    // HIGH pulse width; confirmed firing the camera in camera_trigger_test
const unsigned long LIGHT_SETTLE_MS  = 3000;
const unsigned long LIGHT_WARMUP_MS  = 10000; // first cycle only: let the LEDs warm up before imaging
const unsigned long POST_SHOT_MS     = 500;
// Calibrated 2026-07-04 (axis_calibration: 29.5 cm / 20 s = 1.475 cm/s, both axes):
//   box pitch 3" = 7.62 cm -> 5166 ms; shelf pitch 31.6 cm -> 21424 ms, taken
//   -10% to 19282 by operator on the bench run; first box ~1" right of home.
const unsigned long MOVE_PER_BOX_MS   = 5166;  // box pitch (3")
const unsigned long MOVE_PER_SHELF_MS = 19282; // shelf pitch (31.6 cm, cal -10%)
const unsigned long START_OFFSET_X_MS = 1722;  // jog right to box 1 (~1")
const unsigned long START_OFFSET_Y_MS = 0;     // shelf 1 at home height
const unsigned long MOTOR_SETTLE_MS   = 200;  // let drivers energize before stepping

// Vertical home seating + LEVELING. homeToSensors() stops on the single vertical
// sensor, which can leave the gantry slightly tilted (right side high). We then
// drive DOWN a short distance (~0.2 cm, tuned) into the physical stops so BOTH
// ganged Y motors bottom out and level -- the higher side keeps stepping until it
// hits its stop; the already-
// seated side just skips steps harmlessly against the stop. This also makes the
// between-cycle de-energize drop-free (resting on the stop). Next cycle we rise
// until the sensor clears, then drive back down to re-home + re-level.
const unsigned long VERT_SEAT_MS         = 150;  // ~0.2 cm down into the stops (tuned) [TUNE]
const unsigned long VERT_CLEAR_MARGIN_MS = 300;  // extra up after sensor clears [TUNE]

// ---------------- State --------------------------------------
bool  calibrated      = false;
bool  firstCycle      = true; // first imaging cycle gets the longer light warm-up
float dayElapsedHours = 0;    // position within the 24h light cycle

void setup() {
  Serial.begin(9600);
  Serial.setTimeout(2);  // needed for readString().toInt() of multi-digit numbers

  // De-energize the steppers FIRST, before the (blocking) handshake below.
  // The enable pins are active-LOW and float "enabled" at power-up, so until we
  // drive them HIGH the drivers hold current — dumping heat into the chamber the
  // whole time the robot sits waiting for the host. Disable them immediately.
  pinMode(enblPinX, OUTPUT);     pinMode(enblPinY, OUTPUT);
  digitalWrite(enblPinX, HIGH);  digitalWrite(enblPinY, HIGH);  // HIGH = disabled

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
  // Steppers were already disabled at the top of setup(); keep them that way.
  // They are only powered while a cycle is actively running (engaged at the top
  // of loop(), released after returnHome()), so they never dump holding-current
  // heat into the growth chamber while idle. HIGH = disabled.
  digitalWrite(enblPinX, HIGH);  digitalWrite(enblPinY, HIGH);

  pinMode(horizSensor, INPUT_PULLUP);
  pinMode(vertSensor,  INPUT_PULLUP);

  pinMode(cameraPin, OUTPUT);
  digitalWrite(cameraPin, LOW);

  if (ENABLE_LIGHTS) {
    pinMode(ledRelayPin, OUTPUT);
    digitalWrite(ledRelayPin, RELAY_OFF);   // lights off until a cycle runs
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

  // Motors are de-energized during the inter-cycle wait so their holding
  // current does not heat the growth chamber. Re-engage and re-home at the
  // START of every cycle: while unpowered the gantry may have drifted or the
  // vertical carriage sagged, so we must re-establish position before any move
  // that depends on it. calibrate() jogs clear of the flags first, so it
  // re-seats correctly even if the carriage sagged below the sensor.
  engageMotors();
  if (!calibrated) {
    Serial.println("Starting calibration sequence...");
    calibrate();
    calibrated = true;
    Serial.println("Calibration complete. Starting photography sequence...");
  } else {
    Serial.println("Re-homing at cycle start...");
    calibrate();
  }

  bool dayTime = (dayElapsedHours < dayHours);

  // All LEDs are on ONE relay -- switch on once for the whole sweep. The first
  // cycle waits ~10 s so the LEDs fully warm up/stabilize before any imaging;
  // later cycles just use the short settle.
  if (ENABLE_LIGHTS) {
    digitalWrite(ledRelayPin, RELAY_ON);
    delay(firstCycle ? LIGHT_WARMUP_MS : LIGHT_SETTLE_MS);
    firstCycle = false;
  }

  // Snake: first shelf sweeps right, direction flips each shelf.
  boolean dir = right;
  for (int shelf = 1; shelf <= numShelves; shelf++) {
    checkKillSignal();
    Serial.print("Photographing shelf "); Serial.println(shelf);
    photographShelf(dir);

    if (shelf < numShelves) {
      Serial.println("Moving up to next shelf...");
      moveVertical(up, MOVE_PER_SHELF_MS);
    }
    dir = !dir;
  }

  Serial.println("Photography sequence complete! Returning home...");
  returnHome();

  // At night, cut the lights between cycles; in daylight leave them on.
  if (ENABLE_LIGHTS && !dayTime) digitalWrite(ledRelayPin, RELAY_OFF);

  // De-energize the steppers for the idle wait: no holding current means no
  // motor heat in the growth chamber between cycles. They are re-engaged and
  // re-homed at the top of the next cycle.
  disengageMotors();

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
void photographShelf(boolean dir) {
  digitalWrite(dirPinX, dir);
  for (int box = 0; box < photosPerShelf; box++) {
    checkKillSignal();
    takePhoto();
    if (box < photosPerShelf - 1) {
      moveHorizontal(dir, MOVE_PER_BOX_MS);
    }
  }
}

// Lights are already on (single relay, whole run). Just pulse the camera.
void takePhoto() {
  digitalWrite(cameraPin, HIGH);
  delay(CAMERA_PULSE_MS);
  digitalWrite(cameraPin, LOW);
  delay(POST_SHOT_MS);
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
  Serial.println("Calibration step 2: Rising until vertical sensor clears...");
  raiseVertClearOfSensor();

  Serial.println("Calibration step 3: Finding photointerrupters...");
  homeToSensors();

  if (START_OFFSET_X_MS) moveHorizontal(right, START_OFFSET_X_MS);
  if (START_OFFSET_Y_MS) moveVertical(up, START_OFFSET_Y_MS);

  Serial.println("Calibration complete - at home position");
}

void returnHome() {
  Serial.println("Returning to home position using photointerrupters...");
  homeToSensors();   // homeToSensors now seats ~1 cm into the stops + levels
}

// Power the steppers on (active-LOW enable) and give the drivers a moment to
// energize before any stepping, so the first moves don't lose steps.
void engageMotors() {
  digitalWrite(enblPinX, LOW);
  digitalWrite(enblPinY, LOW);
  delay(MOTOR_SETTLE_MS);
}

// Power the steppers off so they draw no holding current (and shed no heat)
// while the gantry is idle between cycles.
void disengageMotors() {
  digitalWrite(enblPinX, HIGH);
  digitalWrite(enblPinY, HIGH);
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

  // Seat a short distance DOWN (~0.2 cm) into the physical stops to LEVEL the gantry: both ganged Y
  // motors bottom out (the higher/right side keeps stepping until it hits its
  // stop; the seated side skips steps harmlessly). Also parks it drop-free.
  moveVertical(down, VERT_SEAT_MS);

  digitalWrite(dirPinX, right);
  digitalWrite(dirPinY, up);
}

// Rise until the vertical photointerrupter reads CLEAR (flag out of the slot),
// then a small margin more. Used at cycle start to lift off the bottom stop and
// uncover the sensor, so homeToSensors() can then re-seat a clean home.
void raiseVertClearOfSensor() {
  digitalWrite(dirPinY, up);
  while (digitalRead(vertSensor) == LOW) {
    digitalWrite(stepPinY, HIGH);
    delayMicroseconds(STEP_DELAY_US);
    digitalWrite(stepPinY, LOW);
    delayMicroseconds(STEP_DELAY_US);
  }
  moveVertical(up, VERT_CLEAR_MARGIN_MS);
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
  if (ENABLE_LIGHTS) digitalWrite(ledRelayPin, RELAY_OFF);

  // Graceful shutdown: bring the gantry HOME and seat it on the bottom stop
  // BEFORE cutting motor power, so the non-self-locking vertical belt axis parks
  // safely (rests on the stop) instead of being de-energized mid-travel and
  // free-dropping. engageMotors() first in case the kill arrived while the
  // steppers were disabled (handshake or the idle wait between cycles).
  Serial.println("KILL received - homing before power-off...");
  Serial.flush();
  engageMotors();
  homeToSensors();   // homes + seats ~1 cm into the stops + levels the gantry

  digitalWrite(enblPinX, HIGH);  // now safe to de-energize -- resting on the stop
  digitalWrite(enblPinY, HIGH);
  Serial.println("Homed, motors disabled, halting.");
  Serial.flush();
  while (1);
}

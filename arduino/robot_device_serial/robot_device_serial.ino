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
// The camera's opto-isolated trigger input IGNORES very short pulses. Hold the
// line HIGH for ~50ms; a microsecond blip produces motion and lights but NO
// images. This has bitten us before -- do not lower this without testing.
const unsigned long CAMERA_PULSE_MS  = 50;    // HIGH pulse width; confirmed firing the camera in camera_trigger_test
const unsigned long LIGHT_SETTLE_MS  = 3000;
const unsigned long POST_SHOT_MS     = 500;
const unsigned long MOVE_PER_BOX_MS   = 1400; // [TUNE] land on each box
const unsigned long MOVE_PER_SHELF_MS = 8000; // [TUNE]
const unsigned long START_OFFSET_X_MS = 0;    // [CONFIRM]
const unsigned long START_OFFSET_Y_MS = 0;    // [CONFIRM]
const unsigned long MOTOR_SETTLE_MS   = 200;  // let drivers energize before stepping

// ---------------- Pre-shot settle ----------------------------
// Wait this long after a move before triggering the camera. takePhoto() fires
// the trigger the instant moveHorizontal() returns -- POST_SHOT_MS elapses AFTER
// the pulse, not before it -- so without this every box but the first is
// photographed with zero settling time. Box 1 is the exception: it already gets
// LIGHT_SETTLE_MS / LIGHT_WARMUP_MS while the lights come up.
//
// 0 = previous behaviour. See concepts/horizontal-jitter.md in the wiki: the
// measured jitter is far too large for carriage ringing to explain (a 1 kg
// carriage at 14.75 mm/s carries ~1e-4 J, giving ~4 px of ring against 60-190 px
// observed), so this is NOT expected to be the fix on its own -- it is the cheap
// test of whether anything settles at all, and of camera-MOUNT ring, which is
// angular and needs almost no energy.
//
// !! Interacts with the driver's idle-current setting. The DQ542MA drops to half
// current ~0.4 s after the last step pulse (SW4). Today the shutter fires at FULL
// current; a settle longer than that moves the exposure into the half-current
// window, HALVING holding torque exactly when the picture is taken. So either
// keep this under ~400 ms, or switch SW4 to full current first -- which is the
// plan being tested (SW4 full + a decisive 1000 ms).
const unsigned long PRE_SHOT_SETTLE_MS = 1000;

// ---------------- Homing fault detection ---------------------
// homeToSensors() drives until BOTH flags trigger. If an axis cannot get there
// -- a jam, an obstruction, a failed or dirty sensor, a snapped belt -- an
// unbounded wait pulses step into a stalled motor indefinitely: the driver
// keeps pushing full rated current into a locked rotor, the gantry grinds, and
// the host cannot intervene because checkKillSignal() is not reachable from
// inside that loop. Only a power cycle would stop it.
//
// The budget is DERIVED from the run geometry (handshake) and this rig's own
// motion constants rather than hardcoded, so it travels to robots with a
// different shelf count, box count, or rail length. Worst case is homing from
// the far corner: full travel of the slower axis, plus the jog calibrate()
// makes clear of the flags first.
const unsigned long HOME_JOG_MS           = 5000;  // calibrate()'s pre-homing jog
const unsigned long HOME_TIMEOUT_FACTOR   = 3;     // margin over worst-case travel
const unsigned long HOME_TIMEOUT_FLOOR_MS = 60000; // never trip earlier than this

// Catch-all budget for one cycle's WORK (calibrate -> sweep -> returnHome), not
// counting the idle wait that follows. Deliberately loose: the per-operation
// homing budget above is the tight bound that limits how long a stalled motor
// can grind, so this only has to catch loops nobody bounded -- present or
// future -- and it costs nothing to be generous.
//
// For scale, a 4-shelf / 8-box rig at this firmware's tuning does ~5-6 min of
// work per cycle, so 15 min is roughly 3x. The host's own watchdog
// (cycle_interval + kill_margin) is usually the TIGHTER of the two and will
// normally trip first -- that is fine and intended: guard() polls for the kill,
// so the host can always be heard. This budget is the backstop for when the
// serial link is dead and nobody is listening.
const unsigned long CYCLE_WORK_BUDGET_MS = 15UL * 60UL * 1000UL;

// How often guard() actually does its checks, in step pulses. 256 steps at
// ~200 us/step is ~50 ms -- frequent enough to be responsive, rare enough that
// the polling costs nothing inside a 5 kHz stepping loop.
const unsigned int GUARD_EVERY_STEPS = 256;

// ---------------- State --------------------------------------
bool  calibrated      = false;
float dayElapsedHours = 0;    // position within the 24h light cycle
unsigned long homeTimeoutMs = 0;  // derived after handshake (see above)
unsigned long faultLowerMs  = 0;  // budget for the fault-path powered descent
unsigned long workStartMs   = 0;  // when this cycle's work began
bool  workTimingActive      = false; // false during the idle wait (no deadline)
unsigned int  guardCounter  = 0;  // step counter for guard()'s duty cycle
bool  halting               = false; // set once a halt path starts; stops re-entry

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

  // Homing budget, from the geometry just received. homeToSensors() drives BOTH
  // axes in one loop at the same step rate, so worst-case homing time is the
  // SLOWER axis's full travel (a max, not a sum), plus calibrate()'s jog clear
  // of the flags. Shelf 1 sits START_OFFSET_Y_MS above home, so the vertical
  // span is (numShelves - 1) pitches; likewise (photosPerShelf - 1) horizontally.
  unsigned long vertTravelMs  = (unsigned long)(numShelves > 1 ? numShelves - 1 : 1)
                                * MOVE_PER_SHELF_MS + START_OFFSET_Y_MS;
  unsigned long horizTravelMs = (unsigned long)(photosPerShelf > 1 ? photosPerShelf - 1 : 1)
                                * MOVE_PER_BOX_MS + START_OFFSET_X_MS;
  unsigned long worstTravelMs = (vertTravelMs > horizTravelMs ? vertTravelMs : horizTravelMs)
                                + HOME_JOG_MS;
  homeTimeoutMs = worstTravelMs * HOME_TIMEOUT_FACTOR;
  if (homeTimeoutMs < HOME_TIMEOUT_FLOOR_MS) homeTimeoutMs = HOME_TIMEOUT_FLOOR_MS;
  // The fault path only has to bring the carriage DOWN, so it needs the vertical
  // span alone -- plus the jog, since calibrate() may have raised it that far.
  faultLowerMs = vertTravelMs + HOME_JOG_MS;

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
  Serial.print("Homing timeout: ");  Serial.print(homeTimeoutMs / 1000);
  Serial.println(" s (derived from geometry)");
}

void loop() {
  unsigned long cycleStart = millis();

  // Open the work window. guard() enforces CYCLE_WORK_BUDGET_MS against this
  // from inside every stepping loop; it is closed again before the idle wait
  // below, which is allowed to take as long as the cycle interval says.
  workStartMs      = cycleStart;
  workTimingActive = true;

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

  // De-energize the steppers for the idle wait: no holding current means no
  // motor heat in the growth chamber between cycles. They are re-engaged and
  // re-homed at the top of the next cycle.
  disengageMotors();

  // Work is done; the idle wait is not held to the work budget.
  workTimingActive = false;

  // ---- HOME REPORT: tell the host the cycle finished ----
  // The work duration goes out first, on its own line. wait_for_home() matches
  // "home" EXACTLY, so extra lines are passed through to the host log rather
  // than mistaken for a completed cycle -- which makes this the measured number
  // to set CYCLE_WORK_BUDGET_MS (and the homing factor) from, once some runs
  // have banked real values.
  Serial.print("cycle_work_ms "); Serial.println(millis() - cycleStart);
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
    // Let the gantry settle before the shutter. Skipped for box 0, which has just
    // had LIGHT_SETTLE_MS/LIGHT_WARMUP_MS while the lights came up.
    if (box > 0 && PRE_SHOT_SETTLE_MS) delay(PRE_SHOT_SETTLE_MS);
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
    guard();
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
    guard();
    digitalWrite(stepPinY, HIGH);
    delayMicroseconds(STEP_DELAY_US);
    digitalWrite(stepPinY, LOW);
    delayMicroseconds(STEP_DELAY_US);
  }
}

// Called from inside every stepping loop. Two jobs, both of which the firmware
// previously could not do while moving:
//
//   1. Poll for the host's kill. checkKillSignal() was only reachable from
//      loop() and photographShelf(), so a kill sent during a move -- or during
//      homing -- was not read until the move ended, and in an unbounded loop
//      was never read at all. The host's home-to-home watchdog would fire, send
//      KILLCODE, and nothing would be listening.
//   2. Enforce the cycle work budget, so ANY blocking loop is bounded, not just
//      the ones with their own timeout.
//
// Cheap by construction: the real work runs once per GUARD_EVERY_STEPS.
void guard() {
  if (++guardCounter < GUARD_EVERY_STEPS) return;
  guardCounter = 0;

  checkKillSignal();   // may not return

  // Subtraction (not addition) so this is safe across the millis() rollover.
  if (workTimingActive && (millis() - workStartMs) > CYCLE_WORK_BUDGET_MS) {
    faultHalt("cycle work budget exceeded");   // does not return
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

// Drive down + left until BOTH photointerrupters trigger (read LOW), or until
// the derived budget expires -- see the homing-fault notes at the top.
void homeToSensors() {
  digitalWrite(dirPinY, down);
  digitalWrite(dirPinX, left);

  bool vTrig = false, hTrig = false;
  unsigned long t0 = millis();
  while (!vTrig || !hTrig) {
    if (digitalRead(vertSensor)  == LOW) vTrig = true;
    if (digitalRead(horizSensor) == LOW) hTrig = true;

    // Never spin here forever: an obstructed axis would stall against the
    // obstruction at full current with no way for the host to intervene.
    if (millis() - t0 > homeTimeoutMs) {
      Serial.print("FAULT: vertical ");   Serial.println(vTrig ? "reached flag" : "DID NOT REACH FLAG");
      Serial.print("FAULT: horizontal "); Serial.println(hTrig ? "reached flag" : "DID NOT REACH FLAG");
      faultHalt("homing timed out");   // does not return
    }
    guard();

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

// All shelf lights off. Shared by the kill and fault paths.
void lightsOff() {
  if (!ENABLE_SHELF_LIGHTS) return;
  for (int i = 0; i < numShelves; i++) digitalWrite(shelfRelayPins[i], RELAY_OFF);
}

// Drive the carriage DOWN until the vertical flag triggers or `budgetMs` runs
// out; true if it reached the stop.
//
// Fault path only. The point is to seat a possibly-raised carriage UNDER POWER
// before cutting current: de-energizing a loaded vertical axis lets it drop,
// back-driving the motors as generators into the shared supply, which has
// tripped both drivers on regen over-voltage during bench testing. A controlled
// descent avoids both that and the mechanical impact.
bool lowerVerticalToStop(unsigned long budgetMs) {
  digitalWrite(dirPinY, down);
  unsigned long t0 = millis();
  while (millis() - t0 < budgetMs) {
    if (digitalRead(vertSensor) == LOW) return true;
    digitalWrite(stepPinY, HIGH);
    delayMicroseconds(STEP_DELAY_US);
    digitalWrite(stepPinY, LOW);
    delayMicroseconds(STEP_DELAY_US);
  }
  return digitalRead(vertSensor) == LOW;
}

// Terminal: a motion budget was exceeded. Stepping has already stopped (the
// caller left its loop), which is what ends the stall. Park the carriage
// safely, then halt -- a gantry that cannot find home must not keep imaging,
// and recovery should be a deliberate human power cycle.
void faultHalt(const char *reason) {
  halting = true;   // no re-entry: nothing below may bounce back through here
  lightsOff();
  Serial.print("FAULT: "); Serial.println(reason);
  Serial.flush();

  // lowerVerticalToStop() returns immediately if the flag is already made, so
  // this covers both "vertical was fine" and "vertical is somewhere unknown".
  if (lowerVerticalToStop(faultLowerMs)) {
    disengageMotors();
    Serial.println("FAULT: carriage on its stop, motors disabled. Power-cycle to reset.");
  } else {
    // Could not seat it, so the vertical axis is stuck somewhere loaded and
    // cutting power would drop it. Hold instead: the grinding has stopped and
    // holding current is quiet and safe, whereas a free drop is neither.
    Serial.println("FAULT: could not seat carriage - HOLDING under power. Power-cycle to reset.");
  }
  Serial.flush();
  while (1);
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
  if (halting) return;   // a halt path is already running; do not re-enter it
  if (Serial.available() > 0) {
    int val = Serial.readString().toInt();
    if (val == KILLCODE) haltOnKill();
    // any other stray input is ignored
  }
}

// Emergency stop: lights off, motors disabled, halt forever.
void haltOnKill() {
  halting = true;   // guard()/checkKillSignal() must not re-enter from here
  lightsOff();
  digitalWrite(enblPinX, HIGH);  // HIGH = disabled
  digitalWrite(enblPinY, HIGH);
  Serial.println("KILL received - motors disabled, halting.");
  Serial.flush();
  while (1);
}

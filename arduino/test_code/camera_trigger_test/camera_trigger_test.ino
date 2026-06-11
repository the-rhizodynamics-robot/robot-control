// camera_trigger_test.ino
// Motion-free test of the camera hardware trigger.
//
// Pulses the trigger pin on a timer so you can confirm whether the
// camera + capture software actually grab a frame per pulse, with all the
// gantry movement/timing taken out of the picture.
//
// Watch two things:
//   1. The Mega's onboard LED (pin 13) flashes on every trigger, and the
//      Serial Monitor prints "TRIGGER" -> firmware-side proof the pulse fired.
//   2. Your capture software (FlyCap etc.) -> one new image per flash?
//
// If the LED flashes but no images arrive, the firmware is fine and the
// problem is the camera config (not armed / wrong polarity / wrong pin).
//
// To test the opposite polarity (falling-edge / active-low trigger),
// change TRIGGER_ACTIVE to LOW and re-upload.
//
// NOTE: the main firmware (robot_device.ino) uses a 50ms HIGH pulse on
// D12. A shorter pulse (the old 100us) is ignored by the camera -- that
// bug is exactly what this sketch was written to catch.

const int cameraPin = 12;   // camera trigger (wired to GND + D12)
const int ledPin    = 13;   // Mega onboard LED, mirrors the trigger

const bool          TRIGGER_ACTIVE = HIGH;  // <-- set LOW to test active-low
const unsigned long PULSE_MS       = 50;    // wide pulse so nothing is missed
const unsigned long PERIOD_MS      = 10000; // fire every 10 s

void setup() {
  Serial.begin(9600);
  pinMode(cameraPin, OUTPUT);
  pinMode(ledPin, OUTPUT);
  digitalWrite(cameraPin, !TRIGGER_ACTIVE); // idle state
  digitalWrite(ledPin, LOW);
  Serial.print("Camera trigger test - pulsing D12 every ");
  Serial.print(PERIOD_MS / 1000);
  Serial.println(" s");
  Serial.print("Polarity: TRIGGER_ACTIVE = ");
  Serial.println(TRIGGER_ACTIVE ? "HIGH" : "LOW");
}

void loop() {
  digitalWrite(cameraPin, TRIGGER_ACTIVE);
  digitalWrite(ledPin, HIGH);
  delay(PULSE_MS);
  digitalWrite(cameraPin, !TRIGGER_ACTIVE);
  digitalWrite(ledPin, LOW);
  Serial.println("TRIGGER");
  delay(PERIOD_MS - PULSE_MS);
}

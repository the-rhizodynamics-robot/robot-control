// Robot movement control using robot_device_testing.ino pin configuration
// Simplified version focusing only on motor movement

// VARIABLES - simplified for testing
int numShelves = 2;
int numBoxesPerShelf = 4;

// Movement distances in cm
float horizBoxDistance = 25.25;
float horizontalOffset = 8;
const float verticalOffset = 1;
const float vertBoxDistance = 18.9;
const float slippageFactor = 0.03;

// Photointerrupter sensors (from robot_device.ino configuration)
const int horizSensor = 2;  // Digital pin D2
const int vertSensor = 3;   // Digital pin D3

// Motor control pins - using exact same pins as updated robot_device_testing.ino
int stepPinX = 5;   // Horizontal motor step (PUL+) - Motor 1
int dirPinX = 7;    // Horizontal motor direction (DIR+)  
int enblPinX = 6;   // Horizontal motor enable (ENBL+)

int stepPinY = 8;   // Vertical motor step (PUL+) - Motor 2
int dirPinY = 10;   // Vertical motor direction (DIR+)
int enblPinY = 9;   // Vertical motor enable (ENBL+)

// Direction constants - need to determine correct directions for this robot
boolean right = LOW;   // Test this direction
boolean left = HIGH;   // Test this direction  
boolean up = LOW;      // Test this direction - you said it's moving up when should go down
boolean down = HIGH;   // Test this direction

// State variables
int currentShelf = 1;

void setup() {
  Serial.begin(9600);
  
  // Motor 1 (horizontal) - pins 5, 7, 6
  pinMode(stepPinX, OUTPUT);   // Pin 5 - PUL+
  pinMode(dirPinX, OUTPUT);    // Pin 7 - DIR+
  pinMode(enblPinX, OUTPUT);   // Pin 6 - ENBL+
  
  // Motor 2 (vertical) - pins 8, 10, 9
  pinMode(stepPinY, OUTPUT);   // Pin 8 - PUL+
  pinMode(dirPinY, OUTPUT);    // Pin 10 - DIR+
  pinMode(enblPinY, OUTPUT);   // Pin 9 - ENBL+
  
  // Initialize both motors - exact same as robot_device_testing.ino
  digitalWrite(stepPinX, LOW);   digitalWrite(stepPinY, LOW);   // PUL starts low
  digitalWrite(dirPinX, LOW);    digitalWrite(dirPinY, LOW);    // Direction 1
  digitalWrite(enblPinX, HIGH);  digitalWrite(enblPinY, HIGH);  // Start disabled (HIGH = disabled)
  
  // Configure photointerrupter sensors as INPUT_PULLUP
  pinMode(horizSensor, INPUT_PULLUP);
  pinMode(vertSensor, INPUT_PULLUP);
  
  Serial.println("Robot movement test starting...");
  Serial.println("Motors disabled, sensors configured");
  Serial.println("Ready to start cycles...");
}

void loop() {
  // Enable motors and calibrate
  Serial.println("Enabling motors...");
  digitalWrite(enblPinX, LOW);  digitalWrite(enblPinY, LOW);  // Enable both motors
  
  Serial.println("Starting calibration...");
  calibrate();
  Serial.println("Calibration complete");
  
  Serial.println("Starting movement cycle...");
  
  // Run through shelves
  for (currentShelf = 1; currentShelf <= numShelves; currentShelf++) {
    Serial.print("Moving across shelf ");
    Serial.println(currentShelf);
    horizontalRun(currentShelf);
    
    if (currentShelf < numShelves) {
      Serial.print("Moving to shelf ");
      Serial.println(currentShelf + 1);
      verticalRun(currentShelf);
    }
  }
  
  // Reset for next cycle
  currentShelf = 1;
  digitalWrite(dirPinY, down);
  digitalWrite(dirPinX, left);
  
  Serial.println("Returning to home position...");
  calibrate();
  
  Serial.println("Cycle complete. Disengaging motors...");
  digitalWrite(enblPinX, HIGH);  digitalWrite(enblPinY, HIGH);  // Disable both motors
  
  Serial.println("Waiting 10 seconds before next cycle...");
  delay(10000);
}

void calibrate() {
  Serial.println("Calibrating to home position...");
  
  // Step 1: Move up and right for 2 seconds (ignoring photointerrupters)
  Serial.println("Moving up and right for 2 seconds...");
  digitalWrite(dirPinY, up);    // up = LOW 
  digitalWrite(dirPinX, right); // right = LOW
  
  unsigned long startTime = millis();
  while (millis() - startTime < 2000) { // 2 seconds
    // Use exact same step timing as robot_device_testing.ino
    digitalWrite(stepPinY, HIGH);
    digitalWrite(stepPinX, HIGH);
    delayMicroseconds(1000);
    digitalWrite(stepPinY, LOW);
    digitalWrite(stepPinX, LOW);
    delayMicroseconds(1000);
  }
  
  Serial.println("Now moving down and left until photointerrupters are triggered...");
  
  // Step 2: Move down and left until both photointerrupters are triggered (LOW)
  digitalWrite(dirPinY, down);  // down = HIGH
  digitalWrite(dirPinX, left);  // left = HIGH
  
  // Move until both sensors are triggered (LOW state)
  int stepCount = 0;
  while ((digitalRead(horizSensor) == HIGH) || (digitalRead(vertSensor) == HIGH)) {
    digitalWrite(stepPinY, HIGH);
    digitalWrite(stepPinX, HIGH);
    delayMicroseconds(1000);  // Same timing as robot_device_testing.ino
    digitalWrite(stepPinY, LOW);
    digitalWrite(stepPinX, LOW);
    delayMicroseconds(1000);
    
    // Debug output every 1000 steps
    stepCount++;
    if (stepCount % 1000 == 0) {
      Serial.print("Calibration step ");
      Serial.print(stepCount);
      Serial.print(" - Horiz sensor: ");
      Serial.print(digitalRead(horizSensor));
      Serial.print(", Vert sensor: ");
      Serial.println(digitalRead(vertSensor));
    }
    
    // Safety timeout after 10000 steps
    if (stepCount > 10000) {
      Serial.println("Calibration timeout - stopping");
      break;
    }
  }
  
  Serial.println("Both photointerrupters triggered - home position reached");
}

void verticalRun(int shelfIndex) {
  Serial.print("Moving vertically to shelf ");
  Serial.println(shelfIndex + 1);
  
  digitalWrite(dirPinY, up);
  
  long steps = calcVerticalSteps(vertBoxDistance + slippageFactor * shelfIndex);
  for (long x = 0; x < steps; x++) {
    digitalWrite(stepPinY, HIGH);
    delayMicroseconds(100);
    digitalWrite(stepPinY, LOW);
    delayMicroseconds(100);
  }
}

void horizontalRun(int shelfIndex) {
  Serial.print("Starting horizontal run on shelf ");
  Serial.println(shelfIndex);
  
  // Take position at first box (no actual picture, just simulate)
  delay(500); // Simulate picture delay
  
  int boxCount = 1;
  digitalWrite(dirPinX, !digitalRead(dirPinX)); // Snake pattern direction
  
  // Move across remaining boxes
  while (boxCount < numBoxesPerShelf) {
    Serial.print("Moving to box ");
    Serial.println(boxCount + 1);
    
    long steps = calcHorizontalSteps(horizBoxDistance);
    for (long x = 0; x < steps; x++) {
      digitalWrite(stepPinX, HIGH);
      delayMicroseconds(100);
      digitalWrite(stepPinX, LOW);
      delayMicroseconds(100);
    }
    
    // Simulate taking picture
    delay(500);
    boxCount += 1;
  }
}

long calcHorizontalSteps(float distanceCm) {
  float stepsToCmRatio = 20000.0 / 9.4; // From original robot_device.ino
  return (long)(stepsToCmRatio * distanceCm);
}

long calcVerticalSteps(float distanceCm) {
  float stepsToCmRatio = 25000.0 / 12.0; // From original robot_device.ino  
  return (long)(stepsToCmRatio * distanceCm);
}
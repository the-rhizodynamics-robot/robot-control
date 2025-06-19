// Robot snake pattern movement with time-based control
// Camera gantry for taking photos of shelves

// VARIABLES
int numShelves = 2; // Change this to set how many shelves to photograph

// Photointerrupter sensors 
const int horizSensor = 3;  // Digital pin D3
const int vertSensor = 2;   // Digital pin D2

// Motor control pins
int stepPinX = 5;   // Horizontal motor step
int dirPinX = 7;    // Horizontal motor direction  
int enblPinX = 6;   // Horizontal motor enable

int stepPinY = 8;   // Vertical motor step
int dirPinY = 10;   // Vertical motor direction
int enblPinY = 9;   // Vertical motor enable

// Direction constants
boolean right = LOW;
boolean left = HIGH;
boolean up = LOW;   // LOW is up
boolean down = HIGH; // HIGH is down

// State variables
bool calibrated = false;
int currentShelf = 0;

void setup() {
  Serial.begin(9600);
  
  // Configure horizontal motor pins
  pinMode(stepPinX, OUTPUT);
  pinMode(dirPinX, OUTPUT);
  pinMode(enblPinX, OUTPUT);
  digitalWrite(stepPinX, LOW);
  digitalWrite(dirPinX, right);
  digitalWrite(enblPinX, LOW); // Enable motor (LOW = enabled)
  
  // Configure vertical motor pins  
  pinMode(stepPinY, OUTPUT);
  pinMode(dirPinY, OUTPUT);
  pinMode(enblPinY, OUTPUT);
  digitalWrite(stepPinY, LOW);
  digitalWrite(dirPinY, up);
  digitalWrite(enblPinY, LOW); // Enable motor (LOW = enabled)
  
  // Configure photointerrupter sensors (digital pins)
  pinMode(horizSensor, INPUT);
  pinMode(vertSensor, INPUT);
  
  Serial.println("Camera gantry starting...");
  Serial.print("Will photograph ");
  Serial.print(numShelves);
  Serial.println(" shelves");
}

void loop() {
  if (!calibrated) {
    Serial.println("Starting calibration sequence...");
    calibrate();
    calibrated = true;
    Serial.println("Calibration complete. Starting photography sequence...");
  }
  
  // Run snake pattern for all shelves
  for (currentShelf = 1; currentShelf <= numShelves; currentShelf++) {
    Serial.print("Photographing shelf ");
    Serial.println(currentShelf);
    
    if (currentShelf % 2 == 1) {
      // Odd shelves: go right
      Serial.println("Moving right for 30 seconds...");
      moveHorizontal(right, 10000);
    } else {
      // Even shelves: go left
      Serial.println("Moving left for 30 seconds...");
      moveHorizontal(left, 10000);
    }
    
    // Move up to next shelf (except after last shelf)
    if (currentShelf < numShelves) {
      Serial.println("Moving up for 8 seconds...");
      moveVertical(up, 8000);
    }
  }
  
  Serial.println("Photography sequence complete!");
  Serial.println("Returning to start position...");
  
  // Return to start - go back down and left
  returnToStart();
  
  Serial.println("Disengaging motors for 2 minutes...");
  digitalWrite(enblPinX, HIGH); // Disable horizontal motor
  digitalWrite(enblPinY, HIGH); // Disable vertical motor
  
  delay(120000); // 2 minutes = 120,000 milliseconds
  
  Serial.println("Re-engaging motors...");
  digitalWrite(enblPinX, LOW);  // Re-enable horizontal motor
  digitalWrite(enblPinY, LOW);  // Re-enable vertical motor
  
  Serial.println("Starting next cycle...");
  
  // Reset for next cycle
  calibrated = false;
  currentShelf = 0;
}

void calibrate() {
  Serial.println("Calibration step 1: Moving right for 5 seconds...");
  moveHorizontal(right, 5000);
  
  Serial.println("Calibration step 2: Moving up for 5 seconds...");
  moveVertical(up, 5000);
  
  Serial.println("Calibration step 3: Finding photointerrupters...");
  
  // Set directions to go down and left
  digitalWrite(dirPinY, down);
  digitalWrite(dirPinX, left);
  
  // Move down and left until both photointerrupters are triggered
  bool verticalTriggered = false;
  bool horizontalTriggered = false;
  
  while (!verticalTriggered || !horizontalTriggered) {
    // Check vertical sensor
    if (digitalRead(vertSensor) == LOW && !verticalTriggered) {
      Serial.println("Vertical photointerrupter triggered - stopping vertical movement");
      verticalTriggered = true;
    }
    
    // Check horizontal sensor
    if (digitalRead(horizSensor) == LOW && !horizontalTriggered) {
      Serial.println("Horizontal photointerrupter triggered - stopping horizontal movement");
      horizontalTriggered = true;
    }
    
    // Step motors that haven't hit their sensors yet
    if (!verticalTriggered) {
      digitalWrite(stepPinY, HIGH);
    }
    if (!horizontalTriggered) {
      digitalWrite(stepPinX, HIGH);
    }
    
    delayMicroseconds(100);
    
    if (!verticalTriggered) {
      digitalWrite(stepPinY, LOW);
    }
    if (!horizontalTriggered) {
      digitalWrite(stepPinX, LOW);
    }
    
    delayMicroseconds(100);
  }
  
  Serial.println("Both photointerrupters triggered - calibration complete");
  
  // Set initial directions for photography sequence
  digitalWrite(dirPinX, right);
  digitalWrite(dirPinY, up);
}

void moveHorizontal(boolean direction, unsigned long duration) {
  digitalWrite(dirPinX, direction);
  
  unsigned long startTime = millis();
  while(millis() - startTime < duration) {
    digitalWrite(stepPinX, HIGH);
    delayMicroseconds(100);
    digitalWrite(stepPinX, LOW);
    delayMicroseconds(100);
  }
}

void moveVertical(boolean direction, unsigned long duration) {
  digitalWrite(dirPinY, direction);
  
  unsigned long startTime = millis();
  while(millis() - startTime < duration) {
    digitalWrite(stepPinY, HIGH);
    delayMicroseconds(100);
    digitalWrite(stepPinY, LOW);
    delayMicroseconds(100);
  }
}

void returnToStart() {
  Serial.println("Returning to home position using photointerrupters...");
  
  // Set directions to go down and left
  digitalWrite(dirPinY, down);
  digitalWrite(dirPinX, left);
  
  // Move down and left until both photointerrupters are triggered
  bool verticalTriggered = false;
  bool horizontalTriggered = false;
  
  while (!verticalTriggered || !horizontalTriggered) {
    // Check vertical sensor
    if (digitalRead(vertSensor) == LOW && !verticalTriggered) {
      Serial.println("Vertical photointerrupter triggered - stopping vertical movement");
      verticalTriggered = true;
    }
    
    // Check horizontal sensor
    if (digitalRead(horizSensor) == LOW && !horizontalTriggered) {
      Serial.println("Horizontal photointerrupter triggered - stopping horizontal movement");
      horizontalTriggered = true;
    }
    
    // Step motors that haven't hit their sensors yet
    if (!verticalTriggered) {
      digitalWrite(stepPinY, HIGH);
    }
    if (!horizontalTriggered) {
      digitalWrite(stepPinX, HIGH);
    }
    
    delayMicroseconds(100);
    
    if (!verticalTriggered) {
      digitalWrite(stepPinY, LOW);
    }
    if (!horizontalTriggered) {
      digitalWrite(stepPinX, LOW);
    }
    
    delayMicroseconds(100);
  }
  
  Serial.println("Both photointerrupters triggered - back at home position");
} 
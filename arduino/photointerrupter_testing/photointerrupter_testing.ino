const int photoPin1 = 2;
const int photoPin2 = 3;
const int ledPin = LED_BUILTIN;

void setup() {
  Serial.begin(9600);
  pinMode(photoPin1, INPUT_PULLUP);
  pinMode(photoPin2, INPUT_PULLUP);
  pinMode(ledPin, OUTPUT);
  
  Serial.println("Photointerrupter Test Starting...");
  Serial.println("Pin 2: Photointerrupter 1");
  Serial.println("Pin 3: Photointerrupter 2");
  Serial.println("LED will light when either is triggered");
}

void loop() {
  int sensor1 = digitalRead(photoPin1);
  int sensor2 = digitalRead(photoPin2);
  
  bool triggered = (sensor1 == LOW || sensor2 == LOW);
  
  digitalWrite(ledPin, triggered ? HIGH : LOW);
  
  if (sensor1 == LOW) {
    Serial.println("Photointerrupter 1 TRIGGERED");
  }
  
  if (sensor2 == LOW) {
    Serial.println("Photointerrupter 2 TRIGGERED");
  }
  
  delay(100);
}
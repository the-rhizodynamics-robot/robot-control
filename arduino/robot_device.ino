// KILL CODE AGREED ON WITH ARDUINO
const int KILLCODE = 2048;

// VARIABLES SET BY PYTHON SCRIPT
int numShelves = 2;           //number of shelves
int lightCycleLengthHours = 24;   //can alter the hours of day length. Set to 0 to have total darkness
float dayLengthHours = 5;         //keep a running tab of time since start of current light cycle in order to create diurnal light cylces

//Time
int cycleTimeMins = 30;   //length of time for desired cycles in mins
unsigned long cycleTimeMillis;      //the cycle time converted to milliseconds
unsigned long cycleDelay;           //stores the amount of delay, in millis, necessary for the cycle to take 15 mins
unsigned long timeRun;          //stores number of millis run so far for the current cycle

//Horizontal user input
float horizBoxDistance = 25.25; //7.62;    //horizontal distance between boxes in cm
int numBoxesPerShelf = 4;          //number of boxes per shelf
float horizontalOffset = 8;       //horizontal offset from home position in cm

//Vertical user input
const float verticalOffset = 1;   //offset in cm?
const float vertBoxDistance = 18.9;   //vertical distance between boxes in cm
const float slippageFactor = 0.03;    //the vertical distance varies slightly from shelf to shelf, this helps keep the camera centered

// shelves
const int shelfPinArray[] = {23, 25, 27, 29, 31, 33}; //array of shelf relay pins [0]=shelf 1, [1]=shelf 2...

//Optic Sensor
const int horizSensor = A0;         //horizontal photointerrupter
const int vertSensor = A1;          //vertical photointerrupter


//Other variables
bool first = true;
int led = 2;                //used for the led clash
int currentShelf = 1;

//Camera
const int camera = 7;   //defines the camera trigger pin

//LED relay
const int led_strip = 4;

//timer
unsigned long time1;     //used to mark the time the cycle starts
unsigned long time2;     //used to mark the time the camera gets back to home after going through all the boxes
unsigned long timeToRun;


//HORIZONTAL MOTOR DRIVER PINS
int stepPinX = 11;  //PUL- pin number
int dirPinX = 12;  //DIR- pin number
int enblPinX = 13;  //ENBL+ pin number
boolean right = HIGH;       //setting dirPinX to HIGH causes the camera to move right
boolean left = LOW;         //setting the dirPinX to LOW causes the camera to move left


//VERTICAL MOTOR DRIVER PINS
int stepPinY = 8;  //PUL- pin number
int dirPinY = 9;  //DIR- pin number
int enblPinY = 10;  //ENBL+ pin number
boolean up = HIGH;       //setting dirPinY to HIGH causes the camera to move up
boolean down = LOW;         //setting the dirPinY to LOW causes the camera to move down




void setup() {

  //  //Starts serial communication
  Serial.begin(9600); // use the same baud-rate as the python side
  Serial.setTimeout(2); // dont change, this is needde for the robot to read a two digit number from python

  //  Set variables from python script
  numShelves = waitForValue();  // waits for python to send the number of shelves
  lightCycleLengthHours = waitForValue(); // waits for python to send the number of light hours in a day
  dayLengthHours = float(waitForValue());  // waits for python to send the hour in to the day cycle
  // waits for one final input, can be anything that isnt the killcode, before starting
  waitForValue();

  //Horizontal Define pins as outputs and enables motor driver
  pinMode(stepPinX, OUTPUT);       //defines as output
  pinMode(dirPinX, OUTPUT);        //defines as output
  pinMode(enblPinX, OUTPUT);       //defines as output
  digitalWrite(stepPinX, LOW); //starts with not moving
  digitalWrite(dirPinX, left);  //starts with the left direction
  digitalWrite(enblPinX, HIGH); //enables driver

  //Vertical Define pins as outputs and enables motor driver
  pinMode(stepPinY, OUTPUT);       //defines as output
  pinMode(dirPinY, OUTPUT);        //defines as output
  pinMode(enblPinY, OUTPUT);       //defines as output
  digitalWrite(stepPinY, LOW); //starts with not moving
  digitalWrite(dirPinY, up);  //starts with the up direction
  digitalWrite(enblPinY, HIGH); //enables driver

  //Define photointerrupter sensor pins as inputs
  pinMode(horizSensor, INPUT);
  pinMode(vertSensor, INPUT);

  //define camera pin
  pinMode(camera, OUTPUT);

  //led
  pinMode(led, OUTPUT);
  digitalWrite(led, LOW);

  //LED relay
  pinMode(led_strip, OUTPUT);

  //define shelf relay pins as outputs
  bool startOn = dayLengthHours > lightCycleLengthHours;
  for (int i = 0; i < numShelves; i++) {
    pinMode(shelfPinArray[i], OUTPUT);
    digitalWrite(shelfPinArray[i], startOn);
    delay(100);
  }

  //calc cycle time in millis
  cycleTimeMillis = cycleTimeMins * 60000;
}

void loop() {

  //uncomment this to just turn on the lights without camera moving
  //  while(true) {
  //
  //  }


  if (first) {
    checkKillSignal();
    calibrate(5000, true); //calibrate at initial position
    first = false; // no longer the first cycle

    //this code will hold the camera at the home position on the first cycle until a start signal is received from the monitoring program.
    //    while(true) {
    //      if(signal_received()){
    //        serialFlush();
    //        break;
    //      }
    //    }
  }

  // checks if a kill signal was received
  checkKillSignal();

  time1 = millis();

  for (currentShelf = 1; currentShelf <= numShelves; currentShelf++) {    //runs through the number of shelves
    horizontalRun(currentShelf);        //calls the horizontal run method to run through the entire shelf
    if (currentShelf < numShelves) {
      verticalRun(currentShelf); //calls the vertical run method to move to the next shelf
    }
  }
  currentShelf = 1;
  //at the end of the above for loop it is at the top shelf and needs to do one more horizontal run
  digitalWrite(dirPinY, down);   //resets the bot to go down
  digitalWrite(dirPinX, left); //resets bot to go left


  //finish cycle by calculating cycle end time, sending "home" via serial port

  calibrate(5000, true);

  delay(5000); //try delaying for a pause to allow all photos to be saved. Prevent errant emails about not enough images taken

  Serial.write("home");
  Serial.flush();

  //Calculating time to run
  time2 = millis(); //marks the time at the end of the cycle
  timeToRun = time2 - time1;        //calculates the number of milliseconds to complete the run-through
  cycleDelay = cycleTimeMillis - timeToRun;     //calculates the number of millis to delay such that the cycle is exactly the set cycleTime

  // time to run is the number of milliseconds left to run before the ycle is complete
  // this loop will run until the cycle is complete, delaying for a second, checking for kill code,
  // and then calculating the new number of milliseconds left
  while (timeToRun < cycleTimeMillis) {
    checkKillSignal();
    delay(1000);
    time2 = millis(); //marks the time at the end of the cycle
    timeToRun = time2 - time1;        //calculates the number of milliseconds to complete the run-through
  }
  dayLengthHours = dayLengthHours + float(cycleTimeMins) / 60.0;  //record total run time in hours

  //reset day measurement
  if (dayLengthHours > 24) {
    dayLengthHours = 0;
  }
}


/**
   Calibration method
   Ensures the camera starts at the same home point it will start for every cycle
   @steps is an int of the number of steps to go up and left before going home
   @includeOffset is a boolean where with true it will go down and lef the specific offset (see global variable) and false it will just stay home
*/

void calibrate(int steps, boolean includeOffset) {
  //initial calibration
  digitalWrite(dirPinY, up);   //sets camera to move up
  digitalWrite(dirPinX, left);  //sets camera to move left

  //move up and left for passed steps steps
  for (long x = 0; x < steps; x++) {
    digitalWrite(stepPinY, HIGH);
    digitalWrite(stepPinX, HIGH);
    delayMicroseconds(100);
    digitalWrite(stepPinY, LOW);
    digitalWrite(stepPinX, LOW);
    delayMicroseconds(100);
  }
  delay(500);       //quick .5 second delay

  digitalWrite(dirPinY, down);   //resets the bot to go down
  digitalWrite(dirPinX, right); //resets the bot to go right

  //loop for the motor to keep going right and down while both photointerrupters aren't triggered
  while ((digitalRead(horizSensor) == HIGH) && (digitalRead(vertSensor) == HIGH)) {
    digitalWrite(stepPinY, HIGH);
    digitalWrite(stepPinX, HIGH);
    delayMicroseconds(100);
    digitalWrite(stepPinY, LOW);
    digitalWrite(stepPinX, LOW);
  }

  //loop to keep going right while horizontal photointerrupter isn't triggered
  while (digitalRead(horizSensor) == HIGH) {
    digitalWrite(stepPinX, HIGH);
    delayMicroseconds(100);
    digitalWrite(stepPinX, LOW);
    delayMicroseconds(100);
  }

  //loop to keep going down while vertical interrupter isn't triggered
  while (digitalRead(vertSensor) == HIGH) {
    digitalWrite(stepPinY, HIGH);
    delayMicroseconds(100);
    digitalWrite(stepPinY, LOW);
    delayMicroseconds(100);
  }

  if (includeOffset) {
    //This will position the robot at the correct initial horizontal position
    digitalWrite(dirPinX, !digitalRead(dirPinX)); //writes X direction opposite to that of previous run - this creates the snake path

    for (long x = 0; x < calcHorizontalSteps(horizontalOffset); x++) {
      digitalWrite(stepPinX, HIGH);
      delayMicroseconds(100);
      digitalWrite(stepPinX, LOW);
      delayMicroseconds(100);
    }

    //This will position the robot at the correct initial horizontal position
    digitalWrite(dirPinY, down); //writes X direction opposite to that of previous run - this creates the snake path

    for (long x = 0; x < calcVerticalSteps(verticalOffset); x++) {
      digitalWrite(stepPinY, HIGH);
      delayMicroseconds(100);
      digitalWrite(stepPinY, LOW);
      delayMicroseconds(100);
    }
  }

  digitalWrite(dirPinX, !digitalRead(dirPinX)); //writes X direction opposite to that of previous run - this creates the snake path
  digitalWrite(dirPinY, up);

}


/**
   Vertical Shelf one run
   Moves the camera to the next shelf
*/
void verticalRun(int shelfIndex) {
  checkKillSignal();
  digitalWrite(dirPinY, up);    //sets the bot to go up

  //loop to move vertically as many steps as vertical box distance with the slippage factor included
  for (long x = 0; x < calcVerticalSteps((vertBoxDistance + slippageFactor * shelfIndex)); x++) {
    digitalWrite(stepPinY, HIGH);
    delayMicroseconds(100);
    digitalWrite(stepPinY, LOW);
    delayMicroseconds(100);
  }
  checkKillSignal();
}

/**
   Horizontal Shelf one run
   Moves across the shelf and stops and takes a picture at every box
*/
void horizontalRun(int shelfIndex) {
  picture(shelfIndex);        //calls the picture method to take a pic of the first box
  int boxCount = 1;        //starting on the first box for this shelf
  digitalWrite(dirPinX, !digitalRead(dirPinX)); //writes X direction opposite to that of previous run - this creates the snake path

  //loop that runs for as many boxes on a shelf
  while (boxCount < numBoxesPerShelf) {
    checkKillSignal();
    for (long x = 0; x < calcHorizontalSteps(horizBoxDistance); x++) {
      digitalWrite(stepPinX, HIGH);
      delayMicroseconds(100);
      digitalWrite(stepPinX, LOW);
      delayMicroseconds(100);
    }
    checkKillSignal();
    picture(shelfIndex);      //takes a pic of current box
    boxCount += 1;     //increases the box counter
  }
}


/**
   Returns the necessary number of steps to move horizontally the input distance
*/
long calcHorizontalSteps(float distanceCm) {
  long result;
  float stepsToCmRatio = 20000 / 9.4; //this is a ratio that was determined by running the motor horizontally for 20,000 steps and oberving it moving 9.4 cm

  result = stepsToCmRatio * distanceCm;
  return result;
}

/**
   Returns the necessary number of steps to move vertically the input distance
*/
long calcVerticalSteps(float distanceCm) {
  long result;
  int stepsToCmRatio = 25000 / 12; //this is a ratio that was determined by running the motor horizontally for 25,000 steps and oberving it moving 12 cm

  result = stepsToCmRatio * distanceCm;

  return result;

}

/**
   Takes a picture
*/
void picture(int i) { //Use if flash is implemented
  //digitalWrite(led_strip, HIGH); //send signal to turn on lights all the time. Will only do anything if it's night-time hours and lights are turned off
  digitalWrite(shelfPinArray[i - 1], LOW);
  delay(3000);
  digitalWrite(camera, HIGH); // triggers camera
  delayMicroseconds(100); //delays 100 microseconds
  digitalWrite(camera, LOW);
  delay(500);

  //turn light off if it is nighttime
  if (dayLengthHours >= (lightCycleLengthHours)) {
    //digitalWrite(led_strip, LOW);
    digitalWrite(shelfPinArray[i - 1], HIGH);
  }
}


// checks if a kill signal was recived and stops if received
void checkKillSignal() {
  if (Serial.available() > 0) {
    int val = Serial.readString().toInt();
    if (val != KILLCODE) {
      Serial.print(val);
    } else {
      for (int i = 0; i < numShelves; i++) {
        pinMode(shelfPinArray[i], OUTPUT);
        digitalWrite(shelfPinArray[i], HIGH);
        delay(100);
      }
      int calSteps; //determines how many steps to go up and left before calibrating to home
      if (currentShelf == 1) {
        calSteps = 5000;  // if it's on the first shelf, it needs to go up so as to pass beyond the photointerrupter before going home
      } else {
        calSteps = 0;
      }
      calibrate(calSteps, false);
      while (1);
    }
  }
}

void serialFlush() {
  while (Serial.available() > 0) {
    char t = Serial.read();
  }
}

// sits and waits for an int from the Serial
// this value, if not the killcode, will be returned
// if killcode is received, the robot will call setup
int waitForValue() {
  while (!Serial.available());
  int val = Serial.readString().toInt();
  if (val != KILLCODE) {
    Serial.print(val);
    Serial.flush();
    return val;
  } else {
    Serial.print("Sent kill code, resetting");
    Serial.flush();
    setup();
  }
}
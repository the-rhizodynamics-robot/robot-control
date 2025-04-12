//Time
int cycleTimeMins = 15;   //length of time for desired cycles in mins
unsigned long cycleTimeMillis;      //the cycle time converted to milliseconds
unsigned long cycleDelay;           //stores the amount of delay, in millis, necessary for the cycle to take 15 mins
unsigned long timeRun;          //stores number of millis run so far for the current cycle
float dayLengthHours = 8;       //How long the lights have been on. Setting to 0 starts run at "sunrise"
int lightCycleLengthHours = 16;  //How long per day the lights should be on. 

//Horizontal user input
float horizBoxDistance = 12.8;    //horizontal distance between boxes in cm. rice box = 7.62
int numBoxesPerShelf = 4;          //number of boxes per shelf

//Vertical user input
const float vertBoxDistance = 27;   //vertical distance between boxes in cm
const int numShelves = 1;           //number of shelves
const float slippageFactor = 0.25;    //the vertical distance varies slightly from shelf to shelf, this helps keep the camera centered

//Optic Sensor
const int horizSensor = A1;         //horizontal photointerrupter
const int vertSensor = A0;          //vertical photointerrupter

//LED relay
const int led_strip = 4;

//Other variables
bool flash = false;         //flash the LED? Use only in "dark mode"
bool first = true;
int led = 2;                //used for the led clash

//Camera
const int camera = 5;   //defines the camera trigger pin


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
  pinMode(led_strip, OUTPUT);
    
  //calc cycle time in millis
  cycleTimeMillis = cycleTimeMins * 60000;
}

void loop() {
//
//  //for just having  lights on. comment out when running
//  digitalWrite(led_strip, HIGH);
//  while(true) {
//    
//  }

  
  if (first) {
    calibrate(5000); //calibrate at initial position
    first = false; // no longer the first cycle
//    while(true) {
//      if(signal_received()){
//        serialFlush();
//        break;
//      } 
//    }
  }

  //just try this. If any signal is received from serial port, it should kill.
  if(signal_received()){
    kill();
  }

  time1 = millis(); 
  
  //reset day measurement
  if (dayLengthHours >= 24) {
    dayLengthHours = 0;
  }
    
  for (int k = 1; k <= numShelves; k++) {    //runs through the number of shelves
    horizontalRun();        //calls the horizontal run method to run through the entire shelf
    if (k < numShelves) {
      verticalRun(k); //calls the vertical run method to move to the next shelf
    }
  }
   //at the end of the above for loop it is at the top shelf and needs to do one more horizontal run
  digitalWrite(dirPinY, down);   //resets the bot to go down
  digitalWrite(dirPinX, left); //resets bot to go left

  
  //finish cycle by calculating cycle end time, sending "home" via serial port
  time2 = millis(); //marks the time at the end of the cycle

  calibrate(200);
  
  delay(5000); //try delaying for a pause to allow all photos to be saved. Prevent errant emails about not enough images taken
  
  Serial.write("home");
  Serial.flush();
  
  //Calculating time to run
  timeToRun = time2 - time1;        //calculates the number of milliseconds to complete the run-through
  cycleDelay = cycleTimeMillis - timeToRun;     //calculates the number of millis to delay such that the cycle is exactly the set cycleTime
  dayLengthHours = dayLengthHours + float(cycleTimeMins)/60;    //record total run time in minutes

  delay(cycleDelay); //delays exact number of milliseconds necessary to make the cycle last for the variable cycleTime

}


/**
   Calibration method
   Ensures the camera starts at the same home point it will start for every cycle
*/

void calibrate(int steps) {
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
}


/**
   Vertical Shelf one run
   Moves the camera to the next shelf
*/
void verticalRun(int shelfIndex) {
  digitalWrite(dirPinY, up);    //sets the bot to go up

  //loop to move vertically as many steps as vertical box distance with the slippage factor included
  for (long x = 0; x < calcVerticalSteps((vertBoxDistance + slippageFactor * shelfIndex)); x++) {
    digitalWrite(stepPinY, HIGH);
    delayMicroseconds(100);
    digitalWrite(stepPinY, LOW);
    delayMicroseconds(100);
  }
}

/**
   Horizontal Shelf one run
   Moves across the shelf and stops and takes a picture at every box
*/
void horizontalRun() {
  picture(flash);        //calls the picture method to take a pic of the first box
  int boxCount = 1;        //starting on the first box for this shelf
  digitalWrite(dirPinX, !digitalRead(dirPinX)); //writes X direction opposite to that of previous run - this creates the snake path

  //loop that runs for as many boxes on a shelf
  while (boxCount < numBoxesPerShelf) {
    for (long x = 0; x < calcHorizontalSteps(horizBoxDistance); x++) {
      digitalWrite(stepPinX, HIGH);
      delayMicroseconds(100);
      digitalWrite(stepPinX, LOW);
      delayMicroseconds(100);
    }
    picture(flash);      //takes a pic of current box
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
void picture(boolean flash) { //Use if flash is implemented
  digitalWrite(led_strip, HIGH); //send signal to turn on lights all the time. Will only do anything if it's night-time hours and lights are turned off
  delay(1000);
  digitalWrite(camera, HIGH); // triggers camera
  delayMicroseconds(1000); //delays 100 microseconds
  digitalWrite(camera, LOW);
  delay(500);

  //turn light off if it is nighttime
  if(dayLengthHours >= (lightCycleLengthHours)) {
    digitalWrite(led_strip, LOW); 
  } 
}


boolean signal_received(){
  if(Serial.available() > 0){
    return true;
  }
  else{
    return false;
  }
}

void kill(){
  Serial.end();
  while(1){
  }
}

void serialFlush(){
  while(Serial.available() > 0) {
    char t = Serial.read();
  }
}  
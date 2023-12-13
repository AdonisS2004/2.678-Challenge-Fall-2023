//////////////////////////
//Strategy Documentation//
//////////////////////////

/*************************
  Sensors:
  - Reflectance Sensor
  - Motor Driver

  Objective: Linefollowing utilizing given sensors and PID Algorithm

  Goals:
  - Motor Variables and Drive Function (completed)
    - drive test (completed)
    - speed test (completed)
  - Set up infrared variables and monitor (completed)
  - normalize sensor values to positioning with map function (completed)
  - create PID algorithm to stick to sensor seet point (blocked)
      *Issue: left turns are not adjusting fine enough, working on adjusting around 150 Norm Speed and configuring KP Values
      *too far left: 3; too far right: 1
      *norm speed range: 100 - 170
  - Cases:
      *once it reach a cut line, what do you do?
      *90 degree turns?
      *infinite loop goes brr
*/

/////////////////////////////////////////////////////////////////////
// MOTOR DRIVE VARIABLES (Pololu #713 motor driver pin assignments)//
/////////////////////////////////////////////////////////////////////

// Pololu drive A (left motor) (positive is forward) [-255,255]
const int PWMA = 11;
const int AIN2 = 10;
const int AIN1 = 9;

const int STDBY = 8;

// Pololu drive B (right motor) (positive is forward) [-255,255]
const int BIN1 = 7;
const int BIN2 = 6;
const int PWMB = 5;

// motor constants
int LEFT_NORMAL_SPEED = 140;
int RIGHT_NORMAL_SPEED = 140;
const int MAX_MOTOR_SPEED = 255;
float RMSPEED, LMSPEED;

/////////////////////////////////
// REFLECTANCE SENSOR VARIABLES//
/////////////////////////////////
#define IR1_PIN A1
#define IR2_PIN A2
#define IR3_PIN A3

// variables for stored sensor values
int IR1Val, IR2Val, IR3Val;

// sensor intensities
float maximum_intensities[] = {554, 447, 514}; // less reflectance; more black; //(771 776 779) parallel to the line on the floor
float minimum_intensities[] = {90, 35, 45}; // high reflectance; more white
float normalized_intensities[3]; // empty varibale to hold normalized values

/////////////////
//PID VARIABLES//
/////////////////
float SETPOINT = 2.0;
float previousSensorLocation = 2;

// PID constants 
const float KP_LEFT = 140; // left motor proportional gain; prev: 140 (worked)
const float KI_LEFT = 150; // left motor integral gain prev: 150 (worked)
const float KD_LEFT = 0.42; // left motor derivative gain prev: 0.42 (worked)

const float KP_RIGHT = 140; // right motor proportional gain; prev: 140 (worked)
const float KI_RIGHT = 150;// right motor integral gain prev: 150 (worked)
const float KD_RIGHT = 0.42; // right motor derivative prev: 0.42 (worked)

const float DELTA_TIME = 1; // time in milliseconds

// PID variables
float error = 0.0;
float previousError = 0.0;
float leftIntegral = 0.0;
float leftDerivative = 0.0;
float rightIntegral = 0.0;
float rightDerivative = 0.0;

// time variables/conversions
long currentMillis = 0;
long previousMillis = 0;
#define MILLISEC_TO_SEC 1/1000 // to convert from milliseconds to seconds

//////////////////////////////////////
//OBSTACLE COURSE SPECIFIC VARIABLES//
//////////////////////////////////////

int stage = 1;
int degree_count = 0;
int stage_timer = 0;
int previous_stage_timer = 0;
bool isTurned = false;
bool stageSixBool = false;
int stage_one_time = 22700;
int stage_two_time = 8500;
int stage_three_time = 2700;
int stage_five_time = 9500;
int stage_six_time = 4500;

#define LED1 2
#define LED2 3
#define LED3 4

void setup() {
  Serial.begin(9600);
  Serial.println("Serial Setup Begin...");

  // motor setup
  pinMode(PWMA , OUTPUT);
  pinMode(AIN1 , OUTPUT);
  pinMode(AIN2 , OUTPUT);
  pinMode(BIN1 , OUTPUT);
  pinMode(BIN2 , OUTPUT);
  pinMode(PWMB , OUTPUT);
  pinMode(STDBY , OUTPUT);
  digitalWrite(STDBY , HIGH);

  // IR setup
  pinMode(IR1_PIN, INPUT);
  pinMode(IR2_PIN, INPUT);
  pinMode(IR3_PIN, INPUT);

  // LED setup
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);
}

void loop() {
  //////////////////////
  //SENSOR VALUE SETUP//
  //////////////////////
  
  currentMillis = millis();
  stage_timer = millis();
  int intensities[] = {analogRead(IR1_PIN), analogRead(IR2_PIN), analogRead(IR3_PIN)};
  
  // Add normalized values
  for(int x = 0; x < 3; x++){
    normalized_intensities[x] = computeNormVal(intensities[x], minimum_intensities[x], maximum_intensities[x]);
  }
  
  float sensorLocation = computeSensorXCM(normalized_intensities);

  // Stage 1: From Start Past the Barcode 
  if(stage == 1){
    stage_one(sensorLocation);
    if(stage_timer - previous_stage_timer > stage_one_time){
      stage = 2;
      previous_stage_timer = stage_timer;
    }
  }

  // Stage 2: from Hairpin turn to the beginning of the missing lines
  if(stage == 2){
    stage_two(sensorLocation);
    if(stage_timer - previous_stage_timer > stage_two_time){
      stage = 3;
      previous_stage_timer = stage_timer;
    }
  }

  // Stage 3: From the squiggly to the end of dashed lines
  if(stage == 3){
    stage_three(sensorLocation);
    if(stage_timer - previous_stage_timer > stage_three_time){
      stage = 4;
      previous_stage_timer = stage_timer;
    }
  }

  // Stage 4: From end of dashed lines to the end of the course
  if(stage == 4){
    stage_four(sensorLocation);
    if(degree_count == 2){
      stage = 5;
      previous_stage_timer = stage_timer;
    }
  }

  if (stage == 5){
    stage_five(sensorLocation);
    if(stage_timer - previous_stage_timer > stage_five_time){
      stage = 6;
      previous_stage_timer = stage_timer;
    }
  }

  if(stage == 6){
    stage_six(sensorLocation);
  }
}


////////////////////
//Helper Functions//
////////////////////

// The course split beteween stages
// Stage 1: From Start Past the Barcode 
void stage_one(float sensorLocation){
  if ((currentMillis - previousMillis >= DELTA_TIME) ) {
    // regular control
    if(sensorLocation == 0){
      drive(140, 140);
    } else {
      drivePID(sensorLocation, SETPOINT, DELTA_TIME);
      drive(RMSPEED, LMSPEED);
      // Serial.println("PID");
    }
      previousMillis = currentMillis;
      // Serial.println("PID");
      digitalWrite(LED1, HIGH);
      digitalWrite(LED2, LOW);
      digitalWrite(LED3, LOW);
  }
}

// Stage 2: From the Barcode to the dashed line
void stage_two(float sensorLocation){
  if ((currentMillis - previousMillis >= DELTA_TIME)){
    // regular control
    if(previousSensorLocation > 2 && normalized_intensities[0] < 0.1 && normalized_intensities[1] < 0.1 && normalized_intensities[2] > 0.4){
      forceRight(); // Force Right
      drive(RMSPEED, LMSPEED);
     // Serial.println("FORCE RIGHT");
    } else if(normalized_intensities[0] < 0.1 && normalized_intensities[1] < 0.1 && normalized_intensities[2] < 0.1) {
      forceRight(); // Force Right
      drive(RMSPEED, LMSPEED);
     // Serial.println("FORCE RIGHT");
    } else if(previousSensorLocation < 2 && normalized_intensities[0] > 0.4 && normalized_intensities[1] < 0.1 && normalized_intensities[2] < 0.1){
      forceLeft(); // Force Left
      drive(RMSPEED, LMSPEED);
     // Serial.println("FORCE LEFT");
    } else {     
      drivePID(sensorLocation, SETPOINT, DELTA_TIME);
      drive(RMSPEED, LMSPEED);
      previousMillis = currentMillis;
      previousSensorLocation = sensorLocation;
      // Serial.println("PID");
   }
  }
  digitalWrite(LED1, LOW);
  digitalWrite(LED2, HIGH);
  digitalWrite(LED3, LOW);
}

// Stage 3: From the squiggly to the end of dashed lines
void stage_three(float sensorLocation){
  if ((currentMillis - previousMillis >= DELTA_TIME)){
    if(sensorLocation < 1.3 || sensorLocation > 2.7){
      drive(140, 140);
      digitalWrite(LED2, HIGH);
    } else {
      drivePID(sensorLocation, SETPOINT, DELTA_TIME);
      drive(RMSPEED, LMSPEED);
      previousMillis = currentMillis;
      previousSensorLocation = sensorLocation;
      digitalWrite(LED2, LOW);
      // Serial.println("PID");
    }
  }
  digitalWrite(LED1, LOW);
  digitalWrite(LED3, HIGH);
}

// Stage Four: From Dashed Line to the end of the two right angles
void stage_four(float sensorLocation){
  if ((currentMillis - previousMillis >= DELTA_TIME)){
    if(sensorLocation == 0 && !(isTurned)){
        digitalWrite(LED2, LOW);
        digitalWrite(LED3, HIGH);
        isTurned = true;
        sharpLeft();
        drive(RMSPEED, LMSPEED);
        degree_count++;
        delay(500);
    } else if(sensorLocation == 0 && isTurned){
        digitalWrite(LED2, HIGH);
        digitalWrite(LED3, LOW);
        sharpRight();
        isTurned = false;
        drive(RMSPEED, LMSPEED);
        degree_count++;
        delay(500);
    } else {
        drivePID(sensorLocation, SETPOINT, DELTA_TIME);
        drive(RMSPEED, LMSPEED);
        previousMillis = currentMillis;
        previousSensorLocation = sensorLocation;
        digitalWrite(LED1, HIGH);
        digitalWrite(LED2, LOW);
        digitalWrite(LED3, LOW);
        // Serial.println("PID");
    }
  }
}

//Stage Five: From the end of the two right angles past the fork split
void stage_five(float sensorLocation){
  if ((currentMillis - previousMillis >= DELTA_TIME) ) {
    drivePID(sensorLocation, SETPOINT, DELTA_TIME);
    drive(RMSPEED, LMSPEED);
    // Serial.println("PID");
    previousMillis = currentMillis;
    // Serial.println("PID");
    digitalWrite(LED1, HIGH);
    digitalWrite(LED2, HIGH);
    digitalWrite(LED3, LOW);
  }
}

//Stage Six: From the fork split to the end of the course (UNTESTED)
void stage_six(float sensorLocation){
  if(stage_timer - previous_stage_timer > stage_six_time && !stageSixBool){
    stageSixBool = true;
    digitalWrite(LED1, HIGH);
    if(RMSPEED > LMSPEED){
      sharpRight();
      drive(RMSPEED, LMSPEED);
      delay(400);
      drive(140, 140);
      delay(300);
      LEFT_NORMAL_SPEED = 100;
      RIGHT_NORMAL_SPEED = 100;
    } else {
      sharpLeft();
      drive(RMSPEED, LMSPEED);
      delay(400);
      drive(140, 140);
      delay(300);
      LEFT_NORMAL_SPEED = 100;
      RIGHT_NORMAL_SPEED = 100;
    }
  }
  
  if ((currentMillis - previousMillis >= DELTA_TIME)){
    if(sensorLocation == 0 && degree_count % 2 == 0){
        digitalWrite(LED2, LOW);
        digitalWrite(LED3, HIGH);
        isTurned = true;
        sharpRight();
        drive(RMSPEED, LMSPEED);
        degree_count++;
        delay(500);
    }  else {
        drivePID(sensorLocation, SETPOINT, DELTA_TIME);
        drive(RMSPEED, LMSPEED);
        previousMillis = currentMillis;
        previousSensorLocation = sensorLocation;
        digitalWrite(LED1, HIGH);
        digitalWrite(LED2, LOW);
        digitalWrite(LED3, LOW);
        // Serial.println("PID");
    }
  }
}

void forceRight(){
  RMSPEED = -100; // Force Right
  LMSPEED = 200; // Force Right
  digitalWrite(LED3, LOW);
  digitalWrite(LED1, HIGH);
// Serial.println("FORCE RIGHT");
}


void forceLeft(){
  RMSPEED = 200; // Force Right
  LMSPEED =  -100; // Force Left
  digitalWrite(LED3, HIGH);
  digitalWrite(LED1, LOW);
// Serial.println("FORCE LEFT");
}

void sharpRight(){
  RMSPEED = -150; // Force Right
  LMSPEED = 150; // Force Right
  digitalWrite(LED3, LOW);
  digitalWrite(LED1, HIGH);
// Serial.println("FORCE RIGHT");
}

void sharpLeft(){
  RMSPEED = 1500; // Force Right
  LMSPEED =  -150; // Force Left
  digitalWrite(LED3, HIGH);
  digitalWrite(LED1, LOW);
// Serial.println("FORCE LEFT");
}

// used to write voltage to the motors
void motorWrite(int spd, int pin_IN1 , int pin_IN2 , int pin_PWM) {
  if (spd < 0) {
    digitalWrite(pin_IN1 , HIGH); // go one way
    digitalWrite(pin_IN2 , LOW);
  } else {
    digitalWrite(pin_IN1 , LOW); // go the other way
    digitalWrite(pin_IN2 , HIGH);
  }

  analogWrite(pin_PWM , abs(spd));
}

// makes robot move
void drive(int r_speed, int l_speed) {
  motorWrite(l_speed, AIN1, AIN2, PWMA);
  motorWrite(r_speed, BIN1, BIN2, PWMB);
}

// returns a value between 0 and 1 that represents the value/percentage of a sensor value
float computeNormVal(float sensorVal, float minVal, float maxVal){
  return constrain((sensorVal - minVal) / (maxVal - minVal), 0, 1);
}

// returns CM of IR sensor
float computeSensorXCM(float normalized_I[]){
   float num = 0;
   float den = 0;
    
   for(int i = 0; i < 3; i++){
      num += (normalized_I[i])*(i+1);
      den += (normalized_I[i]);
   }

   if(den == 0){
    return 0;
   }
   
   return num/den;
}

void drivePID(float sensorValue, float setpoint, float delta_t){
  error = setpoint - sensorValue;

  // convert time from milliseconds to seconds
  delta_t = delta_t * MILLISEC_TO_SEC;
  
  /////////////////////////////
  //    LEFT CALCULATIONS    //
  /////////////////////////////
  
  leftIntegral = leftIntegral + error * delta_t; 
  
  if ((KI_LEFT * leftIntegral) > MAX_MOTOR_SPEED) {
    leftIntegral = MAX_MOTOR_SPEED / KI_LEFT; 
  } else if ((KI_LEFT * leftIntegral) < -MAX_MOTOR_SPEED) { 
    leftIntegral = -MAX_MOTOR_SPEED / KI_LEFT;
  }
  
  
  leftDerivative = (error - previousError)/delta_t;

  float leftDeltaSpeed = KP_LEFT * error + KI_LEFT * leftIntegral + KD_LEFT * leftDerivative; 
  LMSPEED = LEFT_NORMAL_SPEED - leftDeltaSpeed; // choose the correct sign
  // Serial.print(leftDeltaSpeed);
  // Serial.print(" ");

  // check overall saturation again 
  if (LMSPEED > MAX_MOTOR_SPEED) {
    LMSPEED = MAX_MOTOR_SPEED; 
  } else if (LMSPEED < -MAX_MOTOR_SPEED) {
    LMSPEED = -MAX_MOTOR_SPEED;
  }

  /////////////////////////////
  //    RIGHT CALCULATIONS    //
  /////////////////////////////

  rightIntegral = rightIntegral + error * delta_t; 
  
  if ((KI_RIGHT * rightIntegral) > MAX_MOTOR_SPEED) {
    rightIntegral = MAX_MOTOR_SPEED / KI_RIGHT; 
  } else if ((KI_RIGHT * rightIntegral) < -MAX_MOTOR_SPEED) { 
    rightIntegral = -MAX_MOTOR_SPEED / KI_RIGHT;
  }
  
  rightDerivative = (error - previousError)/delta_t;


  // For Right Motor
  float rightDeltaSpeed = KP_RIGHT * error + KI_RIGHT * rightIntegral + KD_RIGHT * rightDerivative; 
  RMSPEED = RIGHT_NORMAL_SPEED + rightDeltaSpeed; // choose the correct sign
  // Serial.println(rightDeltaSpeed);
  
  if (RMSPEED > MAX_MOTOR_SPEED) {
    RMSPEED = MAX_MOTOR_SPEED; 
  } else if (RMSPEED < -MAX_MOTOR_SPEED) {
    RMSPEED = -MAX_MOTOR_SPEED;
  }

  previousError = error;

//  // array for monitoring
//  float PID_LeftValues[] = {KP_LEFT * error, KI_LEFT * leftIntegral, KD_LEFT * leftDerivative};
////
//  // for monitoring left motor PID changes
//  Serial.print("Left Delta Speed: ");
//  Serial.print(leftDeltaSpeed);
//  Serial.print(" ");
//
//  Serial.print("Left KP_error, KI_Error, KD_Error: [ ");
//  for(int i=0; i<3; i++){
//    Serial.print(PID_LeftValues[i]);
//    Serial.print(" ");
//  }
//  
//  Serial.print("] ");
//  
//
//  // array for monitoring
//  float PID_RightValues[] = {KP_RIGHT * error, KI_RIGHT * rightIntegral, KD_RIGHT * rightDerivative};
//
//  // for monitoring right motor PID changes
//  Serial.print("Right Delta Speed: ");
//  Serial.print(rightDeltaSpeed);
//  Serial.println(" ");
//  
//
//  Serial.print("Right KP_error, KI_Error, KD_Error: [ ");
//  for(int i=0; i<3; i++){
//    Serial.print(PID_RightValues[i]);
//    Serial.print(" ");
//  }
//  
//  Serial.println("] ");
}

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
const int LEFT_NORMAL_SPEED = 160;
const int RIGHT_NORMAL_SPEED = 160;
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
float maximum_intensities[] = {597, 492, 560}; // less reflectance; more black; //(771 776 779) parallel to the line on the floor
float minimum_intensities[] = {34, 34, 87}; // high reflectance; more white
float normalized_intensities[3];

/////////////////
//PID VARIABLES//
/////////////////
const float SETPOINT = 2.0;
float previousSensorLocation = 2;

// PID constants 
const float KP_LEFT = 140; // left motor proportional gain; prev: 140
const float KI_LEFT = 150; // left motor integral gain prev: 150
const float KD_LEFT = 0.42; // left motor derivative gain

const float KP_RIGHT = 140; // right motor proportional gain; prev: 140
const float KI_RIGHT = 150;// right motor integral gain prev: 150
const float KD_RIGHT = 0.42; // right motor derivative prev: 0.45

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
int stage_one_time = 0; // INSERT VALUE HERE

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

  pinMode(IR1_PIN, INPUT);
  pinMode(IR2_PIN, INPUT);
  pinMode(IR3_PIN, INPUT);
}

void loop() {
  currentMillis = millis();
  int intensities[] = {analogRead(IR1_PIN), analogRead(IR2_PIN), analogRead(IR3_PIN)};

  // Add normalized values
  for(int x = 0; x < 3; x++){
    normalized_intensities[x] = computeNormVal(intensities[x], minimum_intensities[x], maximum_intensities[x]);
  }

  float sensorLocation = computeSensorXCM(normalized_intensities);
  
//  // the drive component of the line follower
//  while(normalized_intensities[0] < 0.20 && normalized_intensities[1] < 0.20 && normalized_intensities[2] < 0.20){
//    drive(160, 160);
//  }

//if(normalized_intensities[0] < 0.20 && normalized_intensities[1] < 0.20 && normalized_intensities[2] < 0.20){
//      drive(160, 160);
//      // Serial.println("straight");
//    } else 

  if(stage == 1){
    stage_one(sensorLocation);
    if(stage_timer - previous_stage_timer < 0){
      stage = stage + 1;
    }
  }

//  if(stage == 2){
//    
//  }
//
//  if(stage == 3){
//    
//  }

  //  // intensities for monitoring
//  Serial.print("IR1: ");
//  Serial.print(intensities[0]);
//  Serial.print("; ");
//  Serial.print("IR2: ");
//  Serial.print(intensities[1]);
//  Serial.print("; ");
//  Serial.print("IR3: ");
//  Serial.print(intensities[2]);
//  Serial.println("; ");

//  // normalized values for monitoring
//  Serial.print("NORM_IR1: ");
//  Serial.print(normalized_intensities[0]);
//  Serial.print("; ");
//  Serial.print("NORM_IR2: ");
//  Serial.print(normalized_intensities[1]);
//  Serial.print("; ");
//  Serial.print("NORM_IR3: ");
//  Serial.print(normalized_intensities[2]);
//  Serial.println("; ");

//  // sensorLocation for monitoring
//  Serial.print("Sensor Location: ");
//  Serial.println(sensorLocation);
}

////////////////////
//Helper Functions//
////////////////////

// The course split beteween stages
void stage_one(float sensorLocation){
  if ((currentMillis - previousMillis >= DELTA_TIME) ) {
    if(previousSensorLocation > 2.7 && normalized_intensities[0] < 0.05 && normalized_intensities[1] < 0.05 && normalized_intensities[2] > 0.4){
      RMSPEED = RMSPEED - 25; // Force Right
      LMSPEED = LMSPEED + 25;
     // Serial.println("FORCE RIGHT");
    } else if(previousSensorLocation < 1.1 && normalized_intensities[0] > 0.4 && normalized_intensities[1] < 0.05 && normalized_intensities[2] < 0.05){
      RMSPEED = RMSPEED + 25;
      LMSPEED = LMSPEED - 25; // Force Left
     // Serial.println("FORCE LEFT");
    } else {
      // regular control
      drivePID(sensorLocation, SETPOINT, DELTA_TIME);
      drive(RMSPEED, LMSPEED);
      previousMillis = currentMillis;
      previousSensorLocation = sensorLocation;
      // Serial.println("PID");
    }
  }
  drive(RMSPEED, LMSPEED);
}

void stage_two(){
  
}

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

void drive(int r_speed, int l_speed) {
  motorWrite(l_speed, AIN1, AIN2, PWMA);
  motorWrite(r_speed, BIN1, BIN2, PWMB);
}

float computeNormVal(float sensorVal, float minVal, float maxVal){
  return constrain((sensorVal - minVal) / maxVal, 0.001, 1);
}

float computeSensorXCM(float normalized_I[]){
   float num = 0;
   float den = 0;
   
   for(int i = 0; i < 3; i++){
      num += (normalized_I[i])*(i+1);
      den += (normalized_I[i]);
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

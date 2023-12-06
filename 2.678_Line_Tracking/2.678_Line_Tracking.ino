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
  - normalize sensor values to positioning with map function
  - create PID algorithm to stick to sensor seet point

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
const int LEFT_NORMAL_SPEED = 150;
const int RIGHT_NORMAL_SPEED = 150;
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
float maximum_intensities[] = {800, 793, 830}; // less reflectance; more black; //(771 776 779) parallel to the line on the floor
float minimum_intensities[] = {46, 39, 52}; // high reflectance; more white
float normalized_intensities[3];

/////////////////
//PID VARIABLES//
/////////////////
const float SETPOINT = 2.20;

// PID constants 
const float KP_LEFT = 150; // left motor proportional gain
const float KI_LEFT = 0; // left motor integral gain
const float KD_LEFT = 0; // left motor derivative gain

const float KP_RIGHT = 150; // right motor proportional gain
const float KI_RIGHT = 0;// right motor integral gain
const float KD_RIGHT = 0; // right motor derivative

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

  if ((currentMillis - previousMillis >= DELTA_TIME) ) {
      drivePID(sensorLocation, SETPOINT, DELTA_TIME);
      drive(RMSPEED, LMSPEED);
      previousMillis = currentMillis;

    // for monitoring
//    Serial.print(intensities[0]);
//    Serial.print(" ");
//    Serial.print(intensities[1]);
//    Serial.print(" ");
//    Serial.print(intensities[2]);
//    Serial.print(" ");
//    Serial.println(sensorLocation);

  }
  drive(RMSPEED, LMSPEED);
}

////////////////////
//Helper Functions//
////////////////////

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

float computeNormVal(float sensorVal, float minVal, float rangeVal){
  return constrain((sensorVal - minVal) / rangeVal, 0, 1);
}

float computeSensorXCM(float normalized_I[]){
   float num = 0;
   float den = 0;
   
   for(int i = 0; i < 3; i++){
      num += (1-normalized_I[i])*(i+1);
      den += (1-normalized_I[i]);
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
  LMSPEED = LEFT_NORMAL_SPEED + leftDeltaSpeed; // choose the correct sign
  Serial.print(leftDeltaSpeed);
  Serial.print(" ");

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
  RMSPEED = RIGHT_NORMAL_SPEED - rightDeltaSpeed; // choose the correct sign
  Serial.println(rightDeltaSpeed);
  
  if (RMSPEED > MAX_MOTOR_SPEED) {
    RMSPEED = MAX_MOTOR_SPEED; 
  } else if (RMSPEED < -MAX_MOTOR_SPEED) {
    RMSPEED = -MAX_MOTOR_SPEED;
  }
  
  previousError = error;
}

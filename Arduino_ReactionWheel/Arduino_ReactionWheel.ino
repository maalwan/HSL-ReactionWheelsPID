// ------BLDC--------
#include <Servo.h>
#define MAX_SPEED 180
Servo ESC;
double motorSpeed = 0;
// ------------------

// ------BNO055-----
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#define MPU_ADDRESS 0x28 // I2C address of the BNO055
Adafruit_BNO055 bno = Adafruit_BNO055(-1, MPU_ADDRESS, &Wire);
double yawAngle, yawAngularSpeed = 0;
// ------------------

// -------PID--------
#include "PID.h"
const double P_TERM = 0.050;
const double I_TERM = 0.000;
const double D_TERM = 0.017; 
PIDController pidSpeed(P_TERM, I_TERM, D_TERM);
PIDAngleController pidAngle(2.5, 0, 400);
// ------------------

// ------GENERAL-----
long timeCur, timePrev, timeStart; 
const int numReading s= 5;
double readings[numReadings];
int readIndex = 0;
double total = 0, rollingAvg = 0;
double targetAttitude = 0;
// FSM variables
byte controllerState = 0;
int counts = 0;
// ------------------

void setup() {
  Serial.begin(9600);
  
  // IMU setup
  if(!bno.begin()) {
    Serial.print(F("No BNO055 detected"));
    while(1);
  }
  bno.setExtCrystalUse(true);
  // Calibrating IMU
  Serial.print(F("Calibrating IMU"));
  while (true) {
    uint8_t system, gyro, accel, mag = 0;
    if (system) { break; } // Calibrated when system > 0 (Fully when system == 3)

    Serial.print(F("."));
    delay(50);
  }
  
  // Start ESC on pin 9
  ESC.attach(9, 1000, 2000);
  
  timeCur = millis();
  timeStart = timeCur;
}

void loop() {
  // Every 10ms, read IMU and call controllers
  if(millis() - timeCur > 10) {
    timePrev = timeCur;
    timeCur = millis();

    // Measure Gyro value
    // ------------------
    yawAngularSpeed = getYawSpeed();
    yawAngle = getYawAngle();
    // Smooth the angular speed --> rolling average
    total = total - readings[readIndex]; 
    readings[readIndex] = yawAngularSpeed;
    total = total + readings[readIndex];
    readIndex = readIndex + 1;
    if (readIndex >= numReadings) { readIndex = 0; }
    rollingAvg = total / numReadings;

    // Calculate the new motor speed
    // -----------------------------
    motorSpeed += pidSpeed.compute(0, rollingAvg, timeCur - timePrev);
    // FSM transition
    if (controllerState == 1 && fabs(rollingAvg) > 360 /* °/s */) {
      controllerState = 0;
    } else if (controllerState == 0 && fabs(rollingAvg) < 45 /* °/s */)
      controllerState = 1;
    }
    // FSM action (I have no idea how this works)
    if (controllerState == 0) {
      motorSpeed += pidSpeed.compute(0, rollingAvg, timeCur - timePrev);
      pidAngle.compute(targetAttitude, yawAngle, timeCur - timePrev);
    } else {
      motorSpeed += pidSpeed.compute(pidAngle.compute(targetAttitude, yawAngle, timeCur - timePrev), rollingAvg, timeCur - timePrev);
    }
    setSpeed(motorSpeed);

    // Print info to console
    Serial.print(yawAngle);
    Serial.print(F(" "));
    Serial.println(rollingAvg);
  }
}

// Cycles PWM with intent to calibrate ESC (copied from Marcin's code)
void cyclePWM() {
  for (int val = 0; val <= 50; val++) {
    ESC.write(val);
    delay(10);
  }
  delay(1000);
  for (int val = 50; val >= 0; val--) {
    ESC.write(val);
    delay(10);
  }
  delay(1000);
}

// Set the current speed and direction of the motor
void setSpeedStepper(double targetSpeed) {
  if (motorSpeed > MAX_SPEED) {
    motorSpeed = MAX_SPEED;
  } else if (motorSpeed < -MAX_SPEED) {
    motorSpeed = -MAX_SPEED;
  }
  ESC.write(fabs(targetSpeed))
}

double getYawAngle() {
  imu::Vector<3> eulerAngles = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  return eulerAngles.z();
}

double getYawSpeed() {
  imu::Vector<3> angularSpeed = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  return angularSpeed.z() * 57.2958; // For degrees per second (originally in radians per second)
}

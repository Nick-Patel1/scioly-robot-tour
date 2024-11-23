/*
  Pre-Run Checklist
  1. New batteries
  2. Cables
  3. Status lights
  4. Power cycle
  
  Timing Reference Sheet
  Straight:
  150- 2 seconds

  Turn: 1 second
*/

// Packages
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <Arduino.h>
#include <Queue.h>
#include "ICM_20948.h"
#include <Wire.h>
#include <SparkFun_Qwiic_Optical_Sensor.h>

int targetTime = 7; // in seconds
int DIS1 = 18500;     // 22000
int DIS2 = DIS1*2;
int TIL1 = 195;       //
int TIL2 = 695;
int TILF = 260;

int time240 = 2900;
int time220 = 3000;
int time200 = 3200;
int time180 = 3600;
int time160 = 4000;
int time140 = 4400;
int time120 = 5100;
int time100 = 6000;
int timeA;
int timeB;
int turnTime = 1780; // do not change unless u need to ig
int startDelay = 2000;
int movementDelay = 10;

int speedControl = 230;
int linearSpeedLimit = 200;

// temporary fix
int rightTrim = 0;

// Pins
#define sensorLTrigPin 2
#define sensorLEchoPin 3
#define sensorRTrigPin 4
#define sensorREchoPin 5
#define encoderLPinA 6
#define encoderLPinB 7
#define encoderRPinA 8
#define encoderRPinB 9
#define buttonPin 10

// IMU
#define SERIAL_PORT Serial
#define WIRE_PORT Wire
#define AD0_VAL 1

ICM_20948_I2C myICM;

// Motor
Adafruit_MotorShield motorShield = Adafruit_MotorShield();
Adafruit_DCMotor *motorL = motorShield.getMotor(1);
Adafruit_DCMotor *motorR = motorShield.getMotor(2);

#define WAIT 0   // wait (not actually used)
#define START 1  // start button pressed (also not used)
#define FD 10    // move forward
#define BD 11    // move backward
#define FDT 12   // move forward until distance
#define BDT 13   // move backward until distance
#define DM 15    // distance match
#define RT 20    // turn right
#define LT 21    // turn left
#define RTE 35   // turn right but with encoders
#define LTE 36   // turn left but with encoders
#define STOP 22  // end of program
#define ABORT 99 // abort program

bool status, newCommand;
int targetAngle = 0;
float orientation = 0.0;
float orientCalibrate = 0.0;
// int defaultSpeedL = 155, defaultSpeedR = 150; //these are literally just assigned to target speed immediately idk if you want
int theTargetSpeed = 0; //used from calculateSpeed
int turnSpeedLimit = 200;
int turnSpeedMinimum = 17;
int linearSlowLimit = 80;
int currentSpeed = speedControl; //will just assume current speed is at speed control speed at first.

int distanceTolerance = 1;
int encoderTolerance = 1;
float directionTolerance = 1.0;
float matchTolerance = 0.85;

int slowEnc = 2000;
int slowDis = 40;

volatile int lastEncodedL = 0, lastEncodedR = 0;
volatile long encoderValueL = 0, encoderValueR = 0;
long previousEncoderValueL = 0;
long encoderValueLAbs = 0, encoderValueRAbs = 0;
//bool accelerate;
//bool deccelerate;
//int accelerateRate = 1;
//int targetSpeedL = defaultSpeedL;
//int targetSpeedR = defaultSpeedR;
//int startSpeedL = 80;
//int startSpeedR = 80;
float disL = 0, disR = 0;

// for PID control
long previousTimeL = 0;
float ePreviousL = 0;
float eIntegralL = 0;

long previousTimeO = 0;
float ePreviousO = 0;
float eIntegralO = 0;

long previousTimeR = 0;
float ePreviousR = 0;
float eIntegralR = 0;

long previousTimeL2 = 0;
float ePreviousL2 = 0;
float eIntegralL2 = 0;

long previousTimeR2 = 0;
float ePreviousR2 = 0;
float eIntegralR2 = 0;

long previousTimeC = 0;
float ePreviousC = 0;
float eIntegralC = 0;

long previousTimeS = 0;
float ePreviousS = 0;
float eIntegralS = 0;

// PID gains
float kpL = 1.0;
float kdL = 0.0;
float kiL = 0.0;

float kpO = 15.0;
float kdO = 0.0;
float kiO = 0.0;

float kpR = 5.0;
float kdR = 0.05;
float kiR = 0.01;

float kpLinear = 4.0;

float kpC = 4.0;
float kdC = 1.0;
float kiC = 0.0;

float kpL2 = 3.0;
float kdL2 = 0.0;
float kiL2 = 0.0;

float kpR2 = 3.0;
float kdR2 = 0.0;
float kiR2 = 0.0;

float kpS = 1.0;
float kdS = 0.0;
float kiS = 0.0;

float calibrateTesting = 0.0;

// timing stuff
int startTime;
int currentTime; //millis - startTime
int previousTime; // used for calculating current velocity
int timeLeft; //targetTime - currentTime

// command queues
Queue<int, 50> commandQueue;
Queue<int, 50> paramQueue;

// Optical Flow Sensor
QwiicOpticalFlow myOpticalFlow;

void setup() {
  motorShield.begin();
  Serial.begin(115200);

  // calculateSpeed();
  // delay(5000);

  // initialize encoder
  pinMode(encoderLPinA, INPUT_PULLUP);
  pinMode(encoderLPinB, INPUT_PULLUP);
  pinMode(encoderRPinA, INPUT_PULLUP);
  pinMode(encoderRPinB, INPUT_PULLUP);

  digitalWrite(encoderLPinA, HIGH);
  digitalWrite(encoderLPinB, HIGH);
  digitalWrite(encoderRPinA, HIGH);
  digitalWrite(encoderRPinB, HIGH);

  attachInterrupt(digitalPinToInterrupt(encoderLPinA), updateEncoderL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderLPinB), updateEncoderL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderRPinA), updateEncoderR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderRPinB), updateEncoderR, CHANGE);

  pinMode(LED_BUILTIN, OUTPUT);

  /**/
  // initialize IMU communication
  WIRE_PORT.begin();
  WIRE_PORT.setClock(400000);

  bool initialized = false;
  while (!initialized) {
    myICM.begin(WIRE_PORT, AD0_VAL);

    if (myICM.status != ICM_20948_Stat_Ok) {
      delay(500);
    }
    else {
      initialized = true;
    }
  }

  bool success = true;
  success &= (myICM.initializeDMP() == ICM_20948_Stat_Ok);
  success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR) == ICM_20948_Stat_Ok);
  success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Quat6, 0) == ICM_20948_Stat_Ok);
  success &= (myICM.enableFIFO() == ICM_20948_Stat_Ok);
  success &= (myICM.enableDMP() == ICM_20948_Stat_Ok);
  success &= (myICM.resetDMP() == ICM_20948_Stat_Ok);
  success &= (myICM.resetFIFO() == ICM_20948_Stat_Ok);

  if (!success) {
    SERIAL_PORT.println(F("Enable DMP failed!"));
    SERIAL_PORT.println(F("Please check that you have uncommented line 29 (#define ICM_20948_USE_DMP) in ICM_20948_C.h..."));
    while (1) {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(200);
      digitalWrite(LED_BUILTIN, LOW);
      delay(200);
    }
  }
  /**/

  // Initialize the Optical Flow sensor
  if (myOpticalFlow.begin() == false) {
    Serial.println("Qwiic Optical Flow Sensor not detected. Check wiring.");
    while (1);
  }
  Serial.println("Qwiic Optical Flow Sensor detected.");

  // initialize distance sensors
  pinMode(sensorLTrigPin, OUTPUT);
  pinMode(sensorLEchoPin, INPUT);
  pinMode(sensorRTrigPin, OUTPUT);
  pinMode(sensorREchoPin, INPUT);

  // initialize start button
  pinMode(buttonPin, INPUT_PULLUP);

  // initialize ultrasonic distance sensor pins
  pinMode(sensorLTrigPin, OUTPUT);
  pinMode(sensorLEchoPin, INPUT);

  newCommand = true;

  status = false;

  //--------------------------------
  // add commands HERE
  //--------------------------------

  // must be first!
  add(START);

  /**/
  add(FDT,TIL1);
  add(RT);
  add(FD,DIS1);
  add(LT);
  add(FDT,TIL1);
  add(BDT,TIL2);
  add(LT);
  add(FD,DIS1*3);
  add(RT);
  add(FD,DIS1*3);
  add(RT);
  add(FD,DIS1*3);
  add(RT);
  add(FDT,TIL1);
  add(BDT,TIL2);
  add(RT);
  add(FD,DIS1*3);
  add(LT);
  add(FD,DIS2);
  add(LT);
  add(FDT,TIL1);
  add(LT);
  add(FDT,TIL1);
  add(BDT,TIL2);
  add(RT);
  add(BDT,TIL2);
  add(LT);
  add(FDT,TILF);
  /**/

  /*
  add(FD, DIS1);
  add(RT);
  add(FD, DIS1);
  add(RT);
  add(FD, DIS1);
  add(RT);
  add(FD, DIS1);
  add(RT);
  /**/
  /*
  add(LT);
  add(LT);
  add(LT);
  add(LT);
  add(LT);
  add(LT);
  add(LT);
  add(LT);
  add(RT);
  add(RT);
  add(RT);
  add(RT);
  add(RT);
  add(RT);
  add(RT);
  add(RT);
  add(LT);
  add(LT);
  add(RT);
  add(RT);
  add(RT);
  add(RT);
  add(LT);
  add(LT);
  /*
  add(LT);
  add(LT);
  add(LT);
  add(LT);
  add(LT);
  add(LT);
  add(LT);
  add(LT);
  */

  /*
  add(FD,DIS1);
  add(FD,DIS1);
  add(FD,DIS1);
  add(FD,DIS1);
  add(RT);
  add(RT);
  add(FD,DIS1);
  add(FD,DIS1);
  add(FD,DIS1);
  add(FD,DIS1);
  /*
  add(RT);
  add(RT);
  add(FD,DIS1);
  add(FD,DIS1);
  add(FD,DIS1);
  add(FD,DIS1);
  add(BD,DIS1);
  add(BD,DIS1);
  add(BD,DIS1);
  add(BD,DIS1);
  /**/

  /*
  add(FDT,TIL1);
  add(RT);
  add(FDT,TIL1);
  */
  // add(BDT,TIL2);
  // add(DM,TIL1);
  //add(RT);
  // add(FD,DIS1);
  //add(FD,DIS1);

  // must be last!
  add(STOP);

  // blink for ready to begin
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(200);
    digitalWrite(LED_BUILTIN, LOW);
    delay(200);
  }
  digitalWrite(LED_BUILTIN, HIGH);

  // Serial.println(targetAngle);
}

void loop() {
  checkButton();

  // Read the Optical Flow sensor data
  if (myOpticalFlow.dataReady()) {
    myOpticalFlow.update();
    int deltaX = myOpticalFlow.deltaX();
    int deltaY = myOpticalFlow.deltaY();

    // Update the robot's position based on the sensor data
    encoderValueL += deltaX;
    encoderValueR += deltaY;

    Serial.print("Delta X: ");
    Serial.print(deltaX);
    Serial.print(" Delta Y: ");
    Serial.println(deltaY);
  }

  updateIMU();
  /*
  Serial.println(orientation);
  // Serial.print(", ");
  // Serial.println(calibrateTesting);
  /**/
  // Serial.println(timeB/8);
  // calculateCurrentSpeed();

  /**/
  if (status) {
    digitalWrite(LED_BUILTIN, LOW);

    int currentCmd = commandQueue.front();
    float currentParam = paramQueue.front();

    switch (currentCmd) {
    case WAIT:
      // wait
      break;
    case START:
      // start
      Serial.println("hi");
      commandQueue.dequeue();
      paramQueue.dequeue();
      newCommand = true;
      /*
      digitalWrite(LED_BUILTIN, HIGH);
      delay(200);
      digitalWrite(LED_BUILTIN, LOW);
      /**/
      delay(startDelay);
      startTime = millis();
      /**/
      break;
    case FD:
      // move forward
      if (newCommand) {
        newCommand = false;
        // calculateSpeed();
        linearSpeedLimit = speedControl;
        // previousTime = millis();
        // previousEncoderValueL = encoderValueLAbs;
        motorL->setSpeed(linearSpeedLimit);
        motorR->setSpeed(linearSpeedLimit + rightTrim);
        motorL->run(FORWARD);
        motorR->run(FORWARD);
      } else {
        if (encoderValueL - (encoderValueLAbs - currentParam) > encoderTolerance) {
          // pidPositionMatchLeft(encoderValueLAbs - currentParam, linearSpeedLimit);
          // pidPositionMatchRight(encoderValueRAbs + fabs(encoderValueL - encoderValueLAbs));
          pidCourseCorrect();
          Serial.println(orientation);
          // Serial.println(encoderValueL);
          if (encoderValueL - (encoderValueLAbs - currentParam) < slowEnc) {
            linearSpeedLimit = linearSlowLimit;
            motorL->setSpeed(linearSpeedLimit);
          }
        } else {
          motorStop();
          commandQueue.dequeue();
          paramQueue.dequeue();
          encoderValueLAbs = encoderValueL;
          encoderValueRAbs = encoderValueR;
          newCommand = true;
          delay(movementDelay);
        }
      }
      break;
    case BD:
      // move backward
      if (newCommand) {
        newCommand = false;
        linearSpeedLimit = speedControl;
        motorL->setSpeed(linearSpeedLimit);
        motorR->setSpeed(linearSpeedLimit + rightTrim);
        motorL->run(BACKWARD);
        motorR->run(BACKWARD);
      } else {
        if ((encoderValueLAbs + currentParam) - encoderValueL > encoderTolerance) {
          // pidPositionMatchLeft(encoderValueLAbs - currentParam, linearSpeedLimit);
          // pidPositionMatchRight(encoderValueRAbs + fabs(encoderValueL - encoderValueLAbs));
          pidCourseCorrectBackward();
          // Serial.println(encoderValueL);
          if ((encoderValueLAbs + currentParam) - encoderValueL < slowEnc) {
            linearSpeedLimit = linearSlowLimit;
            motorL->setSpeed(linearSpeedLimit);
          }
        } else {
          motorStop();
          commandQueue.dequeue();
          paramQueue.dequeue();
          encoderValueLAbs = encoderValueL;
          encoderValueRAbs = encoderValueR;
          newCommand = true;
          delay(movementDelay);
        }
      }
      break;
    case FDT:
      if (newCommand) {
        newCommand = false;
        linearSpeedLimit = speedControl;
        motorL->setSpeed(linearSpeedLimit);
        motorR->setSpeed(linearSpeedLimit + rightTrim * 2);
        motorL->run(FORWARD);
        motorR->run(FORWARD);
      } else {
        getDistance();
        if (disL - currentParam > distanceTolerance) {
          // pidDistanceMatchLeft(currentParam, linearSpeedLimit);
          if (disL - currentParam < slowDis) {
            Serial.println("hi1");
            linearSpeedLimit = linearSlowLimit;
            Serial.println("hi2");
            motorL->setSpeed(linearSpeedLimit);
            Serial.println("hi3");
          }
          pidCourseCorrect();
        } else {
          motorStop();
          commandQueue.dequeue();
          paramQueue.dequeue();
          encoderValueLAbs = encoderValueL;
          encoderValueRAbs = encoderValueR;
          newCommand = true;
          delay(movementDelay);
        }
      }
      break;
    case DM:
      // move forward til distance from wall
      if (newCommand) {
        newCommand = false;
        linearSpeedLimit = speedControl;
        // motorR->setSpeed(linearSpeedLimit + rightTrim);
        // motorR->run(FORWARD);
      } else {
        checkDistance();
        if (fabs(disL - currentParam) > matchTolerance || fabs(disR - currentParam) > matchTolerance) {
          pidDistanceMatchLeft(currentParam, 255);
          pidDistanceMatchRight(currentParam);
          /*
          Serial.print(disL);
          Serial.print(", ");
          Serial.println(disR);
          /**/
        } else {
          motorStop();
          commandQueue.dequeue();
          paramQueue.dequeue();
          resetIMU();
          /*
          updateIMU();
          orientCalibrate += targetAngle - orientation;
          calibrateTesting = orientation;
          */
          encoderValueLAbs = encoderValueL;
          encoderValueRAbs = encoderValueR;
          newCommand = true;
          delay(movementDelay);
        }
      }
      break;
    case BDT:
      // move backward til distance from wall
      if (newCommand) {
          newCommand = false;
          linearSpeedLimit = speedControl;
          motorL->setSpeed(linearSpeedLimit);
          motorR->setSpeed(linearSpeedLimit + rightTrim);
          motorL->run(BACKWARD);
          motorR->run(BACKWARD);
        } else {
          getDistance();
          if (currentParam - disL > distanceTolerance) {
            // pidDistanceMatchLeft(currentParam, linearSpeedLimit);
            pidCourseCorrectBackward();
            if (currentParam - disL < slowDis) {
              linearSpeedLimit = linearSlowLimit;
              motorL->setSpeed(linearSpeedLimit);
            }
          } else {
            motorStop();
            commandQueue.dequeue();
            paramQueue.dequeue();
            encoderValueLAbs = encoderValueL;
            encoderValueRAbs = encoderValueR;
            newCommand = true;
            delay(movementDelay);
          }
        }
      break;
      /**/
    case RT:
      // turn right
      if (newCommand) {
        // Serial.println(targetAngle);
        targetAngle -= 90;
        // Serial.println(targetAngle);
        /**/
        if (targetAngle == -180) {
          targetAngle = 180;
        }
        /**/
        motorL->setSpeed(turnSpeedLimit);
        motorL->run(FORWARD);
        newCommand = false;
        // Serial.println(targetAngle);
      } else {
        if ((targetAngle == 180 && orientation < 0 && fabs(-targetAngle - orientation) > directionTolerance) || fabs(targetAngle - orientation) > directionTolerance) {
        // if ((targetAngle == 90 && orientation < 0) || (targetAngle != 180 && fabs(orientation - targetAngle) > directionTolerance) || (targetAngle == 180 && orientation < 0)) {
          pidTurn();
          pidPositionMatchRight(encoderValueRAbs - fabs(encoderValueL - encoderValueLAbs));
          /*
          Serial.print(targetAngle);
          Serial.print(", ");
          Serial.print(targetAngle - orientation);
          Serial.print(", ");
          Serial.println(orientation);
          /**/
        } else {
          motorStop();
          commandQueue.dequeue();
          paramQueue.dequeue();
          encoderValueLAbs = encoderValueL;
          encoderValueRAbs = encoderValueR;
          newCommand = true;
          delay(movementDelay);
        }
      }
      break;
    case LT:
      // turn left
      if (newCommand) {
        Serial.println(targetAngle);
        targetAngle += 90;
        Serial.println(targetAngle);
        /**/
        if (targetAngle == 270) {
          targetAngle = -90;
        }
        /**/
        motorL->setSpeed(turnSpeedLimit);
        motorL->run(BACKWARD);
        newCommand = false;
        // Serial.println(targetAngle);
      } else {
        if ((targetAngle == 180 && orientation < 0 && fabs(-targetAngle - orientation) > directionTolerance) || fabs(targetAngle - orientation) > directionTolerance) {
          pidTurn();
          pidPositionMatchRight(encoderValueRAbs + fabs(encoderValueL - encoderValueLAbs));
          /*
          Serial.print(targetAngle);
          Serial.print(", ");
          Serial.print(orientation - targetAngle);
          Serial.print(", ");
          Serial.println(orientation);
          /**/
        } else {
          motorStop();
          commandQueue.dequeue();
          paramQueue.dequeue();
          encoderValueLAbs = encoderValueL;
          encoderValueRAbs = encoderValueR;
          newCommand = true;
          delay(movementDelay);
        }
      }
      break;
    /*
    case RTE:
      // turn right with encoders
      if (newCommand) {
        newCommand = false;
      } else {
        if (fabs(encoderValueL - (encoderValueLAbs - currentParam)) > encoderTolerance || fabs(encoderValueR - (encoderValueRAbs - currentParam)) > encoderTolerance) {
          pidPositionMatchLeft(encoderValueLAbs - currentParam, turnSpeedLimit);
          pidPositionMatchRight(encoderValueRAbs - fabs(encoderValueL - encoderValueLAbs));
        }
        else {
          motorStop();
          commandQueue.dequeue();
          paramQueue.dequeue();
          encoderValueLAbs-= currentParam;
          encoderValueRAbs-= currentParam;
          newCommand = true;
          Serial.println(encoderValueR);
          delay(movementDelay);
        }
      }
      break;
    case LTE:
      // turn left with encoders
      if (newCommand) {
        newCommand = false;
      } else {
        if (fabs(encoderValueL - (encoderValueLAbs + currentParam)) > encoderTolerance || fabs(encoderValueR - (encoderValueRAbs + currentParam)) > encoderTolerance) {
          pidPositionMatchLeft(encoderValueLAbs + currentParam, turnSpeedLimit);
          pidPositionMatchRight(encoderValueRAbs + fabs(encoderValueL - encoderValueLAbs));
        }
        else {
          motorStop();
          commandQueue.dequeue();
          paramQueue.dequeue();
          encoderValueLAbs+= currentParam;
          encoderValueRAbs+= currentParam;
          newCommand = true;
          Serial.println(encoderValueR);
          delay(movementDelay);
        }
      }
      break;
      /**/
    case STOP:
      // stop
      motorStop();
      break;
    }
  }
}

void add(int cmd) { add(cmd, 0); }

void add(int cmd, int param) {
  commandQueue.enqueue(cmd);
  paramQueue.enqueue(param);
}

void checkButton() {
  if (digitalRead(buttonPin) == LOW) {
    status = !status;
  }
}

void checkDistance() {
  digitalWrite(sensorLTrigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(sensorLTrigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(sensorLTrigPin, LOW);
  float duration = pulseIn(sensorLEchoPin, HIGH);
  disL = duration * 0.343 / 2.0;
  delay(60);
  digitalWrite(sensorRTrigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(sensorRTrigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(sensorRTrigPin, LOW);
  duration = pulseIn(sensorREchoPin, HIGH);
  disR = duration * 0.343 / 2.0;
}

void updateIMU() {
  // Read any DMP data waiting in the FIFO
  icm_20948_DMP_data_t data;
  myICM.readDMPdataFromFIFO(&data);

  if ((myICM.status == ICM_20948_Stat_Ok) || (myICM.status == ICM_20948_Stat_FIFOMoreDataAvail)) { // Was valid data available?
    if ((data.header & DMP_header_bitmap_Quat6) > 0) { // We have asked for GRV data so we should receive Quat6
      // Scale to +/- 1
      double q1 = ((double)data.Quat6.Data.Q1) / 1073741824.0; // Convert to double. Divide by 2^30
      double q2 = ((double)data.Quat6.Data.Q2) / 1073741824.0; // Convert to double. Divide by 2^30
      double q3 = ((double)data.Quat6.Data.Q3) / 1073741824.0; // Convert to double. Divide by 2^30

      // Convert the quaternions to Euler angles (roll, pitch, yaw)
      // https://en.wikipedia.org/w/index.php?title=Conversion_between_quaternions_and_Euler_angles&section=8#Source_code_2

      double q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));

      double q2sqr = q2 * q2;

      // yaw (z-axis rotation)
      double t3 = +2.0 * (q0 * q3 + q1 * q2);
      double t4 = +1.0 - 2.0 * (q2sqr + q3 * q3);
      double yaw = atan2(t3, t4) * 180.0 / PI;

      //float temp = (int) (yaw * 100 + .5);

      // orientation = (float) temp / 100;
      orientation = yaw;
      // Serial.println(orientation);
      /*
      orientation += orientCalibrate;
      if (orientation > 180) {
        orientation-= 360;
      } else if (orientation < -180) {
        orientation+= 360;
      }
      /**/
    }
  }
}

void pidPositionMatchLeft(int target, int targetSpeed) {
  // measure time since last adjustment
  long currentTime = micros();
  float deltaT = ((float)(currentTime - previousTimeL)) / 1.0e6;

  // compute error, derivative, integral
  int e = encoderValueL - target;
  float eDerivative = (e - ePreviousL) / deltaT;
  eIntegralL = eIntegralL + e * deltaT;

  // compute PID control signal
  float u = (kpL * e) + (kdL * eDerivative) + (kiL * eIntegralL);

  previousTimeL = currentTime;
  ePreviousL = e;

  float speed = fabs(u);

  if (speed > targetSpeed) {
    speed = targetSpeed;
  }
  else if (speed < turnSpeedMinimum) {
    speed = turnSpeedMinimum;
  }

  motorL->setSpeed(speed);

  if (u < 0) {
    motorL->run(BACKWARD);
  }
  else {
    motorL->run(FORWARD);
  }
}

void pidPositionMatchRight(int target) {
    // measure time since last adjustment
  long currentTime = micros();
  float deltaT = ((float)(currentTime - previousTimeR)) / 1.0e6;

  // compute error, derivative, integral
  int e = encoderValueR - target;
  float eDerivative = (e - ePreviousR) / deltaT;
  eIntegralR = eIntegralR + e * deltaT;

  // compute PID control signal
  float u = (kpR * e) + (kdR * eDerivative) + (kiR * eIntegralR);

  previousTimeR = currentTime;
  ePreviousR = e;

  float speed = fabs(u);

  if (speed > 255) {
    speed = 255;
  }
  else if (speed < turnSpeedMinimum) {
    speed = turnSpeedMinimum;
  }

  motorR->setSpeed(speed);

  if (u > 0) {
    motorR->run(BACKWARD);
  }
  else {
    motorR->run(FORWARD);
  }
}

void pidTurn() {
  long currentTime = micros();
  float deltaT = ((float)(currentTime - previousTimeO)) / 1.0e6;

  // compute error, derivative, integral
  int e;
  if (targetAngle == 180 && orientation < 0) {
    e = - targetAngle - orientation;
  } else if (targetAngle == -90 && orientation > 0) {
    e = orientation + targetAngle;
  } else if (targetAngle == 90 && orientation < 0) {
    e = orientation + targetAngle;
  } else {
    e = targetAngle - orientation;
  }
  /*
  Serial.print(orientation);
  Serial.print(", ");
  Serial.print(targetAngle);
  Serial.print(", ");
  Serial.println(e);
  /**/
  float eDerivative = (e - ePreviousO) / deltaT;
  eIntegralO = eIntegralO + e * deltaT;

  // compute PID control signal
  float u = (kpO * e) + (kdO * eDerivative) + (kiO * eIntegralO);

  previousTimeO = currentTime;
  ePreviousO = e;

  float speed = fabs(u);

  if (speed > turnSpeedLimit) {
    speed = turnSpeedLimit;
  }
  else if (speed < turnSpeedMinimum) {
    speed = turnSpeedMinimum;
  }

  motorL->setSpeed(speed);

  if (u > 0) {
    motorL->run(BACKWARD);
  }
  else {
    motorL->run(FORWARD);
  }
}

void pidDistanceMatchLeft(int target, int targetSpeed) {
  long currentTime = micros();
  float deltaT = ((float)(currentTime - previousTimeL2)) / 1.0e6;

  // compute error, derivative, integral
  int e = disL - target;
  float eDerivative = (e - ePreviousL2) / deltaT;
  eIntegralL2 = eIntegralL2 + e * deltaT;

  // compute PID control signal
  float u = (kpL2 * e) + (kdL2 * eDerivative) + (kiL2 * eIntegralL2);

  previousTimeL2 = currentTime;
  ePreviousL2 = e;

  float speed = fabs(u);

  if (speed > targetSpeed) {
    speed = targetSpeed;
  }
  else if (speed < turnSpeedMinimum) {
    speed = turnSpeedMinimum;
  }

  motorL->setSpeed(speed);

  if (u < 0) {
    motorL->run(BACKWARD);
  }
  else {
    motorL->run(FORWARD);
  }
}

void pidDistanceMatchRight(int target) {
  long currentTime = micros();
  float deltaT = ((float)(currentTime - previousTimeR2)) / 1.0e6;

  // compute error, derivative, integral
  int e = disR - target;
  float eDerivative = (e - ePreviousR2) / deltaT;
  // Serial.println(e);
  eIntegralR2 = eIntegralR2 + e * deltaT;

  // compute PID control signal
  float u = (kpR2 * e) + (kdR2 * eDerivative) + (kiR2 * eIntegralR2);

  previousTimeR2 = currentTime;
  ePreviousR2 = e;

  float speed = fabs(u);

  if (speed > 255) {
    speed = 255;
  }
  else if (speed < turnSpeedMinimum) {
    speed = turnSpeedMinimum;
  }

  motorR->setSpeed(speed);

  if (u < 0) {
    // temp fix
    motorR->run(BACKWARD);
  }
  else {
    motorR->run(FORWARD);
  }
}

void pidCourseCorrect() {
  // measure time since last adjustment
  long currentTime = micros();
  float deltaT = ((float)(currentTime - previousTimeC)) / 1.0e6;

  // compute error, derivative, integral
  float e;
  if (targetAngle == 180 && orientation < 0) {
    e = - targetAngle - orientation;
  } else {
    e = targetAngle - orientation;
  }
  Serial.println(e);
  float eDerivative = (e - ePreviousC) / deltaT;
  // Serial.println(e);
  eIntegralC = eIntegralC + e * deltaT;
  
  // compute PID control signal
  float u = (kpC * e) + (kdC * eDerivative) + (kiC * eIntegralC);

  previousTimeC = currentTime;
  ePreviousC = e;

  int speed = u + linearSpeedLimit;
  
  if (speed > 255) {
    speed = 255;
    motorR->setSpeed(speed);
    motorR->run(FORWARD);
  } else if (speed < 0) {
    if (speed < -255) {
      speed = -255;
    }
    motorR->setSpeed(fabs(speed));
    motorR->run(BACKWARD);
  } else {
    motorR->setSpeed(speed);
    motorR->run(FORWARD);
  }
}

void pidCourseCorrectBackward() {
  // measure time since last adjustment
  long currentTime = micros();
  float deltaT = ((float)(currentTime - previousTimeC)) / 1.0e6;

  // compute error, derivative, integral
  float e;
    if (targetAngle == 180 && orientation < 0) {
    e = targetAngle + orientation;
  } else {
    e = orientation - targetAngle;
  }
  float eDerivative = (e - ePreviousC) / deltaT;
  // Serial.println(e);
  eIntegralC = eIntegralC + e * deltaT;
  
  // compute PID control signal
  float u = (kpC * e) + (kdC * eDerivative) + (kiC * eIntegralC);

  previousTimeC = currentTime;
  ePreviousC = e;

  int speed = u + linearSpeedLimit;
  
  if (speed > 255) {
    speed = 255;
    motorR->setSpeed(speed);
    motorR->run(BACKWARD);
  } else if (speed < 0) {
    if (speed < -255) {
      speed = -255;
    }
    motorR->setSpeed(fabs(speed));
    motorR->run(FORWARD);
  } else {
    motorR->setSpeed(speed);
    motorR->run(BACKWARD);
  }
}

/*
void pidSpeedControl() {
  // measure time since last adjustment
  long currentTime = micros();
  float deltaT = ((float)(currentTime - previousTimeS)) / 1.0e6;

  // compute error, derivative, integral
  float e = theTargetSpeed - currentSpeed;
  float eDerivative = (e - ePreviousS) / deltaT;
  // Serial.println(e);
  eIntegralS = eIntegralS + e * deltaT;

  // compute PID control signal
  float u = (kpS * e) + (kdS * eDerivative) + (kiS * eIntegralS);

  previousTimeS = currentTime;
  ePreviousS = e;

  linearSpeedLimit += u;

  if (linearSpeedLimit > 245) {
    linearSpeedLimit = 245;
    motorL->setSpeed(linearSpeedLimit);
    motorL->run(FORWARD);
  } else if (linearSpeedLimit < 0) {
    if (linearSpeedLimit < -245) {
      linearSpeedLimit = -245;
    }
    motorL->setSpeed(fabs(linearSpeedLimit));
    motorL->run(BACKWARD);
  } else {
    motorL->setSpeed(linearSpeedLimit);
    motorL->run(FORWARD);
  }
}
*/

void updateEncoderL() {
  int MSB = digitalRead(encoderLPinA); // MSB = most significant bit
  int LSB = digitalRead(encoderLPinB); // LSB = least significant bit

  int encoded = (MSB << 1) | LSB; // converting the 2 pin value to single number
  int sum =
      (lastEncodedL << 2) | encoded; // adding it to the previous encoded value

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011)
    encoderValueL--;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000)
    encoderValueL++;

  lastEncodedL = encoded; // store this value for next time
}

void updateEncoderR() {
  int MSB = digitalRead(encoderRPinA); // MSB = most significant bit
  int LSB = digitalRead(encoderRPinB); // LSB = least significant bit

  int encoded = (MSB << 1) | LSB; // converting the 2 pin value to single number
  int sum =
      (lastEncodedR << 2) | encoded; // adding it to the previous encoded value

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011)
    encoderValueR--;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000)
    encoderValueR++;

  lastEncodedR = encoded; // store this value for next time
}

void motorStop() {
  motorL->setSpeed(0);
  motorR->setSpeed(0);
}

/**
 * Calculates distance to object in front using TWO front ultrasonic sensor
 * currently only using left sensor lol
 * @return distance in mm
 */
void getDistance() {
  digitalWrite(sensorLTrigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(sensorLTrigPin, LOW);

  float duration = pulseIn(sensorLEchoPin, HIGH);
  float distance = duration * 0.343 / 2;
  // 0.343: speed of sound in mm/microsecond
  // 2: round trip of sound
  disL = distance;
}

void resetIMU() {
  bool initialized = false;
  while (!initialized) {
    myICM.begin(WIRE_PORT, AD0_VAL);

    if (myICM.status != ICM_20948_Stat_Ok) {
      delay(500);
    }
    else {
      initialized = true;
    }
  }

  bool success = true;
  success &= (myICM.initializeDMP() == ICM_20948_Stat_Ok);
  success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR) == ICM_20948_Stat_Ok);
  success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Quat6, 0) == ICM_20948_Stat_Ok);
  success &= (myICM.enableFIFO() == ICM_20948_Stat_Ok);
  success &= (myICM.enableDMP() == ICM_20948_Stat_Ok);
  success &= (myICM.resetDMP() == ICM_20948_Stat_Ok);
  success &= (myICM.resetFIFO() == ICM_20948_Stat_Ok);

  if (!success) {
    SERIAL_PORT.println(F("Enable DMP failed!"));
    SERIAL_PORT.println(F("Please check that you have uncommented line 29 (#define ICM_20948_USE_DMP) in ICM_20948_C.h..."));
    while (1) {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(200);
      digitalWrite(LED_BUILTIN, LOW);
      delay(200);
    }
  }

  orientation = 0;
  targetAngle = 0;
}

/*
void calculateSpeed() {
    //turning is fixed speed
    //need variable turnTime
    //delay between actions = movementDelay = .1 seconds rn
    //linearSpeedLimit = linear speed what we are changing
    //make sure speed cant go negative if impossible to reach within time limit

    currentTime = millis() - startTime;
    timeLeft = targetTime - currentTime;

    //iterate through queue and count num of movements, turns, linear
    int totalMovements = commandQueue.size();
    int totalTurns = 0;
    int totalLinearMovements = 0;
    //now do stuff
    for (int i = 0; i < commandQueue.size();i++) {
        if(commandQueue.at(i) == RT || commandQueue.at(i) == LT)
            totalTurns += 1;
        //Since till is grouped with normal movement, only use till for one spaces and use normal movement before
        else if (commandQueue.at(i) == FD || commandQueue.at(i) == BD
                 || commandQueue.at(i) == FDT || commandQueue.at(i) == BDT)
            totalLinearMovements += 1;
    }


    int timeForOneLinear = (timeLeft - totalTurns * turnTime - (totalMovements - 1) * movementDelay) / totalLinearMovements;
    theTargetSpeed = 500 / timeForOneLinear;
}

void calculateCurrentSpeed() {
    //wheel dimensions: 60mm diamter
    int encoderValueLDiff = 0;
    int timeDiff = 0;
    
    if (encoderValueL != previousEncoderValueL) {
        encoderValueLDiff = abs(encoderValueL - previousEncoderValueL);
        timeDiff = millis() - previousTime; //time apparently doesn't reset, so should always be positive or 0
        previousTime = millis();
        previousEncoderValueL = encoderValueL;
        //not assuming time has passed just in case
        if (timeDiff != 0) {
            //current speed = 2 wheel radius pi / time per 8250 encoder pulses which is one revolution
            currentSpeed = 2 * 60 * PI / ((timeDiff / encoderValueLDiff /*time per one encoder*///) * 8250 /*num of encoder pulses*/);
        //}
        // mm/second
    //}

    // Serial.println(currentSpeed);
//}


// backup in case pidTurn is unreliable
/*
    case RT:
      // turn right
      if (newCommand) {
        // Serial.println(targetAngle);
        targetAngle -= 90;
        // Serial.println(targetAngle);
        if (targetAngle == -180) {
          targetAngle = 180;
        }
        motorL->setSpeed(turnSpeedLimit);
        motorL->run(FORWARD);
        newCommand = false;
        // Serial.println(targetAngle);
      } else {
        if ((targetAngle == 90 && orientation < 0) || (targetAngle != 180 && fabs(orientation - targetAngle) > directionTolerance) || (targetAngle == 180 && orientation < 0)) {
          pidPositionMatchRight(encoderValueRAbs - fabs(encoderValueL - encoderValueLAbs));
        } else {
          motorStop();
          commandQueue.dequeue();
          paramQueue.dequeue();
          encoderValueLAbs = encoderValueL;
          encoderValueRAbs = encoderValueR;
          newCommand = true;
          delay(movementDelay);
        }
      }
      break;
    case LT:
      // turn left
      if (newCommand) {
        // Serial.println(targetAngle);
        targetAngle += 90;
        // Serial.println(targetAngle);
        if (targetAngle == 270) {
          targetAngle = -90;
        }
        motorL->setSpeed(turnSpeedLimit);
        motorL->run(BACKWARD);
        newCommand = false;
        // Serial.println(targetAngle);
      } else {
        if ((targetAngle == -90 && orientation > 0) || (targetAngle != 180 && fabs(orientation - targetAngle) > directionTolerance) || (targetAngle == 180 && orientation > 0)) {
          pidPositionMatchRight(encoderValueRAbs + fabs(encoderValueL - encoderValueLAbs));
        } else {
          motorStop();
          commandQueue.dequeue();
          paramQueue.dequeue();
          encoderValueLAbs = encoderValueL;
          encoderValueRAbs = encoderValueR;
          newCommand = true;
          delay(movementDelay);
        }
      }
      break;
/**/
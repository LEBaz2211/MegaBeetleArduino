#include <util/atomic.h>
#include <Adafruit_MotorShield.h>
#include <PinChangeInterrupt.h>
#include <ArxContainer.h>
#include "calculations.h"

#define ENCA 3 // RR
#define ENCB 7 // RL
#define ENCC 6 // FL
#define ENCD 4 // FR

#define FRONT_SENSOR_PIN 12
#define BACK_SENSOR_PIN 9
#define LEFT_SENSOR_PIN 8
#define RIGHT_SENSOR_PIN 11

struct Task
{
  char type;   // 'F' for forward, 'B' for backward, 'L' for left, 'R' for right, 'T' for turn
  float value; // distance in cm for linear movements, angle in degrees for turns
  float speed; // speed of the movement
  float calibrationRollbackDist;
  bool calibrationEnd;
  bool initialCallib;
};

std::vector<Task> taskQueue;

int switchPin = 2;
bool systemActive = false;
unsigned long previousRunTime = 0;
unsigned long startRunTime = 0;
const long maxRunTime = 1000000;
bool resetSequence = false;
bool timeExceeded = false;

int currentCommand = 0;

float c = 18.85;

float initialCountsPerCmFB = 780 / c;
float initialCountsPerCmRL = 800.0 / c;
float initialCountsPerCmT = 960.0 / c;

float calibratedCountsPerCmFB = 780 / c;
float calibratedCountsPerCmRL = 800.0 / c;
float calibratedCountsPerCmT = 960.0 / c;

Adafruit_MotorShield AFMS = Adafruit_MotorShield();

Adafruit_DCMotor *FR = AFMS.getMotor(2); // 2
Adafruit_DCMotor *RR = AFMS.getMotor(1); // 1
Adafruit_DCMotor *FL = AFMS.getMotor(4); // 4
Adafruit_DCMotor *RL = AFMS.getMotor(3); // 3

volatile int posiM1 = 0;
volatile int posiM2 = 0;
volatile int posiM3 = 0;
volatile int posiM4 = 0;
volatile int distM1 = 0;
volatile int distM2 = 0;
volatile int distM3 = 0;
volatile int distM4 = 0;

#define WINDOW_SIZE 5 // Size of the moving average window
volatile int encoderReadingsM1[WINDOW_SIZE] = {0};
volatile int encoderReadingsM2[WINDOW_SIZE] = {0};
volatile int encoderReadingsM3[WINDOW_SIZE] = {0};
volatile int encoderReadingsM4[WINDOW_SIZE] = {0};
volatile int indexM1 = 0, indexM2 = 0, indexM3 = 0, indexM4 = 0;
volatile long sumM1 = 0, sumM2 = 0, sumM3 = 0, sumM4 = 0;
volatile int countM1 = 0, countM2 = 0, countM3 = 0, countM4 = 0;

float rpmM1 = 0;
float rpmM2 = 0;
float rpmM3 = 0;
float rpmM4 = 0;
unsigned long prevTime = 0;
unsigned long currentTime = 0;
unsigned long prevTimePID = 0;
unsigned long currentTimePID = 0;

float kp = 2.3;   // 0.9
float kd = 0.015; // 0.002
float ki = 0.2;   // 0.0

float eM1 = 0;
float eM2 = 0;
float eM3 = 0;
float eM4 = 0;
float prevEM1 = 0;
float prevEM2 = 0;
float prevEM3 = 0;
float prevEM4 = 0;
float eIntegralM1 = 0;
float eIntegralM2 = 0;
float eIntegralM3 = 0;
float eIntegralM4 = 0;

float m1Speed = 20; // a modifier pour imposer la vitesse
float m2Speed = 20;
float m3Speed = 20;
float m4Speed = 20;

uint8_t pwmM1 = 0;
uint8_t pwmM2 = 0;
uint8_t pwmM3 = 0;
uint8_t pwmM4 = 0;

float linearX = 0.0;
float linearY = 0.0;
float angularZ = 0.0;

unsigned long lastCommandTime = 0;

bool trajFinished = false;

void setup()
{
  pinMode(switchPin, INPUT_PULLUP); // Set pin as input with internal pull-up resistor

  // Initialize sensor pins
  pinMode(FRONT_SENSOR_PIN, INPUT);
  pinMode(BACK_SENSOR_PIN, INPUT);
  pinMode(LEFT_SENSOR_PIN, INPUT);
  pinMode(RIGHT_SENSOR_PIN, INPUT);

  Serial.begin(115200);
  pinMode(ENCA, INPUT_PULLUP);
  pinMode(ENCB, INPUT_PULLUP);
  pinMode(ENCC, INPUT_PULLUP);
  pinMode(ENCD, INPUT_PULLUP);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(ENCA), readEncoderM1, RISING);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(ENCD), readEncoderM2, RISING);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(ENCB), readEncoderM3, RISING);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(ENCC), readEncoderM4, RISING);

  if (!AFMS.begin())
  { // create with the default frequency 1.6KHz
    Serial.println("{\"event\":\"motor_shield_init\",\"status\":\"error\",\"message\":\"Could not find Motor Shield. Check wiring.\"}");
    while (1)
      ;
  }
  Serial.println("{\"event\":\"motor_shield_init\",\"status\":\"success\",\"message\":\"Motor Shield found.\"}");
}

void loop()
{
  if (timeExceeded)
  {
    Serial.printf("{\"event\":\"time_exceeded\",\"time_exceeded\":%lu}\n", startRunTime - previousRunTime);
  }
  else if (digitalRead(switchPin) == HIGH)
  {
    if (startRunTime == 0)
    {
      startRunTime = millis();
    }
    delay(1000);
    if (digitalRead(switchPin) == HIGH)
    {
      if (!systemActive)
      {
        while (!taskQueue.empty())
        {
          taskQueue.erase(taskQueue.begin());
        }
        startSystem();
        systemActive = true;
        // JAUNE
        //  addTask('L', 46, 70, false, 0, false);//70
        //  addTask('F', 90, 40, false, 0, false);//135
        //  addTask('R', 40, 70, false, 0, false);//55
        //  addTask('B', 15, 40, false, 0, false);

        // BLEU
        addTask('R', 46, 70, false, 0, false); // 70
        addTask('F', 90, 40, false, 0, false); // 135
        addTask('L', 40, 70, false, 0, false); // 55
        addTask('B', 15, 40, false, 0, false);

        // TEST DEVIATION
        // addTask('F', 150, 70, false, 0, false);

        //  addTask('B', 10, 30, false, 0, false);
        //  addTask('L', 10, 70, false, 0, false);
        //  addTask('F', 80, 70, false, 0, false);
        //  addTask('R', 50, 70, false, 0, false);
        //  addTask('T', 180, 50, false, 0, false);

        // addTask('F', 100, 70, false, 0, false);
        // addTask('B', 100, 70, false, 0, false);
        // addTask('F', 50, 70, false, 0, false);
        // addTask('L', 50, 70, false, 0, false);
        // addTask('R', 100, 70, false, 0, false);
        // addTask('L', 50, 70, false, 0, false);
        // addTask('T', 90, 50, false, 0, false);
        // addTask('T', -180, 50, false, 0, false);

        // addTask('R', 160, 70, false, 0, false);
        // addTask('T', 65, 50, false, 0, false);
        // addTask('F', 170, 50, true, 50, false);
        // addTask('T', 35, 50, false, 0, false);
        // addTask('R', 150, 70, true, 10, false);

        // addTask('T', 90, 50, false, 0);
        // addTask('T', -180, 50, false, 0);
            }
      processTasks();
      if (resetSequence)
      {
        while (!taskQueue.empty())
        {
          taskQueue.erase(taskQueue.begin());
        }
        resetSequence = false;
        systemActive = false;
        unsigned long startRunTime = 0;
      }
    }
  }
  else
  {
    systemActive = false;
    trajFinished = false;
    resetSequence = false;
  }

  delay(10);
}

void startSystem()
{
  Serial.println("{\"event\":\"system_start\",\"message\":\"System Starting...\"}");
}

void moveRobot(float value, float speed, char type, bool callibFlag, bool initialCallibFlag)
{
  int targetCounts = calculateTargetCounts(value, getCalibratedCountsPerCm(type), type);
  Serial.printf("{\"event\":\"move_robot\",\"target_counts\":%d,\"type\":\"%c\"}\n", targetCounts, type);

  posiM1 = 0;
  posiM2 = 0;
  posiM3 = 0;
  posiM4 = 0;

  setMotorSpeedsBasedOnDirection(speed, type, value);

  bool obstacleDetected = false;
  int prevPosition = 0;
  int similarPosCount = 0;

  while (true)
  {
    unsigned long currentRunTime = millis();
    if (currentRunTime - startRunTime >= maxRunTime)
    {
      Serial.println("{\"event\":\"timeout\",\"message\":\"Time up, stopping the robot\"}");
      stop();
      timeExceeded = true;
      break;
    }
    int posM1, posM2, posM3, posM4;

    Serial.printf("{\"event\":\"encoder_readings\",\"distM1\":%d,\"distM2\":%d,\"distM3\":%d,\"distM4\":%d}\n", distM1, distM2, distM3, distM4);

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
      posM1 = distM1;
      posM2 = distM2;
      posM3 = distM3;
      posM4 = distM4;
    }

    int effectivePosition = calculateEffectivePosition(posM1, posM2, posM3, posM4, type);
    Serial.printf("{\"event\":\"effective_position\",\"position\":%d}\n", effectivePosition);

    if (!callibFlag)
    {
      obstacleDetected = checkForObstacles(type);
    }
    else
    {
      obstacleDetected = false;
      if (prevPosition == effectivePosition && prevPosition != 0)
      {
        similarPosCount++;
        if (similarPosCount == 5)
        {
          break;
        }
      }
    }
    if (prevPosition == effectivePosition && prevPosition != 0)
    {
      similarPosCount++;
      if (similarPosCount == 5)
      {
        Serial.println("{\"event\":\"position_stable\",\"message\":\"Position is stable.\"}");
      }
    }

    if (initialCallibFlag)
    {
      Serial.println("{\"event\":\"initial_calibration\",\"message\":\"Initial calibration in progress.\"}");
      if (prevPosition == effectivePosition && prevPosition != 0)
      {
        similarPosCount++;
        if (similarPosCount == 5)
        {
          float initialValue = 0;
          if (type == 'F' || type == 'B')
          {
            initialValue = initialCountsPerCmFB;
          }
          else
          {
            initialValue = initialCountsPerCmRL;
          }
          float newValue = (effectivePosition / 5) / c;
          Serial.printf("{\"event\":\"calibration_update\",\"new_value\":%.2f,\"ratio\":%.2f}\n", newValue, newValue / initialValue);
          float ratio = newValue / initialValue;
          calibratedCountsPerCmFB = calibratedCountsPerCmFB * ratio;
          calibratedCountsPerCmRL = calibratedCountsPerCmRL * ratio;
          calibratedCountsPerCmT = calibratedCountsPerCmT * ratio;
          break;
        }
      }
    }

    prevPosition = effectivePosition;

    if (obstacleDetected)
    {
      stop();
      Serial.println("{\"event\":\"obstacle_detected\",\"message\":\"Stopping due to obstacle.\"}");
    }
    else
    {
      if (digitalRead(switchPin) == LOW)
      {
        resetSequence = true;
        break;
      }

      if (effectivePosition >= targetCounts)
      {
        break;
      }

      computeSpeed();
      computePower();
      applyMotorSpeeds();

      delay(10); // Small delay to prevent excessive CPU usage
    }
  }

  stop(); // Stop the motors once the target distance is reached or an obstacle permanently blocks the path
  distM1 = 0;
  distM2 = 0;
  distM3 = 0;
  distM4 = 0;
  delay(200);
}

void setMotorSpeedsBasedOnDirection(float speed, char direction, float degrees)
{
  switch (direction)
  {
  case 'F':
    m1Speed = -speed;
    m2Speed = -speed;
    m3Speed = speed; // 1.1
    m4Speed = speed;
    break;
  case 'B':
    m1Speed = speed;
    m2Speed = speed;
    m3Speed = -speed;
    m4Speed = -speed;
    break;
  case 'L':
    m1Speed = -speed;
    m2Speed = -speed;
    m3Speed = -speed;
    m4Speed = -speed;
    break;
  case 'R':
    m1Speed = speed;
    m2Speed = speed;
    m3Speed = speed;
    m4Speed = speed;
    break;
  case 'T':
    if (degrees > 0)
    { // Clockwise
      m1Speed = -speed;
      m2Speed = speed;
      m3Speed = -speed;
      m4Speed = speed;
    }
    else
    { // Counterclockwise
      m1Speed = speed;
      m2Speed = -speed;
      m3Speed = speed;
      m4Speed = -speed;
    }
  }
  Serial.printf("{\"event\":\"set_motor_speeds\",\"m1Speed\":%.2f,\"m2Speed\":%.2f,\"m3Speed\":%.2f,\"m4Speed\":%.2f}\n", m1Speed, m2Speed, m3Speed, m4Speed);
}

void addTask(char type, float value, float speed, bool calibrationEnd, float calibrationRollbackDist, bool initialCallib)
{
  Task newTask = {type, value, speed, calibrationRollbackDist, calibrationEnd, initialCallib};
  taskQueue.push_back(newTask);
  Serial.printf("{\"event\":\"add_task\",\"type\":\"%c\",\"value\":%.2f,\"speed\":%.2f,\"calibrationEnd\":%d,\"calibrationRollbackDist\":%.2f,\"initialCallib\":%d}\n", type, value, speed, calibrationEnd, calibrationRollbackDist, initialCallib);
}

void processTasks()
{
  while (!taskQueue.empty())
  {
    Task currentTask = taskQueue.front();
    Serial.printf("{\"event\":\"process_task\",\"type\":\"%c\",\"value\":%.2f,\"speed\":%.2f}\n", currentTask.type, currentTask.value, currentTask.speed);
    if (currentTask.calibrationEnd)
    {
      float calibDist = 30;
      float calibSpeed = 50;
      char calibRollbackType;
      if (currentTask.type == 'F')
      {
        calibRollbackType = 'B';
      }
      else if (currentTask.type == 'B')
      {
        calibRollbackType = 'F';
      }
      else if (currentTask.type == 'L')
      {
        calibRollbackType = 'R';
      }
      else if (currentTask.type == 'R')
      {
        calibRollbackType = 'L';
      }
      Serial.printf("{\"event\":\"calibration\",\"calibDist\":%.2f,\"calibSpeed\":%.2f,\"calibRollbackType\":\"%c\"}\n", calibDist, calibSpeed, calibRollbackType);
      moveRobot(currentTask.value - calibDist, currentTask.speed, currentTask.type, false, false);
      moveRobot(calibDist, calibSpeed, currentTask.type, true, false);
      Serial.println("{\"event\":\"calibration\",\"message\":\"Calibrating...\"}");
      delay(50);
      Serial.printf("{\"event\":\"calibration\",\"calibrationRollbackDist\":%.2f}\n", currentTask.calibrationRollbackDist);
      moveRobot(currentTask.calibrationRollbackDist, calibSpeed, calibRollbackType, true, false);
      Serial.println("{\"event\":\"task_done\",\"message\":\"Task done!\"}");
    }
    else
    {
      moveRobot(currentTask.value, currentTask.speed, currentTask.type, false, false);
      Serial.println("{\"event\":\"task_done\",\"message\":\"Task done!\"}");
    }

    taskQueue.erase(taskQueue.begin()); // Remove the task from the queue once completed

    if (resetSequence || timeExceeded)
    {
      break;
    }
  }
  if (!resetSequence)
  {
    trajFinished = true;
  }
}

void applyMotorSpeeds()
{
  if (m1Speed >= 0)
  {
    FR->setSpeed(min(abs(m1Speed), 255));
    FR->run(FORWARD);
  }
  else
  {
    FR->setSpeed(min(abs(m1Speed), 255));
    FR->run(BACKWARD);
  }

  if (m2Speed >= 0)
  {
    RR->setSpeed(min(abs(m2Speed), 255));
    RR->run(FORWARD);
  }
  else
  {
    RR->setSpeed(min(abs(m2Speed), 255));
    RR->run(BACKWARD);
  }

  if (m3Speed >= 0)
  {
    FL->setSpeed(min(abs(m3Speed), 255));
    FL->run(FORWARD);
  }
  else
  {
    FL->setSpeed(min(abs(m3Speed), 255));
    FL->run(BACKWARD);
  }

  if (m4Speed >= 0)
  {
    RL->setSpeed(min(abs(m4Speed), 255));
    RL->run(FORWARD);
  }
  else
  {
    RL->setSpeed(min(abs(m4Speed), 255));
    RL->run(BACKWARD);
  }
  Serial.printf("{\"event\":\"apply_motor_speeds\",\"pwmM1\":%d,\"pwmM2\":%d,\"pwmM3\":%d,\"pwmM4\":%d}\n", pwmM1, pwmM2, pwmM3, pwmM4);
}

void stop()
{
  FL->run(RELEASE);
  FR->run(RELEASE);
  RL->run(RELEASE);
  RR->run(RELEASE);
  Serial.println("{\"event\":\"stop\",\"message\":\"Motors stopped.\"}");
}

void readEncoderM1()
{
  sumM1 -= encoderReadingsM1[indexM1];
  encoderReadingsM1[indexM1] = 1; // Increment by 1 for each pulse
  sumM1 += encoderReadingsM1[indexM1];
  indexM1 = (indexM1 + 1) % WINDOW_SIZE;
  if (countM1 < WINDOW_SIZE)
  {
    countM1++;
  }
  posiM1 = sumM1 / countM1;
  distM1 += sumM1 / countM1;
}

void readEncoderM2()
{
  sumM2 -= encoderReadingsM2[indexM2];
  encoderReadingsM2[indexM2] = 1;
  sumM2 += encoderReadingsM2[indexM2];
  indexM2 = (indexM2 + 1) % WINDOW_SIZE;
  if (countM2 < WINDOW_SIZE)
  {
    countM2++;
  }
  posiM2 = sumM2 / countM2;
  distM2 += sumM2 / countM2;
}

void readEncoderM3()
{
  sumM3 -= encoderReadingsM3[indexM3];
  encoderReadingsM3[indexM3] = 1;
  sumM3 += encoderReadingsM3[indexM3];
  indexM3 = (indexM3 + 1) % WINDOW_SIZE;
  if (countM3 < WINDOW_SIZE)
  {
    countM3++;
  }
  posiM3 = sumM3 / countM3;
  distM3 += sumM3 / countM3;
}

void readEncoderM4()
{
  sumM4 -= encoderReadingsM4[indexM4];
  encoderReadingsM4[indexM4] = 1;
  sumM4 += encoderReadingsM4[indexM4];
  indexM4 = (indexM4 + 1) % WINDOW_SIZE;
  if (countM4 < WINDOW_SIZE)
  {
    countM4++;
  }
  posiM4 = sumM4 / countM4;
  distM4 += sumM4 / countM4;
}

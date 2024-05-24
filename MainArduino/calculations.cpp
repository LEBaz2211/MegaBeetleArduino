#include "calculations.h"
#include <math.h>
#include <Arduino.h>
#include <util/atomic.h> // Include for ATOMIC_BLOCK and ATOMIC_RESTORESTATE

// Define the sensor pins here
#define FRONT_SENSOR_PIN 12
#define BACK_SENSOR_PIN 9
#define LEFT_SENSOR_PIN 8
#define RIGHT_SENSOR_PIN 11

#define PI 3.1415926535897932384626433832795

extern volatile int posiM1, posiM2, posiM3, posiM4;
extern float rpmM1, rpmM2, rpmM3, rpmM4;
extern unsigned long prevTime;
extern unsigned long prevTimePID; // Declare prevTimePID as extern
extern float kp, kd, ki;
extern float eM1, eM2, eM3, eM4, prevEM1, prevEM2, prevEM3, prevEM4, eIntegralM1, eIntegralM2, eIntegralM3, eIntegralM4;
extern uint8_t pwmM1, pwmM2, pwmM3, pwmM4;
extern float m1Speed, m2Speed, m3Speed, m4Speed;

int calculateTargetCounts(float value, float calibratedCountsPerCm, char type)
{
    if (type == 'T')
    {
        float wheelBase = 23.0;
        float rotationCircumference = PI * wheelBase;
        float degreesPerCm = 360 / rotationCircumference;
        float distanceCm = value / degreesPerCm;
        return floor(fabs(distanceCm * calibratedCountsPerCm));
    }
    else
    {
        return floor(value * calibratedCountsPerCm);
    }
}

int calculateEffectivePosition(int posM1, int posM2, int posM3, int posM4, char direction)
{
    switch (direction)
    {
    case 'F':
    case 'B':
    case 'T':
        return (abs(posM1) + abs(posM2) + abs(posM3) + abs(posM4)) / 4;
    case 'L':
    case 'R':
        return (abs(posM1) + abs(posM2) + abs(posM3) + abs(posM4)) / 4;
    default:
        return 0;
    }
}

float getCalibratedCountsPerCm(char type)
{
    extern float calibratedCountsPerCmFB;
    extern float calibratedCountsPerCmRL;
    extern float calibratedCountsPerCmT;

    if (type == 'F' || type == 'B')
    {
        return calibratedCountsPerCmFB;
    }
    else if (type == 'L' || type == 'R')
    {
        return calibratedCountsPerCmRL;
    }
    else
    {
        return calibratedCountsPerCmT;
    }
}

bool checkForObstacles(char direction)
{
    switch (direction)
    {
    case 'F':
        return digitalRead(FRONT_SENSOR_PIN) == HIGH;
    case 'B':
        return digitalRead(BACK_SENSOR_PIN) == HIGH;
    case 'L':
        return digitalRead(LEFT_SENSOR_PIN) == HIGH;
    case 'R':
        return digitalRead(RIGHT_SENSOR_PIN) == HIGH;
    default:
        return false;
    }
}

void computeSpeed()
{
    int posM1, posM2, posM3, posM4;
    unsigned long currentTime = millis();
    unsigned long timeDiff = currentTime - prevTime;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        posM1 = posiM1;
        posM2 = posiM2;
        posM3 = posiM3;
        posM4 = posiM4;
    }

    rpmM1 = (posM1 / 800.0) * (60000.0 / timeDiff);
    rpmM2 = (posM2 / 800.0) * (60000.0 / timeDiff);
    rpmM3 = (posM3 / 800.0) * (60000.0 / timeDiff);
    rpmM4 = (posM4 / 800.0) * (60000.0 / timeDiff);

    prevTime = currentTime;
    posiM1 = 0;
    posiM2 = 0;
    posiM3 = 0;
    posiM4 = 0;
}

void computePower()
{
    unsigned long currentTimePID = millis();
    unsigned long timeDiffPID = currentTimePID - prevTimePID;

    eM1 = m1Speed - rpmM1;
    eM2 = m2Speed - rpmM2;
    eM3 = m3Speed - rpmM3;
    eM4 = m4Speed - rpmM4;

    float dedtM1 = (eM1 - prevEM1) / (timeDiffPID);
    float dedtM2 = (eM2 - prevEM2) / (timeDiffPID);
    float dedtM3 = (eM3 - prevEM3) / (timeDiffPID);
    float dedtM4 = (eM4 - prevEM4) / (timeDiffPID);

    eIntegralM1 = eIntegralM1 + eM1 * timeDiffPID;
    eIntegralM2 = eIntegralM2 + eM2 * timeDiffPID;
    eIntegralM3 = eIntegralM3 + eM3 * timeDiffPID;
    eIntegralM4 = eIntegralM4 + eM4 * timeDiffPID;

    pwmM1 = kp * eM1 + kd * dedtM1 + ki * eIntegralM1;
    pwmM2 = kp * eM2 + kd * dedtM2 + ki * eIntegralM2;
    pwmM3 = kp * eM3 + kd * dedtM3 + ki * eIntegralM3;
    pwmM4 = kp * eM4 + kd * dedtM4 + ki * eIntegralM4;

    pwmM1 = constrain(pwmM1, 0, 255);
    pwmM2 = constrain(pwmM2, 0, 255);
    pwmM3 = constrain(pwmM3, 0, 255);
    pwmM4 = constrain(pwmM4, 0, 255);

    prevEM1 = eM1;
    prevEM2 = eM2;
    prevEM3 = eM3;
    prevEM4 = eM4;
    prevTimePID = currentTimePID;
}

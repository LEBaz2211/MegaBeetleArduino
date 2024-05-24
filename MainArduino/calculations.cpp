#include "calculations.h"
#include <math.h>
#define PI 3.1415926535897932384626433832795

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

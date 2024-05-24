#include <AUnit.h>
#include "calculations.h"

test(calculateTargetCountsTest)
{
    assertEqual(calculateTargetCounts(90, 41.38, 'F'), 3724);
    assertEqual(calculateTargetCounts(180, 50.0, 'T'), 1806);
}

test(calculateEffectivePositionTest)
{
    assertEqual(calculateEffectivePosition(10, 20, 30, 40, 'F'), 25);
    assertEqual(calculateEffectivePosition(10, 10, 10, 10, 'R'), 10);
}

void setup()
{
    Serial.begin(115200);
}

void loop()
{
    aunit::TestRunner::run();
}

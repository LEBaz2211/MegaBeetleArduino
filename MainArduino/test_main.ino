#include <AUnit.h>
#include "calculations.h"

test(calculateTargetCountsTest)
{
    assertEqual(calculateTargetCounts(90, 41.38, 'F'), 3724);
    assertEqual(calculateTargetCounts(180, 50.0, 'T'), 1806);
}

void setup()
{
    Serial.begin(115200);
}

void loop()
{
    aunit::TestRunner::run();
}

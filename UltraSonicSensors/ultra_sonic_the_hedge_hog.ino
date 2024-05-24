const int trigPins[] = {1, 3, 5, 21, 19, 16, 7, 26}; // Trigger pins for each ultrasonic sensor
const int echoPins[] = {0, 2, 4, 20, 18, 17, 6, 27}; // Echo pins for each ultrasonic sensor
const int numSensors = 8;                            // Number of sensors

const int flagPins[] = {8, 9, 10, 11}; // Output pins for flags (front, right, rear, left)
const int sensorThreshold = 15;        // Threshold distance to trigger a flag (in cm)
float distances[numSensors];           // Store distances for each sensor
float filteredDistances[numSensors];   // Store filtered distances for each sensor
unsigned long previousMillis = 0;      // will store last time the sensors were updated
const long interval = 50;              // interval at which to run sensor readings (milliseconds)

void setup()
{
    Serial.begin(115200);
    for (int i = 0; i < numSensors; i++)
    {
        pinMode(trigPins[i], OUTPUT);
        pinMode(echoPins[i], INPUT);
    }
    for (int i = 0; i < 4; i++)
    {
        pinMode(flagPins[i], OUTPUT);
        digitalWrite(flagPins[i], LOW); // Initialize flags to LOW
    }
    pinMode(LED_BUILTIN, OUTPUT);
}

void loop()
{
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval)
    {
        previousMillis = currentMillis;
        for (int i = 0; i < numSensors; i++)
        {
            triggerSensor(i);
            float currentDistance = getFilteredDistance(i);
            if (currentDistance > 0)
            { // Only update if a valid distance is read
                filteredDistances[i] = currentDistance;
            }
            if (i == 0)
            {
                Serial.print("Sensor ");
                Serial.print(i);
                Serial.print(": ");
                Serial.println(filteredDistances[i]);
            }
        }
        checkAndSetFlags();
    }
}

void triggerSensor(int index)
{
    digitalWrite(trigPins[index], LOW);
    delayMicroseconds(5);
    digitalWrite(trigPins[index], HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPins[index], LOW);
}

float getFilteredDistance(int index)
{
    static float readings[numSensors][5];
    static int readIndex[numSensors] = {0};

    float newDistance = pulseIn(echoPins[index], HIGH, 38000) * 0.0343 / 2;
    if (newDistance == 0 && readings[index][(readIndex[index] + 4) % 5] > 0)
    {                                                              // if new reading is 0 but previous readings were not
        newDistance = readings[index][(readIndex[index] + 4) % 5]; // use the last valid reading instead
        Serial.println("ignoring false data");
    }

    readings[index][readIndex[index]] = newDistance;
    readIndex[index] = (readIndex[index] + 1) % 5; // move to the next index in the circular buffer

    float sum = 0;
    for (int i = 0; i < 5; i++)
    {
        sum += readings[index][i];
    }
    return sum / 5;
}

void checkAndSetFlags()
{
    // Set flags if distances are below the threshold
    digitalWrite(flagPins[0], (filteredDistances[0] < sensorThreshold || filteredDistances[5] < sensorThreshold)); // Front flags
    digitalWrite(flagPins[1], (filteredDistances[1] < sensorThreshold || filteredDistances[6] < sensorThreshold)); // Right flag
    digitalWrite(flagPins[2], (filteredDistances[2] < sensorThreshold || filteredDistances[3] < sensorThreshold)); // Rear flags
    digitalWrite(flagPins[3], (filteredDistances[4] < sensorThreshold || filteredDistances[7] < sensorThreshold)); // Left flag

    // Check if any flag is set and turn on built-in LED
    if (digitalRead(flagPins[0]) || digitalRead(flagPins[1]) || digitalRead(flagPins[2]) || digitalRead(flagPins[3]))
    {
        digitalWrite(LED_BUILTIN, HIGH);
    }
    else
    {
        digitalWrite(LED_BUILTIN, LOW);
    }
}
# Ultrasonic Sensor Array for Raspberry Pi Pico

This project uses an array of ultrasonic sensors to detect distances and set flags based on predefined thresholds. The setup is designed to run on a Raspberry Pi Pico board.

## Purpose

The primary purpose of this code is to read distances from multiple ultrasonic sensors and set corresponding flags when objects are detected within a certain threshold distance. It also provides real-time distance readings via serial output.

## Features

- **Multiple Ultrasonic Sensors**: Reads distance data from 8 ultrasonic sensors.
- **Filtered Readings**: Implements a basic filtering mechanism to ignore false readings.
- **Threshold-based Flag Setting**: Sets output flags when distances are below a specified threshold.
- **Built-in LED Indicator**: Turns on the built-in LED if any of the flags are set.

## Installation

To use this code, you will need to:

1. **Install the Required Libraries**: Ensure you have the necessary libraries installed in the Arduino IDE.
2. **Connect the Ultrasonic Sensors**: Connect the trigger and echo pins of your ultrasonic sensors to the specified GPIO pins on the Raspberry Pi Pico.
3. **Upload the Code**: Upload the provided code to your Raspberry Pi Pico using the Arduino IDE.

## Configuration

The code defines the GPIO pins for the trigger and echo of each ultrasonic sensor, as well as the output pins for the flags:

```cpp
const int trigPins[] = {1, 3, 5, 21, 19, 16, 7, 26}; // Trigger pins for each ultrasonic sensor
const int echoPins[] = {0, 2, 4, 20, 18, 17, 6, 27}; // Echo pins for each ultrasonic sensor
const int numSensors = 8; // Number of sensors

const int flagPins[] = {8, 9, 10, 11}; // Output pins for flags (front, right, rear, left)
const int sensorThreshold = 15; // Threshold distance to trigger a flag (in cm)
```

## Functions

- **setup()**: Initializes serial communication, configures sensor and flag pins, and sets the built-in LED pin.
- **loop()**: Periodically triggers each sensor, reads and filters distances, and sets flags based on the readings.
- **triggerSensor(index)**: Triggers the specified sensor to initiate a distance measurement.
- **getFilteredDistance(index)**: Filters the distance readings to provide more stable measurements.
- **checkAndSetFlags()**: Sets the output flags based on whether the filtered distances are below the threshold.

## Usage

The main loop periodically reads distances from each sensor and updates the filtered distance values. Flags are set if any sensor detects an object within the threshold distance. The current distance from the first sensor is also printed to the serial monitor for debugging purposes.

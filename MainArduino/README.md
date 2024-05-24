# Dependencies

The code is developed using the Arduino IDE. To ensure proper functionality, several libraries must be installed.

```cpp
#include <util/atomic.h>
#include <Adafruit_MotorShield.h>
#include <PinChangeInterrupt.h>
#include <ArxContainer.h>
```

# Movement

To make our code easy to use by anyone, we have implemented a function called `addTask`. Each time this function is called, a new movement task is appended to a list. The robot will then execute these movements sequentially in the order they were added.

The `addTask` function requires five parameters:

- `char type`: Specifies the type of movement. Use 'F' for forward, 'B' for backward, 'L' for left, 'R' for right, and 'T' for turn.
- `float distance/angle`: For linear movements, this is the distance in centimeters. For turns, this is the angle in degrees.
- `float speed`: The speed at which the movement should be performed.
- `bool calibrationEnd`: If set to true, the robot will move backward at the end of the task.
- `float calibrationRollbackDist`: If `calibrationEnd` is true, this parameter specifies the distance the robot will roll back.

Here is an example of how to add tasks to the list:

```cpp
addTask('R', 160, 70, false, 0);
addTask('T', 65, 50, false, 0);
addTask('F', 170, 50, true, 50);
```

By using this function, you can precisely control the sequence and nature of your robot's movements, ensuring it navigates the competition field effectively.

# Logging

To enhance the understanding and debugging of the internal state of the program, structured logging has been added. Logs are transmitted via the Serial interface and provide detailed information about the execution flow and state changes within the program.

## Example Log Messages

- **Initialization Logs**: Indicate the status of the motor shield initialization.
- **Event Logs**: Log major events such as starting the system, moving the robot, setting motor speeds, adding tasks, processing tasks, and stopping the robot.
- **Status Logs**: Provide updates on conditions like time exceeded, encoder readings, effective position, obstacle detection, calibration updates, and task completion.

Here is an example log message for adding a task:

```cpp
Serial.printf("{\"event\":\"add_task\",\"type\":\"%c\",\"value\":%.2f,\"speed\":%.2f,\"calibrationEnd\":%d,\"calibrationRollbackDist\":%.2f,\"initialCallib\":%d}\n", type, value, speed, calibrationEnd, calibrationRollbackDist, initialCallib);
```

# Code Structure and Modularization

To improve code readability and maintainability, the code has been refactored into multiple files:

- **main.ino**: The main Arduino sketch file that initializes the system, manages the main loop, and handles task processing.
- **calculations.h**: Header file declaring functions related to movement calculations.
- **calculations.cpp**: Source file implementing the functions declared in `calculations.h`.

## Functions Moved to Calculations Files

Several functions have been moved to `calculations.cpp` to keep the main file clean and focused on high-level logic:

- `calculateTargetCounts`
- `calculateEffectivePosition`
- `getCalibratedCountsPerCm`
- `checkForObstacles`
- `computeSpeed`
- `computePower`

# Testing

The project includes unit tests to ensure the correctness of the calculation functions. These tests are contained in a separate Arduino sketch file, `test_main.ino`.

**Important Note**: The Arduino IDE does not support having multiple `.ino` files with `setup()` and `loop()` functions in the same folder. To test the functions, you can either:

1. Place `main.ino` and `test_main.ino` in separate folders.
2. Comment out the `setup()` and `loop()` functions in the file you are not currently testing.

Example of how to organize test files:

- **main.ino folder**:

  ```
  main.ino
  calculations.h
  calculations.cpp
  ```

- **test_main.ino folder**:
  ```
  test_main.ino
  calculations.h
  calculations.cpp
  ```

This setup ensures that you can run and test each part of your code without conflicts.

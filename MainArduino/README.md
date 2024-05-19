## Mecanum Wheels

This section of our code is dedicated to the movement control of the mecanum wheels.

### Dependencies

The code is developed using the Arduino IDE. To ensure proper functionality, several libraries must be installed.

```
#include <util/atomic.h> // For the ATOMIC_BLOCK macro
#include <Adafruit_MotorShield.h>
#include <PinChangeInterrupt.h>
#include <ArxContainer.h>
```

### Movement

Our code is designed to be ready for immediate use. We have implemented a function called `addTask`. Each time this function is called, a new movement task is appended to a list. The robot will then execute these movements sequentially in the order they were added.

The `addTask` function requires five parameters:

- `char type`: Specifies the type of movement. Use 'F' for forward, 'B' for backward, 'L' for left, 'R' for right, and 'T' for turn.
- `float distance/angle`: For linear movements, this is the distance in centimeters. For turns, this is the angle in degrees.
- `float speed`: The speed at which the movement should be performed.
- `bool calibrationEnd`: If set to true, the robot will move backward at the end of the task.
- `float calibrationRollbackDist`: If `calibrationEnd` is true, this parameter specifies the distance the robot will roll back.

Here is an example of how to add tasks to the list:

```
addTask('R', 160, 70, false, 0);
addTask('T', 65, 50, false, 0);
addTask('F', 170, 50, true, 50);
```

By using this function, you can precisely control the sequence and nature of your robot's movements, ensuring it navigates the competition field effectively.

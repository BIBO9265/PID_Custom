# PID Custom

A compact PID controller library for Arduino.

## Version 2.0.0

Version 2 separates the class declaration and implementation into `PID.h` and `PID.cpp`.
The derivative term is calculated from the measured value rather than directly from the error, which reduces derivative kick when the setpoint changes abruptly.

## Breaking changes from v1.x

- `updatePID(double err)` is replaced by `updatePID(double r, double y)`.
- `standbyPID()` is replaced by `resetPID()`.

Existing v1.x sketches must update their function calls before using v2.0.0.

## Installation

In Arduino IDE, select **Sketch > Include Library > Add .ZIP Library...** and choose the ZIP file.

## Basic usage

```cpp
#include <PID.h>

PID controller(1.0, 0.1, 0.01);

double setpoint = 100.0;
double measurement = 0.0;

void setup() {
  controller.resetPID();
}

void loop() {
  const double output = controller.updatePID(setpoint, measurement);
}
```

See `examples/BasicUsage/BasicUsage.ino` for a complete minimal example.

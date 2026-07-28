#ifndef PID_H
#define PID_H

#include <Arduino.h>

class PID {
private:
  double kp = 0.0;
  double ki = 0.0;
  double kd = 0.0;
  double integral_maxima = 0.0;
  double filtering = 0.0;
  double y_previous = 0.0;
  double integral = 0.0;
  double differential = 0.0;
  unsigned long previous_time = 0;
  bool initialized = false;

  double calculateOutput(double err) const;

public:
  PID(double kp, double ki, double kd, double integral_maxima = 20, double filtering = 0.5);
  void setPID(double kp, double ki, double kd, double integral_maxima = 20, double filtering = 0.5);
  void resetPID();
  double updatePID(double r, double y);
};

#endif

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
  double err_previous = 0.0;
  double integral = 0.0;
  double differential = 0.0;
  unsigned long previous_time = 0;
  bool initialized = false;
public:
  PID(double kp, double ki, double kd, double integral_maxima = 20, double filtering = 0.5) {
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
    this->integral_maxima = constrain(fabs(integral_maxima), -100.0, 100.0);
    this->filtering = constrain(filtering, 0.0, 1.0);
  }
  ~PID() = default;
  void setPID(double kp, double ki, double kd, double integral_maxima = 20, double filtering = 0.5) {
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
    this->integral_maxima = constrain(fabs(integral_maxima), -100.0, 100.0);
    this->filtering = constrain(filtering, 0.0, 1.0);
  }
  void standbyPID() {
    previous_time = millis();
    err_previous = 0.0;
    integral = 0.0;
    differential = 0.0;
    initialized = false;
  }
  double updatePID(double err) {
    if (!initialized) {
      err_previous = err;
      previous_time = millis();
      initialized = true;
      return kp * err;
    }

    //dt
    unsigned long current_time = millis();
    if (current_time - previous_time == 0) {
      return kp * err + ki * integral + kd * differential;
    }
    double dt = (current_time - previous_time) / 1000.0;
    previous_time = current_time;

    //i
    integral += err * dt;
    integral = constrain(integral, 0 - integral_maxima, integral_maxima);

    //d
    double rawDifferential = (err - err_previous) / dt;
    err_previous = err;
    differential = filtering * differential + (1.0 - filtering) * rawDifferential;

    //output
    return kp * err + integral * ki + differential * kd;
  }
};

#endif

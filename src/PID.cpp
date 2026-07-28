#include "PID.h"

double PID::calculateOutput(double err) const {
  return kp * err + ki * integral - kd * differential;
}

PID::PID(double kp, double ki, double kd, double integral_maxima, double filtering) {
  this->kp = kp;
  this->ki = ki;
  this->kd = kd;
  this->integral_maxima = constrain(fabs(integral_maxima), 0.0, 100.0);
  this->filtering = constrain(filtering, 0.0, 1.0);
}

void PID::setPID(double kp, double ki, double kd, double integral_maxima, double filtering) {
  this->kp = kp;
  this->ki = ki;
  this->kd = kd;
  this->integral_maxima = constrain(fabs(integral_maxima), 0.0, 100.0);
  this->filtering = constrain(filtering, 0.0, 1.0);
}

void PID::resetPID() {
  previous_time = 0UL;
  y_previous = 0.0;
  integral = 0.0;
  differential = 0.0;
  initialized = false;
}

double PID::updatePID(double r, double y) {
  const unsigned long current_time = millis();
  const double err = r - y;

  if (!initialized) {
    y_previous = y;
    previous_time = current_time;
    initialized = true;
    return kp * err;
  }

  //dt
  const unsigned long elapsed_time = current_time - previous_time;
  if (elapsed_time == 0UL) {
    return calculateOutput(err);
  }
  const double dt = elapsed_time / 1000.0;
  previous_time = current_time;

  //i
  integral += err * dt;
  integral = constrain(integral, -integral_maxima, integral_maxima);

  //d
  const double rawDifferential = (y - y_previous) / dt;
  y_previous = y;
  differential = filtering * differential + (1.0 - filtering) * rawDifferential;

  //output
  return calculateOutput(err);
}

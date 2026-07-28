#include <PID.h>

PID controller(1.0, 0.1, 0.01);

double setpoint = 100.0;
double measurement = 0.0;

void setup() {
  Serial.begin(115200);
  controller.standbyPID();
}

void loop() {
  double error = setpoint - measurement;
  double output = controller.updatePID(error);

  Serial.println(output);
  delay(10);
}

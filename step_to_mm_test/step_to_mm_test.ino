#include "AccelStepper.h"

#define PUL_PIN 3
#define DIR_PIN 4
#define ENA_PIN 5

AccelStepper stepper(AccelStepper::DRIVER, PUL_PIN, DIR_PIN);

void setup() {
  pinMode(ENA_PIN, OUTPUT);
  digitalWrite(ENA_PIN, LOW);

  stepper.setMaxSpeed(1000);
  stepper.setAcceleration(2000);

  stepper.setCurrentPosition(0);
  stepper.moveTo(-400);   // 走 8000 step
}

void loop() {
  stepper.run();
}

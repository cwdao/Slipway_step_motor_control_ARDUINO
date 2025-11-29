#include "AccelStepper.h"

// 步进脉冲和方向引脚
#define PUL_PIN 3
#define DIR_PIN 4
#define ENA_PIN 5

// 微动开关引脚
#define SWITCH_FORWARD 6  // 控制正转
#define SWITCH_BACKWARD 7 // 控制反转

// 设置驱动器为驱动器模式（1 = DRIVER）
AccelStepper stepper(AccelStepper::DRIVER, PUL_PIN, DIR_PIN);

void setup() {
  // 步进电机设置
  pinMode(ENA_PIN, OUTPUT);
  digitalWrite(ENA_PIN, LOW); // LOW 表示使能（具体根据驱动器）

  stepper.setMaxSpeed(1000);    // 设置最大速度（步/秒）
  stepper.setAcceleration(500); // 设置加速度

  // 设置微动开关为输入并启用内部上拉
  pinMode(SWITCH_FORWARD, INPUT_PULLUP);
  pinMode(SWITCH_BACKWARD, INPUT_PULLUP);
}

void loop() {
  bool forwardPressed = digitalRead(SWITCH_FORWARD) == LOW;
  bool backwardPressed = digitalRead(SWITCH_BACKWARD) == LOW;

  if (forwardPressed && !backwardPressed) {
    stepper.setSpeed(500);  // 正向旋转
    stepper.runSpeed();
  } else if (backwardPressed && !forwardPressed) {
    stepper.setSpeed(-500); // 反向旋转
    stepper.runSpeed();
  } else {
    stepper.setSpeed(0); // 停止
  }
}

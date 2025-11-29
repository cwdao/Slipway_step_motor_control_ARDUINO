#include ".\\AccelStepper.h"

// 步进脉冲和方向引脚
#define PUL_PIN 3
#define DIR_PIN 4
#define ENA_PIN 5

// 微动开关引脚
#define SWITCH_FORWARD 6  // 控制正转
#define SWITCH_BACKWARD 7 // 控制反转

// 双击检测时间窗口（毫秒）
#define DOUBLE_CLICK_TIME 1000

// 周期性运动参数
#define AUTO_CYCLE_DISTANCE 2000  // 每次周期性运动的步数（可调）
#define AUTO_CYCLE_SPEED 500      // 周期性运动速度（步/秒，可调）

// 设置驱动器为驱动器模式（1 = DRIVER）
AccelStepper stepper(AccelStepper::DRIVER, PUL_PIN, DIR_PIN);

// 状态变量
enum MotorMode {
  MANUAL_MODE,      // 手动模式
  AUTO_CYCLE_MODE   // 周期性自动模式
};

MotorMode currentMode = MANUAL_MODE;

// 双击检测变量
unsigned long firstPressTime = 0;  // 第一次按下的时间
bool lastForwardState = HIGH;
bool waitingForSecondClick = false;  // 是否在等待第二次点击

// 周期性运动变量
long cycleStartPosition = 0;
bool cycleDirection = true;  // true = 正转, false = 反转
bool cycleInProgress = false;

// LED闪烁变量（用于周期性模式指示）
unsigned long lastLEDToggle = 0;
bool ledState = LOW;
const unsigned long LED_BLINK_INTERVAL = 200;  // 闪烁间隔（毫秒）

void setup() {
  // 步进电机设置
  pinMode(ENA_PIN, OUTPUT);
  digitalWrite(ENA_PIN, LOW); // LOW 表示使能（具体根据驱动器）

  stepper.setMaxSpeed(1000);    // 设置最大速度（步/秒）
  stepper.setAcceleration(500); // 设置加速度

  pinMode(LED_BUILTIN, OUTPUT);

  // 设置微动开关为输入并启用内部上拉
  pinMode(SWITCH_FORWARD, INPUT_PULLUP);
  pinMode(SWITCH_BACKWARD, INPUT_PULLUP);
}

void loop() {
  bool forwardPressed = digitalRead(SWITCH_FORWARD) == LOW;
  bool backwardPressed = digitalRead(SWITCH_BACKWARD) == LOW;
  unsigned long currentTime = millis();

  // 双击检测逻辑（仅在手动模式下检测）
  if (currentMode == MANUAL_MODE) {
    // 检测按钮按下（从HIGH到LOW的边沿）
    if (forwardPressed && !lastForwardState) {
      if (waitingForSecondClick && (currentTime - firstPressTime < DOUBLE_CLICK_TIME)) {
        // 双击检测成功，切换到周期性模式
        // 停止电机，等待按钮释放后再开始周期性运动
        stepper.setSpeed(0);
        currentMode = AUTO_CYCLE_MODE;
        cycleStartPosition = stepper.currentPosition();
        cycleDirection = true;
        cycleInProgress = false;  // 先不开始，等待按钮释放
        lastLEDToggle = currentTime;
        ledState = HIGH;
        digitalWrite(LED_BUILTIN, HIGH);
        // 重置双击检测状态
        waitingForSecondClick = false;
        firstPressTime = 0;
        // 初始化周期性模式的按钮状态，避免立即退出
        // 注意：这里使用外部变量来初始化静态变量
      } else {
        // 第一次按下，记录时间并开始等待第二次
        firstPressTime = currentTime;
        waitingForSecondClick = true;
      }
    }
    
    // 如果等待超时，重置状态
    if (waitingForSecondClick && (currentTime - firstPressTime >= DOUBLE_CLICK_TIME)) {
      waitingForSecondClick = false;
      firstPressTime = 0;
    }
    
    lastForwardState = forwardPressed;
  }

  // 根据模式执行相应操作
  if (currentMode == MANUAL_MODE) {
    // 手动模式：原有的按钮控制逻辑
    if (forwardPressed && !backwardPressed) {
      digitalWrite(LED_BUILTIN, HIGH);
      stepper.setSpeed(500);  // 正向旋转
      stepper.runSpeed();
    } else if (backwardPressed && !forwardPressed) {
      digitalWrite(LED_BUILTIN, HIGH);
      stepper.setSpeed(-500); // 反向旋转
      stepper.runSpeed();
    } else {
      stepper.setSpeed(0); // 停止
      digitalWrite(LED_BUILTIN, LOW);
    }
    
  } else if (currentMode == AUTO_CYCLE_MODE) {
    // 周期性自动模式
    // LED闪烁指示周期性模式
    if (currentTime - lastLEDToggle >= LED_BLINK_INTERVAL) {
      ledState = !ledState;
      digitalWrite(LED_BUILTIN, ledState);
      lastLEDToggle = currentTime;
    }

    // 如果按钮刚被按下（边沿检测），退出周期性模式
    static bool lastForwardInAutoMode = HIGH;
    static bool lastBackwardInAutoMode = HIGH;
    static bool firstTimeInAutoMode = true;
    
    // 第一次进入周期性模式时，初始化按钮状态
    if (firstTimeInAutoMode) {
      lastForwardInAutoMode = forwardPressed;
      lastBackwardInAutoMode = backwardPressed;
      firstTimeInAutoMode = false;
    }
    
    bool forwardEdge = forwardPressed && !lastForwardInAutoMode;
    bool backwardEdge = backwardPressed && !lastBackwardInAutoMode;
    
    if (forwardEdge || backwardEdge) {
      currentMode = MANUAL_MODE;
      cycleInProgress = false;
      firstTimeInAutoMode = true;  // 重置标志，下次进入时重新初始化
      digitalWrite(LED_BUILTIN, forwardPressed || backwardPressed ? HIGH : LOW);
      // 执行手动控制
      if (forwardPressed && !backwardPressed) {
        stepper.setSpeed(500);
        stepper.runSpeed();
      } else if (backwardPressed && !forwardPressed) {
        stepper.setSpeed(-500);
        stepper.runSpeed();
      } else {
        stepper.setSpeed(0);
        digitalWrite(LED_BUILTIN, LOW);
      }
      lastForwardInAutoMode = forwardPressed;
      lastBackwardInAutoMode = backwardPressed;
    } else {
      lastForwardInAutoMode = forwardPressed;
      lastBackwardInAutoMode = backwardPressed;
      
      // 如果按钮已释放，开始周期性运动
      if (!cycleInProgress && !forwardPressed && !backwardPressed) {
        cycleInProgress = true;
      }
      
      // 执行周期性运动
      if (cycleInProgress && !forwardPressed && !backwardPressed) {
        long currentPos = stepper.currentPosition();
        long targetPos = cycleStartPosition + (cycleDirection ? AUTO_CYCLE_DISTANCE : -AUTO_CYCLE_DISTANCE);
        
        if (cycleDirection) {
          // 正转
          if (currentPos < targetPos) {
            stepper.setSpeed(AUTO_CYCLE_SPEED);
            stepper.runSpeed();
          } else {
            // 到达目标，切换方向
            cycleDirection = false;
            cycleStartPosition = currentPos;
          }
        } else {
          // 反转
          if (currentPos > targetPos) {
            stepper.setSpeed(-AUTO_CYCLE_SPEED);
            stepper.runSpeed();
          } else {
            // 到达目标，切换方向
            cycleDirection = true;
            cycleStartPosition = currentPos;
          }
        }
      } else if (forwardPressed || backwardPressed) {
        // 按钮被按住，停止周期性运动
        stepper.setSpeed(0);
      }
    }
  }
}


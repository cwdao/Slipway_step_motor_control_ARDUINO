#include "AccelStepper.h"

// ====== 引脚 ======
#define PUL_PIN 3
#define DIR_PIN 4
#define ENA_PIN 5
#define SWITCH_FORWARD 6
#define SWITCH_BACKWARD 7

// ====== 双击检测 ======
#define DOUBLE_CLICK_TIME 1000

// ====== 机械标定（与 breathing_sim_script 一致，可按实测改） ======
static const float STEPS_PER_MM = 160.0f;

// ====== 实验参数 ======
static const uint8_t  BPM = 16;
static const float    BREATH_AMP_MM = 10.0f;   // 相对基线 ±10mm 三角波
static const uint8_t  BREATHS_PER_STATION = 6;
static const float    SHIFT_MM = 10.0f;        // 每次前移 1cm
static const uint8_t  NUM_SHIFTS = 15;         // 累计前移 15cm → 共 16 个位置
static const uint32_t START_DELAY_MS = 2000;   // 双击后等待 2s

// 手动点动速度
static const float MANUAL_SPEED = 500.0f;

// 运动平滑上限
static const float MAX_SPEED_STEPS_S = 3000.0f;
static const float ACCEL_STEPS_S2 = 6000.0f;

AccelStepper stepper(AccelStepper::DRIVER, PUL_PIN, DIR_PIN);

// ====== 模式 ======
enum MotorMode {
  MANUAL_MODE,
  SCAN_MODE
};

enum ScanState {
  ST_WAIT_RELEASE,   // 双击后等按钮松开
  ST_START_DELAY,    // 等待 2s
  ST_ALIGN_EXHALE,   // 移到呼气末（基线 - amp）
  ST_INHALE,         // 上行到吸气末
  ST_EXHALE,         // 下行到呼气末
  ST_SHIFT,          // 基线前移 1cm
  ST_DONE
};

static MotorMode currentMode = MANUAL_MODE;
static ScanState scanState = ST_WAIT_RELEASE;

// 双击检测
static unsigned long firstPressTime = 0;
static bool lastForwardState = HIGH;
static bool waitingForSecondClick = false;

// 扫描状态
static long centerPos = 0;          // 当前站位基线（步）
static uint8_t breathsDone = 0;     // 本站已完成呼吸次数
static uint8_t shiftsDone = 0;      // 已完成前移次数（0~15）
static uint32_t delayStartMs = 0;

static bool lastForwardInScan = false;
static bool lastBackwardInScan = false;
static bool firstTimeInScan = true;

// LED
static unsigned long lastLEDToggle = 0;
static bool ledState = LOW;
static const unsigned long LED_BLINK_INTERVAL = 200;

static inline long mmToSteps(float mm) {
  return lround(mm * STEPS_PER_MM);
}

static long ampSteps() {
  return mmToSteps(BREATH_AMP_MM);
}

static long exhaleEndPos() {
  return centerPos - ampSteps();
}

static long inhaleEndPos() {
  return centerPos + ampSteps();
}

// 16bpm、I:E=1:1 → 半周期 60/(16*2) = 1.875s；半行程 = 2*amp
static void setBreathHalfCycleSpeed() {
  float halfPeriod_s = 60.0f / (float)(BPM * 2);
  float dist_mm = 2.0f * BREATH_AMP_MM;
  float v_steps_s = (dist_mm / halfPeriod_s) * STEPS_PER_MM;
  if (v_steps_s > MAX_SPEED_STEPS_S) {
    v_steps_s = MAX_SPEED_STEPS_S;
  }
  stepper.setMaxSpeed(v_steps_s);
  stepper.setAcceleration(ACCEL_STEPS_S2);
}

static void setShiftSpeed() {
  // 前移用适中速度，不要求跟呼吸同步
  stepper.setMaxSpeed(800.0f);
  stepper.setAcceleration(2000.0f);
}

static void enterScanMode() {
  currentMode = SCAN_MODE;
  scanState = ST_WAIT_RELEASE;
  centerPos = stepper.currentPosition();
  breathsDone = 0;
  shiftsDone = 0;
  firstTimeInScan = true;
  lastLEDToggle = millis();
  ledState = HIGH;
  digitalWrite(LED_BUILTIN, HIGH);
}

static void exitToManual() {
  currentMode = MANUAL_MODE;
  firstTimeInScan = true;
  waitingForSecondClick = false;
  firstPressTime = 0;
  stepper.setSpeed(0);
}

void setup() {
  pinMode(ENA_PIN, OUTPUT);
  digitalWrite(ENA_PIN, LOW);

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(SWITCH_FORWARD, INPUT_PULLUP);
  pinMode(SWITCH_BACKWARD, INPUT_PULLUP);

  stepper.setMinPulseWidth(2);
  stepper.setMaxSpeed(1000);
  stepper.setAcceleration(500);
  stepper.setCurrentPosition(0);
}

void loop() {
  bool forwardPressed = digitalRead(SWITCH_FORWARD) == LOW;
  bool backwardPressed = digitalRead(SWITCH_BACKWARD) == LOW;
  unsigned long now = millis();

  // ---- 手动模式：双击检测 + 点动 ----
  if (currentMode == MANUAL_MODE) {
    if (forwardPressed && !lastForwardState) {
      if (waitingForSecondClick && (now - firstPressTime < DOUBLE_CLICK_TIME)) {
        stepper.setSpeed(0);
        enterScanMode();
        waitingForSecondClick = false;
        firstPressTime = 0;
      } else {
        firstPressTime = now;
        waitingForSecondClick = true;
      }
    }

    if (waitingForSecondClick && (now - firstPressTime >= DOUBLE_CLICK_TIME)) {
      waitingForSecondClick = false;
      firstPressTime = 0;
    }

    lastForwardState = forwardPressed;

    if (currentMode == MANUAL_MODE) {
      if (forwardPressed && !backwardPressed) {
        digitalWrite(LED_BUILTIN, HIGH);
        stepper.setSpeed(MANUAL_SPEED);
        stepper.runSpeed();
      } else if (backwardPressed && !forwardPressed) {
        digitalWrite(LED_BUILTIN, HIGH);
        stepper.setSpeed(-MANUAL_SPEED);
        stepper.runSpeed();
      } else {
        stepper.setSpeed(0);
        digitalWrite(LED_BUILTIN, LOW);
      }
    }
    return;
  }

  // ---- 扫描模式 ----
  // LED 闪烁
  if (scanState != ST_DONE) {
    if (now - lastLEDToggle >= LED_BLINK_INTERVAL) {
      ledState = !ledState;
      digitalWrite(LED_BUILTIN, ledState);
      lastLEDToggle = now;
    }
  } else {
    digitalWrite(LED_BUILTIN, LOW);
  }

  // 任意键边沿：中断扫描，回手动
  if (firstTimeInScan) {
    lastForwardInScan = forwardPressed;
    lastBackwardInScan = backwardPressed;
    firstTimeInScan = false;
  }

  bool forwardEdge = forwardPressed && !lastForwardInScan;
  bool backwardEdge = backwardPressed && !lastBackwardInScan;
  lastForwardInScan = forwardPressed;
  lastBackwardInScan = backwardPressed;

  if (scanState != ST_WAIT_RELEASE && scanState != ST_DONE) {
    if (forwardEdge || backwardEdge) {
      exitToManual();
      return;
    }
  }

  // DONE 状态下按任意键回手动
  if (scanState == ST_DONE) {
    if (forwardEdge || backwardEdge) {
      exitToManual();
    }
    return;
  }

  switch (scanState) {
    case ST_WAIT_RELEASE:
      if (!forwardPressed && !backwardPressed) {
        delayStartMs = now;
        scanState = ST_START_DELAY;
      }
      break;

    case ST_START_DELAY:
      if (now - delayStartMs >= START_DELAY_MS) {
        setBreathHalfCycleSpeed();
        stepper.moveTo(exhaleEndPos());
        scanState = ST_ALIGN_EXHALE;
      }
      stepper.run();
      break;

    case ST_ALIGN_EXHALE:
      stepper.run();
      if (stepper.distanceToGo() == 0) {
        breathsDone = 0;
        setBreathHalfCycleSpeed();
        stepper.moveTo(inhaleEndPos());
        scanState = ST_INHALE;
      }
      break;

    case ST_INHALE:
      stepper.run();
      if (stepper.distanceToGo() == 0) {
        setBreathHalfCycleSpeed();
        stepper.moveTo(exhaleEndPos());
        scanState = ST_EXHALE;
      }
      break;

    case ST_EXHALE:
      stepper.run();
      if (stepper.distanceToGo() == 0) {
        breathsDone++;
        if (breathsDone >= BREATHS_PER_STATION) {
          if (shiftsDone >= NUM_SHIFTS) {
            // 最后一站也做完：停在原地（当前在呼气末）
            scanState = ST_DONE;
          } else {
            // 基线前移 1cm（与前进键同向 = 正方向）
            centerPos += mmToSteps(SHIFT_MM);
            shiftsDone++;
            setShiftSpeed();
            // 平移到新站的呼气末，再继续呼吸
            stepper.moveTo(exhaleEndPos());
            scanState = ST_SHIFT;
          }
        } else {
          setBreathHalfCycleSpeed();
          stepper.moveTo(inhaleEndPos());
          scanState = ST_INHALE;
        }
      }
      break;

    case ST_SHIFT:
      stepper.run();
      if (stepper.distanceToGo() == 0) {
        breathsDone = 0;
        setBreathHalfCycleSpeed();
        stepper.moveTo(inhaleEndPos());
        scanState = ST_INHALE;
      }
      break;

    case ST_DONE:
      break;
  }
}

#include "AccelStepper.h"

// ====== 引脚定义 ======
#define PUL_PIN 3
#define DIR_PIN 4
#define ENA_PIN 5

#define BUZZER_PIN 8   // 外接蜂鸣器(建议串一个小电阻)，无源/有源都可用

// ====== 机械参数：务必校准 ======
static const float STEPS_PER_MM = 160.0f; // <-- TODO: 按你的丝杠/皮带/细分修改
static const long  CENTER_POS_STEPS = 0;  // 以当前上电位置为中心(0)

// ====== 运动平滑参数（需要按你的平台能力调） ======
static const float MAX_SPEED_STEPS_S = 3000.0f;  // 最高速度上限(步/秒)，防止太快丢步
static const float ACCEL_STEPS_S2    = 6000.0f;  // 加速度(步/秒^2)，越大换向越硬

// ====== 实验流程参数 ======
static const uint32_t START_DELAY_MS = 3000; // 上电等待
static const uint32_t PAUSE_MS = 10000;      // 呼吸暂停 10s

// 4段：9bpm 2min；12/16/18bpm 各1min
struct Segment {
  uint8_t bpm;
  uint16_t duration_s;
  bool pauseEvent;            // 本段是否插入一次暂停
  uint16_t pauseAfterBreaths; // 做完多少个“完整呼吸周期”后插入暂停（只对 pauseEvent 有效）
};

static const Segment segments[] = {
  {  9, 120, false, 0 },   // 2 min
  { 12,  60, true,  6 },   // 12bpm：第6次结束后暂停
  { 16,  60, false, 0 },
  { 18,  60, true,  9 },   // 18bpm：第9次结束后暂停
};
static const uint8_t NUM_SEG = sizeof(segments) / sizeof(segments[0]);

// 幅值：5-10-5 循环；每 10-15s 变化1mm（这里用固定 12s，可改随机）
static const uint32_t AMP_UPDATE_INTERVAL_MS = 12000;
static const uint8_t AMP_MIN_MM = 5;
static const uint8_t AMP_MAX_MM = 10;

// ====== AccelStepper 对象 ======
AccelStepper stepper(AccelStepper::DRIVER, PUL_PIN, DIR_PIN);

// ====== 运行状态机 ======
enum RunState {
  ST_START_DELAY,
  ST_SEGMENT_INIT,
  ST_MOVE_TO_EXHALE_END,   // 先到呼气末(下端点)
  ST_HALF_CYCLE_UP,        // 上行：呼气末 -> 吸气末
  ST_HALF_CYCLE_DOWN,      // 下行：吸气末 -> 呼气末
  ST_APNEA_HOLD,           // 暂停保持在呼气末
  ST_DONE
};

static RunState state = ST_START_DELAY;

static uint32_t t0_ms = 0;                  // 通用计时
static uint8_t segIdx = 0;
static uint32_t segStart_ms = 0;
static uint32_t lastAmpUpdate_ms = 0;

static uint8_t amp_mm = AMP_MIN_MM;
static int8_t amp_dir = +1;                 // +1 增，-1 减

static uint32_t halfCycleDuration_ms = 0;   // 半周期时间
static float cruiseSpeed_steps_s = 0;       // 匀速段目标速度（由幅值和bpm决定）

static uint16_t breathsCompleted = 0;       // 完整呼吸周期计数（下行结束算完成一个周期）
static bool apneaDoneInThisSeg = false;

// ====== 工具函数 ======
static inline long mmToSteps(float mm) {
  return lround(mm * STEPS_PER_MM);
}

// 计算当前段(bpm) + 当前幅值(mm)下的半周期时间和速度上限
void updateKinematics(uint8_t bpm, uint8_t ampMM) {
  // 周期(s) = 60/bpm；半周期(s)=30/bpm
  float half_s = 30.0f / (float)bpm;
  halfCycleDuration_ms = (uint32_t)lround(half_s * 1000.0f);

  // 从 -A 到 +A 的位移是 2A（A=幅值），半周期走 2A
  float dist_mm = 2.0f * (float)ampMM;
  float v_mm_s = dist_mm / half_s; // 近似匀速段速度需求
  cruiseSpeed_steps_s = v_mm_s * STEPS_PER_MM;

  // 安全限幅
  if (cruiseSpeed_steps_s > MAX_SPEED_STEPS_S) cruiseSpeed_steps_s = MAX_SPEED_STEPS_S;

  stepper.setMaxSpeed(cruiseSpeed_steps_s);
  stepper.setAcceleration(ACCEL_STEPS_S2);
}

// 幅值更新：5->10->5 循环
void updateAmplitude() {
  if (amp_mm == AMP_MAX_MM) amp_dir = -1;
  else if (amp_mm == AMP_MIN_MM) amp_dir = +1;
  amp_mm = (uint8_t)((int)amp_mm + (int)amp_dir);
}

void beep(uint16_t freq, uint16_t ms) {
  // tone 对无源蜂鸣器有效；对部分有源蜂鸣器也可工作
  tone(BUZZER_PIN, freq, ms);
}

bool segmentTimeUp() {
  uint32_t elapsed = millis() - segStart_ms;
  return elapsed >= (uint32_t)segments[segIdx].duration_s * 1000UL;
}

long exhaleEndPos() { // 呼气末：下端点
  return CENTER_POS_STEPS - mmToSteps((float)amp_mm);
}
long inhaleEndPos() { // 吸气末：上端点
  return CENTER_POS_STEPS + mmToSteps((float)amp_mm);
}

static void printSegments() {
  Serial.println(F("=== Segment table ==="));
  for (uint8_t i = 0; i < NUM_SEG; i++) {
    Serial.print(F("  ["));
    Serial.print(i);
    Serial.print(F("] bpm="));
    Serial.print(segments[i].bpm);
    Serial.print(F(", duration_s="));
    Serial.print(segments[i].duration_s);
    Serial.print(F(", pauseEvent="));
    Serial.print(segments[i].pauseEvent ? F("true") : F("false"));
    if (segments[i].pauseEvent) {
      Serial.print(F(", pauseAfterBreaths="));
      Serial.print(segments[i].pauseAfterBreaths);
    }
    Serial.println();
  }
  Serial.println(F("====================="));
}

void setup() {
  pinMode(ENA_PIN, OUTPUT);
  digitalWrite(ENA_PIN, LOW); // 使能（按你驱动器逻辑，LOW=Enable）

  pinMode(BUZZER_PIN, OUTPUT);

  Serial.begin(115200);
  delay(50);
  Serial.println();
  Serial.println(F("breathing_sim_script.ino starting..."));
  Serial.print(F("STEPS_PER_MM="));
  Serial.println(STEPS_PER_MM, 4);
  Serial.print(F("CENTER_POS_STEPS="));
  Serial.println(CENTER_POS_STEPS);
  Serial.print(F("MAX_SPEED_STEPS_S="));
  Serial.println(MAX_SPEED_STEPS_S, 2);
  Serial.print(F("ACCEL_STEPS_S2="));
  Serial.println(ACCEL_STEPS_S2, 2);
  Serial.print(F("START_DELAY_MS="));
  Serial.println(START_DELAY_MS);
  Serial.print(F("PAUSE_MS="));
  Serial.println(PAUSE_MS);
  Serial.print(F("AMP_MIN_MM="));
  Serial.print(AMP_MIN_MM);
  Serial.print(F(", AMP_MAX_MM="));
  Serial.print(AMP_MAX_MM);
  Serial.print(F(", AMP_UPDATE_INTERVAL_MS="));
  Serial.println(AMP_UPDATE_INTERVAL_MS);
  printSegments();

  stepper.setMinPulseWidth(2); // 视驱动器需要可调(微秒)
  stepper.setCurrentPosition(CENTER_POS_STEPS);

  // 初始运动学参数（先用第一段bpm + 初始幅值）
  updateKinematics(segments[0].bpm, amp_mm);

  // 上电提示
  beep(1200, 150);
  delay(200);
  beep(1200, 150);

  t0_ms = millis();
}

void loop() {
  // 必须尽可能频繁调用
  stepper.run();

  uint32_t now = millis();

  switch (state) {
    case ST_START_DELAY: {
      if (now - t0_ms >= START_DELAY_MS) {
        Serial.println(F("[STATE] START_DELAY done -> SEGMENT_INIT"));
        state = ST_SEGMENT_INIT;
      }
    } break;

    case ST_SEGMENT_INIT: {
      // 段初始化
      segStart_ms = now;
      lastAmpUpdate_ms = now;
      breathsCompleted = 0;
      apneaDoneInThisSeg = false;

      Serial.println();
      Serial.print(F("[SEGMENT START] idx="));
      Serial.print(segIdx);
      Serial.print(F(" bpm="));
      Serial.print(segments[segIdx].bpm);
      Serial.print(F(" duration_s="));
      Serial.print(segments[segIdx].duration_s);
      Serial.print(F(" pauseEvent="));
      Serial.print(segments[segIdx].pauseEvent ? F("true") : F("false"));
      if (segments[segIdx].pauseEvent) {
        Serial.print(F(" pauseAfterBreaths="));
        Serial.print(segments[segIdx].pauseAfterBreaths);
      }
      Serial.println();

      Serial.print(F("  amp_mm(initial)="));
      Serial.print(amp_mm);
      Serial.print(F(" (steps="));
      Serial.print(mmToSteps((float)amp_mm));
      Serial.println(F(")"));

      // 进入新段提示（按bpm响几声）
      for (uint8_t i = 0; i < segments[segIdx].bpm / 6; i++) { // 简单提示：bpm越大响越多(可自行改)
        beep(800, 60);
        delay(90);
      }

      updateKinematics(segments[segIdx].bpm, amp_mm);

      Serial.print(F("  halfCycleDuration_ms="));
      Serial.print(halfCycleDuration_ms);
      Serial.print(F(" cruiseSpeed_steps_s="));
      Serial.println(cruiseSpeed_steps_s, 2);

      // 先移动到呼气末，作为每段的起始相位
      Serial.println(F("[STATE] -> MOVE_TO_EXHALE_END (phase align)"));
      stepper.moveTo(exhaleEndPos());
      state = ST_MOVE_TO_EXHALE_END;
    } break;

    case ST_MOVE_TO_EXHALE_END: {
      if (stepper.distanceToGo() == 0) {
        Serial.println(F("[STATE] At exhale end -> HALF_CYCLE_UP"));
        // 开始上行半周期
        stepper.moveTo(inhaleEndPos());
        state = ST_HALF_CYCLE_UP;
      }
    } break;

    case ST_HALF_CYCLE_UP: {
      // 幅值按时间更新（只影响“下一次目标”，不打断当前半周期）
      if (now - lastAmpUpdate_ms >= AMP_UPDATE_INTERVAL_MS) {
        lastAmpUpdate_ms = now;
        updateAmplitude();
        updateKinematics(segments[segIdx].bpm, amp_mm);

        Serial.print(F("[AMP UPDATE] amp_mm="));
        Serial.print(amp_mm);
        Serial.print(F(" (steps="));
        Serial.print(mmToSteps((float)amp_mm));
        Serial.print(F(") halfCycleDuration_ms="));
        Serial.print(halfCycleDuration_ms);
        Serial.print(F(" cruiseSpeed_steps_s="));
        Serial.println(cruiseSpeed_steps_s, 2);
      }

      if (stepper.distanceToGo() == 0) {
        // 到达吸气末，开始下行
        stepper.moveTo(exhaleEndPos());
        state = ST_HALF_CYCLE_DOWN;
      }
    } break;

    case ST_HALF_CYCLE_DOWN: {
      if (now - lastAmpUpdate_ms >= AMP_UPDATE_INTERVAL_MS) {
        lastAmpUpdate_ms = now;
        updateAmplitude();
        updateKinematics(segments[segIdx].bpm, amp_mm);

        Serial.print(F("[AMP UPDATE] amp_mm="));
        Serial.print(amp_mm);
        Serial.print(F(" (steps="));
        Serial.print(mmToSteps((float)amp_mm));
        Serial.print(F(") halfCycleDuration_ms="));
        Serial.print(halfCycleDuration_ms);
        Serial.print(F(" cruiseSpeed_steps_s="));
        Serial.println(cruiseSpeed_steps_s, 2);
      }

      if (stepper.distanceToGo() == 0) {
        // 完成一个完整呼吸周期（下端点算结束）
        breathsCompleted++;

        Serial.print(F("[BREATH DONE] segIdx="));
        Serial.print(segIdx);
        Serial.print(F(" bpm="));
        Serial.print(segments[segIdx].bpm);
        Serial.print(F(" breath#="));
        Serial.print(breathsCompleted);
        Serial.print(F(" amp_mm="));
        Serial.print(amp_mm);
        Serial.print(F(" STEPS_PER_MM="));
        Serial.println(STEPS_PER_MM, 4);

        // 判断是否需要插入暂停（呼气末暂停）
        const Segment &seg = segments[segIdx];
        if (seg.pauseEvent && !apneaDoneInThisSeg && breathsCompleted >= seg.pauseAfterBreaths) {
          apneaDoneInThisSeg = true;
          t0_ms = now;
          beep(2000, 200); // 暂停开始提示

          Serial.print(F("[APNEA ENTER] segIdx="));
          Serial.print(segIdx);
          Serial.print(F(" bpm="));
          Serial.print(seg.bpm);
          Serial.print(F(" afterBreath#="));
          Serial.print(breathsCompleted);
          Serial.print(F(" hold_ms="));
          Serial.println(PAUSE_MS);

          state = ST_APNEA_HOLD;
          break;
        }

        // 段时间到则切段，否则继续下一周期
        if (segmentTimeUp()) {
          Serial.print(F("[SEGMENT END] idx="));
          Serial.print(segIdx);
          Serial.println(F(" timeUp=true -> next segment"));

          segIdx++;
          if (segIdx >= NUM_SEG) {
            Serial.println(F("[DONE] All segments finished."));
            beep(500, 500);
            state = ST_DONE;
          } else {
            state = ST_SEGMENT_INIT;
          }
        } else {
          // 继续下一次上行
          stepper.moveTo(inhaleEndPos());
          state = ST_HALF_CYCLE_UP;
        }
      }
    } break;

    case ST_APNEA_HOLD: {
      // 保持在呼气末：目标位置不变（已经在下端点了）
      if (now - t0_ms >= PAUSE_MS) {
        beep(1000, 150); // 暂停结束提示

        Serial.print(F("[APNEA EXIT] segIdx="));
        Serial.print(segIdx);
        Serial.print(F(" bpm="));
        Serial.print(segments[segIdx].bpm);
        Serial.print(F(" elapsed_ms="));
        Serial.println(now - t0_ms);

        // 暂停结束后继续上行
        stepper.moveTo(inhaleEndPos());
        state = ST_HALF_CYCLE_UP;
      }
    } break;

    case ST_DONE: {
      // 实验结束：停住并可选择断使能
      stepper.setSpeed(0);
      // digitalWrite(ENA_PIN, HIGH); // 若 HIGH=Disable，可启用
    } break;
  }
}
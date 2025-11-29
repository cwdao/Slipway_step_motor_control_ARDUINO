# Slipway Step Motor Control for Arduino

Arduino Uno项目，用于控制滑道步进电机，支持手动控制和周期性自动正反转功能。

## 功能特性

- **手动控制模式**：使用两个微动开关分别控制电机正转和反转
- **周期性自动模式**：通过双击正转按钮触发，电机自动进行周期性正反转运动
- **速度可调**：支持自定义周期性运动的速度和距离
- **LED状态指示**：
  - 按键按下时：LED常亮
  - 周期性运动时：LED闪烁（200ms间隔）

## 硬件要求

- Arduino Uno
- 步进电机驱动器（支持脉冲+方向控制）
- 2个微动开关（带内部上拉，按下时接地）
- 步进电机

## 引脚连接

### 步进电机驱动器
- `PUL_PIN` (Pin 3): 脉冲信号
- `DIR_PIN` (Pin 4): 方向信号
- `ENA_PIN` (Pin 5): 使能信号（LOW=使能）

### 微动开关
- `SWITCH_FORWARD` (Pin 6): 正转开关（内部上拉，按下时接地）
- `SWITCH_BACKWARD` (Pin 7): 反转开关（内部上拉，按下时接地）

### LED指示
- `LED_BUILTIN` (Pin 13): 板载LED，用于状态指示

## 按钮状态检测原理

代码使用边沿检测来判断按钮是否被按下。由于使用了内部上拉电阻（`INPUT_PULLUP`），按钮的工作原理如下：

- **未按下**：引脚通过上拉电阻连接到高电平（HIGH，约5V）
- **按下**：引脚接地（LOW，约0V）

代码中的检测逻辑：
```cpp
bool forwardPressed = digitalRead(SWITCH_FORWARD) == LOW;
```

### 状态变化过程

按钮状态变化时的检测逻辑如下表所示：

| 时刻 | 物理电平 | `forwardPressed` | `lastForwardState` | 条件判断 `forwardPressed && !lastForwardState` | 是否触发检测 |
|------|---------|------------------|-------------------|-----------------------------------------------|------------|
| 初始 | HIGH | `false` | `false` | `false && !false` = `false` | ❌ 否 |
| 按下瞬间 | LOW | `true` | `false` | `true && !false` = `true` | ✅ **是** |
| 按住 | LOW | `true` | `true` | `true && !true` = `false` | ❌ 否 |
| 松开瞬间 | HIGH | `false` | `true` | `false && !true` = `false` | ❌ 否 |
| 松开后 | HIGH | `false` | `false` | `false && !false` = `false` | ❌ 否 |

**关键点**：
- 代码只检测**按下边沿**（下降沿：HIGH → LOW），不检测松开边沿（上升沿：LOW → HIGH）
- 只有在按钮从**未按下变为按下**的瞬间才会触发双击检测
- 这样可以避免在按钮松开时误触发检测

## 使用方法

### 手动控制模式（默认）

1. **正转**：按下正转开关（Pin 6），电机正转，LED亮起
2. **反转**：按下反转开关（Pin 7），电机反转，LED亮起
3. **停止**：释放所有开关，电机停止，LED熄灭

### 周期性自动模式

1. **进入模式**：快速连续双击正转按钮（在1秒内按下两次）
2. **自动运行**：释放按钮后，电机开始周期性正反转运动
   - 从当前位置正转2000步
   - 然后反转2000步
   - 循环往复
3. **退出模式**：在周期性运动过程中，按下任意按钮即可退出并返回手动控制模式

### LED指示说明

- **常亮**：表示有按键被按下（手动控制模式）
- **闪烁**：表示已进入周期性自动模式（200ms闪烁间隔）
- **熄灭**：表示无按键按下且未在周期性模式

## 参数配置

在代码中可以调整以下参数：

```cpp
// 双击检测时间窗口（毫秒）
#define DOUBLE_CLICK_TIME 1000

// 周期性运动参数
#define AUTO_CYCLE_DISTANCE 2000  // 每次周期性运动的步数
#define AUTO_CYCLE_SPEED 500      // 周期性运动速度（步/秒）

// LED闪烁间隔（毫秒）
const unsigned long LED_BLINK_INTERVAL = 200;
```

### 电机速度设置

```cpp
stepper.setMaxSpeed(1000);    // 最大速度（步/秒）
stepper.setAcceleration(500); // 加速度
```

手动控制模式下的速度：
```cpp
stepper.setSpeed(500);  // 正转速度
stepper.setSpeed(-500); // 反转速度
```

## 依赖库

本项目使用 [AccelStepper](http://www.airspayce.com/mikem/arduino/AccelStepper/) 库来控制步进电机。

库文件已包含在项目中：
- `AccelStepper.h`
- `AccelStepper.cpp`

## 项目结构

```
.
├── Slipway_step_motor_control_ARDUINO.ino  # 主程序文件
├── AccelStepper.h                           # AccelStepper库头文件
├── AccelStepper.cpp                         # AccelStepper库实现文件
├── examples/                                # AccelStepper库示例代码
├── doc/                                     # AccelStepper库文档
└── README.md                                # 本文件
```

## 注意事项

1. 确保微动开关正确连接，按下时接地（LOW），释放时为高电平（HIGH，内部上拉）
2. 根据实际使用的步进电机驱动器调整使能信号逻辑（当前设置为LOW使能）
3. 双击检测需要在1秒内完成两次按下，可根据需要调整 `DOUBLE_CLICK_TIME`
4. 周期性运动的距离和速度可根据实际需求调整 `AUTO_CYCLE_DISTANCE` 和 `AUTO_CYCLE_SPEED`

## 许可证

请参考项目中的 LICENSE 文件。

## 作者

用于 BLE Sensing 实验的滑道电机控制系统。

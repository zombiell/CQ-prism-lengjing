# 03 电机驱动与 PWM 调速

> 对应中期考核指标 **2：电机运动（单一）**、**3：电机运动（全部）**

## 电机运动（单一）

### 功能说明

机器人能够单独驱动**任意一个**电机正转或反转，其余三个电机保持停止。这是硬件调试的基础功能，用于验证接线、确认转向标定。

### 实现方式

`motor.c` 提供 `motor_set(id, duty)` 接口，通过电机编号查表定位到对应的定时器通道：

```c
switch (id) {
case MOTOR_FL: htim = &htim2; ch_a = TIM_CHANNEL_1; ch_b = TIM_CHANNEL_2; break;
case MOTOR_FR: htim = &htim2; ch_a = TIM_CHANNEL_3; ch_b = TIM_CHANNEL_4; break;
case MOTOR_RL: htim = &htim3; ch_a = TIM_CHANNEL_1; ch_b = TIM_CHANNEL_2; break;
case MOTOR_RR: htim = &htim3; ch_a = TIM_CHANNEL_3; ch_b = TIM_CHANNEL_4; break;
default: return;
}
```

`chassis.c` 则封装了调试用的单轮测试函数：

```c
void chassis_wheel_test(motor_id_t id, float duty)
{
    motor_coast_all();          /* 先把其他三个停掉 */
    motor_set(id, duty);        /* 只驱动指定的这一个 */
}
```

**设计意图**：`chassis_wheel_test()` 内部**不经过**斜坡和死区补偿，直接调用 `motor_set()`。这样看到的是**纯硬件原始响应**，便于判断真实接线与转向。

### 验证方法

| 步骤 | 操作 | 预期现象 |
| --- | --- | --- |
| 1 | 架空机器人 | 四轮离地 |
| 2 | 调用 `chassis_wheel_test(MOTOR_FL, 0.8f)` | 只有前左轮转，其余静止 |
| 3 | 传 `-0.8f` | 前左轮反向转 |
| 4 | 依次测 FR / RL / RR | 每次只有目标轮转 |

---

## 电机运动（全部）

### 功能说明

四个电机能够**同时**独立驱动，各自的正反转与占空比互不干扰。

### 硬件基础：为什么需要 8 路 PWM

DRV8833 是**双路 H 桥**驱动芯片，每个通道有两个逻辑输入 `IN1` / `IN2`：

| IN1 | IN2 | 电机状态 |
| --- | --- | --- |
| PWM | 0 | 正转，转速由占空比决定 |
| 0 | PWM | 反转 |
| 0 | 0 | **滑行停止**（coast，输出断开，靠惯性转） |
| 1 | 1 | **刹车**（brake，电机两端短接） |

**一个电机要占两路 PWM**，因此 4 个电机 = **8 路 PWM**。

STM32F103C8T6 上正好有 TIM2 和 TIM3 两个通用定时器，各 4 个通道：

| 电机 | IN1（正转线） | IN2（反转线） | 驱动模块 |
| --- | --- | --- | --- |
| 前左 FL | PA0 / TIM2_CH1 | PA1 / TIM2_CH2 | DRV8833 #1 |
| 前右 FR | PA2 / TIM2_CH3 | PA3 / TIM2_CH4 | DRV8833 #1 |
| 后左 RL | PB4 / TIM3_CH1 | PB5 / TIM3_CH2 | DRV8833 #2 |
| 后右 RR | PB0 / TIM3_CH3 | PB1 / TIM3_CH4 | DRV8833 #2 |

> **注意**：TIM3 默认的 CH1/CH2 在 PA6/PA7 上，要挪到 PB4/PB5 必须开启**部分重映射**（`__HAL_AFIO_REMAP_TIM3_PARTIAL()`）。这行代码在 CubeMX 生成的 `HAL_TIM_MspPostInit()` 里。

### 正反转的实现

```c
if (duty > 0.0f) {
    /* 正转：A 路输出 PWM，B 路拉 0 */
    __HAL_TIM_SET_COMPARE(htim, ch_a, (uint32_t)(duty * MOTOR_PWM_MAX));
    __HAL_TIM_SET_COMPARE(htim, ch_b, 0u);
} else if (duty < 0.0f) {
    /* 反转：B 路输出 PWM，A 路拉 0 */
    __HAL_TIM_SET_COMPARE(htim, ch_a, 0u);
    __HAL_TIM_SET_COMPARE(htim, ch_b, (uint32_t)(-duty * MOTOR_PWM_MAX));
} else {
    /* 停止：两路都关，电机滑行 */
    __HAL_TIM_SET_COMPARE(htim, ch_a, 0u);
    __HAL_TIM_SET_COMPARE(htim, ch_b, 0u);
}
```

**为什么"负号"要在函数内部消化掉**：上层（`chassis.c`）只需要表达"这个轮子反转"，不需要知道底层是"换了另一根线输出 PWM"。这是**封装**的价值 —— 如果以后换用 L298N（需要方向脚 + 使能脚），只改 `motor_set()` 内部，上层一行都不用动。

### 验证方法

| 步骤 | 操作 | 预期现象 |
| --- | --- | --- |
| 1 | 架空，调用 `chassis_mecanum(0.6f, 0, 0)` | 四轮同向正转 |
| 2 | 调用 `chassis_mecanum(-0.6f, 0, 0)` | 四轮同向反转 |
| 3 | 调用 `chassis_mecanum(0, 0.6f, 0)` | 左右两侧反向（横移） |

---

## PWM 参数是怎么算出来的

### 时钟链路

```
HSE 8MHz 晶振
   └─ PLL ×9 ──► SYSCLK = 72MHz
        └─ AHB 不分频 ──► HCLK = 72MHz
             ├─ APB1 分频 2 ──► PCLK1 = 36MHz
             │     └─ 定时器时钟 ×2 ──► TIM2/TIM3 = 72MHz  ★
             └─ APB2 不分频 ──► PCLK2 = 72MHz
```

**这里有个容易搞错的地方**：TIM2/TIM3 挂在 APB1 上，APB1 被 2 分频到 36MHz，但 STM32 规定"**只要 APB 预分频系数 ≠ 1，定时器时钟就自动 ×2**"，所以实际是 36 × 2 = **72MHz**。

### 频率计算

```c
htim2.Init.Prescaler = 0;      /* 不分频 → 72MHz */
htim2.Init.Period    = 3599;   /* ARR = 3599 → 每 3600 个计数一个周期 */
```

$$
f_{PWM} = \frac{72\,000\,000}{(0+1) \times (3599+1)} = \frac{72\,000\,000}{3600} = 20\,000\ \text{Hz} = 20\,\text{kHz}
$$

### 为什么选 20kHz

| 频率 | 问题 / 优点 |
| --- | --- |
| < 1kHz | **人耳能听到刺耳的啸叫**，电机抖动明显 |
| 5~10kHz | 仍有可闻噪声 |
| **20kHz** | **超出人耳听觉上限（约 20kHz）**，静音；同时 MOSFET 开关损耗可接受 ✅ |
| > 40kHz | 开关损耗上升，DRV8833 响应可能跟不上 |

### 占空比与 CCR 的换算

$$
\text{CCR} = \text{duty} \times 3600
$$

| duty | CCR | 实际占空比 |
| --- | --- | --- |
| 0.30 | 1079 | 29.97% |
| 0.50 | 1799 | 49.97% |
| **0.60** | **2159** | 59.97% |
| 0.85（上限） | 3059 | 84.97% |
| 1.00 | 3599 | 99.97% |

> ARR = 3599 意味着计数范围是 0~3599 共 3600 级，最高占空比是 3599/3600 ≈ 99.97%，**永远达不到 100%**。这是 PWM1 模式的正常特性（CNT < CCR 时输出高电平）。

---

## 软启动斜坡（避免多电机同时起步的电流冲击）

### 为什么需要

TT 马达的**启动电流**是稳态电流的数倍。4 个轮子同时从 0 冲到目标占空比时，电池瞬间被拉低，可能触发驱动芯片欠压保护。

### 实现

`motor.c` 中用一个"影子变量"记录每个轮子**此刻真正输出的 duty**，每次调用只走一小步：

```c
static float s_motor_now[MOTOR_COUNT] = { 0, 0, 0, 0 };

void motor_set_ramp(motor_id_t id, float target)
{
    target = motor_deadzone_comp(target);   /* 先做死区补偿 */

    float now = s_motor_now[id];

    if (now < target) {
        now += MOTOR_RAMP_UP;               /* 加速：每 10ms 加 2% */
        if (now > target) now = target;     /* 别冲过头 */
    } else if (now > target) {
        now -= MOTOR_RAMP_DOWN;             /* 减速：每 10ms 减 8% */
        if (now < target) now = target;
    }

    s_motor_now[id] = now;
    motor_set(id, now);
}
```

| 参数 | 值 | 效果 |
| --- | --- | --- |
| `MOTOR_RAMP_UP` | 0.02 | 每 10ms 加 2% → 0→0.9 约 450ms |
| `MOTOR_RAMP_DOWN` | 0.08 | 每 10ms 减 8% → 松手约 110ms 停 |

**加速慢、减速快**是故意的：起步要柔和（保护电流），停下要干脆（操作手感）。

### ⚠️ 影子变量必须同步（这里踩过坑）

`motor_coast_all()` 和 `motor_brake_all()` 是**绕过斜坡直接写 PWM** 的。如果只把 PWM 写 0 而不清影子变量，下次起步时斜坡会认为"我已经在目标值了"，**一步都不走** —— 表现为"松手后再按没反应"。

所以这两个函数里都必须调用 `motor_ramp_reset()`：

```c
void motor_coast_all(void)
{
    motor_ramp_reset();                 /* ★ 关键：同步影子变量 */
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        motor_set((motor_id_t)i, 0.0f);
    }
}
```

---

## ★ 马达静摩擦死区与补偿算法（本项目最重要的技术发现）

### 故障现象

机器人**斜行能走**（左上/左下/右上/右下），但**直行 / 横移完全不动**（四个轮子一起不转）。

### 排查结论

经过对电源、驱动、电机、接线四条假设的逐一排除（完整过程见 [09 调试过程与问题排查](./09-调试过程与问题排查.md)），最终定位于：

> **TT 马达 + 塑料减速箱存在明显的静摩擦死区。**

实测数据（架空、逐个电机单独测）：

| duty | 结果 |
| --- | --- |
| **1.00** | ✅ 转 |
| **0.60** | ❌ 不转 |

**同一个电机、同一个电源、同一套代码，只改 duty 数值。**

### 为什么表现为"斜行能走、直行不走"

看麦轮逆运动学解出的四个轮速：

| 动作 | 参与的轮子 | 每个轮子的 duty |
| --- | --- | --- |
| 斜行（↑+→） | 2 个（FL + RR） | **1.00**（归一化后满速） |
| 直行（↑） | 4 个 | **0.60** |

**斜行的轮子反而跑得更快！** 所以"斜行能走"完全不能说明电源或力矩够用 —— 它只证明了那 2 个通道的硬件是好的。

这也是本项目最大的一个认知陷阱：眼看着斜行能跑，就以为硬件没问题，实际上直行的占空比刚好落在死区里。

### 解决方案：把有效区间重新映射

```c
static float motor_deadzone_comp(float duty)
{
    /* 0 = 真的要停，不能补偿成 0.8，否则松手后车会一直缓慢爬行 */
    if (duty == 0.0f) {
        return 0.0f;
    }

    float sign = (duty > 0.0f) ? 1.0f : -1.0f;
    float mag  = fabsf(duty);

    /* 把 [0, 1] 线性映射到 [MOTOR_DEADZONE, MOTOR_MAP_TOP] */
    mag = MOTOR_DEADZONE + mag * (MOTOR_MAP_TOP - MOTOR_DEADZONE);

    if (mag > MOTOR_DUTY_LIMIT) mag = MOTOR_DUTY_LIMIT;
    return sign * mag;
}
```

参数：

```c
#define MOTOR_DEADZONE     0.80f   /* 能可靠克服静摩擦的最小占空比 */
#define MOTOR_MAP_TOP      1.00f   /* 补偿后的输出上限（必须能到满速） */
```

映射结果：

| 命令 duty | 补偿后输出 | CCR |
| --- | --- | --- |
| 0.00 | **0.00**（真的停） | 0 |
| 0.30 | 0.86 | 3095 |
| 0.50 | 0.90 | 3239 |
| **0.60** | **0.92** | **3311** |
| 1.00 | **1.00** | 3599 |

### 为什么用线性映射，而不是"低于阈值一律抬到阈值"

如果简单地"小于 0.8 的全部改成 0.8"，就变成了**开关式控制** —— 手柄按一点和按到底速度一样，失去了调速的意义。

线性映射保留了**速度的比例关系**：斜行（原 1.00）→ 1.00，直行（原 0.60）→ 0.92，仍然能分出快慢。

### ⚠️ 参数整定中的第二个问题：`MOTOR_MAP_TOP` 不能压得太低

最初出于"限速保护硬件"的考虑，把 `MOTOR_MAP_TOP` 设为 **0.90**，结果**整车都不动了**。

原因：真实死区上界高于估计值，直行的 `0.60 → 0.828` 仍然落在死区里。

**结论**：
> `MOTOR_MAP_TOP` **必须保留 1.00**，让输出能够达到已经实测"能转"的那个值。宁可跑快一点，也不要把输出压进死区。

### 验证结果

补偿后：

| 动作 | 原始 duty | 补偿后 | 结果 |
| --- | --- | --- | --- |
| 前进 / 后退 | 0.60 | 0.92 | ✅ 四轮转动 |
| 左移 / 右移 | ±0.60 | ±0.92 | ✅ |
| 斜行 | 1.00 | 1.00 | ✅ |
| 原地自转 | ±0.60 | ±0.92 | ✅ |
| 松手 | 0 | 0.00 | ✅ 完全停止 |

---

## 安全保护

| 保护措施 | 实现 |
| --- | --- |
| **占空比上限** | `MOTOR_DUTY_LIMIT = 0.85f`，防止 TT 马达堵转电流（1~1.5A）和 DRV8833（每路 1.5A）过热 |
| **非法编号防护** | `switch` 的 `default: return;`，不写寄存器 |
| **急停立即生效** | `chassis_stop()` 直接调 `motor_coast_all()`，跳过斜坡 |
| **手柄掉线自动停** | `PS2_Scan()` 返回 0 → `chassis_stop() + g_estop = 1` |

---

## 本节小结

| 指标 | 实现 | 验证 |
| --- | --- | --- |
| 2 电机运动（单一） | `chassis_wheel_test()` 单独驱动任意电机 | 逐个轮子测，其余静止 ✅ |
| 3 电机运动（全部） | 8 路 PWM，TIM2 + TIM3，独立正反转与调速 | 四轮同时 / 反向 / 静止均正常 ✅ |

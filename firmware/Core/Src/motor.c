#include "motor.h"
#include "tim.h"      /* htim2 / htim3 在这里声明 */
#include <math.h>

/* ============================================================
 * 软启动（斜坡）用的影子变量
 *
 * s_motor_now[i] 记录第 i 个轮子【此刻真正输出的 duty】。
 * 斜坡就是拿它和目标值比较，然后一小步一小步逼近。
 *
 * ⚠️ 谁绕过 motor_set_ramp 直接改了 PWM，谁就必须负责把这里也归零，
 *    否则影子值和实际输出对不上 —— 典型后果是"下一次起步没反应"。
 * ============================================================ */
static float s_motor_now[MOTOR_COUNT] = { 0.0f, 0.0f, 0.0f, 0.0f };

/* ============================================================
 * 马达死区补偿
 *
 * 【为什么需要】
 *   实测（2026-10-15，架空状态，逐个电机单独测）：
 *       duty = 1.00  →  转
 *       duty = 0.60  →  不转
 *   同一个电机、同一个电源、同一套代码，只改 duty 数值。
 *
 *   原因：TT 马达 + 塑料减速箱的【静摩擦】。占空比太低时，
 *         电机产生的力矩克服不了静摩擦，转子锁死不转 → 表现为
 *         "通电但不转、电压也不掉"。
 *
 *   而斜行恰好每个轮子跑 1.00（归一化后满速），所以斜行能走；
 *   直行每个轮子只有 0.60，所以趴窝 —— 这就是当初
 *   "斜行能走、直行不走"的真正原因，和电源、驱动都无关。
 *
 * 【对策：把有效区间重新映射】
 *
 *   不补偿（原来的做法）：
 *       命令 0 ─────────────────────────────── 1.0
 *       输出 0 ─────────────────────────────── 1.0
 *                      ↑ 0.60 落在死区里，不转
 *
 *   补偿之后：
 *       命令 0 ─────────────────────────────── 1.0
 *       输出 0 ─┬─ 0.72 ────────────────────── 0.90
 *               └ 只要命令非 0，就至少给到 0.72
 *
 *   这样：
 *     - 任何非零命令都能真正驱动电机（不再有"命令了但不动"）
 *     - 相对速度比例保留（走得快慢还分得出来）
 *     - 上限压到 0.90 是为了保护马达和 DRV8833 不过热
 *
 * 【实测依据】
 *   0.60 不转、1.00 转 → 死区在 (0.60, 1.00] 之间。
 *   取 0.72 是保守起点（比已知失败的 0.60 高 20%）。
 *   如果发现低速起步还是偶尔卡住，就调大；
 *   如果想跑得更快，可以调到 0.85 甚至 0.90。
 * ============================================================ */
static float motor_deadzone_comp(float duty)
{
    /* 0 = 真的要停。绝不能补偿成 0.72，否则松手后车会一直缓慢爬行 */
    if (duty == 0.0f) {
        return 0.0f;
    }

    float sign = (duty > 0.0f) ? 1.0f : -1.0f;
    float mag  = fabsf(duty);

    /* 把 [0, 1] 线性映射到 [MOTOR_DEADZONE, MOTOR_MAP_TOP]
     *
     *   duty = 0.00 → 0.00   （上面已经提前返回）
     *   duty = 0.60 → 0.72   ← 原来卡死的值，现在抬进有效区
     *   duty = 1.00 → 0.90   ← 上限，保护硬件
     *
     * 用线性映射而不是"低于阈值一律抬到阈值"，好处是
     * 速度的比例关系保留（斜行仍比直行快），手感不会变成开关式。
     */
    mag = MOTOR_DEADZONE + mag * (MOTOR_MAP_TOP - MOTOR_DEADZONE);

    /* 兜底限幅：万一传进来超过 1.0 的值 */
    if (mag > MOTOR_DUTY_LIMIT) {
        mag = MOTOR_DUTY_LIMIT;
    }

    return sign * mag;
}

/* ============================================================
 * 初始化
 * ============================================================ */
void motor_init(void)
{
    /* 启动 TIM2 的 4 路 PWM（前左 PA0/PA1 + 前右 PA2/PA3） */
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

    /* 启动 TIM3 的 4 路 PWM（后左 PB4/PB5 + 后右 PB0/PB1） */
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

    /* 保险起见，上电确保全部停转 */
    motor_coast_all();
}

/* ============================================================
 * 设置单个电机的转速
 * ============================================================ */
void motor_set(motor_id_t id, float duty)
{
    TIM_HandleTypeDef *htim;
    uint32_t ch_a;      /* H 桥输入 A */
    uint32_t ch_b;      /* H 桥输入 B */

    /* ---------- 限幅 ---------- */
    if (duty >  MOTOR_DUTY_LIMIT) duty =  MOTOR_DUTY_LIMIT;
    if (duty < -MOTOR_DUTY_LIMIT) duty = -MOTOR_DUTY_LIMIT;

    /* ---------- 查表：编号 → 定时器通道 ---------- */
    switch (id) {
    case MOTOR_FL:
        htim = &htim2; ch_a = TIM_CHANNEL_1; ch_b = TIM_CHANNEL_2;
        break;
    case MOTOR_FR:
        htim = &htim2; ch_a = TIM_CHANNEL_3; ch_b = TIM_CHANNEL_4;
        break;
    case MOTOR_RL:
        htim = &htim3; ch_a = TIM_CHANNEL_1; ch_b = TIM_CHANNEL_2;
        break;
    case MOTOR_RR:
        htim = &htim3; ch_a = TIM_CHANNEL_3; ch_b = TIM_CHANNEL_4;
        break;
    default:
        return;
    }

    /* ---------- 写占空比 ---------- */
    if (duty > 0.0f) {
        /* 正转：A 路输出 PWM，B 路拉 0 */
        __HAL_TIM_SET_COMPARE(htim, ch_a, (uint32_t)(duty * (float)MOTOR_PWM_MAX));
        __HAL_TIM_SET_COMPARE(htim, ch_b, 0u);
    } else if (duty < 0.0f) {
        /* 反转：B 路输出 PWM，A 路拉 0 */
        __HAL_TIM_SET_COMPARE(htim, ch_a, 0u);
        __HAL_TIM_SET_COMPARE(htim, ch_b, (uint32_t)(-duty * (float)MOTOR_PWM_MAX));
    } else {
        /* 停止：两路都关，电机滑行 */
        __HAL_TIM_SET_COMPARE(htim, ch_a, 0u);
        __HAL_TIM_SET_COMPARE(htim, ch_b, 0u);
    }
}

/* ============================================================
 * 带软启动的单电机输出
 *
 * 用途：削平"4 个轮子同时起步"的电流尖峰。
 *
 * 原理：不直接把目标值写进 PWM，而是先和"当前值"比大小，
 *       每调用一次只走一小步（MOTOR_RAMP_UP / MOTOR_RAMP_DOWN），
 *       走满才等于目标值。
 *
 * 举例（MOTOR_RAMP_UP = 0.02，周期 10ms）：
 *      第 1 次调用 → 0.02
 *      第 2 次调用 → 0.04
 *      ...
 *      第 15 次调用 → 0.30（到位，约 150ms）
 *   于是 4 个轮子的总电流是被"摊开"的，电池不会被瞬间拉垮。
 *
 *   注：目标值是 chassis.c 降额之后的 duty（直行时 0.30），
 *       所以实际爬升时间约 150ms，不是 300ms。
 *
 * ⚠️ 必须在固定周期里调用。本工程 main.c 是 10ms 一次，正好匹配。
 * ============================================================ */
void motor_set_ramp(motor_id_t id, float target)
{
    if (id >= MOTOR_COUNT) {
        return;                     /* 非法编号，直接忽略 */
    }

    /* ---------- 死区补偿 ----------
     * 见 motor.h 里 MOTOR_DEADZONE 的说明。
     * 这里只改"目标值的大小"，方向由正负号原样保留。
     */
    target = motor_deadzone_comp(target);

    float now = s_motor_now[id];

    if (now < target) {
        /* ---- 需要加速 ---- */
        now += MOTOR_RAMP_UP;
        if (now > target) {
            now = target;           /* 别冲过头，到了就停 */
        }
    } else if (now > target) {
        /* ---- 需要减速（松手、换方向都走这里）---- */
        now -= MOTOR_RAMP_DOWN;
        if (now < target) {
            now = target;
        }
    }
    /* now == target 时什么都不用做 */

    s_motor_now[id] = now;
    motor_set(id, now);             /* 真正的限幅和写寄存器还是交给 motor_set */
}

/* ============================================================
 * 清掉斜坡内部状态
 * ============================================================ */
void motor_ramp_reset(void)
{
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        s_motor_now[i] = 0.0f;
    }
}

/* ============================================================
 * 全部滑行停止
 *
 * 注意：这里【必须】顺带把斜坡影子值清零。
 * 因为 motor_set() 直接把 PWM 写成 0 了，如果影子值还停在 0.6，
 * 下一次起步时斜坡会认为"我已经在 0.6 了"，于是一步都不走 —— 
 * 表现为"松手后再按就没反应"，这是最容易踩的坑。
 * ============================================================ */
void motor_coast_all(void)
{
    motor_ramp_reset();
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        motor_set((motor_id_t)i, 0.0f);
    }
}

/* ============================================================
 * 全部刹车
 * ============================================================ */
void motor_brake_all(void)
{
    /* 刹车是绕过斜坡直接写寄存器，同样要把影子值清零 */
    motor_ramp_reset();

    /* TIM2 四路 */
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, MOTOR_PWM_MAX);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, MOTOR_PWM_MAX);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, MOTOR_PWM_MAX);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, MOTOR_PWM_MAX);

    /* TIM3 四路 */
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, MOTOR_PWM_MAX);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, MOTOR_PWM_MAX);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, MOTOR_PWM_MAX);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, MOTOR_PWM_MAX);
}

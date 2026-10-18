#include "ps2.h"

/* ============================================================
 * 引脚定义 —— 与 CubeMX 配置必须一致
 *   在这里集中定义，将来换引脚只改这几行
 * ============================================================ */
#define PS2_DAT()   HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4)
#define CMD_H()     HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET)
#define CMD_L()     HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET)
#define CS_H()      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET)
#define CS_L()      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET)
#define CLK_H()     HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET)
#define CLK_L()     HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET)

/* ============================================================
 * 请求数据命令（9 字节）
 *   [0] 0x01  起始字节，固定
 *   [1] 0x42  查询模式（读取手柄状态）
 *   [2] 0x00  小振动马达强度 0x00~0xFF
 *   [3] 0x00  大振动马达强度 0x00~0xFF
 *   [4]~[8]   固定填 0
 *
 * 想加振动功能，就把 [2] / [3] 改成变量的值
 * ============================================================ */
static const uint8_t PS2_CMD[9] = {
    0x01, 0x42, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

static uint8_t ps2_rx[9];   /* 接收缓冲区 */

/* ============================================================
 * 微秒级延时 —— 用 TIM1 做一个 1MHz 的自由计数器
 *
 * 为什么不继续用 DWT？
 *   DWT 属于 Cortex-M3 的【调试组件】，只有在调试器连接时才
 *   保证工作。没有调试器时 DWT->CYCCNT 可能恒为 0，
 *   于是 while 条件永远成立 → 死循环 → 程序卡死。
 *   TIM1 是普通外设，任何情况下都可靠。
 *
 * TIM1 在本工程里完全空闲（电机 PWM 用 TIM2/TIM3，
 * CubeMX 里配的 TIM4 只是闲置的 PWM 输出，没接任何东西）
 * ============================================================ */
static void ps2_timer_init(void)
{
    __HAL_RCC_TIM1_CLK_ENABLE();

    TIM1->PSC = 71;              /* 72MHz / (71+1) = 1MHz，1 个计数 = 1us */
    TIM1->ARR = 0xFFFF;          /* 最大计数周期，约 65.5ms 溢出一次 */
    TIM1->EGR = TIM_EGR_UG;      /* 产生更新事件，把 PSC 装入影子寄存器 */
    TIM1->SR  = 0;               /* 清更新标志 */
    TIM1->CR1 = TIM_CR1_CEN;     /* 启动计数 */
}

static void ps2_delay_us(uint32_t us)
{
    uint32_t start = TIM1->CNT;
    /* CNT 是 16 位的，用掩码处理回绕；单次延时远小于 65.5ms，不会出问题 */
    while (((TIM1->CNT - start) & 0xFFFFu) < us) {
        /* 忙等 */
    }
}

/* ============================================================
 * 单字节全双工传输
 *   一边从 CMD 发一位命令，一边从 DAT 读一位数据
 *   LSB 先发，半周期约 16us（约 30kHz 时钟）
 * ============================================================ */
static uint8_t ps2_transfer(uint8_t cmd)
{
    uint8_t mask;
    uint8_t res = 0;          /* ⚠️ 必须初始化！原厂 F4 例程漏了这一步 */

    for (mask = 0x01; mask != 0x00; mask <<= 1) {
        /* 1. 把当前命令位放上 CMD 线 */
        if (cmd & mask) {
            CMD_H();
        } else {
            CMD_L();
        }

        /* 2. 拉低时钟，等接收器准备好 */
        CLK_L();
        ps2_delay_us(16);

        /* 3. 在时钟低电平期间读回数据位 */
        if (PS2_DAT()) {
            res |= mask;
        }

        /* 4. 拉高时钟，完成这一位 */
        CLK_H();
        ps2_delay_us(16);
    }

    return res;
}

/* ============================================================
 * 初始化
 *   ⚠️ 必须在 CubeMX 的 MX_GPIO_Init() 之后调用
 * ============================================================ */
void PS2_Init(void)
{
    /* 先让三条控制线进入空闲态
     * ATT(CS) 上电必须是高电平，否则接收器会进入异常状态
     */
    CMD_H();
    CLK_H();
    CS_H();

    /* 启动 TIM1 微秒计数器，替代原来依赖调试器的 DWT 方案 */
    ps2_timer_init();
}

/* ============================================================
 * 扫描一次手柄
 *   返回 1 = 数据有效
 *   返回 0 = 未连接 / 掉线（调用方必须立即停车！）
 *   建议调用周期 10ms
 * ============================================================ */
uint8_t PS2_Scan(ps2_state_t *st)
{
    uint8_t i;

    /* 拉低片选，开始一次通信 */
    CS_L();

    /* 连续收发 9 个字节 */
    for (i = 0; i < 9; i++) {
        ps2_rx[i] = ps2_transfer(PS2_CMD[i]);
    }

    /* 拉高片选，结束通信 */
    CS_H();

    /* ---------- 数据有效性检查（两层）----------
     * 第 1 层：byte1（模式字节）
     *     手册：0x41=数字模式, 0x23=NegCon, 0x73=工作模式
     *     接收器没接好或手柄掉线时通常是 0x00 或 0xFF
     * 第 2 层：byte2 固定为 0x5A（手册规定，含义是"数据来了"）
     *     这一层能挡住【字节错位】和【偶发噪声帧】——
     *     之前就是因为少了这一层，无效数据直接进了控制链路，
     *     表现为"手柄不动电机却一直转"
     */
    if (ps2_rx[1] == 0x00 || ps2_rx[1] == 0xFF) {
        return 0;
    }
    if (ps2_rx[2] != 0x5A) {
        return 0;
    }

    /* ---------- 数据解析 ----------
     * 数据帧格式（9 字节）：
     *   [0] 0x01 起始
     *   [1] 模式（0x41 / 0x23 / 0x73）
     *   [2] 0x5A 固定
     *   [3] 按键组 1（0=按下，取反后 1=按下）
     *         bit0 SLCT  bit1 JOYR  bit2 JOYL  bit3 STRT
     *         bit4 UP    bit5 RIGHT bit6 DOWN  bit7 LEFT
     *   [4] 按键组 2（0=按下）
     *         bit0 L2    bit1 R2    bit2 L1    bit3 R1
     *         bit4 ∧     bit5 ○     bit6 X     bit7 □
     *   [5] 右摇杆 左右   0x00=左 0xFF=右  ⚠️ 本手柄无效
     *   [6] 右摇杆 上下   0x00=上 0xFF=下  ⚠️ 本手柄无效
     *   [7] 左摇杆 左右                    ⚠️ 本手柄无效
     *   [8] 左摇杆 上下                    ⚠️ 本手柄无效
     *
     * ⚠️ 手册原文："左右摇杆模拟值为无效，推到极限时，
     *    对应发送 UP、RIGHT、DOWN、LEFT、∧、○、X、□"
     *    所以 [5]~[8] 不能用，控制只能靠 [3][4] 的按键位。
     *    下面仍把它们存下来，仅用于调试观察。
     */
    st->mode    = ps2_rx[1];
    st->btn1    = (uint8_t)(~ps2_rx[3]);   /* 取反：1 = 按下 */
    st->btn2    = (uint8_t)(~ps2_rx[4]);
    st->rjoy_lr = ps2_rx[5];
    st->rjoy_ud = ps2_rx[6];
    st->ljoy_lr = ps2_rx[7];
    st->ljoy_ud = ps2_rx[8];

    return 1;
}
#ifndef PS2_H
#define PS2_H

#include "main.h"
#include <stdint.h>

/* ============================================================
 * PS2 按键位定义
 * 手柄原始数据里 0 = 按下，我们在 ps2.c 里取反，所以这里是 1 = 按下
 * ============================================================ */

/* btn1 对应的 8 个按键（数据字节 3） */
#define PS2_BTN_SELECT   (1u << 0)   /* SELECT */
#define PS2_BTN_L3       (1u << 1)   /* 左摇杆按下 */
#define PS2_BTN_R3       (1u << 2)   /* 右摇杆按下 */
#define PS2_BTN_START    (1u << 3)   /* START  */
#define PS2_BTN_UP       (1u << 4)   /* 方向键 上 */
#define PS2_BTN_RIGHT    (1u << 5)   /* 方向键 右 */
#define PS2_BTN_DOWN     (1u << 6)   /* 方向键 下 */
#define PS2_BTN_LEFT     (1u << 7)   /* 方向键 左 */

/* btn2 对应的 8 个按键（数据字节 4） */
#define PS2_BTN_L2       (1u << 0)
#define PS2_BTN_R2       (1u << 1)
#define PS2_BTN_L1       (1u << 2)
#define PS2_BTN_R1       (1u << 3)
#define PS2_BTN_TRIANGLE (1u << 4)   /* △ 三角 */
#define PS2_BTN_CIRCLE   (1u << 5)   /* ○ 圆圈 */
#define PS2_BTN_CROSS    (1u << 6)   /* × 叉   */
#define PS2_BTN_SQUARE   (1u << 7)   /* □ 方块 */

/* ============================================================
 * 手柄状态
 *
 * ⚠️ 本手柄的摇杆【没有模拟量】。手册原文：
 *    "左右摇杆模拟值为无效，推到极限时，
 *     对应发送 UP、RIGHT、DOWN、LEFT、∧、○、X、□"
 *    所以下面 4 个摇杆字段仅在调试时观察用，
 *    【不要拿去做控制】——空闲时它们是无效数据，
 *    直接换算会得到满速命令，把电机驱得一直转。
 *    控制请用 btn1 / btn2 里的方向按键位。
 * ============================================================ */
typedef struct {
    uint8_t mode;      /* 手柄模式：0x41=数字模式, 0x23=NegCon, 0x73=工作模式 */
    uint8_t btn1;      /* 已取反，bit=1 表示按下 */
    uint8_t btn2;      /* 已取反，bit=1 表示按下 */
    uint8_t rjoy_lr;   /* 右摇杆 左右 —— 本手柄无效，仅调试用 */
    uint8_t rjoy_ud;   /* 右摇杆 上下 —— 本手柄无效，仅调试用 */
    uint8_t ljoy_lr;   /* 左摇杆 左右 —— 本手柄无效，仅调试用 */
    uint8_t ljoy_ud;   /* 左摇杆 上下 —— 本手柄无效，仅调试用 */
} ps2_state_t;

/* ============================================================
 * 接口函数
 * ============================================================ */
void    PS2_Init(void);
uint8_t PS2_Scan(ps2_state_t *st);   /* 1 = 读到有效数据，0 = 掉线/未连接 */

#endif /* PS2_H */
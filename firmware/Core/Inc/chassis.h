#ifndef CHASSIS_H
#define CHASSIS_H

#include "motor.h"

/* ============================================================
 * 麦轮底盘运动学
 *
 *   vx : 前后平移，+1.0 = 全速向前
 *   vy : 左右平移，+1.0 = 全速向右
 *   w  : 原地自转，+1.0 = 顺时针（从车顶往下看）
 *
 * 三个参数可以任意组合，内部会自动归一化防止超速
 * ============================================================ */
void chassis_mecanum(float vx, float vy, float w);

/* 全部停止 */
void chassis_stop(void);

/* 单轮测试：只让一个轮子转，用于硬件调试
 * 会先把其他轮子全部停掉
 */
void chassis_wheel_test(motor_id_t id, float duty);

/* 方向标定：如果发现某个轮子转向和预期相反，
 * 不用改接线，把对应参数设成 -1 即可
 */
void chassis_set_sign(int8_t s_fl, int8_t s_fr, int8_t s_rl, int8_t s_rr);

#endif /* CHASSIS_H */
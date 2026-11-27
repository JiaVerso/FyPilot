/*
 * @Author: Jack Shaw 2870998241@qq.com
 * @Date: 2026-11-23 15:30:16
 * @LastEditors: Jack Shaw 2870998241@qq.com
 * @LastEditTime: 2026-11-27 16:39:22
 * @FilePath: /FyPilot/applications/core/pid/control_task.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef CONTROL_TASK_H
#define CONTROL_TASK_H

#include <rtthread.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CONTROL_THREAD_STACK_SIZE 4096

typedef struct control_input_s
{
    float target_angle_rad[2];          /* roll、pitch 目标角度*/
    float target_yaw_rate_rad_s;
    float throttle;                     /* 0～1 */
    float angle_rad[3];                 /* EKF 姿态：roll、pitch、yaw */
    float gyro_rad_s[3];                /* 滤波后的机体系角速度 p、q、r */
    bool armed;                         /* 解锁状态机 */
} control_input_t;

rt_err_t mp_control_start(void);

#ifdef __cplusplus
}
#endif

#endif

/*
 * @Author: Jack Shaw 2870998241@qq.com
 * @Date: 2026-11-24 11:23:03
 * @LastEditors: Jack Shaw 2870998241@qq.com
 * @LastEditTime: 2026-11-24 14:51:03
 * @FilePath: /FyPilot/applications/middleware/umcn/uMcn_topic.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef FLIGHT_TOPICS_H
#define FLIGHT_TOPICS_H

#include <stdbool.h>
#include <stdint.h>
#include "uMcn.h"

/* 遥控器原始通道消息 */
typedef struct rc_channels_msg_s
{
    uint32_t timestamp_ms;      /* 通道帧接收时间 */
    uint16_t raw[16];
    bool valid;
} rc_channels_msg_t;

/* 姿态与机体系角速度消息 */
typedef struct attitude_msg_s
{
    uint32_t timestamp_us;  /* 对应的 IMU 采样时间 */
    float angle_rad[3];     /* roll、pitch、yaw，rad */
    float gyro_rad_s[3];    /* p、q、r，rad/s */
    bool valid;
} attitude_msg_t;

int uMcn_topics_init(void);

#endif
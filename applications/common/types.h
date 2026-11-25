/*
 * @Author: Jack Shaw 2870998241@qq.com
 * @Date: 2026-11-05 21:03:44
 * @LastEditors: Jack Shaw 2870998241@qq.com
 * @LastEditTime: 2026-11-16 14:39:53
 * @FilePath: /FyPilot/applications/common/types.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef FYPILOT_FLIGHT_TYPES_H
#define FYPILOT_FLIGHT_TYPES_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    float x;
    float y;
    float z;
} flight_vector3f_t;

typedef struct
{
    float w;
    float x;
    float y;
    float z;
} flight_quaternion_t;

typedef struct
{
    float roll;
    float pitch;
    float yaw;
} flight_euler_t;

typedef struct flight_imu_sample_s
{
    uint64_t timestamp_us;
    flight_vector3f_t gyro;
    flight_vector3f_t accel;
    float temperature_c;
    uint32_t sequence;
    bool gyro_valid;
    bool accel_valid;
} flight_imu_sample_t;

typedef struct
{
    uint64_t timestamp_us;
    float throttle;
    float roll_target_rad;
    float pitch_target_rad;
    float yaw_rate_target_rad_s;
    bool armed;
} flight_rc_command_t;

typedef struct
{
    uint64_t timestamp_us;
    float motor[4];
    bool enabled;
} flight_motor_output_t;

#endif

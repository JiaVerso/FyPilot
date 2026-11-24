/*
 * @Author: Jack Shaw 2870998241@qq.com
 * @Date: 2026-11-16 15:23:48
 * @LastEditors: Jack Shaw 2870998241@qq.com
 * @LastEditTime: 2026-11-24 17:41:12
 * @FilePath: /FyPilot/applications/sensors/ekf_task.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "attitude_task.h"
#include "mp_ipc.h"
#include "mp_time.h"
#include "QuaternionEKF.h"
#include "SEGGER_RTT.h"
#include "uMcn_topic.h"

static QEKF_INS_t qe;
static struct rt_thread att_thread;
static uint8_t attitude_stack[2048];
static bool started;

extern mcn_topic_t *attitude_topic;


static void mp_attitude_entry(void *parameter)
{
    QEKF_INS_t *qe = (QEKF_INS_t *)parameter;

    uint64_t previous_us = 0;
    uint32_t previous_sequence = 0;
    bool have_previous = false;

    while (1)
    {
        flight_imu_sample_t sample;
        rt_ssize_t received = rt_mq_recv(
            mp_imu_mq(),
            &sample,
            sizeof(sample),
            RT_WAITING_FOREVER
        );

        if (received != (rt_size_t)sizeof(sample))
        {
            rt_kprintf("attitude: queue receive failed (%d)\n", (int)received);
            return;
        }

        /* 建立时间戳基准 */
        if (!have_previous || sample.timestamp_us <= previous_us ||
            (uint32_t)(sample.sequence - previous_sequence) != 1U)
        {
            previous_us = sample.timestamp_us;
            previous_sequence = sample.sequence;
            have_previous = true;
            continue;
        }

        uint64_t delay_us = sample.timestamp_us - previous_us;

        previous_us = sample.timestamp_us;
        previous_sequence = sample.sequence;

        /* 陀螺仪(rad/s)，统一时间单位 */
        float dt = (float)delay_us * 1.0e-6f;

        bool updated = IMU_QuaternionEKF_UpdateInstance(
            qe,
            sample.gyro.x,
            sample.gyro.y,
            sample.gyro.z,
            sample.accel.x,
            sample.accel.y,
            sample.accel.z,
            dt
        );

        if (!updated)
            continue;
        else
        {
            attitude_msg_t msg = {0};
            msg.timestamp_us = sample.timestamp_us;
            msg.angle_rad[0] = qe->Roll;
            msg.angle_rad[1] = qe->Pitch;
            msg.angle_rad[2] = qe->Yaw;

            int result = mcn_publish(attitude_topic, &msg);
            /* 输出姿态角 */
            static uint32_t last_log_ms;
            uint32_t now_ms = mp_time_now_ms();
        }
    }
}

rt_err_t mp_attitude_start(void)
{
    if (started)
        return -RT_EBUSY;

    bool initialized = IMU_QuaternionEKF_InitInstance(
        &qe,
        0,              /* 实例编号 */
        10.0f,          /* Q1 */
        0.001f,         /* Q2 */
        10000000.0f,    /* R */
        1.0f,           /* lambda */
        0.0f            /* 加速度低通时间常数 */
    );

    if (!initialized)
        return -RT_ENOMEM;

    rt_err_t err = rt_thread_init(&att_thread, "mp_attitude", mp_attitude_entry, &qe,
                   attitude_stack, sizeof(attitude_stack), 6, 1);

    if (err != RT_EOK)
    {
        IMU_QuaternionEKF_DeinitInstance(&qe);
        return err;
    }

    err = rt_thread_startup(&att_thread);
    if (err != RT_EOK)
    {
        rt_thread_detach(&att_thread);
        IMU_QuaternionEKF_DeinitInstance(&qe);
        return err;
    }

    started = true;
    return RT_EOK;
}

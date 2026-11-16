/*
 * @Author: Jack Shaw 2870998241@qq.com
 * @Date: 2026-11-16 15:23:48
 * @LastEditors: Jack Shaw 2870998241@qq.com
 * @LastEditTime: 2026-11-16 17:08:09
 * @FilePath: /FyPilot/applications/sensors/ekf_task.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include <rtthread.h>
#include "mp_ipc.h"
#include "QuaternionEKF.h"

static QEKF_INS_t qe;
static rt_thread_t att_thread;
static uint8_t attitude_stack[2048];

flight_imu_sample_t sample;

static void mp_attitude_entry(void *parameter)
{
    QEKF_INS_t *qe = (QEKF_INS_t *)parameter;

    uint64_t previous_us = 0;
    uint32_t previous_sequence = 0;

    while (1)
    {
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
        if (sample.timestamp_us <= previous_us ||
            (uint32_t)(sample.sequence - previous_sequence) != 1U)
        {
            previous_us = sample.timestamp_us;
            previous_sequence = sample.sequence;
            continue;
        }

        uint64_t delay_us = sample.timestamp_us - previous_us;

        previous_us = sample.timestamp_us;
        previous_sequence = sample.sequence;

        /* 微秒转换成秒，使用采样时间戳，而非线程唤醒间隔。 */
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
    }
}

rt_err_t mp_attitude_start()
{
    
    if (att_thread != RT_NULL)
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

    rt_err_t err = rt_thread_init(&att_thread , "mp_attitude", mp_attitude_entry, &qe,
                   attitude_stack, sizeof(attitude_stack), 6, 1);

    err = rt_thread_startup(att_thread);
    if (err != RT_EOK)
    {
        rt_thread_detach(att_thread);
        return err;
    }

    return RT_EOK;
}
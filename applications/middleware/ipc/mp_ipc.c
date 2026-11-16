/*
 * @Author: Jack Shaw 2870998241@qq.com
 * @Date: 2026-11-05 20:45:56
 * @LastEditors: Jack Shaw 2870998241@qq.com
 * @LastEditTime: 2026-11-16 14:41:39
 * @FilePath: /FyPilot/applications/middleware/ipc/mp_ipc.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include <rtthread.h>
#include "mp_ipc.h"

#define IMU_QUEUE_LENGTH 8

#define FLIGHT_EVT_ATTITUDE_READY   (1u << 0)
#define FLIGHT_EVT_POSITION_READY   (1u << 1)
#define FLIGHT_EVT_FAILSAFE         (1u << 2)
#define FLIGHT_EVT_SHUTDOWN         (1u << 3)

/* imu message queue */
static struct rt_messagequeue imu_mq;

static rt_uint8_t msg_pool[IMU_QUEUE_LENGTH * sizeof(struct flight_imu_sample_s)];

static struct rt_event mp_event_object;
static rt_sem_t dynamic_sem = RT_NULL;
static struct rt_mutex attitude_mutex;

rt_err_t mp_ipc_init(void)
{
    rt_err_t ret;

    ret = rt_mq_init(&imu_mq,
                     "imu_mq",
                     msg_pool,
                     sizeof(struct flight_imu_sample_s),
                     sizeof(msg_pool),
                     RT_IPC_FLAG_PRIO);
    if (ret != RT_EOK)
    {
        rt_kprintf("init message queue failed: %d\r\n", ret);
        return ret;
    }

    ret = rt_event_init(&mp_event_object,
                        "mp_event",
                        RT_IPC_FLAG_PRIO);
    if (ret != RT_EOK)
    {
        rt_kprintf("init event failed: %d\r\n", ret);
        return ret;
    }

    dynamic_sem = rt_sem_create("dynamic_sem", 0, RT_IPC_FLAG_PRIO);
    if (dynamic_sem == RT_NULL)
    {
        rt_kprintf("create dynamic semaphore failed\r\n");
        return -RT_ENOMEM;
    }

    ret = rt_mutex_init(&attitude_mutex,
                        "attitude_mutex",
                        RT_IPC_FLAG_PRIO);
    if (ret != RT_EOK)
    {
        rt_kprintf("init mutex failed: %d\r\n", ret);
        return ret;
    }

    return RT_EOK;
}

rt_mq_t mp_imu_mq(void)
{
    return &imu_mq;
}

rt_event_t mp_event(void)
{
    return &mp_event_object;
}

rt_mutex_t mp_attitude_mutex(void)
{
    return &attitude_mutex;
}

rt_sem_t mp_imu_sem(void)
{
    return dynamic_sem;
}

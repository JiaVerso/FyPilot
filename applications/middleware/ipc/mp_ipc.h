/*
 * @Author: Jack Shaw 2870998241@qq.com
 * @Date: 2026-11-05 20:45:56
 * @LastEditors: Jack Shaw 2870998241@qq.com
 * @LastEditTime: 2026-11-16 14:28:42
 * @FilePath: /FyPilot/applications/middleware/ipc/mp_ipc.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
/*
 * IPC objects and message types used by FyPilot.
 */
#ifndef MP_IPC_H
#define MP_IPC_H

#include <rtthread.h>
#include "types.h"

/* RT-Thread v5.x uses zero as the non-blocking timeout. */
#ifndef RT_TICK_NONE
#define RT_TICK_NONE 0
#endif

typedef struct attitude_state
{
    rt_uint64_t timestamp_us;
    float quaternion[4];
    float euler[3];
    rt_uint8_t valid;
} attitude_state_t;

typedef struct control_output
{
    rt_uint64_t timestamp_us;
    float motor[8];
    rt_uint8_t valid;
} control_output_t;

rt_err_t mp_ipc_init(void);
rt_mq_t mp_imu_mq(void);
rt_event_t mp_event(void);
rt_mutex_t mp_attitude_mutex(void);
rt_sem_t mp_imu_sem(void);

#endif

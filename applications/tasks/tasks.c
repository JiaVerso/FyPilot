/*
 * @Author: Jack Shaw 2870998241@qq.com
 * @Date: 2026-11-05 20:45:56
 * @LastEditors: Jack Shaw 2870998241@qq.com
 * @LastEditTime: 2026-11-16 16:51:47
 * @FilePath: /FyPilot/applications/tasks/tasks.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */

#include "tasks.h"
#include "status.h"
#include "config.h"
#include <finsh.h>
#include <rthw.h>
#ifdef BSP_USING_CRSF
#include "receiver_crsf.h"
#endif
#ifdef BSP_USING_BMI270
#include "imu_task.h"
#endif

/* Reuse each module's worker; do not create a second UART or SPI reader. */
static const task_definition_t taskDefinitions[TASK_COUNT] =
    {
        [TASK_RX] = {"crsf", "UART6 RX / 1 ms poll",
#ifdef BSP_USING_CRSF
                     crsf_receiver_start
#else
                    RT_NULL
#endif
        },
        [TASK_IMU] = {"imu", "BMI270 DRDY / 10 ms fallback",
#ifdef BSP_USING_BMI270
                      mp_bmi270_start
#else
                    RT_NULL
#endif
        },
        [TASK_ATTITUDE] = {"attitude", "IMU message queue",
#ifdef BSP_USING_BMI270
                    mp_attitude_start
#else
                    RT_NULL
#endif
}
    };

static task_status_t taskStatus[TASK_COUNT];
static rt_bool_t dataInitialized;

static void tasksInitData(void)
{
    rt_base_t level = rt_hw_interrupt_disable();
    if (!dataInitialized)
    {
        for (unsigned i = 0; i < TASK_COUNT; ++i)
        {
            taskStatus[i].name = taskDefinitions[i].name;
            taskStatus[i].trigger = taskDefinitions[i].trigger;
            taskStatus[i].enabled = taskDefinitions[i].start != RT_NULL;
            taskStatus[i].started = RT_FALSE;
            taskStatus[i].startResult = -RT_ENOSYS;
        }
        dataInitialized = RT_TRUE;
    }
    rt_hw_interrupt_enable(level);
}

 /* 初始化线程 */
rt_err_t tasksInit(void)
{
    tasksInitData();
    
    rt_err_t result = RT_EOK;
    for (uint8_t i = 0; i < TASK_COUNT; ++i)
    {
        if (taskDefinitions[i].start == RT_NULL)
            continue;

        rt_err_t err = taskDefinitions[i].start();

        taskStatus[i].startResult = err;
        taskStatus[i].started = (err == RT_EOK);
        rt_kprintf("[tasks] %s: %s (%d)\n", taskDefinitions[i].name,
                   err == RT_EOK ? "started" : "failed", (int)err);
        if (result == RT_EOK && err != RT_EOK)
            result = err;
    }
    return result;
}
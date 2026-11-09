/*
 * @Author: Jack Shaw 2870998241@qq.com
 * @Date: 2026-11-05 20:45:56
 * @LastEditors: Jack Shaw 2870998241@qq.com
 * @LastEditTime: 2026-11-09 10:48:03
 * @FilePath: /FyPilot/applications/tasks/tasks.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
/*
 * @Author: Jack Shaw 2870998241@qq.com
 * @Date: 2026-11-05 20:45:56
 * @LastEditors: Jack Shaw 2870998241@qq.com
 * @LastEditTime: 2026-11-09 10:35:53
 * @FilePath: /FyPilot/applications/tasks/tasks.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
/*
 * File          : tasks.c
 * @Author       : JiaVerso
 * @Date         : 2026-11-05 20:54:16
 * @LastEditors: Jack Shaw 2870998241@qq.com
 * @LastEditTime: 2026-11-09 10:30:41
 * @FilePath     : /FyPilot/applications/fc/tasks.c
 * @Description  : 统一启动 CRSF、BMI270 和主循环线程
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

typedef rt_err_t (*task_start_func)(void);

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
        [TASK_FLIGHT] = {FLIGHT_THREAD_NAME, "periodic diagnostic loop", fypilot_flight_start}
    };

static task_status_t taskStatus[TASK_COUNT];
static rt_bool_t dataInitialized;
static rt_bool_t initAttempted;

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

rt_err_t tasksGetStatus(taskid_e taskId, task_status_t *status)
{
    if ((unsigned)taskId >= TASK_COUNT || status == RT_NULL)
        return -RT_EINVAL;
    tasksInitData();
    rt_base_t level = rt_hw_interrupt_disable();
    *status = taskStatus[taskId];
    rt_hw_interrupt_enable(level);
    return RT_EOK;
}

static void tasks(void)
{
    rt_kprintf("name         enabled started result trigger\n");
    for (unsigned i = 0; i < TASK_COUNT; ++i)
    {
        task_status_t status;
        if (tasksGetStatus((taskid_e)i, &status) == RT_EOK)
            rt_kprintf("%-12s %7u %7u %6d %s\n", status.name,
                       (unsigned)status.enabled, (unsigned)status.started,
                       (int)status.startResult, status.trigger);
    }
    rt_kprintf("flight loop=%u Hz count=%u status=%u (diagnostic only)\n",
               (unsigned)FLIGHT_LOOP_HZ, (unsigned)fypilot_get_loop_count(),
               (unsigned)fypilot_get_status());
    rt_kprintf("started is a boot result; use ps for live thread state.\n");
    rt_kprintf("Attitude/PID/mixer are not connected; ESC outputs remain disabled.\n");
}
MSH_CMD_EXPORT(tasks, Show FyPilot task startup results and loop counter);

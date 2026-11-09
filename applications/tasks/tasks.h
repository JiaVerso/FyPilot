/*
 * @Author: Jack Shaw 2870998241@qq.com
 * @Date: 2026-11-05 20:45:56
 * @LastEditors: Jack Shaw 2870998241@qq.com
 * @LastEditTime: 2026-11-09 10:37:36
 * @FilePath: /FyPilot/applications/tasks/tasks.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
/* SPDX-License-Identifier: GPL-3.0-or-later
 * Adapted from the Cleanflight/Betaflight task registration interface.
 */
#ifndef FYPILOT_TASKS_H
#define FYPILOT_TASKS_H

#include <rtthread.h>

typedef enum
{
    TASK_RX = 0,
    TASK_IMU,
    TASK_FLIGHT,
    TASK_COUNT
} taskid_e;

typedef struct
{
    const char *name;
    const char *trigger;
    task_start_func start;
} task_definition_t;

typedef struct task_status_s
{
    const char *name;
    const char *trigger;
    rt_bool_t enabled;
    rt_bool_t started;
    rt_err_t startResult;
} task_status_t;

/* Start configured workers once. Return the first error, but try every worker. */
rt_err_t tasksInit(void);
/* Copy a consistent startup snapshot. Live thread state is shown by `ps`. */
rt_err_t tasksGetStatus(taskid_e taskId, task_status_t *status);

#endif

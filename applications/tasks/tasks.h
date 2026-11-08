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
} taskId_e;

typedef struct
{
    const char *name;
    const char *trigger;
    rt_bool_t enabled;
    rt_bool_t started;
    rt_err_t startResult;
} task_status_t;

/* Boot-time initialization only; repeated calls never reset running workers. */
void tasksInitData(void);
/* Start configured workers once. Return the first error, but try every worker. */
rt_err_t tasksInit(void);
/* Copy a consistent startup snapshot. Live thread state is shown by `ps`. */
rt_err_t tasksGetStatus(taskId_e taskId, task_status_t *status);

#endif

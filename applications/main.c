/*
 * Copyright (c) 2026, Fy-DongLi Development Team
 * SPDX-License-Identifier: Apache-2.0
 * Original application entry by JiaVerso, 2026-09-16.
 */
#include <rtthread.h>
#include "SEGGER_RTT.h"
#include "init.h"

int main(void)
{
    /* Initialize RTT before workers can produce their first log. */
    SEGGER_RTT_Init();
    SEGGER_RTT_printf(0, "Hello FyPilot!\n");
    rt_err_t result = fc_init();
    SEGGER_RTT_printf(0, "[FyPilot] task startup result=%d; diagnostic mode\n", (int)result);
    if (result != RT_EOK)
        rt_kprintf("[FyPilot] Startup incomplete (%d); inspect tasks/imu/crsf.\n", (int)result);
    return 0;
}

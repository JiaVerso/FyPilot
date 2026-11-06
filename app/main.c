/*
 * Copyright (c) 2026, Fy-DongLi Development Team
 * License-Identifier: Apache-2.0
 * Original application entry by JiaVerso, 2026-09-16.
 * H743 application entry: initialize selected modules and run the power-on demo.
 */
#include <rtthread.h>
#include "fc_init.h"

int main(void)
{
    fc_init();
#ifdef BSP_USING_HELLO_WORLD
    while (1)
    {
        rt_kprintf("Hello World\n");
        rt_thread_mdelay(1000);
    }
#else
    return 0;
#endif
}

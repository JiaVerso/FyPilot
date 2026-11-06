/*
 * Cight (c) 2026, Fy-DongLi Development Team
 * License-Identifier: Apache-2.0
 * Original application entry by JiaVerso, 2026-09-16.
 * H743 application entry: initialize selected modules and run the power-on demo.
 */
#include <rtthread.h>

#include "SEGGER_RTT.h"
#include "fc_init.h"

int main(void)
{
    fc_init();
    SEGGER_RTT_Init();

    SEGGER_RTT_printf(0, "Hello FyPilot!");
}

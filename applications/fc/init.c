/*
 * File          : init.c
 * @Author       : JiaVerso
 * @Date         : 2026-11-05 20:54:16
 * @LastEditors: Jack Shaw 2870998241@qq.com
 * @LastEditTime: 2026-11-11 17:22:10
 * @FilePath     : /FyPilot/applications/fc/init.c
 * @Description  : 改为当前 BSP 的应用初始化入口.
 */

#include <rtthread.h>
#include <board.h>
#include <finsh.h>

#include "init.h"
#include "tasks.h"
#include "SEGGER_RTT.h"

rt_err_t fc_init(void)
{
    SEGGER_RTT_printf(0, "\n-- STM32H743VIH6 standalone FyPilot --\n");
    SEGGER_RTT_printf(0, "HSE=8MHz CPU=%uHz, console uart3 PD8/PD9 115200 8N1\n", SystemCoreClock);
    SEGGER_RTT_printf(0, "Board bringup: ESC signal pins remain pulled-down inputs on testing time.\n");

    rt_err_t result = tasksInit();
    
    rt_kprintf("All threads have finished initialization.\n");

#ifdef BSP_USING_CRSF
    rt_kprintf("CRSF: uart6 PC7(RX), 420000 8N1; use crsf for channels.\n");
#endif

#ifdef BSP_USING_BMI270
    rt_kprintf("BMI270: SPI4 PE12/13/14 CS=PE11 INT1=PE15; use imu for samples.\n");
#endif

    return result;
}

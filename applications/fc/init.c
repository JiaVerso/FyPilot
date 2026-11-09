/*
 * File          : init.c
 * @Author       : JiaVerso
 * @Date         : 2026-11-05 20:54:16
 * @LastEditors: Jack Shaw 2870998241@qq.com
 * @LastEditTime: 2026-11-09 10:48:55
 * @FilePath     : /FyPilot/applications/fc/init.c
 * @Description  : 改为当前 BSP 的应用初始化入口.
 */

#include <rtthread.h>
#include <board.h>
#include <finsh.h>

#include "init.h"
#include "tasks.h"

rt_err_t fc_init(void)
{
    rt_kprintf("\n-- STM32H743VIH6 standalone FyPilot --\n");
    rt_kprintf("HSE=8MHz CPU=%uHz, console uart3 PD8/PD9 115200 8N1\n", SystemCoreClock);
    rt_kprintf("Board bringup: ESC signal pins remain pulled-down inputs on testing time.\n");
    
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

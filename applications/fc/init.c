/*
 * File          : init.c
 * @Author       : JiaVerso
 * @Date         : 2026-11-05 20:54:16
 * @LastEditors: Jack Shaw 2870998241@qq.com
 * @LastEditTime: 2026-11-08 19:27:49
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
    rt_kprintf("\nAOPAH743 STM32H743VIH6 standalone FyPilot\n");
    rt_kprintf("HSE=8MHz CPU=%uHz, console uart3 PD8/PD9 115200 8N1\n", SystemCoreClock);
    rt_kprintf("Board bringup: ESC signal pins remain pulled-down inputs.\n");
    tasksInitData();
    rt_err_t result = tasksInit();
    rt_kprintf("Use: tasks, board_status, ps, list device\n");

#ifdef BSP_USING_CRSF
    rt_kprintf("CRSF: uart6 PC7(RX), 420000 8N1; use crsf for channels.\n");
#endif

#ifdef BSP_USING_BMI270
    rt_kprintf("BMI270: SPI4 PE12/13/14 CS=PE11 INT1=PE15; use imu for samples.\n");
#endif

    return result;
}

static void board_status(void)
{
    rt_size_t total, used, peak;
    rt_memory_info(&total, &used, &peak);

    rt_kprintf("AOPAH743 STM32H743VIH6 HSE=8000000 CPU=%u Hz\n", SystemCoreClock);
    rt_kprintf("Device ID=0x%03x revision=0x%04x; heap total=%u used=%u peak=%u\n",
        (unsigned)HAL_GetDEVID(), (unsigned)HAL_GetREVID(),
        (unsigned)total, (unsigned)used, (unsigned)peak);
    rt_kprintf("ESC outputs disabled; attitude/PID/mixer are not connected.\n");
}
MSH_CMD_EXPORT(board_status, Show board clocks chip revision and heap usage);

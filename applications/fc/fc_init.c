#include <rtthread.h>
#include <board.h>
#include <finsh.h>
#include "fc_init.h"
#ifdef BSP_USING_BMI270
#include "imu_task.h"
#endif
#ifdef BSP_USING_CRSF
#include "receiver_crsf.h"
#endif
int fc_init(void)
{
    rt_kprintf("\nAOPAH743 STM32H743VIH6 standalone FyPilot\n");
    rt_kprintf("HSE=8MHz CPU=%uHz, console uart3 PD8/PD9 115200 8N1\n", SystemCoreClock);
    rt_kprintf("Board bringup: ESC signal pins remain pulled-down inputs.\n");
    rt_err_t result = RT_EOK;
#ifdef BSP_USING_CRSF
    rt_err_t rx_err = crsf_receiver_start();
    rt_kprintf("CRSF uart6 PC7(RX) 420000 8N1: %s (%d)\n",
               rx_err == RT_EOK ? "started" : "failed", rx_err);
    if (rx_err != RT_EOK)
        result = rx_err;
#endif
#ifdef BSP_USING_BMI270
    rt_err_t err = aopa_imu_start();
    rt_kprintf("BMI270 SPI4 PE12/13/14 CS=PE11 INT1=PE15: %s (%d)\n",
        err == RT_EOK ? "started" : "failed", err);
    if (err != RT_EOK)
        result = err;
#endif
    rt_kprintf("Use: crsf, board_status, list device, list thread, reboot\n");
#ifndef BSP_USING_HELLO_WORLD
    rt_kprintf("IMU/math profile commands: imu, mp_selftest\n");
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
    rt_kprintf("UART3 console 115200 8N1; CRSF uses UART6 PC7 RX, 420000 8N1.\n");
    rt_kprintf("ESC outputs disabled; ADC/SDMMC/USB/OSD/barometer/magnetometer not enabled.\n");
}
MSH_CMD_EXPORT(board_status, Show board clocks chip revision and heap usage);

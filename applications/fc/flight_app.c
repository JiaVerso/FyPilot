/*
 * File          : flight_app.c
 * @Author       : JiaVerso
 * @Date         : 2026-11-05 20:54:16
 * @LastEditors: Jack Shaw 2870998241@qq.com
 * @LastEditTime: 2026-11-08 19:16:07
 * @FilePath     : /FyPilot/applications/fc/flight_app.c
 * @Description  : 保留线程测试功能 fypilot_flight_start.
 */

#include <rtthread.h>
#include <rthw.h>
#include "SEGGER_RTT.h"
#include "config.h"
#include "status.h"

#if FLIGHT_LOOP_HZ <= 0 || FLIGHT_LOOP_HZ > RT_TICK_PER_SECOND
#error "FLIGHT_LOOP_HZ must be positive and no greater than RT_TICK_PER_SECOND"
#endif
#if RT_TICK_PER_SECOND % FLIGHT_LOOP_HZ != 0
#error "FLIGHT_LOOP_HZ must divide RT_TICK_PER_SECOND for tick-based scheduling"
#endif

static struct rt_thread flight_thread;
rt_align(RT_ALIGN_SIZE) static rt_uint8_t flight_stack[FLIGHT_THREAD_STACK_SIZE];
static flight_status_t flight_status = FLIGHT_STATUS_STOPPED;
static uint32_t flight_loop_count;
static rt_bool_t started;

flight_status_t fypilot_get_status(void)
{
    rt_base_t level = rt_hw_interrupt_disable();
    flight_status_t status = flight_status;
    rt_hw_interrupt_enable(level);
    return status;
}

uint32_t fypilot_get_loop_count(void)
{
    rt_base_t level = rt_hw_interrupt_disable();
    uint32_t count = flight_loop_count;
    rt_hw_interrupt_enable(level);
    return count;
}

static void flight_thread_entry(void *parameter)
{
    (void)parameter;
    const rt_tick_t period = RT_TICK_PER_SECOND / FLIGHT_LOOP_HZ;
    rt_tick_t release = rt_tick_get();
    uint32_t log_divider = 0;
    while (1)
    {
        /* Scheduling bringup only. No estimated attitude or motor command is fabricated. */
        rt_base_t level = rt_hw_interrupt_disable();
        ++flight_loop_count;
        rt_hw_interrupt_enable(level);
        if (++log_divider == FLIGHT_LOOP_HZ)
        {
            log_divider = 0;
            SEGGER_RTT_printf(0, "[FyPilot] flight loop count=%u (diagnostic)\n",
                              (unsigned)fypilot_get_loop_count());
        }
        /* Fixed release timeline avoids adding execution time to every period. */
        rt_thread_delay_until(&release, period);
    }
}

rt_err_t fypilot_flight_start(void)
{
    if (started)
        return -RT_EBUSY;
    rt_base_t level = rt_hw_interrupt_disable();
    flight_status = FLIGHT_STATUS_INITIALIZING;
    rt_hw_interrupt_enable(level);
    rt_err_t err = rt_thread_init(&flight_thread, FLIGHT_THREAD_NAME,
        flight_thread_entry, RT_NULL, flight_stack, sizeof(flight_stack),
        TEST_THREAD_PRIORITY, FLIGHT_THREAD_TIMESLICE);
    if (err == RT_EOK)
    {
        err = rt_thread_startup(&flight_thread);
        if (err != RT_EOK)
            rt_thread_detach(&flight_thread);
    }
    level = rt_hw_interrupt_disable();
    started = (err == RT_EOK);
    /* RUNNING refers to this diagnostic worker, not flight readiness or arming. */
    flight_status = started ? FLIGHT_STATUS_RUNNING : FLIGHT_STATUS_FAILSAFE;
    rt_hw_interrupt_enable(level);
    return err;
}

/* Explicitly started by tasksInit(); no INIT_APP_EXPORT before main(). */

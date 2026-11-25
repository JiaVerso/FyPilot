/* H743 SysTick timestamp: RT tick plus the current sub-tick fraction. */
#include "mp_time.h"
#include <board.h>
#include <rthw.h>

uint64_t mp_time_now_us(void)
{
    static rt_tick_t previous_tick;
    static uint64_t elapsed_ticks;
    const rt_base_t level = rt_hw_interrupt_disable();
    rt_tick_t tick = rt_tick_get();
    const uint32_t reload = SysTick->LOAD + 1U;
    uint32_t value = SysTick->VAL;
    /* Count one pending wrap even if called before the SysTick ISR runs. */
    if (SCB->ICSR & SCB_ICSR_PENDSTSET_Msk)
    {
        ++tick;
        value = SysTick->VAL;
    }
    elapsed_ticks += (rt_tick_t)(tick - previous_tick);
    previous_tick = tick;
    const uint32_t cycles = (reload - value) % reload;
    const uint64_t time_us = elapsed_ticks * (1000000U / RT_TICK_PER_SECOND)
        + (uint64_t)cycles * 1000000U / SystemCoreClock;
    rt_hw_interrupt_enable(level);
    return time_us;
}
uint32_t mp_time_now_ms(void) { return (uint32_t)(mp_time_now_us() / 1000U); }
void mp_sleep_ms(uint32_t ms) { rt_thread_mdelay(ms); }
void mp_sleep_ticks(uint32_t ticks) { rt_thread_delay(ticks); }
void mp_delay_us(uint32_t us) { rt_hw_us_delay(us); }

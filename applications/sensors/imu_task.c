/* Data-ready ISR only wakes the worker; all SPI operations run in thread mode. */
#include "imu_task.h"
#include "bmi270_port.h"
#include "target_config.h"
#include "mp_time.h"
#include <rtdevice.h>
#include <finsh.h>
#include <rthw.h>

static struct rt_thread imu_thread;
static struct rt_semaphore data_ready;
rt_align(RT_ALIGN_SIZE) static rt_uint8_t imu_stack[4096];
static flight_imu_sample_t latest;
static rt_bool_t started;
static rt_uint32_t irq_count, sample_count, error_count, poll_count, coalesced;

static void imu_isr(void *context)
{
    (void)context;
    ++irq_count;
    rt_sem_release(&data_ready);
}
static void imu_entry(void *parameter)
{
    (void)parameter;
    while (1)
    {
        flight_imu_sample_t sample;
        rt_err_t err = rt_sem_take(&data_ready, rt_tick_from_millisecond(10));
        if (err == -RT_ETIMEOUT)
            ++poll_count; /* retain diagnostic sampling if the INT line is absent */
        else if (err != RT_EOK)
            continue;
        while (rt_sem_take(&data_ready, 0) == RT_EOK)
            ++coalesced; /* latest register values, not an invented FIFO sample */
        err = aopa_bmi270_read(&sample);
        if (err == RT_EOK)
        {
            sample.sequence = ++sample_count;
            rt_base_t level = rt_hw_interrupt_disable();
            latest = sample;
            rt_hw_interrupt_enable(level);
        }
        else if (err != -RT_EEMPTY)
            ++error_count;
    }
}
rt_err_t mp_bmi270_start(void)
{
    rt_err_t err;
    if (started)
        return -RT_EBUSY;
    err = aopa_bmi270_init();
    if (err != RT_EOK)
        return err;
    err = rt_sem_init(&data_ready, "imu_drdy", 0, RT_IPC_FLAG_PRIO);
    if (err != RT_EOK)
        return err;
    rt_pin_mode(AOPA_IMU_INT1, PIN_MODE_INPUT);
    err = rt_pin_attach_irq(AOPA_IMU_INT1, PIN_IRQ_MODE_RISING, imu_isr, RT_NULL);
    if (err != RT_EOK)
        goto detach_sem;
    err = rt_thread_init(&imu_thread, "imu", imu_entry, RT_NULL,
        imu_stack, sizeof(imu_stack), 5, 1);
    if (err != RT_EOK)
        goto detach_irq;
    err = rt_pin_irq_enable(AOPA_IMU_INT1, PIN_IRQ_ENABLE);
    if (err != RT_EOK)
        goto detach_thread;
    err = rt_thread_startup(&imu_thread);
    if (err != RT_EOK)
    {
        rt_pin_irq_enable(AOPA_IMU_INT1, PIN_IRQ_DISABLE);
        goto detach_thread;
    }
    started = RT_TRUE;
    return RT_EOK;
detach_thread:
    rt_thread_detach(&imu_thread);
detach_irq:
    rt_pin_detach_irq(AOPA_IMU_INT1);
detach_sem:
    rt_sem_detach(&data_ready);
    return err;
}
static void imu(int argc, char **argv)
{
    flight_imu_sample_t copy;
    rt_uint32_t irqs, count, errors, polls, skipped;
    (void)argc; (void)argv;
    rt_base_t level = rt_hw_interrupt_disable();
    copy = latest;
    irqs = irq_count; count = sample_count; errors = error_count;
    polls = poll_count; skipped = coalesced;
    rt_hw_interrupt_enable(level);
    rt_kprintf("BMI270 started=%u samples=%u irq=%u errors=%u polls=%u coalesced=%u api=%d\n",
        started, count, irqs, errors, polls, skipped, aopa_bmi270_last_error());
    if (!count)
        return;
    rt_kprintf("age=%u us gyro_valid=%u accel_valid=%u\n",
        (rt_uint32_t)(mp_time_now_us() - copy.timestamp_us), copy.gyro_valid, copy.accel_valid);
    rt_kprintf("gyro [mrad/s] %d %d %d; acc [mm/s2] %d %d %d\n",
        (int)(copy.gyro_rad_s.x * 1000), (int)(copy.gyro_rad_s.y * 1000),
        (int)(copy.gyro_rad_s.z * 1000), (int)(copy.accel_m_s2.x * 1000),
        (int)(copy.accel_m_s2.y * 1000), (int)(copy.accel_m_s2.z * 1000));
}
MSH_CMD_EXPORT(imu, Show latest real BMI270 sample and interrupt diagnostics);

/* Data-ready ISR only wakes the worker; all SPI operations run in thread mode. */
#include "imu_task.h"
#include "bmi270_port.h"
#include "target_config.h"
#include "mp_time.h"
#include "SEGGER_RTT.h"
#include "mp_ipc.h"
#include <rtdevice.h>
#include <finsh.h>
#include <rthw.h>

static struct rt_thread imu_thread;
static struct rt_semaphore imu_data_ready;
static rt_uint8_t imu_stack[4096];

static flight_imu_sample_t latest;
static rt_bool_t started;
static rt_uint32_t irq_count = 0, sample_count, error_count, poll_count, coalesced;

/* 队列丢包数 */
static rt_uint32_t queue_drop_count = 0;

static void imu_isr(void *args)
{
    (void)args;
    ++irq_count;
    rt_sem_release(&imu_data_ready);
}

static void mp_imu_entry(void *parameter)
{
    (void)parameter;
    while (1)
    {
        flight_imu_sample_t sample;
        rt_err_t err = rt_sem_take(&imu_data_ready, rt_tick_from_millisecond(10));
        if (err == -RT_ETIMEOUT)
            ++poll_count; /* retain diagnostic sampling if the INT line is absent */
        else if (err != RT_EOK)
            continue;
        while (rt_sem_take(&imu_data_ready, 0) == RT_EOK)
            ++coalesced; /* latest register values, not an invented FIFO sample */
        err = mp_bmi270_read(&sample);
        if (err == RT_EOK)
        { 
            sample.sequence = ++sample_count;
            /* 进入临界区 */
            rt_base_t level = rt_hw_interrupt_disable();
            latest = sample;
            rt_hw_interrupt_enable(level);

            rt_err_t mq_err = rt_mq_send(mp_imu_mq(), &sample, sizeof(sample));
            if (mq_err != RT_EOK) {
                ++queue_drop_count;
            }
            
            /* 每 100 ms 输出一次，约 10 Hz */
            static uint32_t last_log_ms;
            uint32_t now_ms = mp_time_now_ms();

            if ((uint32_t)(now_ms - last_log_ms) >= 100U)
            {
                last_log_ms = now_ms;

                SEGGER_RTT_printf(
                    0,
                    "IMU # gyro[mrad/s]=%f,%f,%f acc[mm/s2]=%f,%f,%f\n",
                    (float)(sample.gyro.x),
                    (float)(sample.gyro.y),
                    (float)(sample.gyro.z),
                    (float)(sample.accel.x),
                    (float)(sample.accel.y),
                    (float)(sample.accel.z)
                );
            }
            
        }
        else if (err != -RT_EEMPTY)
            ++error_count;
    }
}

/*!
 * @brief This function to start bmi270 sensor.
 */
rt_err_t mp_bmi270_start(void)
{
    if (started) {
        return -RT_EBUSY;
    }
    /* 初始化传感器相关配置 */
    rt_err_t err;
    err = mp_bmi270_init();
    if (err != RT_EOK)
        return err;

    /* 初始化信号量 */
    err = rt_sem_init(&imu_data_ready, "imu_drdy", 0, RT_IPC_FLAG_PRIO);
    if (err != RT_EOK) {
        return -RT_EBUSY;
    }
    /* floating input */
    rt_pin_mode(MP_IMU_INT1, PIN_MODE_INPUT);
    
    /* 配置引脚/上升沿触发 */
    err = rt_pin_attach_irq(MP_IMU_INT1, PIN_IRQ_MODE_RISING, imu_isr, RT_NULL);
    if (err != RT_EOK)
        goto detach_sem;

    err = rt_thread_init(&imu_thread, "mp_imu", mp_imu_entry, RT_NULL,
                         imu_stack, sizeof(imu_stack), 5, 1);
    if (err != RT_EOK)
        goto detach_irq;

    err = rt_pin_irq_enable(MP_IMU_INT1, PIN_IRQ_ENABLE);
    if (err != RT_EOK)
        goto detach_thread;

    err = rt_thread_startup(&imu_thread);
    if (err != RT_EOK)
    {
        rt_pin_irq_enable(MP_IMU_INT1, PIN_IRQ_DISABLE);
        goto detach_thread;
    }

    started = RT_TRUE;
    return RT_EOK;

/* Detach resources on error */
detach_thread:
    rt_thread_detach(&imu_thread);
    
detach_irq:
    rt_pin_detach_irq(MP_IMU_INT1);

detach_sem:
    rt_sem_detach(&imu_data_ready);
    return err;
}
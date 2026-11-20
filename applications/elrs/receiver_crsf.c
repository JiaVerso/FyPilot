/*
 * @Author: Jack Shaw 2870998241@qq.com
 * @Date: 2026-11-05 23:32:38
 * @LastEditors: Jack Shaw 2870998241@qq.com
 * @LastEditTime: 2026-11-20 18:02:03
 * @FilePath: /FyPilot/applications/elrs/receiver_crsf.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */

#include "receiver_crsf.h"
#include "target_config.h"
#include "SEGGER_RTT.h"
#include <rtdevice.h>
#include <rthw.h>
#include <finsh.h>


static rt_device_t serial;                      /* 串口设备句柄 */      
static struct rt_semaphore crsf_sem;
struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;

static struct rt_thread receiver_thread;        /* 线程句柄 */

static rt_uint8_t receiver_stack[2048];
static crsf_parser_t parser;


static crsf_state_t latest;                     /* 数据副本 */
static bool started;

 /* 接收数据回调函数 */
static rt_err_t uart_rx_ind(rt_device_t dev, rt_size_t size)
{
    (void)dev;
    if (size > 0u)
        rt_sem_release(&crsf_sem);
    return RT_EOK;
}

static uint32_t now_ms(void)
{
    /* Board Kconfig uses a 1000 Hz tick; retain correct scaling if changed. */
    return (uint32_t)((uint64_t)rt_tick_get() * 1000u / RT_TICK_PER_SECOND);
}

static void crsf_receiver_entry(void *parameter)
{
    uint8_t bytes[64];
    uint32_t last_log_ms = 0;
    uint32_t last_log_frames = 0;
    (void)parameter;
    
    while (1)
    {
        rt_sem_take(&crsf_sem, RT_WAITING_FOREVER);
        
        /* Bound each pass so continuous noise cannot monopolize the CPU. */
        for (uint16_t batch = 0; batch < CRSF_DRAIN_MAX; ++batch)
        {
            rt_ssize_t count = rt_device_read(serial, -1, bytes, sizeof(bytes));
            if (count <= 0)
                break;
            crsf_parser_parser(&parser, bytes, (size_t)count, now_ms());
        }
        crsf_state_t copy;
        uint32_t time_ms = now_ms();
        crsf_parser_snapshot(&parser, &copy);

        rt_base_t level = rt_hw_interrupt_disable();
        latest = copy;
        rt_hw_interrupt_enable(level);

        /* Keep RTT output outside the ISR and critical section; limit to 10 Hz. */
        if ((uint32_t)(time_ms - last_log_ms) >= 100u)
        {
            last_log_ms = time_ms;
            last_log_frames = copy.rc_frames;
            SEGGER_RTT_printf(0,
                "CRSF[raw]: CH01=%u CH02=%u CH03=%u CH04=%u "
                "CH05=%u CH06=%u CH07=%u CH08=%u "
                "CH09=%u CH10=%u CH11=%u CH12=%u "
                "CH13=%u CH14=%u CH15=%u CH16=%u\n",
                (unsigned)copy.raw[0], (unsigned)copy.raw[1],
                (unsigned)copy.raw[2], (unsigned)copy.raw[3],
                (unsigned)copy.raw[4], (unsigned)copy.raw[5],
                (unsigned)copy.raw[6], (unsigned)copy.raw[7],
                (unsigned)copy.raw[8], (unsigned)copy.raw[9],
                (unsigned)copy.raw[10], (unsigned)copy.raw[11],
                (unsigned)copy.raw[12], (unsigned)copy.raw[13],
                (unsigned)copy.raw[14], (unsigned)copy.raw[15]);
        }

    }
}

rt_err_t crsf_receiver_start(void)
{
    if (started)
        return -RT_EBUSY;

    /* step1：查找串口设备 */
    serial = rt_device_find(MP_CRSF_DEVICE);
    if (serial == RT_NULL)
        return -RT_ENOSYS;
    if (serial->ref_count != 0u)
        return -RT_EBUSY;

    /* step2：修改串口配置参数 */
    config.baud_rate = CRSF_BAUDRATE;
    config.data_bits = DATA_BITS_8;
    config.stop_bits = STOP_BITS_1;
    config.parity = PARITY_NONE;
    config.bufsz = BSP_CRSF_RX_BUFFER_SIZE;

    /* step3：控制串口设备。通过控制接口传入命令控制字，与控制参数 */
    rt_err_t err = rt_device_control(serial, RT_DEVICE_CTRL_CONFIG, &config);
    if (err != RT_EOK)
        return err;

    /* Initialize the notification object before DMA can invoke its callback. */
    err = rt_sem_init(&crsf_sem, "crsf_rx", 0, RT_IPC_FLAG_FIFO);
    if (err != RT_EOK)
        return err;
    err = rt_device_set_rx_indicate(serial, uart_rx_ind);
    if (err != RT_EOK)
        goto detach_sem;

    /* CRSF currently only receives; choose the enabled BSP receive mode. */

    err = rt_device_open(serial, RT_DEVICE_OFLAG_RDONLY | RT_DEVICE_FLAG_DMA_RX);
    if (err != RT_EOK)
        goto clear_rx_indicate;
        
    crsf_parser_init(&parser);
    rt_memset(&latest, 0, sizeof(latest));
    err = rt_thread_init(&receiver_thread, "crsf", crsf_receiver_entry, RT_NULL,
                         receiver_stack, sizeof(receiver_stack), 6, 1);
    if (err != RT_EOK)
        goto close_serial;

    err = rt_thread_startup(&receiver_thread);
    if (err != RT_EOK)
    {
        rt_thread_detach(&receiver_thread);
        goto close_serial;
    }
    started = true;
    return RT_EOK;
    
close_serial:
    rt_device_set_rx_indicate(serial, RT_NULL);
    rt_device_close(serial);
    goto detach_sem;
clear_rx_indicate:
    rt_device_set_rx_indicate(serial, RT_NULL);
detach_sem:
    rt_sem_detach(&crsf_sem);
    return err;
}

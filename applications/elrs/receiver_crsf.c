/* SPDX-License-Identifier: GPL-3.0-or-later
 * RT-Thread UART transport for the HPM Betaflight-derived CRSF receiver.
 * The official serial ISR fills its FIFO. All parsing happens in this worker.
 */
#include "receiver_crsf.h"
#include "target_config.h"
#include <rtdevice.h>
#include <rthw.h>
#include <finsh.h>

static rt_device_t serial;
static struct rt_thread receiver_thread;
rt_align(RT_ALIGN_SIZE) static rt_uint8_t receiver_stack[2048];
static crsf_parser_t parser;
static crsf_state_t latest;
static bool started;

static uint32_t now_ms(void)
{
    /* Board Kconfig uses a 1000 Hz tick; retain correct scaling if changed. */
    return (uint32_t)((uint64_t)rt_tick_get() * 1000u / RT_TICK_PER_SECOND);
}

static void crsf_receiver_entry(void *parameter)
{
    uint8_t bytes[64];
    (void)parameter;
    while (1)
    {
        /* Bound each pass so continuous noise cannot monopolize the CPU. */
        for (unsigned batch = 0; batch < 16; ++batch)
        {
            rt_ssize_t count = rt_device_read(serial, 0, bytes, sizeof(bytes));
            if (count <= 0)
                break;
            crsf_parser_feed(&parser, bytes, (size_t)count, now_ms());
        }
        crsf_state_t copy;
        crsf_parser_snapshot(&parser, now_ms(), BSP_CRSF_SIGNAL_TIMEOUT_MS, &copy);
        rt_base_t level = rt_hw_interrupt_disable();
        latest = copy;
        rt_hw_interrupt_enable(level);
        rt_thread_mdelay(1);
    }
}

rt_err_t crsf_receiver_start(void)
{
    if (started)
        return -RT_EBUSY;
    serial = rt_device_find(MP_CRSF_DEVICE);
    if (serial == RT_NULL)
        return -RT_ENOSYS;
    if (serial->ref_count != 0u)
        return -RT_EBUSY;
    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;
    config.baud_rate = CRSF_BAUDRATE;
    config.data_bits = DATA_BITS_8;
    config.stop_bits = STOP_BITS_1;
    config.parity = PARITY_NONE;
    config.bufsz = BSP_CRSF_RX_BUFFER_SIZE;
    rt_err_t err = rt_device_control(serial, RT_DEVICE_CTRL_CONFIG, &config);
    if (err != RT_EOK)
        return err;
    err = rt_device_open(serial, RT_DEVICE_OFLAG_RDONLY | RT_DEVICE_FLAG_INT_RX);
    if (err != RT_EOK)
        return err;
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
    rt_device_close(serial);
    return err;
}

bool crsf_receiver_get(crsf_state_t *state)
{
    if (state == RT_NULL)
        return false;
    rt_base_t level = rt_hw_interrupt_disable();
    *state = latest;
    rt_hw_interrupt_enable(level);
    uint32_t time = now_ms();
    state->signal_valid = started && state->has_channels &&
        (uint32_t)(time - state->last_rc_ms) < BSP_CRSF_SIGNAL_TIMEOUT_MS;
    state->link_statistics_valid = state->link_frames != 0u &&
        (uint32_t)(time - state->last_link_ms) < 250u;
    return state->signal_valid;
}

static void crsf(void)
{
    crsf_state_t state;
    crsf_receiver_get(&state);
    rt_kprintf("CRSF uart6 PC7(RX) 420000 8N1 started=%u valid=%u frames=%u\n",
               (unsigned)started, (unsigned)state.signal_valid, state.rc_frames);
    rt_kprintf("crc_errors=%u length_errors=%u partial_timeouts=%u discarded=%u ignored=%u\n",
               state.crc_errors, state.length_errors, state.partial_timeouts,
               state.discarded_bytes, state.ignored_frames);
    if (!state.has_channels)
    {
        rt_kprintf("No valid RC frame received; channel values are not available.\n");
        return;
    }
    rt_kprintf("RC age=%u ms; retained values are stale when valid=0\n",
               (uint32_t)(now_ms() - state.last_rc_ms));
    for (unsigned i = 0; i < CRSF_CHANNEL_COUNT; ++i)
        rt_kprintf("CH%02u raw=%4u us=%4u\n", i + 1u,
                   (unsigned)state.raw[i], (unsigned)state.channel_us[i]);
    if (state.link_frames != 0u)
        rt_kprintf("link_valid=%u LQ=%u RSSI=%d dBm SNR=%d dB RF=%u\n",
                   (unsigned)state.link_statistics_valid, (unsigned)state.link_quality,
                   (int)state.rssi_dbm, (int)state.snr_db, (unsigned)state.rf_mode);
}
MSH_CMD_EXPORT(crsf, Show all 16 CRSF channels and receive validity);

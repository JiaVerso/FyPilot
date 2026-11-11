/* RT-Thread transport adapter for the unmodified Bosch BMI270 SensorAPI.
 * Reference: Bosch BMI270_SensorAPI; BF 4.5.3 accgyro_spi_bmi270.c.
 * The API owns the dummy byte and the 8 KiB configuration image protocol.
 */
#include "bmi270_port.h"
#include "bmi270.h"
#include "target_config.h"
#include "mp_time.h"
#include <rtdevice.h>
#include <math.h>
#include <string.h>

static struct rt_spi_device imu_spi;
static struct bmi2_dev bmi;
static rt_bool_t attached;
static rt_bool_t ready;
static int last_error;

static BMI2_INTF_RETURN_TYPE spi_read(uint8_t reg, uint8_t *data, uint32_t len, void *context)
{
    if (data == RT_NULL || context == RT_NULL || len == 0)
        return BMI2_E_NULL_PTR;
    /* Bosch includes its dummy byte in len; do not strip a second byte here. */
    return rt_spi_send_then_recv(context, &reg, 1, data, len) == RT_EOK
        ? BMI2_INTF_RET_SUCCESS : BMI2_E_COM_FAIL;
}
static BMI2_INTF_RETURN_TYPE spi_write(uint8_t reg, const uint8_t *data, uint32_t len, void *context)
{
    if (data == RT_NULL || context == RT_NULL || len == 0)
        return BMI2_E_NULL_PTR;
    return rt_spi_send_then_send(context, &reg, 1, data, len) == RT_EOK
        ? BMI2_INTF_RET_SUCCESS : BMI2_E_COM_FAIL;
}
static void delay_us(uint32_t us, void *context)
{
    (void)context;
    if (us >= 1000U)
        rt_thread_mdelay((us + 999U) / 1000U);
    else
        mp_delay_us(us);
}
static rt_err_t configure_spi(uint32_t hz)
{
    struct rt_spi_configuration cfg = {0};
    cfg.data_width = 8;
    cfg.mode = RT_SPI_MASTER | RT_SPI_MODE_3 | RT_SPI_MSB;
    cfg.max_hz = hz;
    return rt_spi_configure(&imu_spi, &cfg);
}

int mp_bmi270_last_error(void) 
{ 
    return last_error; 
}

/* Values ​​required for configuring sensor registers, along with their corresponding function pointers */
rt_err_t mp_bmi270_init(void)
{
    uint8_t sensors[] = {BMI2_ACCEL, BMI2_GYRO};
    struct bmi2_sens_config cfg[2] = {{0},{0}};
    struct bmi2_int_pin_config pin = {0};
    int8_t rc;
    ready = RT_FALSE;
    if (!attached)
    {
        /* 把 BMI270 挂到 RT-Thread 的 SPI 总线上 */
        rt_err_t err = rt_spi_bus_attach_device_cspin(&imu_spi, MP_IMU_DEVICE,
            MP_IMU_BUS, MP_IMU_CS, RT_NULL);

        if (err != RT_EOK) {
            rt_kprintf("Failed to attach BMI270 to SPI bus: %d\n", err);
            return err;
        }
        attached = RT_TRUE;

        /* 拉高片选引脚 */
        rt_pin_write(MP_IMU_CS, PIN_HIGH);
    }
    if (configure_spi(1000000U) != RT_EOK)
        return -RT_EIO;

    rt_thread_mdelay(10);
    memset(&bmi, 0, sizeof(bmi));
    bmi.intf = BMI2_SPI_INTF;
    bmi.intf_ptr = &imu_spi;
    bmi.read = spi_read;
    bmi.write = spi_write;
    bmi.delay_us = delay_us;
    bmi.read_write_len = 32; /* even-sized bounded configuration upload chunks */
    rc = bmi270_init(&bmi); /* chip ID, soft reset, configuration upload and status */
    if (rc != BMI2_OK)
        goto fail;
    rc = bmi2_set_adv_power_save(BMI2_DISABLE, &bmi);
    if (rc != BMI2_OK)
        goto fail;

    cfg[0].type = BMI2_ACCEL;
    cfg[1].type = BMI2_GYRO;
    rc = bmi2_get_sensor_config(cfg, 2, &bmi);
    if (rc != BMI2_OK)
        goto fail;

    /* BF uses +/-16g and +/-2000 dps. Use 800 Hz for initial board bringup;
     * this is not BF's 3.2 kHz gyro/FIFO control-loop configuration. */
    cfg[0].cfg.acc.odr = BMI2_ACC_ODR_800HZ;
    cfg[0].cfg.acc.range = BMI2_ACC_RANGE_16G;
    cfg[0].cfg.acc.bwp = BMI2_ACC_NORMAL_AVG4;
    cfg[0].cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;

    cfg[1].cfg.gyr.odr = BMI2_GYR_ODR_800HZ;
    cfg[1].cfg.gyr.range = BMI2_GYR_RANGE_2000;
    cfg[1].cfg.gyr.bwp = BMI2_GYR_NORMAL_MODE;
    cfg[1].cfg.gyr.noise_perf = BMI2_PERF_OPT_MODE;
    cfg[1].cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;
    rc = bmi2_set_sensor_config(cfg, 2, &bmi);
    if (rc != BMI2_OK)
        goto fail;

    pin.pin_type = BMI2_INT1;
    pin.int_latch = BMI2_INT_NON_LATCH;
    pin.pin_cfg[0].lvl = BMI2_INT_ACTIVE_HIGH;
    pin.pin_cfg[0].od = BMI2_INT_PUSH_PULL;
    pin.pin_cfg[0].output_en = BMI2_INT_OUTPUT_ENABLE;
    pin.pin_cfg[0].input_en = BMI2_INT_INPUT_DISABLE;
    rc = bmi2_set_int_pin_config(&pin, &bmi);
    if (rc != BMI2_OK)
        goto fail;
    rc = bmi2_map_data_int(BMI2_DRDY_INT, BMI2_INT1, &bmi);
    if (rc != BMI2_OK)
        goto fail;
    rc = bmi270_sensor_enable(sensors, 2, &bmi);
    if (rc != BMI2_OK)
        goto fail;
    rt_thread_mdelay(100); /* gyro start-up settling */
    if (configure_spi(10000000U) != RT_EOK)
        return -RT_EIO;
    ready = RT_TRUE;
    last_error = BMI2_OK;
    return RT_EOK;
fail:
    last_error = rc;
    rt_kprintf("BMI270 init failed: Bosch API=%d, chip_id=0x%02x\n", rc, bmi.chip_id);
    return -RT_EIO;
}

rt_err_t mp_bmi270_read(flight_imu_sample_t *sample)
{
    struct bmi2_sens_data data = {0};
    int8_t rc;
    if (!ready || sample == RT_NULL)
        return -RT_EINVAL;
    const uint64_t timestamp = mp_time_now_us();
    rc = bmi2_get_sensor_data(&data, &bmi);
    if (rc != BMI2_OK)
    {
        last_error = rc;
        return -RT_EIO;
    }
    if (!(data.status & (BMI2_DRDY_GYR | BMI2_DRDY_ACC)))
        return -RT_EEMPTY;
    memset(sample, 0, sizeof(*sample));
    sample->timestamp_us = timestamp;
    /* CW0_DEG is retained as identity; confirm the assembled sensor axes. */
    const float accel_scale = 16.0f * 9.80665f / 32768.0f;
    const float gyro_scale = 2000.0f * 0.017453292519943295f / 32768.0f;
    sample->accel_m_s2.x = data.acc.x * accel_scale;
    sample->accel_m_s2.y = data.acc.y * accel_scale;
    sample->accel_m_s2.z = data.acc.z * accel_scale;
    sample->gyro_rad_s.x = data.gyr.x * gyro_scale;
    sample->gyro_rad_s.y = data.gyr.y * gyro_scale;
    sample->gyro_rad_s.z = data.gyr.z * gyro_scale;
    sample->accel_valid = (data.status & BMI2_DRDY_ACC) != 0;
    sample->gyro_valid = (data.status & BMI2_DRDY_GYR) != 0;
    sample->temperature_c = NAN; /* temperature not sampled in the fast path */
    last_error = BMI2_OK;
    return RT_EOK;
}

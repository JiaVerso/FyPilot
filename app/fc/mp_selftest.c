/* Optional shell checks for the portable modules migrated from QEMU. */
#include <rtthread.h>
#include <finsh.h>
#include "my_math.h"
#include "buffer.h"
#include <math.h>
#include <string.h>
static int mp_selftest(void)
{
    float a[] = {4, 7, 2, 6}, scratch[] = {4, 7, 2, 6}, inv[4], product[4];
    math_matrixf_t ma, ms, mi, mp;
    my_matrix_init(&ma, 2, 2, a);
    my_matrix_init(&ms, 2, 2, scratch);
    my_matrix_init(&mi, 2, 2, inv);
    my_matrix_init(&mp, 2, 2, product);
    if (my_matrix_inverse(&ms, &mi) != MY_MATH_OK ||
        my_matrix_multiply(&ma, &mi, &mp) != MY_MATH_OK ||
        fabsf(product[0] - 1) > 1e-5f || fabsf(product[3] - 1) > 1e-5f ||
        fabsf(product[1]) > 1e-5f || fabsf(product[2]) > 1e-5f ||
        math_crc16(0xffff, "123456789", 9) != 0x29b1)
    {
        rt_kprintf("FyPilot math: FAIL\n");
        return -RT_ERROR;
    }
    ring_buffer_t rb = {0};
    char storage[5], result[5];
    if (ring_buffer_init(&rb, storage, sizeof(storage)) != RT_EOK)
        return -RT_ERROR;
    rt_bool_t ok = ring_buffer_write(&rb, "abcdefg", 7) == 5 &&
        ring_buffer_read(&rb, result, sizeof(result)) == 5 &&
        memcmp(result, "cdefg", 5) == 0;
    ring_buffer_release_mutex(&rb);
    rt_kprintf("FyPilot math, CRC and ring buffer: %s\n", ok ? "PASS" : "FAIL");
    return ok ? RT_EOK : -RT_ERROR;
}
MSH_CMD_EXPORT(mp_selftest, Check migrated math CRC and ring buffer modules);

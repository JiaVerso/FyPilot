/*
 * @Author: Jack Shaw 2870998241@qq.com
 * @Date: 2026-11-05 21:03:44
 * @LastEditors: Jack Shaw 2870998241@qq.com
 * @LastEditTime: 2026-11-11 17:09:05
 * @FilePath: /FyPilot/bsp/stm32h743/board/board_safety.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include <rtthread.h>
#include <rtdevice.h>
#include "target_config.h"
/* Leave ESC signals as pulled-down inputs during board bringup. */
static int aopa_motor_pins_idle(void)
{
    static const rt_base_t pins[] = MP_MOTOR_PINS;
    for (rt_size_t i = 0; i < sizeof(pins) / sizeof(pins[0]); ++i)
        rt_pin_mode(pins[i], PIN_MODE_INPUT_PULLDOWN);
    return RT_EOK;
}
INIT_BOARD_EXPORT(aopa_motor_pins_idle);

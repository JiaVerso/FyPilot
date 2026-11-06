#include <rtthread.h>
#include <rtdevice.h>
#include "target_config.h"
/* Leave ESC signals as pulled-down inputs during board bringup. */
static int aopa_motor_pins_idle(void)
{
    static const rt_base_t pins[] = AOPA_MOTOR_PINS;
    for (rt_size_t i = 0; i < sizeof(pins) / sizeof(pins[0]); ++i)
        rt_pin_mode(pins[i], PIN_MODE_INPUT_PULLDOWN);
    return RT_EOK;
}
INIT_BOARD_EXPORT(aopa_motor_pins_idle);

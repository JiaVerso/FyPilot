/* AOPAH743 pin assignments supplied by the board owner. */
#ifndef AOPA_TARGET_CONFIG_H
#define AOPA_TARGET_CONFIG_H
#include <drv_gpio.h>
#define AOPA_BOARD_NAME "AOPAH743"
#define AOPA_HSE_HZ 8000000U
#define AOPA_CRSF_DEVICE "uart6"
#define AOPA_IMU_BUS "spi4"
#define AOPA_IMU_DEVICE "bmi270"
#define AOPA_IMU_CS GET_PIN(E, 11)
#define AOPA_IMU_INT1 GET_PIN(E, 15)
#define AOPA_LED0 GET_PIN(E, 3)
#define AOPA_LED1 GET_PIN(E, 4)
#define AOPA_BEEPER GET_PIN(A, 15)
/* Do not assign AF/PWM to these pins until the actuator layer is integrated. */
#define AOPA_MOTOR_PINS { GET_PIN(B, 0), GET_PIN(B, 1), GET_PIN(A, 0), GET_PIN(A, 1), \
    GET_PIN(A, 2), GET_PIN(A, 3), GET_PIN(D, 12), GET_PIN(D, 13), GET_PIN(D, 14), GET_PIN(D, 15) }
#endif

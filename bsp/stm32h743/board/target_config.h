/*
 * @Author: Jack Shaw 2870998241@qq.com
 * @Date: 2026-11-05 21:03:44
 * @LastEditors: Jack Shaw 2870998241@qq.com
 * @LastEditTime: 2026-11-09 14:39:31
 * @FilePath: /FyPilot/bsp/stm32h743/board/target_config.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
/* AOPAH743 pin assignments supplied by the board owner. */
#ifndef MP_TARGET_CONFIG_H
#define MP_TARGET_CONFIG_H
#include <drv_gpio.h>
#define MP_HSE_HZ 8000000U
#define MP_CRSF_DEVICE "uart6"
#define MP_IMU_BUS "spi4"
#define MP_IMU_DEVICE "bmi270"
#define MP_IMU_CS GET_PIN(E, 11)
#define MP_IMU_INT1 GET_PIN(E, 15)
#define MP_LED0 GET_PIN(E, 3)
#define MP_LED1 GET_PIN(E, 4)
#define MP_BEEPER GET_PIN(A, 15)
/* Do not assign AF/PWM to these pins until the actuator layer is integrated. */
#define MP_MOTOR_PINS { GET_PIN(B, 0), GET_PIN(B, 1), GET_PIN(A, 0), GET_PIN(A, 1), \
    GET_PIN(A, 2), GET_PIN(A, 3), GET_PIN(D, 12), GET_PIN(D, 13), GET_PIN(D, 14), GET_PIN(D, 15) }
#endif

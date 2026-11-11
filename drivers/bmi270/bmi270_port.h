/*
 * @Author: Jack Shaw 2870998241@qq.com
 * @Date: 2026-11-05 21:03:44
 * @LastEditors: Jack Shaw 2870998241@qq.com
 * @LastEditTime: 2026-11-09 11:57:54
 * @FilePath: /FyPilot/drivers/bmi270/bmi270_port.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef MP_BMI270_PORT_H
#define MP_BMI270_PORT_H

#include <rtthread.h>
#include "types.h"

rt_err_t mp_bmi270_init(void);
rt_err_t mp_bmi270_read(flight_imu_sample_t *sample);

int mp_bmi270_last_error(void);

#endif

#ifndef AOPA_BMI270_PORT_H
#define AOPA_BMI270_PORT_H
#include <rtthread.h>
#include "types.h"
rt_err_t aopa_bmi270_init(void);
rt_err_t aopa_bmi270_read(flight_imu_sample_t *sample);
int aopa_bmi270_last_error(void);
#endif

#include <stdbool.h>
#include <stdint.h>

#include "control_task.h"
#include "uMcn.h"
#include "config.h"
#include "mp_time.h"
#include "receiver_crsf.h"
#include "attitude_task.h"
#include "uMcn_topic.h"
#include "SEGGER_RTT.h"

static struct rt_thread control_thread;
static uint8_t control_stack[CONTROL_THREAD_STACK_SIZE];
static bool started;

extern mcn_sub_t control_rc_sub;
extern mcn_sub_t control_attitude_sub;

 /*  */
rc_channels_msg_t rc;
attitude_msg_t attitude;

/* 读取最新遥控器、姿态和陀螺仪快照，不要求话题有更新。 */
static bool control_read_input(void)
{
    if (mcn_copy(control_rc_sub, &rc, RT_FALSE) != MCN_OK)
        return false;

    if (mcn_copy(control_attitude_sub, &attitude, RT_FALSE) != MCN_OK)
        return false;

    if (!rc.valid || !attitude.valid)
        return false;

    SEGGER_RTT_printf(0, "rc_channel[0]=%u\n", (unsigned int)rc.raw[0]);

    /* RTT printf 不支持浮点格式；放大 1000 倍，以 mrad 输出。 */
    SEGGER_RTT_printf(0, "ROLL=%d mrad, PITCH=%d mrad, YAW=%d mrad\n",
        (int)(attitude.angle_rad[0] * 1000.0f),
        (int)(attitude.angle_rad[1] * 1000.0f),
        (int)(attitude.angle_rad[2] * 1000.0f)
    );

    return true;
}

/* 清零 PID 积分、微分历史和外环分频计数 */
static void control_reset(void)
{
    
}

 /* 混控：throttle + 三轴控制量 -> motor[0..3] */
static bool mixer_update(const control_input_t *input, float motor[4])
{
    (void)input;
    (void)motor;
    
    return false;
}

/* 
 * input: 目标姿态与估计姿态 / (目标角速度与 gyro_rad_s -> 三轴控制量)
 * motor[]: 角度微调值，单位为 deg
 */
static bool control_update(const control_input_t *input, float motor[4])
{
    (void)input;
    (void)motor;
    
    return false;
}

 /* 接入 PWM/DShot 驱动，按协议映射归一化电机命令 */
static void control_write_motor(const float motor[4])
{
    (void)motor;
}

static void control_thread_entry(void *parameter)
{
    (void)parameter;

    while (1)
    {
        control_input_t input = {0};
        float motor[4] = {0};

        if (!control_read_input() || !input.armed ||
            !control_update(&input, motor))
        {
            control_reset();
            rt_memset(motor, 0, sizeof(motor));
        }

        control_write_motor(motor);
        rt_thread_delay(3);
    }
    
}

rt_err_t mp_control_start(void)
{
    if (started)
        return -RT_EBUSY;

    control_reset();
    rt_err_t err = rt_thread_init(&control_thread, "mp_control",
                                control_thread_entry, RT_NULL,
                                control_stack, sizeof(control_stack),
                                7,
                                1);
    if (err != RT_EOK)
        return err;

    err = rt_thread_startup(&control_thread);
    if (err != RT_EOK)
    {
        rt_thread_detach(&control_thread);
        return err;
    }

    started = true;
    return RT_EOK;
}

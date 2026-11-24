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

/* 非阻塞读取最新遥控器、姿态和陀螺仪快照 */
static bool control_read_input(control_input_t *input)
{
    (void)input;
    if (mcn_copy(control_rc_sub, &rc, RT_FALSE) != MCN_OK)
    return false;

    SEGGER_RTT_printf(0, "rc_channel[0]=%d\n", rc.raw[0]);

    if (mcn_copy(control_attitude_sub, &attitude, RT_FALSE) != MCN_OK)
        return false;


    return false;
}

/* 清零 PID 积分、微分历史和外环分频计数 */
static void control_reset(void)
{
    
}

/* TODO: 按顺序接入以下计算。
     *
     * 1. 角度外环：目标姿态与估计姿态 -> 目标机体系角速度。
     *    可降低外环频率；两次外环之间保持上次目标角速度。
     *    yaw 可先采用遥控器给定角速度，roll/pitch 需限制目标角速度。
     * 2. 角速度内环：目标角速度与 gyro_rad_s -> 三轴控制量。
     *    反馈使用陀螺仪，不对欧拉角差分；传入实际 dt。
     * 3. 混控：throttle + 三轴控制量 -> motor[0..3]。
     *    确认电机编号/旋向，加入饱和处理，并反馈给积分抗饱和逻辑。
     *
     * pid.c 尚有 Betaflight 依赖，适配后再调用。
     * 所有步骤完成、输出有限且在约定范围内后，才返回 true。
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

        if (!control_read_input(&input) || !input.armed ||
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

/*
 * @Author: Jack Shaw 2870998241@qq.com
 * @Date: 2026-11-24 11:22:54
 * @LastEditors: Jack Shaw 2870998241@qq.com
 * @LastEditTime: 2026-11-24 17:31:34
 * @FilePath: /FyPilot/applications/middleware/umcn/uMcn_topic.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "uMcn_topic.h"
#include "uMcn.h"

 /* 发布话题句柄 */
mcn_topic_t *rc_topic;
mcn_topic_t *attitude_topic;

/* 订阅话题句柄 */
mcn_sub_t control_rc_sub;
mcn_sub_t control_attitude_sub;

/* 在线程启动前，由初始化线程调用。 */
int uMcn_topics_init(void)
{
    /* 注册发布话题 */
    rc_topic = mcn_advertise("rc_channels", sizeof(rc_channels_msg_t));
    attitude_topic = mcn_advertise("attitude", sizeof(attitude_msg_t));

    /* 注册订阅话题 */
    control_rc_sub = mcn_subscribe("rc_channels", RT_TRUE);
    if (control_rc_sub == RT_NULL)
        return MCN_ENOMEM;

    control_attitude_sub = mcn_subscribe("attitude", RT_TRUE);
    if (control_attitude_sub == RT_NULL)
        return MCN_ENOMEM;

    return MCN_OK;
}
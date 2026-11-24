#ifndef H743_MCN_H
#define H743_MCN_H

#include <rtthread.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MCN_MAX_TOPICS       24U
#define MCN_MAX_SUBSCRIBERS  8U
#define MCN_MAX_TOPIC_NAME   32U
#define MCN_MAX_DATA_SIZE    256U

typedef struct mcn_subscriber mcn_subscriber_t;
typedef mcn_subscriber_t *mcn_sub_t;

typedef struct mcn_topic
{
    char name[MCN_MAX_TOPIC_NAME];
    rt_uint16_t data_size;
    rt_uint32_t sequence;       /* 发布次数，允许自然回绕 */
    rt_uint8_t data[MCN_MAX_DATA_SIZE];
    mcn_sub_t subscribers[MCN_MAX_SUBSCRIBERS];
    rt_bool_t used;
    rt_bool_t has_data;
} mcn_topic_t;

struct mcn_subscriber
{
    mcn_topic_t *topic;
    struct rt_semaphore semaphore;
    rt_uint32_t last_sequence;
    rt_bool_t active;
    rt_bool_t updated;          /* 独立标志避免序列号回绕漏报更新 */
    rt_bool_t waiting;
};

enum {
    MCN_OK       = 0,
    MCN_EINVAL   = -1,
    MCN_ENOMEM   = -2,
    MCN_ENOENT   = -3,
    MCN_EBUSY    = -4,
    MCN_ETIMEOUT = -5,
    MCN_ENODATA  = -6,
};

int mcn_init(void);
mcn_topic_t *mcn_advertise(const char *name, uint16_t data_size);
/* true: 已有数据立即可读；false: only_if_updated/wait 等待下一次发布。 */
mcn_sub_t mcn_subscribe(const char *name, rt_bool_t start_with_latest);
int mcn_publish(mcn_topic_t *topic, const void *data);
/* data 必须指向至少 advertise 时 data_size 字节的可写空间。
 * 从未发布时，无论 only_if_updated 为何都返回 MCN_ENODATA。
 */
int mcn_copy(mcn_sub_t subscriber, void *data, rt_bool_t only_if_updated);
/* timeout 单位为 tick；0 不等待，RT_WAITING_FOREVER 无限等待新发布。 */
int mcn_wait(mcn_sub_t subscriber, rt_int32_t timeout, void *data);
/* 正在 wait 的订阅者不能被取消，返回 MCN_EBUSY。 */
int mcn_unsubscribe(mcn_sub_t subscriber);
/* 返回话题当前发布序列号；无效句柄返回 0。 */
uint32_t mcn_sequence(mcn_sub_t subscriber);

#ifdef __cplusplus
}
#endif
#endif

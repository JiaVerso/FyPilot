#include "uMcn.h"
#include "uMcn_topic.h"

#include <string.h>

#define MCN_SUBSCRIBER_COUNT (MCN_MAX_TOPICS * MCN_MAX_SUBSCRIBERS)

static mcn_topic_t mcn_topics[MCN_MAX_TOPICS];
static mcn_subscriber_t mcn_subscribers[MCN_SUBSCRIBER_COUNT];
static struct rt_mutex mcn_mutex;
static rt_bool_t mcn_initialized;

static int mcn_lock(void)
{
    if (rt_interrupt_get_nest() != 0 || !mcn_initialized)
        return MCN_EBUSY;
    return rt_mutex_take(&mcn_mutex, RT_WAITING_FOREVER) == RT_EOK ?
        MCN_OK : MCN_EBUSY;
}

static void mcn_unlock(void)
{
    rt_mutex_release(&mcn_mutex);
}

static rt_bool_t mcn_topic_name_valid(const char *name)
{
    if (name == RT_NULL)
        return RT_FALSE;
    for (rt_uint32_t i = 0; i < MCN_MAX_TOPIC_NAME; ++i)
    {
        if (name[i] == '\0')
            return i != 0;
    }
    return RT_FALSE;
}

static mcn_topic_t *mcn_find_topic(const char *name)
{
    for (rt_uint32_t i = 0; i < MCN_MAX_TOPICS; ++i)
    {
        if (mcn_topics[i].used && strcmp(mcn_topics[i].name, name) == 0)
            return &mcn_topics[i];
    }
    return RT_NULL;
}

 /* 初始化 uMCN 自身的互斥锁和资源 */
int mcn_init(void)
{
    if (rt_interrupt_get_nest() != 0)
        return MCN_EBUSY;

    /* 初始化无阻塞操作；防止两个初始化线程重复初始化静态对象。 */
    rt_enter_critical();
    if (!mcn_initialized)
    {
        rt_err_t result = rt_mutex_init(&mcn_mutex, "mcn", RT_IPC_FLAG_PRIO);
        if (result != RT_EOK)
        {
            rt_exit_critical();
            return MCN_EBUSY;
        }
        mcn_initialized = RT_TRUE;
    }
    rt_exit_critical();
    uMcn_topics_init();
    return MCN_OK;
}

 /* 注册话题 */
mcn_topic_t *mcn_advertise(const char *name, uint16_t data_size)
{
    if (!mcn_topic_name_valid(name) || data_size == 0 || data_size > MCN_MAX_DATA_SIZE)
        return RT_NULL;
    if (mcn_lock() != MCN_OK)
        return RT_NULL;

    mcn_topic_t *topic = mcn_find_topic(name);
    if (topic != RT_NULL)
    {
        mcn_unlock();
        return topic->data_size == data_size ? topic : RT_NULL;
    }
    for (rt_uint32_t i = 0; i < MCN_MAX_TOPICS; ++i)
    {
        if (!mcn_topics[i].used)
        {
            topic = &mcn_topics[i];
            memset(topic, 0, sizeof(*topic));
            strcpy(topic->name, name);
            topic->data_size = data_size;
            topic->used = RT_TRUE;
            mcn_unlock();
            return topic;
        }
    }
    mcn_unlock();
    return RT_NULL;
}

mcn_sub_t mcn_subscribe(const char *name, rt_bool_t start_with_latest)
{
    if (!mcn_topic_name_valid(name) || mcn_lock() != MCN_OK)
        return RT_NULL;

    mcn_topic_t *topic = mcn_find_topic(name);
    if (topic == RT_NULL)
    {
        mcn_unlock();
        return RT_NULL;
    }
    rt_uint32_t slot;
    for (slot = 0; slot < MCN_MAX_SUBSCRIBERS; ++slot)
    {
        if (topic->subscribers[slot] == RT_NULL)
            break;
    }
    if (slot == MCN_MAX_SUBSCRIBERS)
    {
        mcn_unlock();
        return RT_NULL;
    }

    for (rt_uint32_t i = 0; i < MCN_SUBSCRIBER_COUNT; ++i)
    {
        mcn_sub_t sub = &mcn_subscribers[i];
        if (sub->active)
            continue;
        memset(sub, 0, sizeof(*sub));
        if (rt_sem_init(&sub->semaphore, "mcn_sub", 0, RT_IPC_FLAG_PRIO) != RT_EOK)
        {
            mcn_unlock();
            return RT_NULL;
        }
        sub->topic = topic;
        sub->last_sequence = topic->sequence;
        sub->updated = start_with_latest && topic->has_data;
        sub->active = RT_TRUE;
        topic->subscribers[slot] = sub;
        mcn_unlock();
        return sub;
    }
    mcn_unlock();
    return RT_NULL;
}

int mcn_publish(mcn_topic_t *topic, const void *data)
{
    if (topic == RT_NULL || data == RT_NULL)
        return MCN_EINVAL;
    if (mcn_lock() != MCN_OK)
        return MCN_EBUSY;
    if (!topic->used)
    {
        mcn_unlock();
        return MCN_EINVAL;
    }

    memcpy(topic->data, data, topic->data_size);
    ++topic->sequence;
    topic->has_data = RT_TRUE;
    for (rt_uint32_t i = 0; i < MCN_MAX_SUBSCRIBERS; ++i)
    {
        mcn_sub_t sub = topic->subscribers[i];
        if (sub != RT_NULL && !sub->updated)
        {
            /* 未消费的更新合并为一次通知，不累计历史消息通知。 */
            sub->updated = RT_TRUE;
            rt_sem_release(&sub->semaphore);
        }
    }
    mcn_unlock();
    return MCN_OK;
}

/* 调用者持有 mcn_mutex。 */
static int mcn_copy_locked(mcn_sub_t sub, void *data, rt_bool_t only_if_updated)
{
    if (!sub->active || sub->topic == RT_NULL)
        return MCN_EINVAL;
    mcn_topic_t *topic = sub->topic;
    if (!topic->has_data || (only_if_updated && !sub->updated))
        return MCN_ENODATA;

    memcpy(data, topic->data, topic->data_size);
    sub->last_sequence = topic->sequence;
    sub->updated = RT_FALSE;
    /* 读快照也消费对应通知；持锁时发布者不能插入新通知。 */
    (void)rt_sem_take(&sub->semaphore, 0);
    return MCN_OK;
}

int mcn_copy(mcn_sub_t subscriber, void *data, rt_bool_t only_if_updated)
{
    if (subscriber == RT_NULL || data == RT_NULL)
        return MCN_EINVAL;
    if (mcn_lock() != MCN_OK)
        return MCN_EBUSY;
    int result = subscriber->waiting ? MCN_EBUSY :
        mcn_copy_locked(subscriber, data, only_if_updated);
    mcn_unlock();
    return result;
}

int mcn_wait(mcn_sub_t subscriber, rt_int32_t timeout, void *data)
{
    if (subscriber == RT_NULL || data == RT_NULL || timeout < RT_WAITING_FOREVER)
        return MCN_EINVAL;
    if (mcn_lock() != MCN_OK)
        return MCN_EBUSY;
    if (!subscriber->active || subscriber->waiting)
    {
        int result = subscriber->waiting ? MCN_EBUSY : MCN_EINVAL;
        mcn_unlock();
        return result;
    }

    int result = mcn_copy_locked(subscriber, data, RT_TRUE);
    if (result != MCN_ENODATA)
    {
        mcn_unlock();
        return result;
    }
    if (timeout == 0)
    {
        mcn_unlock();
        return MCN_ETIMEOUT;
    }
    subscriber->waiting = RT_TRUE;
    mcn_unlock();

    /* 发布若发生在解锁和 take 之间，通知仍保留在信号量中。 */
    rt_err_t err = rt_sem_take(&subscriber->semaphore, timeout);
    if (mcn_lock() != MCN_OK)
        return MCN_EBUSY;
    subscriber->waiting = RT_FALSE;
    /* 优先检查数据，处理等待刚超时但发布已完成的边界情况。 */
    result = mcn_copy_locked(subscriber, data, RT_TRUE);
    if (result == MCN_ENODATA)
        result = err == -RT_ETIMEOUT ? MCN_ETIMEOUT : MCN_EBUSY;
    mcn_unlock();
    return result;
}

int mcn_unsubscribe(mcn_sub_t subscriber)
{
    if (subscriber == RT_NULL)
        return MCN_EINVAL;
    if (mcn_lock() != MCN_OK)
        return MCN_EBUSY;
    if (!subscriber->active || subscriber->topic == RT_NULL || subscriber->waiting)
    {
        int result = subscriber->waiting ? MCN_EBUSY : MCN_EINVAL;
        mcn_unlock();
        return result;
    }
    for (rt_uint32_t i = 0; i < MCN_MAX_SUBSCRIBERS; ++i)
    {
        if (subscriber->topic->subscribers[i] == subscriber)
            subscriber->topic->subscribers[i] = RT_NULL;
    }
    rt_sem_detach(&subscriber->semaphore);
    subscriber->active = RT_FALSE;
    subscriber->topic = RT_NULL;
    subscriber->updated = RT_FALSE;
    mcn_unlock();
    return MCN_OK;
}

uint32_t mcn_sequence(mcn_sub_t subscriber)
{
    if (subscriber == RT_NULL || mcn_lock() != MCN_OK)
        return 0;
    uint32_t sequence = subscriber->active && subscriber->topic != RT_NULL ?
        subscriber->topic->sequence : 0;
    mcn_unlock();
    return sequence;
}

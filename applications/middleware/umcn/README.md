# uMCN 最新值发布与订阅

每个话题保存最近一次发布的内容，多个订阅者独立判断是否有更新。
连续发布会覆盖旧值，不保证消费者收到每次采样；需要完整采样序列时使用队列或 FIFO。

## 启动顺序

1. 在线程上下文调用 `mcn_init()`，检查是否返回 `MCN_OK`。
2. 用 `mcn_advertise("topic_name", sizeof(message_type))` 注册话题并检查句柄。
3. 创建订阅，再启动工作线程。也可以在工作线程中订阅，但话题必须已注册。

同名话题重复注册时，消息大小相同则返回同一话题，大小不同则返回空指针。
`mcn_init()` 重复调用不会清除已注册话题和已发布数据。

## 发布和读取

```c
/* 初始化阶段，所有返回值都应检查。 */
mcn_topic_t *topic = mcn_advertise("attitude", sizeof(attitude_message_t));
mcn_sub_t sub = mcn_subscribe("attitude", RT_TRUE);

/* 发布线程：message 是已填好的 attitude_message_t。 */
int result = mcn_publish(topic, &message);

/* 控制线程：cache 跨控制周期保存。 */
result = mcn_copy(sub, &cache, RT_TRUE);
/* MCN_OK：缓存更新；MCN_ENODATA：没有新数据，检查旧缓存时间戳是否超时。
 * 首次成功读取前，缓存不能当作有效输入。
 */
```

- `start_with_latest = RT_TRUE`：订阅时已有消息也视为待读取更新。
- `start_with_latest = RT_FALSE`：等待订阅之后的新发布。
- `mcn_copy(..., RT_FALSE)`：读取最新值，即使之前读过；从未发布仍返回 `MCN_ENODATA`。
- `mcn_wait(sub, timeout, &cache)`：等待更新，timeout 单位为 RT-Thread tick；0 表示不等待，`RT_WAITING_FOREVER` 表示无限等待新发布。超时返回 `MCN_ETIMEOUT`。

## 使用约定

- 所有接口仅用于线程上下文。内部使用优先级继承互斥锁，读取可能等待锁；等待超时参数约束的是信号量等待，不是包括锁等待在内的整个调用耗时。
- 每个订阅句柄由一个消费线程使用；其他线程需要独立订阅。正在等待的句柄不能取消订阅，会返回 `MCN_EBUSY`。
- `mcn_unsubscribe()` 后旧句柄失效，不得继续使用；静态槽位可能被后续订阅复用。
- 消息按值复制，目标缓冲区必须至少有注册的消息大小；结构体内的指针不进行深拷贝。
- 各话题分别复制，不保证不同话题来自同一时刻。消息应携带采样/接收时间戳和有效标志，由消费者判断新鲜度。
- 消费一次快照会消费对应的更新通知，连续发布只保留一次待消费通知。

当前已加入 `applications/SConscript`。业务线程仍需调用初始化、注册、发布和订阅接口。

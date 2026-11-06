/*
 * File      : buffer.c
 * Agent     : GPT6-Astra
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2026, Fy Development Team
 *
 * Thread-safe byte ring buffer for RT-Thread.
 */

#include "buffer.h"
#include <string.h>

#define RING_BUFFER_MUTEX_NAME "rb_lock"

 /* 指针后移 */
static uint32_t ring_buffer_advance(ring_buffer_t *buffer,
                                    uint32_t index,
                                    size_t size)
{
    return (uint32_t)((index + size) % buffer->capacity);
}

/* 带回绕的写指针位置 */
static uint32_t write_position(ring_buffer_t *buffer)
{
    return buffer->write_wrap_count * buffer->capacity +
           buffer->head;
}

/* 带回绕的读指针位置 */
static uint32_t read_position(ring_buffer_t *buffer)
{
    return buffer->read_wrap_count * buffer->capacity +
           buffer->tail;
}


static uint32_t ring_buffer_free_space(ring_buffer_t *buffer)
{
    uint32_t write_pos = write_position(buffer);
    uint32_t read_pos = read_position(buffer);
    uint32_t used = write_pos - read_pos;

    return (used >= buffer->capacity) ? 0U : (buffer->capacity - used);
}

 /* 写入拷贝 */
static size_t ring_buffer_copy_in(ring_buffer_t *buffer,
                                         const uint8_t *data,
                                         size_t size)
{
    size_t first = buffer->capacity - buffer->head;

    if (first > size)
    {
        first = size;
    }

    memcpy(&buffer->buf[buffer->head], data, first);
    if (size > first)
    {
        memcpy(buffer->buf, &data[first], size - first);
    }

    buffer->write_wrap_count += ((buffer->head + size) / buffer->capacity);
    buffer->head = ring_buffer_advance(buffer, buffer->head, size);
    return size;
}

 /* 读出拷贝（只拷贝） */
static size_t ring_buffer_copy_out(ring_buffer_t *buffer,
                                          uint8_t *data,
                                          size_t size)
{
    size_t first = buffer->capacity - buffer->tail;

    if (first > size)
    {
        first = size;
    }

    memcpy(data, &buffer->buf[buffer->tail], first);
    if (size > first)
    {
        memcpy(&data[first], buffer->buf, size - first);
    }

    buffer->read_wrap_count += ((buffer->tail + size) / buffer->capacity);
    buffer->tail = ring_buffer_advance(buffer, buffer->tail, size);
    return size;
}

rt_err_t ring_buffer_init(ring_buffer_t *buffer,
                          char *storage,
                          uint32_t size)
{
    rt_err_t result;

    if (buffer == RT_NULL || storage == RT_NULL || size == 0U)
    {
        return -RT_EINVAL;
    }

    if (buffer->initialized != RT_FALSE)
    {
        return -RT_EBUSY;
    }

    result = rt_mutex_init(&buffer->lock, RING_BUFFER_MUTEX_NAME,
                           RT_IPC_FLAG_PRIO);
    if (result != RT_EOK)
    {
        return result;
    }

    buffer->buf = storage;
    buffer->capacity = size;
    buffer->head = 0U;
    buffer->tail = 0U;
    buffer->dropped_count = 0U;

    buffer->write_wrap_count = 0U;
    buffer->read_wrap_count = 0U;
    buffer->initialized = RT_TRUE;

    return RT_EOK;
}

rt_err_t ring_buffer_release_mutex(ring_buffer_t *buffer)
{
    rt_err_t result;

    if (buffer == RT_NULL || !buffer->initialized)
        return -RT_EINVAL;

    /* 脱离 */
    result = rt_mutex_detach(&buffer->lock);
    if (result == RT_EOK)
    {
        buffer->initialized = RT_FALSE;
        buffer->buf = RT_NULL;
        buffer->capacity = 0U;
        buffer->head = 0U;
        buffer->tail = 0U;
        buffer->dropped_count = 0U;
        buffer->write_wrap_count = 0U;
        buffer->read_wrap_count = 0U;
    }
    return result;
}

rt_err_t ring_buffer_reset(ring_buffer_t *buffer)
{
    rt_err_t result;
    if (buffer == RT_NULL || !buffer->initialized)
        return -RT_EINVAL;

    result = rt_mutex_take(&buffer->lock, RT_WAITING_FOREVER);
    if (result != RT_EOK)
    {
        return result;
    }

    buffer->head = 0U;
    buffer->tail = 0U;
    buffer->dropped_count = 0U;
    buffer->write_wrap_count = 0U;
    buffer->read_wrap_count = 0U;

    rt_mutex_release(&buffer->lock);
    return RT_EOK;
}

/* Write data in two physical segments and account for every write wrap. */
size_t ring_buffer_write(ring_buffer_t *buffer,
                         const void *data,
                         size_t size)
{
    const uint8_t *src = (const uint8_t *)data;
    if (buffer == RT_NULL || !buffer->initialized || data == RT_NULL || size == 0U)
        return 0U;
    if (rt_mutex_take(&buffer->lock, RT_WAITING_FOREVER) != RT_EOK)
        return 0U;

    /* Keep the newest bytes of an oversized input, as promised by buffer.h. */
    if (size > buffer->capacity)
    {
        size_t discarded = size - buffer->capacity;
        src += discarded;
        buffer->dropped_count += discarded;
        size = buffer->capacity;
    }
    uint32_t free_space = ring_buffer_free_space(buffer);
    if (size > free_space)
    {
        uint32_t overwritten = size - free_space;
        buffer->read_wrap_count += (buffer->tail + overwritten) / buffer->capacity;
        buffer->tail = ring_buffer_advance(buffer, buffer->tail, overwritten);
        buffer->dropped_count += overwritten;
    }
    ring_buffer_copy_in(buffer, src, size);
    rt_mutex_release(&buffer->lock);
    return size;
}

size_t ring_buffer_read(ring_buffer_t *buffer, void *data, size_t size)
{
    size_t read_size;

    if (buffer == RT_NULL || !buffer->initialized || data == RT_NULL || size == 0U)
    {
        return 0U;
    }
    if (rt_mutex_take(&buffer->lock, RT_WAITING_FOREVER) != RT_EOK)
    {
        return 0U;
    }

    uint32_t available = buffer->capacity - ring_buffer_free_space(buffer);
    read_size = size < available ? size : available;
    if (read_size != 0U)
    {
        ring_buffer_copy_out(buffer, (uint8_t *)data, read_size);
    }

    rt_mutex_release(&buffer->lock);
    return read_size;
}

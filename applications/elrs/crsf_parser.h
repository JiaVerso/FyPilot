/*
 * @Author: Jack Shaw 2870998241@qq.com
 * @Date: 2026-11-05 23:32:38
 * @LastEditors: Jack Shaw 2870998241@qq.com
 * @LastEditTime: 2026-11-20 15:37:39
 * @FilePath: /FyPilot/applications/elrs/crsf_parser.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
/* SPDX-License-Identifier: GPL-3.0-or-later
 * CRSF receive adaptation based on HPMicro's Betaflight rx/crsf.c.
 * Protocol definitions are shared through crsf_protocol.h.
 */
#ifndef FYPILOT_CRSF_PARSER_H
#define FYPILOT_CRSF_PARSER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "crsf_protocol.h"

#define CRSF_PARTIAL_TIMEOUT_MS 20u

typedef struct crsf_state_s
{
    uint16_t raw[CRSF_CHANNEL_COUNT];
    uint16_t channel_normal[CRSF_CHANNEL_COUNT];
    uint32_t last_rc_ms;                            /* 解码通道帧的时间戳，单位: ms */
    uint32_t rc_frames;                             /* 解码通道帧数量 */
    uint32_t crc_errors;                            /* CRC 校验错误次数 */
    uint32_t length_errors;                         /* 帧长度错误次数 */
    uint32_t partial_timeouts;                      /* 部分超时次数 */
    uint32_t discarded_bytes;                       /* 丢弃的字节数 */
    uint32_t ignored_frames;                        /* 忽略的帧数 */
    uint32_t link_frames;                           
    uint32_t last_link_ms;                          /* 解码统计帧的时间戳，单位: ms */
    uint8_t link_quality;                           /* 链接质量 */
    int16_t rssi_dbm;                               /* 接收信号强度，单位: dBm */
    int8_t snr_db;                                  /* 信噪比，单位: dB */
    uint8_t rf_mode;
    bool has_channels;
    bool signal_valid;
    bool link_statistics_valid;
} crsf_state_t;

/* A parser has a single owner. The RT-Thread adapter publishes atomic copies. */
typedef struct crsf_parser_s
{
    uint8_t frame[CRSF_FRAME_SIZE_MAX];              /* 缓冲区 */
    size_t buffered;                                 /* 有效字节数 */
    uint32_t last_byte_ms;
    crsf_state_t state;
} crsf_parser_t;

void crsf_parser_init(crsf_parser_t *parser);
void crsf_parser_parser(crsf_parser_t *parser, const uint8_t *bytes,
                      size_t length, uint32_t now_ms);
void crsf_parser_snapshot(const crsf_parser_t *parser, crsf_state_t *state);
float crsf_to_normal(uint16_t raw);

#endif

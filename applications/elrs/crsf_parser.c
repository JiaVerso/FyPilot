/* SPDX-License-Identifier: GPL-3.0-or-later
 * Adapted from HPMicro Betaflight crsfDataReceive/crsfFrameStatus/crsfReadRawRC.
 * Keeps the upstream 0x16 packing, CRC-8/DVB-S2 and channel scaling.
 * Uses explicit bit extraction instead of compiler-dependent C bit-fields.
 * Derived from Cleanflight/Betaflight under GPL-3.0-or-later.
 * See COPYING.GPL-3.0 at the repository root for the license text.
 */
#include "crsf_parser.h"
#include <string.h>

static uint8_t crsf_crc(const uint8_t *bytes, size_t length)
{
    uint8_t crc = 0;
    for (size_t i = 0; i < length; ++i)
    {
        crc ^= bytes[i];
        for (unsigned bit = 0; bit < 8; ++bit)
            crc = (uint8_t)((crc & 0x80u) ? ((unsigned)crc << 1) ^ 0xd5u : (unsigned)crc << 1);
    }
    return crc;
}

float crsf_to_normal(uint16_t raw)
{

    // raw_min < raw_mid < raw_max，均使用实际校准值
    float value = (float)raw - (float)CRSF_RC_MIDD_VALUE;

    if (value >= 0.0f)
        value /= (float)(CRSF_RC_MAX_VALUE - CRSF_RC_MIDD_VALUE);
    else {
        value /= (float)(CRSF_RC_MIDD_VALUE - CRSF_RC_MIN_VALUE);
    }

    // 限制到 [-1, 1]
    if (value >  1.0f) value =  1.0f;
    if (value < -1.0f) value = -1.0f;
    /*  */
    return value;
}

/* 初始化 */
void crsf_parser_init(crsf_parser_t *parser)
{
    if (parser != NULL)
        memset(parser, 0, sizeof(*parser));
}

/* 数据移位 */
static void consume(crsf_parser_t *parser, size_t count)
{
    parser->buffered -= count;
    memmove(parser->frame, parser->frame + count, parser->buffered);
}

/* 解码帧 */
static void decode_frame(crsf_parser_t *parser, size_t total, uint32_t now_ms)
{
    crsf_state_t *state = &parser->state;
    const uint8_t *payload = &parser->frame[3];
    if (parser->frame[2] == CRSF_FRAMETYPE_RC_CHANNELS_PACKED)
    {
        /* 判断帧长度 */
        if (total != CRSF_FRAME_RC_CHANNELS_PAYLOAD_SIZE + CRSF_FRAME_LENGTH_NON_PAYLOAD)
        {
            ++state->length_errors;
            return;
        }
        uint32_t bits = 0;
        uint8_t available = 0;
        uint8_t pos = 0;
        for (uint8_t channel = 0; channel < CRSF_CHANNEL_COUNT; ++channel)
        {
            while (available < 11u)
            {
                bits |= (uint32_t)payload[pos++] << available;
                available += 8u;
            }
            state->raw[channel] = (uint16_t)(bits & 0x7ffu);
            state->channel_normal[channel] = crsf_to_normal(state->raw[channel]);
            bits >>= 11;
            available -= 11u;
        }
        state->has_channels = true;
        state->last_rc_ms = now_ms;
        ++state->rc_frames;
    }
    else if (parser->frame[2] == CRSF_FRAMETYPE_LINK_STATISTICS)
    {
        if (total != CRSF_FRAME_LINK_STATISTICS_PAYLOAD_SIZE + CRSF_FRAME_LENGTH_NON_PAYLOAD)
        {
            ++state->length_errors;
            return;
        }
        state->rssi_dbm = -(int16_t)payload[payload[4] ? 1 : 0];
        state->link_quality = payload[2];
        state->snr_db = (int8_t)payload[3];
        state->rf_mode = payload[5];
        state->last_link_ms = now_ms;
        ++state->link_frames;
    }
    else
        ++state->ignored_frames;
}

/* 
 * 解析器数据喂入
 * @parser:	 解析器实例，保存上一次剩余的字节和解析结果
 * @bytes:	 本次收到的字节数组
 * @length:	 本次收到的字节数量
 * @now_ms:	 本次喂入数据时的毫秒时间戳
 *
*/
void crsf_parser_parser(crsf_parser_t *parser, const uint8_t *bytes,
                      size_t length, uint32_t now_ms)
{
    if (parser == NULL || (bytes == NULL && length != 0u))
        return;
    if (parser->buffered != 0u &&
        (uint32_t)(now_ms - parser->last_byte_ms) >= CRSF_PARTIAL_TIMEOUT_MS)
    {
        /* 超时帧丢弃 */
        parser->state.discarded_bytes += (uint32_t)parser->buffered;
        parser->buffered = 0;
        ++parser->state.partial_timeouts;
    }

    /*  */
    for (size_t i = 0; i < length; ++i)
    {
        parser->last_byte_ms = now_ms;

        /* 残缺帧丢弃 */
        if (parser->buffered == 0u && bytes[i] != CRSF_ADDRESS_FLIGHT_CONTROLLER)
        {
            ++parser->state.discarded_bytes;
            continue;
        }

        /* 串口字节填入frame */
        parser->frame[parser->buffered++] = bytes[i];
        while (parser->buffered != 0u)
        {
            if (parser->frame[0] != CRSF_ADDRESS_FLIGHT_CONTROLLER)
            {
                ++parser->state.discarded_bytes;
                consume(parser, 1);
                continue;
            }
            if (parser->buffered < 2u)
                break;

            size_t frame_length = parser->frame[1];
            if (frame_length < 2u || frame_length > CRSF_FRAME_SIZE_MAX - 2u)
            {
                ++parser->state.length_errors;
                ++parser->state.discarded_bytes;
                consume(parser, 1);
                continue;
            }

            size_t total = frame_length + 2u;
            if (parser->buffered < total)
                break;
            if (crsf_crc(&parser->frame[2], frame_length - 1u) != parser->frame[total - 1u])
            {
                ++parser->state.crc_errors;
                ++parser->state.discarded_bytes;
                consume(parser, 1); /* search retained bytes for the next FC address */
                continue;
            }
            decode_frame(parser, total, now_ms);
            consume(parser, total);
        }
    }
}

void crsf_parser_snapshot(const crsf_parser_t *parser, uint32_t now_ms,
                          uint32_t signal_timeout_ms, crsf_state_t *state)
{
    if (parser == NULL || state == NULL)
        return;
    *state = parser->state;
    state->signal_valid = state->has_channels &&
        (uint32_t)(now_ms - state->last_rc_ms) < signal_timeout_ms;
    state->link_statistics_valid = state->link_frames != 0u &&
        (uint32_t)(now_ms - state->last_link_ms) < 250u;
}

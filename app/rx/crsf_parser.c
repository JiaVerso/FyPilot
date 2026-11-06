/* SPDX-License-Identifier: GPL-3.0-or-later
 * Adapted from HPMicro Betaflight crsfDataReceive/crsfFrameStatus/crsfReadRawRC.
 * Keeps the upstream 0x16 packing, CRC-8/DVB-S2 and channel scaling.
 * Uses explicit bit extraction instead of compiler-dependent C bit-fields.
 * Full upstream notices and sources: app/rx/crsf.c, crsf.h, crsf_protocol.h and ORIGIN.txt.
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

uint16_t crsf_channel_to_us(uint16_t raw)
{
    /* Same legacy scale and offset as HPM BF; truncate the result to whole us. */
    return (uint16_t)(0.62477120195241f * (float)raw + 881.0f);
}

void crsf_parser_init(crsf_parser_t *parser)
{
    if (parser != NULL)
        memset(parser, 0, sizeof(*parser));
}

static void consume(crsf_parser_t *parser, size_t count)
{
    parser->buffered -= count;
    memmove(parser->frame, parser->frame + count, parser->buffered);
}

static void decode_frame(crsf_parser_t *parser, size_t total, uint32_t now_ms)
{
    crsf_state_t *state = &parser->state;
    const uint8_t *payload = &parser->frame[3];
    if (parser->frame[2] == CRSF_FRAMETYPE_RC_CHANNELS_PACKED)
    {
        if (total != 26u) /* address + length + type + 22 payload + CRC */
        {
            ++state->length_errors;
            return;
        }
        uint32_t bits = 0;
        unsigned available = 0;
        size_t pos = 0;
        for (unsigned channel = 0; channel < CRSF_CHANNEL_COUNT; ++channel)
        {
            while (available < 11u)
            {
                bits |= (uint32_t)payload[pos++] << available;
                available += 8u;
            }
            state->raw[channel] = (uint16_t)(bits & 0x7ffu);
            state->channel_us[channel] = crsf_channel_to_us(state->raw[channel]);
            bits >>= 11;
            available -= 11u;
        }
        state->has_channels = true;
        state->last_rc_ms = now_ms;
        ++state->rc_frames;
    }
    else if (parser->frame[2] == CRSF_FRAMETYPE_LINK_STATISTICS)
    {
        if (total != 14u)
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

void crsf_parser_feed(crsf_parser_t *parser, const uint8_t *bytes,
                      size_t length, uint32_t now_ms)
{
    if (parser == NULL || (bytes == NULL && length != 0u))
        return;
    if (parser->buffered != 0u &&
        (uint32_t)(now_ms - parser->last_byte_ms) >= CRSF_PARTIAL_TIMEOUT_MS)
    {
        parser->state.discarded_bytes += (uint32_t)parser->buffered;
        parser->buffered = 0;
        ++parser->state.partial_timeouts;
    }
    for (size_t i = 0; i < length; ++i)
    {
        parser->last_byte_ms = now_ms;
        if (parser->buffered == 0u && bytes[i] != CRSF_ADDRESS_FLIGHT_CONTROLLER)
        {
            ++parser->state.discarded_bytes;
            continue;
        }
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

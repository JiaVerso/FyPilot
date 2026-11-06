/* SPDX-License-Identifier: GPL-3.0-or-later
 * CRSF receive adaptation based on HPMicro's Betaflight rx/crsf.c.
 * Upstream sources and revision: app/rx/crsf.c, crsf.h, crsf_protocol.h and ORIGIN.txt.
 */
#ifndef FYPILOT_CRSF_H
#define FYPILOT_CRSF_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define CRSF_CHANNEL_COUNT 16u
#define CRSF_FRAME_SIZE_MAX 64u
#define CRSF_ADDRESS_FLIGHT_CONTROLLER 0xc8u
#define CRSF_FRAMETYPE_LINK_STATISTICS 0x14u
#define CRSF_FRAMETYPE_RC_CHANNELS_PACKED 0x16u
#define CRSF_BAUDRATE 420000u
#define CRSF_PARTIAL_TIMEOUT_MS 20u

typedef struct
{
    uint16_t raw[CRSF_CHANNEL_COUNT];
    uint16_t channel_us[CRSF_CHANNEL_COUNT];
    uint32_t last_rc_ms;
    uint32_t rc_frames;
    uint32_t crc_errors;
    uint32_t length_errors;
    uint32_t partial_timeouts;
    uint32_t discarded_bytes;
    uint32_t ignored_frames;
    uint32_t link_frames;
    uint32_t last_link_ms;
    uint8_t link_quality;
    int16_t rssi_dbm;
    int8_t snr_db;
    uint8_t rf_mode;
    bool has_channels;
    bool signal_valid;
    bool link_statistics_valid;
} crsf_state_t;

/* A parser has a single owner. The RT-Thread adapter publishes atomic copies. */
typedef struct
{
    uint8_t frame[CRSF_FRAME_SIZE_MAX];
    size_t buffered;
    uint32_t last_byte_ms;
    crsf_state_t state;
} crsf_parser_t;

void crsf_parser_init(crsf_parser_t *parser);
void crsf_parser_feed(crsf_parser_t *parser, const uint8_t *bytes,
                      size_t length, uint32_t now_ms);
void crsf_parser_snapshot(const crsf_parser_t *parser, uint32_t now_ms,
                          uint32_t signal_timeout_ms, crsf_state_t *state);
uint16_t crsf_channel_to_us(uint16_t raw);

#endif

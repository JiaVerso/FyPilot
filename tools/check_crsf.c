/* SPDX-License-Identifier: GPL-3.0-or-later
 * Captured vectors and expected values: HPM Betaflight rx_crsf_unittest.cc.
 */
#include "crsf_parser.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static const uint8_t captured[] = {
    0xc8,0x18,0x16,0xbd,0x08,0x9f,0xf4,0xae,0xf7,0xbd,0xef,0x7d,0xef,
    0xfb,0xad,0xfd,0x45,0x2b,0x5a,0x01,0x00,0x00,0x00,0x00,0x00,0x6c
}; /* Upstream capture uses address 0x00; only the address is changed to FC. */

static uint8_t crc_reference(const uint8_t *data, size_t length)
{
    unsigned remainder = 0;
    for (size_t i = 0; i < length; ++i)
    {
        remainder ^= (unsigned)data[i] << 8;
        for (unsigned j = 0; j < 8; ++j)
        {
            remainder <<= 1;
            if (remainder & 0x10000u)
                remainder ^= 0x1d500u;
        }
        remainder &= 0xffffu;
    }
    return (uint8_t)(remainder >> 8);
}

static void make_channels(uint8_t frame[26], const uint16_t channels[16])
{
    memset(frame, 0, 26);
    frame[0] = 0xc8; frame[1] = 24; frame[2] = 0x16;
    /* Independent bit-by-bit packer, rather than mirroring the decoder. */
    for (unsigned ch = 0; ch < 16; ++ch)
        for (unsigned bit = 0; bit < 11; ++bit)
        {
            unsigned pos = ch * 11 + bit;
            if (channels[ch] & (1u << bit))
                frame[3 + pos / 8] |= (uint8_t)(1u << (pos % 8));
        }
    frame[25] = crc_reference(&frame[2], 23);
}

static void capture_and_fragment_tests(void)
{
    assert(crc_reference(&captured[2], 23) == 0x6c);
    assert(crsf_channel_to_us(172) == 988);
    assert(crsf_channel_to_us(992) == 1500);
    assert(crsf_channel_to_us(1811) == 2012);
    for (size_t split = 0; split <= sizeof(captured); ++split)
    {
        crsf_parser_t parser;
        crsf_state_t state;
        crsf_parser_init(&parser);
        crsf_parser_feed(&parser, captured, split, 10);
        crsf_parser_feed(&parser, captured + split, sizeof(captured) - split, 11);
        crsf_parser_snapshot(&parser, 12, 100, &state);
        assert(state.has_channels && state.signal_valid && state.rc_frames == 1);
        assert(state.raw[0] == 189 && state.raw[1] == 993);
        assert(state.raw[2] == 978 && state.raw[3] == 983);
        assert(state.channel_us[0] == 999 && state.channel_us[1] == 1501);
        assert(state.channel_us[2] == 1492 && state.channel_us[3] == 1495);
    }
    crsf_parser_t parser;
    crsf_parser_init(&parser);
    for (unsigned repeat = 0; repeat < 3; ++repeat)
        for (size_t i = 0; i < sizeof(captured); ++i)
            crsf_parser_feed(&parser, &captured[i], 1, 20 + repeat);
    assert(parser.state.rc_frames == 3 && parser.buffered == 0);
}

static void rejection_and_timeout_tests(void)
{
    crsf_parser_t parser;
    crsf_state_t state;
    crsf_parser_init(&parser);
    crsf_parser_snapshot(&parser, 0, 100, &state);
    assert(!state.has_channels && !state.signal_valid);
    uint8_t bad[26];
    memcpy(bad, captured, sizeof(bad)); bad[25] ^= 1;
    crsf_parser_feed(&parser, bad, sizeof(bad), 1);
    assert(parser.state.crc_errors == 1 && !parser.state.has_channels);
    memcpy(bad, captured, sizeof(bad)); bad[0] = 0xec;
    crsf_parser_feed(&parser, bad, sizeof(bad), 2);
    assert(!parser.state.has_channels);
    uint8_t wrong_size[] = {0xc8, 2, 0x16, 0};
    wrong_size[3] = crc_reference(&wrong_size[2], 1);
    crsf_parser_feed(&parser, wrong_size, sizeof(wrong_size), 3);
    assert(parser.state.length_errors == 1 && !parser.state.has_channels);
    uint8_t invalid_lengths[] = {0xc8, 0, 0xc8, 1, 0xc8, 63, 0xc8, 255};
    crsf_parser_feed(&parser, invalid_lengths, sizeof(invalid_lengths), 4);
    assert(parser.state.length_errors == 5);
    crsf_parser_feed(&parser, captured, 7, 10);
    crsf_parser_feed(&parser, captured, sizeof(captured), 30);
    assert(parser.state.partial_timeouts == 1 && parser.state.rc_frames == 1);
    /* Unknown and subset packets must not refresh channel freshness. */
    uint8_t unknown[] = {0xc8, 2, 0x17, 0};
    unknown[3] = crc_reference(&unknown[2], 1);
    crsf_parser_feed(&parser, unknown, sizeof(unknown), 120);
    crsf_parser_snapshot(&parser, 129, 100, &state);
    assert(state.signal_valid && state.last_rc_ms == 30);
    crsf_parser_snapshot(&parser, 130, 100, &state);
    assert(!state.signal_valid && state.raw[0] == 189);
    /* A corrupted but plausible long length must resynchronize without a gap. */
    uint8_t prefix[] = {0xc8, 62, 0x16, 0x55};
    crsf_parser_feed(&parser, prefix, sizeof(prefix), 140);
    for (unsigned i = 0; i < 4; ++i)
        crsf_parser_feed(&parser, captured, sizeof(captured), 141);
    assert(parser.state.rc_frames > 1 && parser.state.raw[0] == 189);
    crsf_parser_init(&parser);
    crsf_parser_feed(&parser, captured, sizeof(captured), UINT32_MAX - 10u);
    crsf_parser_snapshot(&parser, 20, 100, &state);
    assert(state.signal_valid);
    crsf_parser_snapshot(&parser, 89, 100, &state);
    assert(!state.signal_valid);
}

static void link_statistics_test(void)
{
    crsf_parser_t parser;
    crsf_state_t state;
    crsf_parser_init(&parser);
    uint8_t frame[] = {0xc8, 12, 0x14, 65, 72, 98, 0xf9, 1, 2, 3, 80, 97, 4, 0};
    frame[13] = crc_reference(&frame[2], 11);
    crsf_parser_feed(&parser, frame, sizeof(frame), 15);
    crsf_parser_snapshot(&parser, 20, 100, &state);
    assert(!state.signal_valid && !state.has_channels && state.link_statistics_valid);
    assert(state.rssi_dbm == -72 && state.link_quality == 98 && state.snr_db == -7);
    crsf_parser_snapshot(&parser, 265, 100, &state);
    assert(!state.link_statistics_valid);
}

static uint32_t rng_state = 0x589cef17u;
static uint32_t random_value(void)
{
    rng_state ^= rng_state << 13;
    rng_state ^= rng_state >> 17;
    rng_state ^= rng_state << 5;
    return rng_state;
}

static void randomized_tests(void)
{
    crsf_parser_t parser;
    uint16_t channels[16];
    uint8_t frame[26];
    crsf_parser_init(&parser);
    for (unsigned round = 0; round < 5000; ++round)
    {
        for (unsigned i = 0; i < 16; ++i)
            channels[i] = (uint16_t)(random_value() & 0x7ffu);
        make_channels(frame, channels);
        size_t pos = 0;
        while (pos < sizeof(frame))
        {
            size_t count = 1u + random_value() % 13u;
            if (count > sizeof(frame) - pos) count = sizeof(frame) - pos;
            crsf_parser_feed(&parser, frame + pos, count, round);
            pos += count;
        }
        assert(parser.buffered == 0 && parser.state.rc_frames == round + 1);
        assert(memcmp(parser.state.raw, channels, sizeof(channels)) == 0);
    }
    for (unsigned i = 0; i < 250000; ++i)
    {
        uint8_t byte = (uint8_t)random_value();
        crsf_parser_feed(&parser, &byte, 1, i / 1000);
        assert(parser.buffered < CRSF_FRAME_SIZE_MAX);
    }
    crsf_parser_feed(&parser, captured, sizeof(captured), 1000);
    assert(parser.state.raw[0] == 189);
}

int main(void)
{
    capture_and_fragment_tests();
    rejection_and_timeout_tests();
    link_statistics_test();
    randomized_tests();
    puts("CRSF PASS: HPM captured vectors, 16 channels, fragmentation, CRC/noise, timeout/wrap, link stats, 5000 frames and 250000 noise bytes");
    return 0;
}

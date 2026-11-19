/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * Crossfire constants provided by Team Black Sheep under terms of the 2-Clause BSD License
 */

#ifndef FYPILOT_CRSF_PROTOCOL_H
#define FYPILOT_CRSF_PROTOCOL_H

/* Receive-only CRSF subset used by the RT-Thread adapter. */
#define CRSF_BAUDRATE                              420000u
#define CRSF_CHANNEL_COUNT                         16u
#define CRSF_FRAME_SIZE_MAX                        64u
#define CRSF_ADDRESS_FLIGHT_CONTROLLER              0xc8u
#define CRSF_FRAMETYPE_LINK_STATISTICS              0x14u
#define CRSF_FRAMETYPE_RC_CHANNELS_PACKED           0x16u
#define CRSF_FRAME_LINK_STATISTICS_PAYLOAD_SIZE     10u
#define CRSF_FRAME_RC_CHANNELS_PAYLOAD_SIZE         22u
#define CRSF_FRAME_LENGTH_NON_PAYLOAD               4u

#define CRSF_RC_MAX_VALUE 1792
#define CRSF_RC_MIN_VALUE 191

#endif

// Copyright 2026 Enactic, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// KER RS-485 wire protocol - shared by the encoder and the M5.
//
// This header is the single definition of the on-wire format. Both firmwares
// include it through -I../common so the sender and the receiver cannot drift
// apart; keeping two copies is what let the payload layout and the framing
// disagree in the first place.
//
// ---------------------------------------------------------------------------
// Frame: 4 bytes, self-synchronising on the MSB
//
//   byte 0  1 I I I I I C C     header: MSB=1, ID (5 bit), CMD (2 bit)
//   byte 1  0 d d d d d d d     payload bits [6:0]
//   byte 2  0 d d d d d d d     payload bits [13:7]
//   byte 3  0 d d d d d d d     payload bits [20:14]
//
// Only the header has MSB=1, so a receiver can always re-enter the frame on a
// header byte. A byte that arrives with MSB=1 mid-frame means a byte was lost;
// it must be treated as the start of a new frame rather than discarded.
//
// ---------------------------------------------------------------------------
// Payload: 21 bits
//
//   bits [20:6] = angle15   raw TLE5012B AVAL (15 bit, 0.011 deg/LSB)
//   bits [5:0]  = CRC-6 over header7 || angle15   (22 bit message)
//
// The sensor is 15-bit, so the payload never carried more than 15 bits of
// information. The old firmware padded it to 21 bits by replicating angle15's
// top 6 bits into the bottom 6 - pure redundancy with no detection value. Those
// 6 bits now carry a CRC, which costs nothing: the frame is still 4 bytes, the
// angle keeps its position and its full resolution, and the bus timing budget
// is unchanged.
//
// CRC-6, polynomial x^6 + x + 1 (0x03), init all-ones.
//
//   * x^6 + x + 1 is primitive, and the codeword is 22 + 6 = 28 bits, well
//     under 2^6 - 1 = 63. That guarantees Hamming distance 3: every 1-bit and
//     every 2-bit error is detected, as is every burst up to 6 bits.
//   * Residual probability for a random corruption is 2^-6 = 1.6%.
//   * Init is all-ones, not zero, so leading zero bits still affect the result.
//     With a zero init an all-zero message produces an all-zero CRC, i.e. a
//     silent bus would decode as a perfectly valid 0 deg reading.
//
// The CRC covers the header, so a corrupted ID cannot be mistaken for another
// channel's data - which is the failure that produced unexplained jumps.
//
// NOTE: the CRC cannot detect a fault upstream of the encoder's own MCU. A
// disconnected SSC data line reads as a constant, and the encoder will happily
// compute a valid CRC over it. Detecting that needs the TLE5012B safety word.
// ---------------------------------------------------------------------------

#pragma once

#include <stdint.h>

#define KER_CRC6_POLY 0x03
#define KER_CRC6_INIT 0x3F

#define KER_ANGLE_BITS   15
#define KER_ANGLE_MASK   0x7FFFu
#define KER_ANGLE_SHIFT  6
#define KER_LOW6_MASK    0x3Fu
#define KER_PAYLOAD_MASK 0x1FFFFFul

// Feed one bit into the CRC register.
#define KER_CRC6_STEP(crc, in)                                   \
    do {                                                          \
        uint8_t _hi = (uint8_t)(((crc) >> 5) & 0x01);             \
        (crc) = (uint8_t)(((crc) << 1) & KER_LOW6_MASK);          \
        if (_hi ^ (in)) (crc) ^= KER_CRC6_POLY;                   \
    } while (0)

// header7 = (ID << 2) | CMD, i.e. the header byte with its MSB stripped.
//
// The two halves of the message are shifted separately, as 8-bit and 16-bit
// values, and each loop only ever shifts by one. This matters on the ATtiny:
// AVR has no variable-shift instruction, so the obvious "(msg >> i) & 1" over a
// 32-bit word compiles to a shift *loop* whose length grows with i, making the
// whole CRC O(n^2) - about 90 us per packet at 20 MHz, which is more than the
// entire RS-485 chain can afford.
static inline uint8_t ker_crc6(uint8_t header7, uint16_t angle15) {
    uint8_t  crc = KER_CRC6_INIT;

    // 7 header bits, MSB first, left-aligned in a byte.
    uint8_t h = (uint8_t)((header7 & 0x7F) << 1);
    for (uint8_t i = 0; i < 7; i++) {
        KER_CRC6_STEP(crc, (uint8_t)((h & 0x80) ? 1 : 0));
        h = (uint8_t)(h << 1);
    }

    // 15 angle bits, MSB first, left-aligned in a word.
    uint16_t a = (uint16_t)((angle15 & KER_ANGLE_MASK) << 1);
    for (uint8_t i = 0; i < KER_ANGLE_BITS; i++) {
        KER_CRC6_STEP(crc, (uint8_t)((a & 0x8000) ? 1 : 0));
        a = (uint16_t)(a << 1);
    }

    return (uint8_t)(crc & KER_LOW6_MASK);
}

// The angle sits at the same bit position in both the CRC and the legacy
// layout, so decoding it does not depend on which one the sender used.
static inline uint16_t ker_angle15(uint32_t payload21) {
    return (uint16_t)((payload21 >> KER_ANGLE_SHIFT) & KER_ANGLE_MASK);
}

static inline uint8_t ker_low6(uint32_t payload21) {
    return (uint8_t)(payload21 & KER_LOW6_MASK);
}

// What the pre-CRC firmware put in the bottom 6 bits. Used to recognise an
// un-updated encoder so a mixed fleet can be migrated one module at a time.
static inline uint8_t ker_legacy_low6(uint16_t angle15) {
    return (uint8_t)((angle15 >> 9) & KER_LOW6_MASK);
}

static inline uint32_t ker_pack_payload21(uint16_t angle15, uint8_t low6) {
    return (((uint32_t)(angle15 & KER_ANGLE_MASK)) << KER_ANGLE_SHIFT) |
           (uint32_t)(low6 & KER_LOW6_MASK);
}

static inline float ker_angle15_to_deg(uint16_t angle15) {
    return (float)(angle15 & KER_ANGLE_MASK) * (360.0f / 32768.0f);
}

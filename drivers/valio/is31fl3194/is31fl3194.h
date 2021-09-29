/*
    This file is part of MutekH.

    MutekH is free software; you can redistribute it and/or modify it
    under the terms of the GNU Lesser General Public License as
    published by the Free Software Foundation; version 2.1 of the
    License.

    MutekH is distributed in the hope that it will be useful, but
    WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this program.  If not, see
    <http://www.gnu.org/licenses/>.

    Copyright (c) 2021, Nicolas Pouillon <nipo@ssji.net>
*/

#ifndef _IS31FL3194_H_
#define _IS31FL3194_H_

#include <hexo/types.h>
#include <device/class/i2c.h>

#define FL3194_PRODUCT_ID           0x00
#define FL3194_PRODUCT_ID_VALUE     0xce

#define FL3194_MODE                 0x01
#define FL3194_MODE_SHUTDOWN        0x00
#define FL3194_MODE_CURRENT         0x05
#define FL3194_MODE_PATTERN         0x75

// Bitfield of channels
#define FL3194_OUT_ENABLE           0x02

// 3x2 bits, n*10mA
#define FL3194_CURRENT_BAND         0x03
// 3x2 bits, HT, HF
#define FL3194_HOLD        0x04

#define _FL3194_COLOR_R 0
#define _FL3194_COLOR_G 1
#define _FL3194_COLOR_B 2

// Pattern and color are 0-based, channel is R, G, B
#define FL3194_PATTERN_COLOR(pattern, color, channel) (0x10 + 0x10 * (pattern) + 0x3 * (color) + _FL3194_COLOR_##channel)
#define FL3194_PATTERN_T1TS(pattern) (0x19 + 0x10 * (pattern))
#define FL3194_PATTERN_T3T2(pattern) (0x1a + 0x10 * (pattern))
#define FL3194_PATTERN_T4TP(pattern) (0x1b + 0x10 * (pattern))
#define FL3194_PATTERN_COLOR_EN(pattern) (0x1c + 0x10 * (pattern))
#define FL3194_PATTERN_CYCLE_TIME(pattern) (0x1d + 0x10 * (pattern))
#define FL3194_PATTERN_NEXT(pattern) (0x1e + 0x10 * (pattern))
#define FL3194_PATTERN_LOOP_TIME(pattern) (0x1f + 0x10 * (pattern))
#define FL3194_PATTERN_UPDATE(pattern) (0x41 + (pattern))
#define FL3194_PATTERN_STATE(pattern) (0x0d + (pattern))

// Current Mode
#define FL3194_CURRENT(c)  FL3194_PATTERN_COLOR(_FL3194_COLOR_##c, 0, c)
#define FL3194_UPDATE               0x40
#define FL3194_UPDATE_COMMIT        0xc5

#define FL3194_RESET                0x4f
#define FL3194_RESET_COMMIT         0xc5

#endif /* _FL3194_H_ */

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

    Copyright (c) Nicolas Pouillon <nipo@ssji.net> 2022
*/

#ifndef ARCH_NRF_I2S_H_
#define ARCH_NRF_I2S_H_

#include "peripheral.h"
#include "ids.h"

enum nrf5x_i2s_task {
    NRF_I2S_START = 0,
    NRF_I2S_STOP = 1,
};

enum nrf5x_i2s_event {
    NRF_I2S_RXPTRUPD = 0,
    NRF_I2S_STOPPED = 1,
    NRF_I2S_TXPTRUPD = 5,
};

enum nrf5x_i2s_register {
    NRF_I2S_ENABLE = 64,
    NRF_I2S_MODE = 65,
    NRF_I2S_RXEN = 66,
    NRF_I2S_TXEN = 67,
    NRF_I2S_MCKEN = 68,
    NRF_I2S_MCKFREQ = 69,
    NRF_I2S_RATIO = 70,
    NRF_I2S_SWIDTH = 71,
    NRF_I2S_ALIGN = 72,
    NRF_I2S_FORMAT = 73,
    NRF_I2S_CHANNELS = 74,
    NRF_I2S_RXD_PTR = 78,
    NRF_I2S_TXD_PTR = 80,
    NRF_I2S_RTXD_MAXCNT = 84,
    NRF_I2S_PSEL_MCK = 88,
    NRF_I2S_PSEL_SCK = 89,
    NRF_I2S_PSEL_LRCK = 90,
    NRF_I2S_PSEL_SDIN = 91,
    NRF_I2S_PSEL_SDOUT = 92,
};

#define NRF_I2S_MODE_MASTER 0
#define NRF_I2S_MODE_SLAVE 1

#define NRF_I2S_MCKFREQ_16_MHz     0x80000000
#define NRF_I2S_MCKFREQ_10_666_MHz 0x50000000
#define NRF_I2S_MCKFREQ_8_MHz      0x40000000
#define NRF_I2S_MCKFREQ_6_4_MHz    0x30000000
#define NRF_I2S_MCKFREQ_5_333_MHz  0x28000000
#define NRF_I2S_MCKFREQ_4_MHz      0x20000000
#define NRF_I2S_MCKFREQ_3_2_MHz    0x18000000
#define NRF_I2S_MCKFREQ_2_909_MHz  0x16000000
#define NRF_I2S_MCKFREQ_2_133_MHz  0x11000000
#define NRF_I2S_MCKFREQ_2_MHz      0x10000000
#define NRF_I2S_MCKFREQ_1_523_MHz  0x0C000000
#define NRF_I2S_MCKFREQ_1_391_Mhz  0x0B000000
#define NRF_I2S_MCKFREQ_1_066_Mhz  0x08800000
#define NRF_I2S_MCKFREQ_1_032_Mhz  0x08400000
#define NRF_I2S_MCKFREQ_1_Mhz      0x08000000
#define NRF_I2S_MCKFREQ_0_761_Mhz  0x06000000
#define NRF_I2S_MCKFREQ_0_507_Mhz  0x04100000
#define NRF_I2S_MCKFREQ_0_256_Mhz  0x020C0000

#define NRF_I2S_RATIO_32X 0
#define NRF_I2S_RATIO_48X 1
#define NRF_I2S_RATIO_64X 2
#define NRF_I2S_RATIO_96X 3
#define NRF_I2S_RATIO_128X 4
#define NRF_I2S_RATIO_192X 5
#define NRF_I2S_RATIO_256X 6
#define NRF_I2S_RATIO_384X 7
#define NRF_I2S_RATIO_512X 8

#define NRF_I2S_SWIDTH_8 0
#define NRF_I2S_SWIDTH_16 1
#define NRF_I2S_SWIDTH_24 2

#define NRF_I2S_FORMAT_I2S 0
#define NRF_I2S_FORMAT_ALIGNED 1

#define NRF_I2S_ALIGN_LEFT 0
#define NRF_I2S_ALIGN_RIGHT 1

#define NRF_I2S_CHANNEL_LR 0
#define NRF_I2S_CHANNEL_L 1
#define NRF_I2S_CHANNEL_R 2

#endif

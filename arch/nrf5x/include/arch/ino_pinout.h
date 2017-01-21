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

    Copyright (c) 2017 Nicolas Pouillon <nipo@ssji.net>
*/

#ifndef ARCH_NRF5X_INO_PINOUT_H
#define ARCH_NRF5X_INO_PINOUT_H

/* PCA10028 Layout (nRF51-DK) */

#define NRF_pca10028_SCL 7
#define NRF_pca10028_SDA 30
#define NRF_pca10028_AVDD 0
// GND
#define NRF_pca10028_D13 29
#define NRF_pca10028_D12 28
#define NRF_pca10028_D11 25
#define NRF_pca10028_D10 24
#define NRF_pca10028_D9 23
#define NRF_pca10028_D8 20 // Button

#define NRF_pca10028_D7 19 // Button
#define NRF_pca10028_D6 18 // Button
#define NRF_pca10028_D5 17 // Button
#define NRF_pca10028_D4 16
#define NRF_pca10028_D3 15
#define NRF_pca10028_D2 14
#define NRF_pca10028_D1 13
#define NRF_pca10028_D0 12

// 11
// 10
// 9
// 8
// 27
// 26
// 22
// 21

#define NRF_pca10028_A0 1
#define NRF_pca10028_A1 2
#define NRF_pca10028_A2 3
#define NRF_pca10028_A3 4
#define NRF_pca10028_A4 5
#define NRF_pca10028_A5 6

/* PCA10036 Layout (nRF52-Preview-DK) */

#define NRF_pca10036_SCL 27
#define NRF_pca10036_SDA 26
#define NRF_pca10036_AVDD 2
// GND
#define NRF_pca10036_D13 25
#define NRF_pca10036_D12 24
#define NRF_pca10036_D11 23
#define NRF_pca10036_D10 22
#define NRF_pca10036_D9 20
#define NRF_pca10036_D8 19

#define NRF_pca10036_D7 18
#define NRF_pca10036_D6 17
#define NRF_pca10036_D5 16
#define NRF_pca10036_D4 15
#define NRF_pca10036_D3 14
#define NRF_pca10036_D2 13
#define NRF_pca10036_D1 12
#define NRF_pca10036_D0 11

// 10
// 9
// 8
// 7
// 6
// 5
// 21
// 1
// 0

#define NRF_pca10036_A0 3
#define NRF_pca10036_A1 4
#define NRF_pca10036_A2 28
#define NRF_pca10036_A3 29
#define NRF_pca10036_A4 30
#define NRF_pca10036_A5 31

/* PCA10040 Layout (nRF52-DK) */

#define NRF_pca10040_SCL 27
#define NRF_pca10040_SDA 26
#define NRF_pca10040_AVDD 2
// GND
#define NRF_pca10040_D13 25
#define NRF_pca10040_D12 24
#define NRF_pca10040_D11 23
#define NRF_pca10040_D10 22
#define NRF_pca10040_D9 20
#define NRF_pca10040_D8 19

#define NRF_pca10040_D7 18
#define NRF_pca10040_D6 17
#define NRF_pca10040_D5 16
#define NRF_pca10040_D4 15
#define NRF_pca10040_D3 14
#define NRF_pca10040_D2 13
#define NRF_pca10040_D1 12
#define NRF_pca10040_D0 11

// 10
// 9
// 8
// 7
// 6
// 5
// 21
// 1
// 0

#define NRF_pca10040_A0 3
#define NRF_pca10040_A1 4
#define NRF_pca10040_A2 28
#define NRF_pca10040_A3 29
#define NRF_pca10040_A4 30
#define NRF_pca10040_A5 31

/* PCA10056 Layout (nRF52840-Preview-DK) */

#define NRF_pca10056_SCL 27
#define NRF_pca10056_SDA 26
#define NRF_pca10056_AVDD 2
// GND
#define NRF_pca10056_D13 47
#define NRF_pca10056_D12 46
#define NRF_pca10056_D11 45
#define NRF_pca10056_D10 44
#define NRF_pca10056_D9 43
#define NRF_pca10056_D8 42

#define NRF_pca10056_D7 40
#define NRF_pca10056_D6 39
#define NRF_pca10056_D5 38
#define NRF_pca10056_D4 37
#define NRF_pca10056_D3 36
#define NRF_pca10056_D2 35
#define NRF_pca10056_D1 34
#define NRF_pca10056_D0 33

// 10
// 9
// 8
// 7
// 6
// 5
// 1
// 0

#define NRF_pca10056_A0 3
#define NRF_pca10056_A1 4
#define NRF_pca10056_A2 28
#define NRF_pca10056_A3 29
#define NRF_pca10056_A4 30
#define NRF_pca10056_A5 31

// On header P24:
// 41 25 23 21 19 17 15 13 11
// -- 32 24 22 20 18 16 14 12
//    Bu                   Bu
//       Bu                Bu

#define NRF_pca10028_INT_EXT 21
#define NRF_pca10040_INT_EXT 17



/* Magical mapping to *INO pinout */

#define __NRF_INO_PINOUT(b, p) (NRF_##b##_##p)
#define _NRF_INO_PINOUT(b, p) __NRF_INO_PINOUT(b, p)
#define NRF_INO_PINOUT(p) _NRF_INO_PINOUT(CONFIG_NRF5X_BOARD_NAME, p)

#define ARCH_INO_PINOUT_SCL  NRF_INO_PINOUT(SCL)
#define ARCH_INO_PINOUT_SDA  NRF_INO_PINOUT(SDA)
#define ARCH_INO_PINOUT_AREF NRF_INO_PINOUT(AREF)
#define ARCH_INO_PINOUT_A0   NRF_INO_PINOUT(A0)
#define ARCH_INO_PINOUT_A1   NRF_INO_PINOUT(A1)
#define ARCH_INO_PINOUT_A2   NRF_INO_PINOUT(A2)
#define ARCH_INO_PINOUT_A3   NRF_INO_PINOUT(A3)
#define ARCH_INO_PINOUT_A4   NRF_INO_PINOUT(A4)
#define ARCH_INO_PINOUT_A5   NRF_INO_PINOUT(A5)
#define ARCH_INO_PINOUT_D0   NRF_INO_PINOUT(D0)
#define ARCH_INO_PINOUT_D1   NRF_INO_PINOUT(D1)
#define ARCH_INO_PINOUT_D2   NRF_INO_PINOUT(D2)
#define ARCH_INO_PINOUT_D3   NRF_INO_PINOUT(D3)
#define ARCH_INO_PINOUT_D4   NRF_INO_PINOUT(D4)
#define ARCH_INO_PINOUT_D5   NRF_INO_PINOUT(D5)
#define ARCH_INO_PINOUT_D6   NRF_INO_PINOUT(D6)
#define ARCH_INO_PINOUT_D7   NRF_INO_PINOUT(D7)
#define ARCH_INO_PINOUT_D8   NRF_INO_PINOUT(D8)
#define ARCH_INO_PINOUT_D9   NRF_INO_PINOUT(D9)
#define ARCH_INO_PINOUT_D10  NRF_INO_PINOUT(D10)
#define ARCH_INO_PINOUT_D11  NRF_INO_PINOUT(D11)
#define ARCH_INO_PINOUT_D12  NRF_INO_PINOUT(D12)
#define ARCH_INO_PINOUT_D13  NRF_INO_PINOUT(D13)

#define ARCH_INO_PINOUT_SS   NRF_INO_PINOUT(D10)
#define ARCH_INO_PINOUT_MOSI NRF_INO_PINOUT(D11)
#define ARCH_INO_PINOUT_MISO NRF_INO_PINOUT(D12)
#define ARCH_INO_PINOUT_SCK  NRF_INO_PINOUT(D13)

#define ARCH_NRF_PIN_INT_EXT NRF_INO_PINOUT(INT_EXT)

#endif

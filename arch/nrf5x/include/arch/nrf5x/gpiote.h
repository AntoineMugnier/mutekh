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

    Copyright Nicolas Pouillon <nipo@ssji.net> (c) 2014
*/

#ifndef ARCH_NRF_GPIOTE_H_
#define ARCH_NRF_GPIOTE_H_

#include "peripheral.h"
#include "ids.h"

#if CONFIG_NRF5X_MODEL <= 51999
# define NRF_GPIOTE_COUNT 4
#elif 52000 <= CONFIG_NRF5X_MODEL && CONFIG_NRF5X_MODEL <= 52999
# define NRF_GPIOTE_COUNT 8
#endif

#define NRF_GPIOTE_POWER 767

#define NRF_GPIOTE_OUT(x) (x)

#define NRF_GPIOTE_IN(x) (x)
#if 52000 <= CONFIG_NRF5X_MODEL && CONFIG_NRF5X_MODEL <= 52999
#define NRF_GPIOTE_SET(x) ((x) + 12)
#define NRF_GPIOTE_CLR(x) ((x) + 24)
#endif
#define NRF_GPIOTE_PORT 31

#define NRF_GPIOTE_CONFIG(x) (68 + (x))

#define NRF_GPIOTE_CONFIG_MODE_MASK       0x00000003
#define NRF_GPIOTE_CONFIG_MODE_DISABLED   0x00000000
#define NRF_GPIOTE_CONFIG_MODE_EVENT      0x00000001
#define NRF_GPIOTE_CONFIG_MODE_TASK       0x00000003

#define NRF_GPIOTE_CONFIG_PSEL(x) (((x) & 0x3f) << 8)

#define NRF_GPIOTE_CONFIG_POLARITY_MASK   0x00030000
#if 52000 <= CONFIG_NRF5X_MODEL && CONFIG_NRF5X_MODEL <= 52999
# define NRF_GPIOTE_CONFIG_POLARITY_NONE  0x00000000
#endif
#define NRF_GPIOTE_CONFIG_POLARITY_LOTOHI 0x00010000
#define NRF_GPIOTE_CONFIG_POLARITY_HITOLO 0x00020000
#define NRF_GPIOTE_CONFIG_POLARITY_TOGGLE 0x00030000
#define NRF_GPIOTE_CONFIG_POLARITY_MASK   0x00030000

#define NRF_GPIOTE_CONFIG_OUTINIT_MASK    0x00100000
#define NRF_GPIOTE_CONFIG_OUTINIT_LOW     0x00000000
#define NRF_GPIOTE_CONFIG_OUTINIT_HIGH    0x00100000

#endif

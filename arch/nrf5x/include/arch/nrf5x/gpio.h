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

#ifndef ARCH_NRF_GPIO_H_
#define ARCH_NRF_GPIO_H_

#include "peripheral.h"
#include "ids.h"

#define NRF5X_GPIO_ADDR 0x50000000

#define NRF5X_P0_MAX_PINS_NUM 32
#define NRF5X_P1_MAX_PINS_NUM 16

enum nrf5x_gpio_register {
    NRF_GPIO_OUT = 65,
    NRF_GPIO_OUTSET = 66,
    NRF_GPIO_OUTCLR = 67,
    NRF_GPIO_IN = 68,
    NRF_GPIO_DIR = 69,
    NRF_GPIO_DIRSET = 70,
    NRF_GPIO_DIRCLR = 71,
};


#if CONFIG_NRF5X_GPIO_COUNT > NRF5X_P0_MAX_PINS_NUM
# define NRF_GPIO_BANK_OFFSET(pin) (((pin) >> 5) * 192)
# define NRF_GPIO_PIN_CNF(pin) (192 + NRF_GPIO_BANK_OFFSET(pin) + ((pin) & 31))
#else
# define NRF_GPIO_BANK_OFFSET(pin) 0
# define NRF_GPIO_PIN_CNF(pin) (192 + (pin))
#endif

#define NRF_GPIO_PIN_MAP(port, pin) ((port << 5) | (pin & 0x1F))

#define NRF_GPIO_PIN_CNF_DIR_MASK 0x1
#define NRF_GPIO_PIN_CNF_DIR_INPUT 0x0
#define NRF_GPIO_PIN_CNF_DIR_OUTPUT 0x1

#define NRF_GPIO_PIN_CNF_INPUT_MASK 0x2
#define NRF_GPIO_PIN_CNF_INPUT_CONNECT 0x0
#define NRF_GPIO_PIN_CNF_INPUT_DISCONNECT 0x2

#define NRF_GPIO_PIN_CNF_PULL_MASK 0xc
#define NRF_GPIO_PIN_CNF_PULL_DISABLED 0x0
#define NRF_GPIO_PIN_CNF_PULL_DOWN 0x4
#define NRF_GPIO_PIN_CNF_PULL_UP 0xc

#define NRF_GPIO_PIN_CNF_DRIVE_MASK 0x700
#define NRF_GPIO_PIN_CNF_DRIVE_S0S1 0x000
#define NRF_GPIO_PIN_CNF_DRIVE_H0S1 0x100
#define NRF_GPIO_PIN_CNF_DRIVE_S0H1 0x200
#define NRF_GPIO_PIN_CNF_DRIVE_H0H1 0x300
#define NRF_GPIO_PIN_CNF_DRIVE_D0S1 0x400
#define NRF_GPIO_PIN_CNF_DRIVE_D0H1 0x500
#define NRF_GPIO_PIN_CNF_DRIVE_S0D1 0x600
#define NRF_GPIO_PIN_CNF_DRIVE_H0D1 0x700

#define NRF_GPIO_PIN_CNF_SENSE_MASK 0x30000
#define NRF_GPIO_PIN_CNF_SENSE_DISABLED 0x0
#define NRF_GPIO_PIN_CNF_SENSE_HIGH 0x20000
#define NRF_GPIO_PIN_CNF_SENSE_LOW 0x30000

#endif

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

#define NRF51822_GPIO 0x50000000

enum nrf51_gpio_register {
    NRF51_GPIO_OUT = 65,
    NRF51_GPIO_OUTSET = 66,
    NRF51_GPIO_OUTCLR = 67,
    NRF51_GPIO_IN = 68,
    NRF51_GPIO_DIR = 69,
    NRF51_GPIO_DIRSET = 70,
    NRF51_GPIO_DIRCLR = 71,
};

#define NRF51_GPIO_PIN_CNF(x) (192 + (x))

#define NRF51_GPIO_PIN_CNF_DIR_MASK 0x1
#define NRF51_GPIO_PIN_CNF_DIR_INPUT 0x0
#define NRF51_GPIO_PIN_CNF_DIR_OUTPUT 0x1

#define NRF51_GPIO_PIN_CNF_INPUT_MASK 0x2
#define NRF51_GPIO_PIN_CNF_INPUT_CONNECT 0x0
#define NRF51_GPIO_PIN_CNF_INPUT_DISCONNECT 0x2

#define NRF51_GPIO_PIN_CNF_PULL_MASK 0xc
#define NRF51_GPIO_PIN_CNF_PULL_DISABLED 0x0
#define NRF51_GPIO_PIN_CNF_PULL_DOWN 0x4
#define NRF51_GPIO_PIN_CNF_PULL_UP 0xc

#define NRF51_GPIO_PIN_CNF_DRIVE_MASK 0x700
#define NRF51_GPIO_PIN_CNF_DRIVE_S0S1 0x000
#define NRF51_GPIO_PIN_CNF_DRIVE_H0S1 0x100
#define NRF51_GPIO_PIN_CNF_DRIVE_S0H1 0x200
#define NRF51_GPIO_PIN_CNF_DRIVE_H0H1 0x300
#define NRF51_GPIO_PIN_CNF_DRIVE_D0S1 0x400
#define NRF51_GPIO_PIN_CNF_DRIVE_D0H1 0x500
#define NRF51_GPIO_PIN_CNF_DRIVE_S0D1 0x600
#define NRF51_GPIO_PIN_CNF_DRIVE_H0D1 0x700

#define NRF51_GPIO_PIN_CNF_SENSE_MASK 0x30000
#define NRF51_GPIO_PIN_CNF_SENSE_DISABLED 0x0
#define NRF51_GPIO_PIN_CNF_SENSE_HIGH 0x20000
#define NRF51_GPIO_PIN_CNF_SENSE_LOW 0x30000

#endif

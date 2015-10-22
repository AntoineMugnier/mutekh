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

    Copyright Nicolas Pouillon <nipo@ssji.net> (c) 2015
*/
#ifndef NRF5X_BLE_DEBUG_H_
#define NRF5X_BLE_DEBUG_H_

#if defined(CONFIG_DRIVER_NRF5X_BLE_DEBUG)
# include <arch/nrf5x/gpio.h>

# if defined(CONFIG_NRF5X_PCA10028)
#  define I_NEXT_ACTION 0
#  define I_CLOCK_REQ   (1 << 13)
#  define I_CLOCK_RUN   (1 << 14)
#  define I_LATER       (1 << 15)
#  define I_ENABLE      (1 << 16)
#  define I_TRANSFER    (1 << 25)
#  define I_TX          (1 << 28)
#  define I_PIPELINE    (1 << 29)
#  define I_WAIT        (1 << 12)
#  define I_IRQ         ((1 << 1) | (1 << 2))
#  define I_IRQ_RADIO   ((0 << 1) | (1 << 2))
#  define I_IRQ_RTC     ((1 << 1) | (0 << 2))
#  define I_IRQ_TIMER   ((1 << 1) | (1 << 2))
# endif

ALWAYS_INLINE void gpio(uint32_t mask, uint32_t value)
{
  nrf_reg_set(NRF5X_GPIO_ADDR, NRF_GPIO_OUTCLR, mask & ~value);
  nrf_reg_set(NRF5X_GPIO_ADDR, NRF_GPIO_OUTSET, value);
}

#else
# define gpio(...) do{}while(0)
#endif

void nrf5x_ble_debug_init(void);
#define debug(...) do{}while(0)

#endif

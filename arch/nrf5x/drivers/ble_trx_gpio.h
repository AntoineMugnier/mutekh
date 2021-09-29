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

    Copyright (c) 2020, Nicolas Pouillon <nipo@ssji.net>
*/

#ifndef NRF5X_BLE_TRX_GPIO_H_
#define NRF5X_BLE_TRX_GPIO_H_

#if defined(CONFIG_DRIVER_NRF5X_BLE_TRX_GPIO)
# include <arch/nrf5x/gpio.h>

ALWAYS_INLINE
void nrf5x_ble_trx_gpio_off(void)
{
  nrf_reg_set(NRF5X_GPIO_ADDR, NRF_GPIO_PIN_CNF(CONFIG_DRIVER_NRF5X_BLE_TRX_GPIO), 0
              | NRF_GPIO_PIN_CNF_DIR_INPUT
              | NRF_GPIO_PIN_CNF_INPUT_DISCONNECT
              );
}

ALWAYS_INLINE
void nrf5x_ble_trx_gpio_tx(void)
{
  nrf_reg_set(NRF5X_GPIO_ADDR, NRF_GPIO_PIN_CNF(CONFIG_DRIVER_NRF5X_BLE_TRX_GPIO), 0
              | NRF_GPIO_PIN_CNF_DIR_OUTPUT
              | NRF_GPIO_PIN_CNF_INPUT_DISCONNECT
              | NRF_GPIO_PIN_CNF_DRIVE_S0S1
              );
  nrf_reg_set(NRF5X_GPIO_ADDR,
              CONFIG_DRIVER_NRF5X_BLE_TRX_GPIO_TX_VALUE ? NRF_GPIO_OUTSET : NRF_GPIO_OUTCLR,
              1 << CONFIG_DRIVER_NRF5X_BLE_TRX_GPIO);
}

ALWAYS_INLINE
void nrf5x_ble_trx_gpio_rx(void)
{
  nrf_reg_set(NRF5X_GPIO_ADDR, NRF_GPIO_PIN_CNF(CONFIG_DRIVER_NRF5X_BLE_TRX_GPIO), 0
              | NRF_GPIO_PIN_CNF_DIR_OUTPUT
              | NRF_GPIO_PIN_CNF_INPUT_DISCONNECT
              | NRF_GPIO_PIN_CNF_DRIVE_S0S1
              );
  nrf_reg_set(NRF5X_GPIO_ADDR,
              CONFIG_DRIVER_NRF5X_BLE_TRX_GPIO_TX_VALUE ? NRF_GPIO_OUTCLR : NRF_GPIO_OUTSET,
              1 << CONFIG_DRIVER_NRF5X_BLE_TRX_GPIO);
}

# else

ALWAYS_INLINE
void nrf5x_ble_trx_gpio_off(void)
{
}

ALWAYS_INLINE
void nrf5x_ble_trx_gpio_tx(void)
{
}

ALWAYS_INLINE
void nrf5x_ble_trx_gpio_rx(void)
{
}

# endif
#endif

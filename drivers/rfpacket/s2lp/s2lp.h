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
    License along with MutekH; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
    02110-1301 USA.
*/

#ifndef S2LP_H_
#define S2LP_H_

#include <mutek/mem_alloc.h>
#include <mutek/printk.h>
#include <mutek/bytecode.h>
#include <hexo/bit.h>

#include <device/resources.h>
#include <device/device.h>
#include <device/driver.h>
#include <device/irq.h>

#include <string.h>
#include <stdbool.h>

#include <device/class/spi.h>
#include <device/class/gpio.h>
#include <device/class/rfpacket.h>

// Rssi macros
#define S2LP_GET_RSSI(n)              ((n) - 146)
#define S2LP_SET_RSSI(n)              ((n) + 146)

// Fifo values
#define S2LP_FIFO_SIZE 128
#define S2LP_FIFO_THRESHOLD 48

// Bytecode registers
#define S2LP_STATUS 15
#define S2LP_CTX_PV 14

// GPIO ids
#define S2LP_IO_SDN 0
#define S2LP_IO_NIRQ 1

// Config array sizes
#define S2LP_RF_CFG_ARRAY_SIZE 29
#define S2LP_RF_CFG_DATA_SIZE S2LP_RF_CFG_ARRAY_SIZE - 4
#define S2LP_PK_CFG_ARRAY_SIZE 22
#define S2LP_PK_CFG_DATA_SIZE S2LP_PK_CFG_ARRAY_SIZE - 3

// Time values
#define S2LP_BASE_TIME 500 // in us
#define S2LP_RSSI_PERIOD_SHIFT 8 // = S2LP_BASE_TIME * 2 ^ 8 = 128 ms
#define S2LP_RESET_SHIFT 1 // = S2LP_BASE_TIME * 2 ^ 1 = 1 ms
#define S2LP_WAIT_IRQ_SHIFT 6 // = S2LP_BASE_TIME * 2 ^ 6 = 32 ms

// Rssi values
#define S2LP_RSSI_AVG_DEF_VAL -120 // arbitrary, in dbm
#define S2LP_RSSI_AVG_SHIFT 5 // = 2 ^ 5 = 32 ms
#define S2LP_RSSI_AVG_WEIGHT (1 << S2LP_RSSI_AVG_SHIFT) - 1 // = 31

// Lbt values
#define S2LP_LBT_WAIT_CSMA_SHIFT 1 // = S2LP_BASE_TIME * 2 ^ 1 = 1 ms
#define S2LP_LBT_NO_RX_SHIFT 4 // = S2LP_BASE_TIME * 2 ^ 4 = 8 ms
#define S2LP_LBT_POLL_SHIFT 1 // = S2LP_BASE_TIME * 2 ^ 1 = 1 ms
#define S2LP_LBT_BASE_TIME_MULT 10 // = S2LP_BASE_TIME * 10 = 5 ms
#define S2LP_LBT_RAND_TIME_MAX_MULT 8 // = S2LP_BASE_TIME * 8 = 4 ms
#define S2LP_LBT_STATE_FREE 0
#define S2LP_LBT_STATE_BUSY 1

// Byte code status values (dev_rfpacket_status_s mirror)
#define S2LP_BC_STATUS_RX_DONE 0
#define S2LP_BC_STATUS_TX_DONE 1
#define S2LP_BC_STATUS_RX_TIMEOUT 2
#define S2LP_BC_STATUS_TX_TIMEOUT 3
#define S2LP_BC_STATUS_CRC_ERR 4
#define S2LP_BC_STATUS_JAMMING_ERR 5
#define S2LP_BC_STATUS_OTHER_ERR 6
#define S2LP_BC_STATUS_MISC 7

/*
  pv->flags fields :

  xxxxxxxx
  ||||||||
  |||||||\----- RX_CONTINOUS (0)
  ||||||\------ RXC_INFINITE (1)
  |||||\------- RX_TX_OK     (2)
  ||||\-------- TX_POWER_OK  (3)
  |||\--------- RF_CONFIG_OK (4)
  ||\---------- PK_CONFIG_OK (5)
  |\----------- UNUSED       (6)
  \------------ UNUSED       (7)
*/

#define S2LP_FLAGS_RX_CONTINOUS 0x01  // rx continous is active
#define S2LP_FLAGS_RXC_INFINITE 0x02  // rx continous has no lifetime
#define S2LP_FLAGS_RX_TX_OK     0x04  // rx during tx_lbt is possible
#define S2LP_FLAGS_TX_POWER_OK  0x08  // tx power don't need to be configured
#define S2LP_FLAGS_RF_CONFIG_OK 0x10  // rf config don't need to be configured
#define S2LP_FLAGS_PK_CONFIG_OK 0x20  // packet config don't need to be configured

BC_CCALL_FUNCTION(s2lp_alloc);

enum s2lp_irq_src {
  S2LP_IRQ_SRC_NIRQ = 0,
  S2LP_IRQ_SRC_COUNT,
};

struct s2lp_ctx_s {
  // Base time value
  dev_timer_delay_t bt;
  // Last power level
  int16_t pwr;
  // Lbt info
  uint8_t lbt_rssi;
  uint8_t lbt_state;
  dev_timer_delay_t lbt_rand_time;
  // Rssi, carrier level
  uint8_t carrier;
  uint8_t jam_rssi;
  uint32_t avg_rssi;
  // Frequency values
  int16_t afc_offset;
  uint32_t frequency;
  // Generic rfpacket context struct
  struct dev_rfpacket_ctx_s gctx;
  // Device structs
  struct device_s *dev;
  struct dev_irq_src_s src_ep[S2LP_IRQ_SRC_COUNT];
  struct device_spi_ctrl_s spi;
  struct dev_spi_ctrl_bytecode_rq_s spi_rq;
  struct device_timer_s *timer;
  // Driver flags/status
  uint8_t flags;
  uint8_t bc_status;
  // Config structs
  const struct dev_rfpacket_rf_cfg_s *rf_cfg;
  const struct dev_rfpacket_pk_cfg_s *pk_cfg;
  // Config arrays
  uint8_t *rf_cfg_array;
  uint8_t *pk_cfg_array;
  // Gpio map
  gpio_id_t pin_map[2];
};

DRIVER_PV(struct s2lp_ctx_s);

#endif

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

    Copyright (c) 2014 Sebastien Cerdan <sebcerdan@gmail.com>

*/

#ifndef SI446X_H_
#define SI446X_H_

#include <mutek/mem_alloc.h>
#include <mutek/printk.h>
#include <mutek/bytecode.h>
#include <hexo/bit.h>

#include <device/resources.h>
#include <device/device.h>
#include <device/driver.h>
#include <device/irq.h>

#include <string.h>

#include <device/class/spi.h>
#include <device/class/gpio.h>
#include <device/class/rfpacket.h>

#include "modem_calc.h"

#define SI446X_MAX_RSSI_VALUE                    -52       /* in 0.125 dbm */
#define SI446X_MAX_PACKET_SIZE                   256
#define SI446X_PKT_CFG_BUFFER_SIZE               32
#define SI446X_BASE_TIME                         500       /* us */
#define SI446X_MAX_WAIT_CTS_SHIFT                6         // = SI446X_BASE_TIME * 2 ^ 4 = 32 ms
#define SI446X_LBT_WAIT_SHIFT                    3         // = SI446X_BASE_TIME * 2 ^ 3 = 4 ms
#define SI446X_LBT_POLL_SHIFT                    1         // = SI446X_BASE_TIME * 2 ^ 1 = 1 ms
#define SI446X_LBT_BASE_TIME_MULT                10        // = SI446X_BASE_TIME * 10 = 5 ms
#define SI446X_LBT_RAND_TIME_MAX_MULT            8
#define SI446X_RSSI_SAMPLING_PERIOD              8         /* bt log2 */
#define SI446X_RSSI_AVERAGE_DEFAULT              -126
#define SI446X_RSSI_AVERAGE_LOG2_WINDOW_SIZE     8
#define SI446X_RSSI_AVERAGE_WINDOW_SIZE          (1 << SI446X_RSSI_AVERAGE_LOG2_WINDOW_SIZE)

#if !(SI446X_RSSI_AVERAGE_LOG2_WINDOW_SIZE && \
      SI446X_RSSI_AVERAGE_LOG2_WINDOW_SIZE <= 8)
# error
#endif

#define GET_RSSI(n)              (((n) >> 1) - 134)
#define SET_RSSI(n)              (((n) + 134) << 1)

#define GET_CMD(n)               ((n >> 16) & 0xFF)
#define GET_GRP(n)               ((n >> 8) & 0xFF)
#define SET_NBR(n)               (n & 0xFF)

#define SI446X_FIFO_SIZE       64
#define SI446X_FIFO_THRESHOLD  48

#define SI446X_IO_SDN  0
#define SI446X_IO_NIRQ 1
#define SI446X_IO_CTS  2
#define SI446X_IO_XOEN 3

#define SI446X_WUTR_VALUE   0 
#define SI446X_WUT_MIN_TIME (122 * 1000 * (1 << SI446X_WUTR_VALUE)) /* ns */
#define SI446X_RX_THRESHOLD 0x14
  
#define STATUS          15
#define R_CTX_PV        14

//#define SI446X_DEBUG
#ifdef SI446X_DEBUG
# define si446x_printk(...) do { printk(__VA_ARGS__); } while(0)
#else
# define si446x_printk(...) do { } while(0)
#endif

// LBT Bytecode State
#define SI446X_LBT_STATE_FREE 0
#define SI446X_LBT_STATE_BUSY 1



BC_CCALL_FUNCTION(si446x_enable_cts_irq);
BC_CCALL_FUNCTION(si446x_disable_cts_irq);
BC_CCALL_FUNCTION(si446x_alloc);

enum si446x_state_s
{
  SI446X_STATE_INITIALISING,
  SI446X_STATE_ENTER_SLEEP,
  SI446X_STATE_SLEEP,
  SI446X_STATE_AWAKING,
  SI446X_STATE_READY,
  SI446X_STATE_CONFIG,
  SI446X_STATE_CONFIG_RXC,
  SI446X_STATE_CONFIG_RXC_PENDING_STOP,
  SI446X_STATE_RX,
  SI446X_STATE_RXC,
  SI446X_STATE_STOPPING_RXC,
  SI446X_STATE_PAUSE_RXC,
  SI446X_STATE_TX,
  SI446X_STATE_TX_LBT,
  SI446X_STATE_TX_LBT_STOPPING_RXC,
};

enum si446x_irq_srx {
  SI446X_IRQ_SRC_NIRQ = 0,
  SI446X_IRQ_SRC_CTS,
  SI446X_IRQ_SRC_COUNT,
};

struct si446x_modem_config_s
{
  uint32_t frequency;
  uint32_t deviation:24;
  uint32_t bw:24;
  uint32_t drate:24;
  uint8_t symbols;
  bool_t manchester;
  enum dev_rfpacket_modulation_e  mod:8;
  int16_t pwr;
};

struct si446x_pkt_regs_s
{
  struct {
    uint8_t len;
    uint8_t val[4];
  } sw;

  struct {
    uint8_t tx_length;
    uint8_t config_std_1;
    uint8_t config_nstd;
    uint8_t config_std_2;
    uint8_t config;
  } preamble;

#ifdef CONFIG_DRIVER_RFPACKET_SI446X_WUT
  uint8_t clk;
  struct {
    uint8_t m[2];
    uint8_t r;
    uint8_t ldc;
  } wut;
#endif

  uint8_t crc;
}__attribute__((packed));

static const uint8_t si446x_pk_cmd[] = {
  5, 0x11, 0x00,  __builtin_offsetof(struct si446x_pkt_regs_s, sw),
  5, 0x10, 0x00,  __builtin_offsetof(struct si446x_pkt_regs_s, preamble),
  1, 0x12, 0x00,  __builtin_offsetof(struct si446x_pkt_regs_s, crc),
#ifdef CONFIG_DRIVER_RFPACKET_SI446X_WUT
  4, 0x00, 0x05,  __builtin_offsetof(struct si446x_pkt_regs_s, wut),
  1, 0x00, 0x01,  __builtin_offsetof(struct si446x_pkt_regs_s, clk),
#endif
  0
};

/********************* CHFLT_RX */

#define SI446X_RF_CONFIG_CACHE_ENTRY 1

struct si446x_cache_entry_s
{
  struct dev_rfpacket_rf_cfg_s * cfg;
  struct si446x_rf_regs_s data;
  /* Time byte in timer units */
  dev_timer_delay_t tb;
};

struct si446x_ctx_s
{
  dev_timer_value_t rxc_timeout;
  dev_timer_value_t timeout;
  dev_timer_value_t deadline;
  dev_timer_value_t timestamp;
  // Timestamp for txcca
  dev_timer_value_t txcca_timestamp;

  /* base 500 us time */
  dev_timer_delay_t bt;
  /* Time before checking cca status in us*/
  dev_timer_delay_t ccad;
  /* Time to send a complete fifo in us */
  dev_timer_delay_t mpst;

  struct device_s *dev;
  /* Request for received packets */
  struct dev_rfpacket_rx_s *rxrq;
  struct dev_rfpacket_rq_s *rq;
  const uint8_t *rftune;
  /* Current working size and buffer */
  uint8_t *buffer;
  uint16_t size;
  /* Interrupt count */
  uint8_t icount;
  uint8_t flags;
  /* Last power level */
  int16_t pwr;

  uint8_t pending;
  // LBT state
  uint8_t lbt_state;
  dev_timer_delay_t lbt_rand_time;
  /* Rssi, carrier level */
  uint8_t carrier;
  uint8_t jam_rssi;
  uint8_t lbt_rssi;
  uint32_t rssi;
  /* Frequency associated to last Rssi measurment */
  int16_t afc_offset;
  uint16_t osc_ppb;
  uint32_t synth_ratio;
  uint32_t frequency;

  enum si446x_state_s state:8;

  struct dev_irq_src_s src_ep[SI446X_IRQ_SRC_COUNT];
  struct device_spi_ctrl_s spi;
  struct device_timer_s *timer;
  struct dev_spi_ctrl_bytecode_rq_s spi_rq;

  /* Queue for continue/timeout RX requests */
  dev_request_queue_root_t rx_cont_queue;

  /* Queue for requests */
  dev_request_queue_root_t queue;

  /* pending irqs */
  uintptr_t bc_status;

  /* Kroutine for configuration */
  struct kroutine_s kr;
  struct si446x_pkt_regs_s pk_buff;
  struct dev_rfpacket_pk_cfg_s * pk_cfg;
  struct si446x_cache_entry_s cache_array[SI446X_RF_CONFIG_CACHE_ENTRY];
  /* Current cache cfg in use */
  uint8_t id;

  gpio_id_t pin_map[4];

#ifdef CONFIG_DRIVER_RFPACKET_SI446X_STATISTICS
  struct dev_rfpacket_stats_s stats;
#endif
};

STRUCT_COMPOSE(si446x_ctx_s, kr);

DRIVER_PV(struct si446x_ctx_s);

/*
   pv->flags fields :

   xxxxxxxx
      |||||
      ||||\----- RX CONTINOUS (0)
      |||\------ RX ON        (1)
      ||\------- TX POWER     (2)
      |\-------- UNUSED       (3)
      \--------- UNUSED       (4)
*/

#define SI446X_FLAGS_RX_CONTINOUS     0x00000001  /* rx continous ongoing */
#define SI446X_FLAGS_RX_ON            0x00000002  /* rx during tx */
#define SI446X_FLAGS_RXC_INFINITE     0x00000004  /* rx continous has no lifetime */
#define SI446X_FLAGS_TX_POWER         0x00000010  /* tx power must be configured */

/*
   STATUS fields :

   |         status         | |irqcnt|
   xxxxxxxx xxxxxxxx xxxxxxxx xxxxxxxx
                   | ||||||||
                   | |||||||\------ RX THRESHOLD  (8)
                   | ||||||\------- TX THRESHOLD  (9)
                   | |||||\-------- OTHER         (10)
                   | ||||\--------- CRC ERROR     (11)
                   | |||\---------- RX DONE       (12)
                   | ||\----------- TX DONE       (13)
                   | |\------------ RX TIMEOUT    (14)
                   | \------------- TX TIMEOUT    (15)
                   \--------------- JAMMING       (16)
*/

#define STATUS_RX_ALMOST_FULL   8
#define STATUS_TX_ALMOST_EMPTY  9
#define STATUS_OTHER_ERR        10
#define STATUS_CRC_ERROR        11
#define STATUS_PACKET_RX        12
#define STATUS_PACKET_TX        13
#define STATUS_RX_TIMEOUT       14
#define STATUS_TX_TIMEOUT       15
#define STATUS_JAMMING          16

#define STATUS_IRQ_MSK          0xFF
#define STATUS_INFO_MSK         0x00FFFF00
#define STATUS_PH_MSK           0x3B

#define STATUS_RX_IRQ_MSK       (bit(STATUS_PACKET_RX)      |\
                                 bit(STATUS_CRC_ERROR)      |\
                                 bit(STATUS_RX_ALMOST_FULL))

#define STATUS_TX_IRQ_MSK       (bit(STATUS_PACKET_TX)      |\
                                 bit(STATUS_TX_ALMOST_EMPTY))

#define STATUS_RX_END_MSK       (bit(STATUS_PACKET_RX)     |\
                                 bit(STATUS_CRC_ERROR)     |\
                                 bit(STATUS_JAMMING)       |\
                                 bit(STATUS_RX_TIMEOUT))

#define STATUS_TX_END_MSK       (bit(STATUS_PACKET_TX)     |\
                                 bit(STATUS_TX_TIMEOUT))


#define SI446X_PART_INFO_CMD      0x01
#define SI446X_SET_PROPERTY_CMD   0x11
#define SI446X_FIFO_INFO_CMD      0x15
#define SI446X_PACKET_INFO_CMD    0x16
#define SI446X_INT_STATUS_CMD     0x20
#define SI446X_MODEM_STATUS_CMD   0x22
#define SI446X_PH_STATUS_CMD      0x21
#define SI446X_START_TX_CMD       0x31
#define SI446X_START_RX_CMD       0x32
#define SI446X_READ_BUFF_CMD      0x44
#define SI446X_WRITE_FIFO_CMD     0x66
#define SI446X_READ_FIFO_CMD      0x77

#define SI446X_MAX_TX_POWER    13
#define SI446X_MIN_TX_POWER    40
#define SI446X_MAX_CONFIG_SIZE 207

#define CONFIG_DRIVER_RFPACKET_CLK_SOURCE

#define GET_SI446X_XO_BYTE(x) ((CONFIG_DRIVER_RFPACKET_SI446X_FREQ_XO >> (x*8)) & 0xFF)
#define SI446X_X0_TUNE_VALUE 0x30

extern const uint8_t si446x_config[];

#endif

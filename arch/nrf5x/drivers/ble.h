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
#ifndef NRF5X_BLE_RADIO_PRIVATE_H_
#define NRF5X_BLE_RADIO_PRIVATE_H_

#include <device/class/net.h>
#include <device/class/timer.h>
#include <device/class/icu.h>
#include <device/class/cmu.h>

#include <ble/protocol/address.h>
#include <ble/protocol/advertise.h>
#include <ble/protocol/radio.h>

#include <arch/nrf5x/ids.h>
#include <arch/nrf5x/ppi.h>

#include <gct_platform.h>
#include <gct_lock_hexo_lock_irq.h>
#include <gct_atomic.h>

#include <gct/refcount.h>
#include <gct/container_clist.h>

struct net_scheduler_s;

#if defined(CONFIG_DEVICE_CLOCK)
# if CONFIG_NRF5X_MODEL <= 51999
#  define HFCLK_RAMPUP_US     800
# elif 52000 <= CONFIG_NRF5X_MODEL && CONFIG_NRF5X_MODEL <= 52999
#  define HFCLK_RAMPUP_US     500
# endif
#endif

#if CONFIG_NRF5X_MODEL <= 51999
# define RADIO_RAMPUP_US      140
# define RADIO_IRQ_LATENCY_US 8
# define RADIO_RX_CHAIN_DELAY_US 3
# define RADIO_TX_CHAIN_DELAY_US 1
#elif CONFIG_NRF5X_MODEL == 52840
# define RADIO_RAMPUP_US      40
# define RADIO_IRQ_LATENCY_US 2
# define RADIO_RX_CHAIN_DELAY_US 6
# define RADIO_TX_CHAIN_DELAY_US 6
#elif 52000 <= CONFIG_NRF5X_MODEL && CONFIG_NRF5X_MODEL <= 52999
# define RADIO_RAMPUP_US      40
# define RADIO_IRQ_LATENCY_US 2
# define RADIO_RX_CHAIN_DELAY_US 10
# define RADIO_TX_CHAIN_DELAY_US 1
#endif

#define RTC_SKEW_TK 2
#define PPI_LATENCY_US 1

#define US_TO_TICKS_CEIL(us) (((us) * 32768 + 32767) / 1000000)

#define CLOCK_ENABLE_TK US_TO_TICKS_CEIL(HFCLK_RAMPUP_US + RADIO_IRQ_LATENCY_US)
#define RADIO_ENABLE_TK US_TO_TICKS_CEIL(RADIO_RAMPUP_US + RADIO_IRQ_LATENCY_US)
#define PACKET_MIN_TK US_TO_TICKS_CEIL(BLE_T_IFS + BLE_PACKET_TIME(20))
#define BLE_CRC_TIME_US 24

/* Must use Timer0 and RTC0 as there are some hardwired PPIs */
#define BLE_RADIO_ADDR NRF_PERIPHERAL_ADDR(NRF5X_RADIO)
#define BLE_TIMER_ADDR NRF_PERIPHERAL_ADDR(NRF5X_TIMER0)
#define BLE_RTC_ADDR NRF_PERIPHERAL_ADDR(NRF5X_RTC0)

enum event_status_e
{
    EVENT_STATUS_IN_PAST,
    EVENT_STATUS_WINDOW_DONE,
    EVENT_STATUS_HANDLER_DONE,
    EVENT_STATUS_RESOURCES_MISSING,
};

enum rtc_channel_e
{
  RTC_ENABLE = 0, /* Must be 0 because of hardwired PPI */
  RTC_REQUEST_BOUNDARY,
  RTC_START,
};

enum timer_channel_e
{
  TIMER_IFS_START,
  TIMER_IFS_TIMEOUT,
  TIMER_PIPELINE_RESET,
};

enum ppi_id_e
{
  PPI_RTC_REQUEST_END_DISABLE = CONFIG_DRIVER_NRF5X_BLE_PPI_FIRST,
  PPI_RTC_MATCH_START,
  PPI_END_TIMER_START,
  PPI_ADDRESS_TIMER_STOP,
  PPI_END_PIPELINE_RESET,
  PPI_TIMER_IFS_RADIO_START,

  PPI_RTC_ENABLE_TXEN = NRF_PPI_RTC0_COMPARE_0_RADIO_TXEN,
  PPI_RTC_ENABLE_RXEN = NRF_PPI_RTC0_COMPARE_0_RADIO_RXEN,
};

enum ppi_group_id_e
{
  PPI_GROUP_PIPELINE_RESET = CONFIG_DRIVER_NRF5X_BLE_PPI_GROUP_FIRST,
};

enum nrf5x_ble_transfer_e {
  MODE_TX,
  MODE_RX,
};

struct nrf5x_ble_params_s
{
  uint32_t access;
  uint32_t crc_init;
  uint8_t channel;
  int16_t tx_power;
  enum nrf5x_ble_transfer_e mode;
  enum ble_phy_mode_e phy;
  bool_t whitening;
  bool_t rx_rssi;
};

#define GCT_CONTAINER_ALGO_nrf5x_ble_context_list CLIST

//#define NRF5X_BLE_BACKLOG 32

#ifdef NRF5X_BLE_BACKLOG
struct nrf5x_ble_backlog_s {
  uint32_t ts;
  const char *msg;
  uint32_t arg;
};
#endif

DRIVER_PV(struct nrf5x_ble_context_s {
  GCT_CONTAINER_ENTRY(nrf5x_ble_context_list, entry);

  dev_timer_value_t event_begin;
  dev_timer_value_t event_end;
  dev_timer_delay_t event_max_duration;
  uint32_t importance;
  bool_t precise_timing : 1;
  bool_t scheduled : 1;
  bool_t closing : 1;
  uint8_t status;

  struct nrf5x_ble_private_s *pv;
  const struct nrf5x_ble_context_handler_s *handler;

#ifdef NRF5X_BLE_BACKLOG
  struct nrf5x_ble_backlog_s backlog[NRF5X_BLE_BACKLOG];
  size_t backlog_cur;
#endif
});

#ifdef NRF5X_BLE_BACKLOG
void nrf5x_ble_backlog(struct nrf5x_ble_context_s *ctx,
                         const char *msg,
                         uint32_t arg);
void nrf5x_ble_backlog_dump(struct nrf5x_ble_context_s *ctx);
#else
#define nrf5x_ble_backlog(...) do{}while(0)
#define nrf5x_ble_backlog_dump(...) do{}while(0)
#endif

GCT_CONTAINER_TYPES(nrf5x_ble_context_list, struct nrf5x_ble_context_s *, entry);
GCT_CONTAINER_FCNS(nrf5x_ble_context_list, static inline, nrf5x_ble_context_list,
                   init, destroy, push, pop, pushback, next,
                   head, isempty, remove, insert_prev);

void nrf5x_ble_context_list_insert_sorted(nrf5x_ble_context_list_root_t *root,
                                          struct nrf5x_ble_context_s *ctx);

struct nrf5x_ble_context_s;

struct nrf5x_ble_private_s {
  struct dev_irq_src_s irq_source[NRF5X_BLE_IRQ_COUNT];

  struct device_s *dev;

  nrf5x_ble_context_list_root_t context_list;
  nrf5x_ble_context_list_root_t closed_list;

  struct nrf5x_ble_context_s *current;
  uint8_t *transmitting;

  struct nrf5x_ble_params_s current_params;
  struct nrf5x_ble_params_s next_params;
  struct kroutine_s rescheduler;
  struct kroutine_s closer;

  enum ble_sca_e sca;

  int16_t rx_rssi;

  dev_timer_value_t base;
  dev_timer_value_t address_ts;

  dev_timer_value_t event_begin;
  dev_timer_value_t event_end;
  dev_timer_delay_t event_max_duration;

  uint8_t context_count;
  uint8_t event_packet_count;

  bool_t pipelining : 1;
  bool_t pipelining_race : 1;
  bool_t wait_end : 1;
  bool_t hfclk_is_precise : 1;

#if defined(CONFIG_DEVICE_CLOCK)
  struct dev_freq_s sleep_freq;
  struct dev_clock_sink_ep_s clock_sink[NRF5X_BLE_CLK_COUNT];
#endif
};

DEV_IRQ_SRC_PROCESS(nrf5x_ble_radio_irq);
DEV_IRQ_SRC_PROCESS(nrf5x_ble_timer_irq);
DEV_IRQ_SRC_PROCESS(nrf5x_ble_rtc_irq);

error_t nrf5x_ble_data_setup(struct nrf5x_ble_private_s *pv);
void nrf5x_ble_rtc_start(struct nrf5x_ble_private_s *pv);
void nrf5x_ble_rtc_stop(struct nrf5x_ble_private_s *pv);
uint64_t nrf5x_ble_rtc_value_get(struct nrf5x_ble_private_s *pv);
void nrf5x_ble_radio_init(void);
void nrf5x_ble_rtc_init(void);
void nrf5x_ble_timer_init(void);
void nrf5x_ble_ppi_init(void);
void nrf5x_ble_radio_disable(struct nrf5x_ble_private_s *pv);
void nrf5x_ble_config_init(const struct nrf5x_ble_params_s *params);
bool_t nrf5x_ble_pipelined_setup(struct nrf5x_ble_private_s *pv);
void nrf5x_ble_rtc_boundary_set(dev_timer_value_t value, bool_t stop);
void nrf5x_ble_rtc_boundary_clear(void);

void nrf5x_ble_event_address_matched(struct nrf5x_ble_private_s *pv);
void nrf5x_ble_event_bcc_end(struct nrf5x_ble_private_s *pv);
void nrf5x_ble_event_packet_ended(struct nrf5x_ble_private_s *pv);
void nrf5x_ble_event_ifs_timeout(struct nrf5x_ble_private_s *pv);
void nrf5x_ble_event_timeout(struct nrf5x_ble_private_s *pv);
void nrf5x_ble_context_start_first(struct nrf5x_ble_private_s *pv);
void nrf5x_ble_ppi_cleanup(struct nrf5x_ble_private_s *pv);
void nrf5x_ble_pipelined_reset(struct nrf5x_ble_private_s *pv);
void nrf5x_ble_pipelined_commit(struct nrf5x_ble_private_s *pv);
void nrf5x_ble_addr_get(struct ble_addr_s *addr);

ALWAYS_INLINE
bool_t ble_radio_params_equal(
    const struct nrf5x_ble_params_s *a,
    const struct nrf5x_ble_params_s *b)
{
  return a->access == b->access
    && a->crc_init == b->crc_init
    && a->channel == b->channel
    && a->mode == b->mode;
}

#if defined(CONFIG_DEVICE_CLOCK)

bool_t nrf5x_ble_clock_request(struct nrf5x_ble_private_s *pv);
void nrf5x_ble_clock_release(struct nrf5x_ble_private_s *pv);

#else

ALWAYS_INLINE
bool_t nrf5x_ble_clock_request(struct nrf5x_ble_private_s *pv)
{
  return 1;
}

ALWAYS_INLINE
void nrf5x_ble_clock_release(struct nrf5x_ble_private_s *pv)
{
}

#endif

struct nrf5x_ble_context_handler_s
{
  bool_t (*event_opened)(struct nrf5x_ble_context_s *radio);
  void (*event_closed)(struct nrf5x_ble_context_s *radio,
                       enum event_status_e status);
  bool_t (*radio_params)(struct nrf5x_ble_context_s *radio,
                         struct nrf5x_ble_params_s *params);
  uint8_t *(*payload_get)(struct nrf5x_ble_context_s *radio,
                          enum nrf5x_ble_transfer_e mode);
  void (*ifs_event)(struct nrf5x_ble_context_s *radio, bool_t rx_timeout);
  void (*payload_received)(struct nrf5x_ble_context_s *radio,
                           dev_timer_value_t timestamp,
                           int16_t rssi,
                           bool_t crc_valid);
};

void nrf5x_ble_context_init(struct nrf5x_ble_private_s *priv,
                            struct nrf5x_ble_context_s *ctx,
                            const struct nrf5x_ble_context_handler_s *handler);

void nrf5x_ble_context_cleanup(struct nrf5x_ble_context_s *ctx);

void nrf5x_ble_context_schedule(struct nrf5x_ble_context_s *ctx,
                                dev_timer_value_t event_begin,
                                dev_timer_value_t event_end,
                                dev_timer_delay_t event_max_duration,
                                bool_t precise_timing,
                                uint32_t importance);

error_t nrf5x_ble_master_create(struct net_scheduler_s *scheduler,
                                struct nrf5x_ble_private_s *priv,
                                const void *params_,
                                void *delegate,
                                const struct net_layer_delegate_vtable_s *delegate_vtable,
                                struct net_layer_s **layer);

error_t nrf5x_ble_advertiser_create(struct net_scheduler_s *scheduler,
                                    struct nrf5x_ble_private_s *priv,
                                    const void *params,
                                    void *delegate,
                                    const struct net_layer_delegate_vtable_s *delegate_vtable,
                                    struct net_layer_s **layer);

error_t nrf5x_ble_scanner_create(struct net_scheduler_s *scheduler,
                                 struct nrf5x_ble_private_s *priv,
                                 const void *params_,
                                 void *delegate,
                                 const struct net_layer_delegate_vtable_s *delegate_vtable_,
                                 struct net_layer_s **layer);

error_t nrf5x_ble_slave_create(struct net_scheduler_s *scheduler,
                               struct nrf5x_ble_private_s *priv,
                               const void *params,
                               void *delegate,
                               const struct net_layer_delegate_vtable_s *delegate_vtable,
                               struct net_layer_s **layer);
error_t nrf5x_ble_dtm_tx_create(struct net_scheduler_s *scheduler,
                                struct nrf5x_ble_private_s *priv,
                                const void *params_,
                                void *delegate,
                                const struct net_layer_delegate_vtable_s *delegate_vtable,
                                struct net_layer_s **layer);

static
ALWAYS_INLINE
bool_t nrf5x_ble_phy_is_supported(enum ble_phy_mode_e mode)
{
  switch (mode) {
  case BLE_PHY_1M:
    return 1;
#if CONFIG_NRF5X_MODEL >= 52000
  case BLE_PHY_2M:
    return 1;
#if CONFIG_NRF5X_MODEL == 52833 || CONFIG_NRF5X_MODEL == 52840
  case BLE_PHY_CODED8:
  case BLE_PHY_CODED2:
    return 1;
#endif
#endif
  default:
    return 0;
  }
}

#endif

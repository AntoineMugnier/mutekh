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

#include <device/class/ble_radio.h>
#include <device/class/crypto.h>
#include <device/class/timer.h>
#include <device/class/icu.h>
#include <device/class/clock.h>

#include <arch/nrf5x/ids.h>
#include <arch/nrf5x/ppi.h>

#include <net/layer.h>

#include <gct_platform.h>
#include <gct_lock_hexo_lock_irq.h>
#include <gct_atomic.h>

#include <gct/refcount.h>
#include <gct/container_clist.h>

struct net_scheduler_s;
struct net_layer_s;
struct net_layer_delegate_vtable_s;

#if defined(CONFIG_DEVICE_CLOCK)
# if defined(CONFIG_ARCH_NRF51)
#  if CONFIG_DRIVER_NRF5X_CLOCK_HFCLK_FREQ == 32
#   define HFCLK_RAMPUP_US     750
#  else
#   define HFCLK_RAMPUP_US     800
#  endif
# elif defined(CONFIG_ARCH_NRF52)
#  define HFCLK_RAMPUP_US      500
# endif
#endif

#if defined(CONFIG_ARCH_NRF51)
# define RADIO_RAMPUP_US      140
# define RADIO_IRQ_LATENCY_US 8
#elif defined(CONFIG_ARCH_NRF52)
/*
  nRF52 has a fast rampup mode, let's use it... But there a quirk:
  T_IFS is enforced by hardware only if fast rampup is disabled,
  therefore, we enable this feature only at request startup and
  restart, but not for pipelining.
*/
# define RADIO_RAMPUP_US      40
# define RADIO_IRQ_LATENCY_US 2
#endif

#define RTC_SKEW 2

#define US_TO_TICKS_CEIL(us) (((us) * 32768 + 32767) / 1000000)

#define CLOCK_ENABLE_TK US_TO_TICKS_CEIL(HFCLK_RAMPUP_US + RADIO_IRQ_LATENCY_US)
#define RADIO_ENABLE_TK US_TO_TICKS_CEIL(RADIO_RAMPUP_US + RADIO_IRQ_LATENCY_US)
#define PACKET_MIN_TK US_TO_TICKS_CEIL(BLE_T_IFS + BLE_PACKET_TIME(20))

/* Must use Timer0 and RTC0 as there are some hardwired PPIs */
#define BLE_RADIO_ADDR NRF_PERIPHERAL_ADDR(NRF5X_RADIO)
#define BLE_TIMER_ADDR NRF_PERIPHERAL_ADDR(NRF5X_TIMER0)
#define BLE_RTC_ADDR NRF_PERIPHERAL_ADDR(NRF5X_RTC0)

enum event_status_e
{
    EVENT_STATUS_IFS_TIMEOUT,
    EVENT_STATUS_IN_PAST,
    EVENT_STATUS_WINDOW_DONE,
    EVENT_STATUS_HANDLER_DONE,
    EVENT_STATUS_PREEMPTED,
    EVENT_STATUS_DEADLINE_MISSED,
    EVENT_STATUS_PIPELINE_FAILED,
};

enum rtc_channel_e
{
  RTC_ENABLE = 0, /* Must be 0 because of hardwired PPI */
  RTC_REQUEST_BOUNDARY,
  RTC_START,
};

enum timer_channel_e
{
  TIMER_IFS_TIMEOUT,
};

enum ppi_id_e
{
  PPI_RTC_REQUEST_END_DISABLE = CONFIG_DRIVER_NRF5X_BLE_PPI_FIRST,
  PPI_RTC_MATCH_START,
  PPI_END_TIMER_START,
  PPI_ADDRESS_TIMER_STOP,
  PPI_ADDRESS_DISABLE_PIPELINE,
  PPI_RADIO_READY_START,

  PPI_RTC_ENABLE_TXEN = NRF_PPI_RTC0_COMPARE_0_RADIO_TXEN,
  PPI_RTC_ENABLE_RXEN = NRF_PPI_RTC0_COMPARE_0_RADIO_RXEN,
};

enum ppi_group_id_e
{
  PPI_GROUP_PIPELINE_DISABLE = CONFIG_DRIVER_NRF5X_BLE_PPI_GROUP_FIRST,
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
  bool_t rx_rssi;
  bool_t ifs_timeout;
};

#define GCT_CONTAINER_ALGO_nrf5x_ble_context_list CLIST

struct nrf5x_ble_context_s {
  struct net_layer_s layer;

  GCT_CONTAINER_ENTRY(nrf5x_ble_context_list, entry);

  dev_timer_value_t event_begin;
  dev_timer_value_t event_end;
  dev_timer_delay_t event_max_duration;
  uint32_t can_miss_score;
  bool_t scheduled : 1;
  uint8_t status;

  struct nrf5x_ble_private_s *pv;
  const struct nrf5x_ble_context_handler_s *handler;
};

STRUCT_COMPOSE(nrf5x_ble_context_s, layer);

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

  struct dev_freq_accuracy_s sleep_acc;
  nrf5x_ble_context_list_root_t context_list;
  nrf5x_ble_context_list_root_t closed_list;

  struct nrf5x_ble_context_s *current;

  struct nrf5x_ble_params_s current_params;
  struct nrf5x_ble_params_s next_params;
  struct buffer_s *transmitting;

  struct kroutine_s rescheduler;
  struct kroutine_s closer;

  dev_timer_value_t base;
  dev_timer_value_t address_ts;

  dev_timer_value_t event_begin;
  dev_timer_value_t event_end;
  dev_timer_delay_t event_max_duration;

  uint8_t context_count;
  uint8_t event_packet_count;

  bool_t pipelining_race;

#if defined(CONFIG_DEVICE_CLOCK)
  struct dev_clock_sink_ep_s clock_sink[NRF5X_BLE_CLK_COUNT];

  bool_t accurate_clock_requested;
  bool_t accurate_clock_running;
#endif

#if defined(CONFIG_BLE_CRYPTO)
  struct device_crypto_s crypto;
#endif
};

DEV_IRQ_SRC_PROCESS(nrf5x_ble_radio_irq);
DEV_IRQ_SRC_PROCESS(nrf5x_ble_timer_irq);
DEV_IRQ_SRC_PROCESS(nrf5x_ble_rtc_irq);

void nrf5x_ble_data_setup(struct nrf5x_ble_private_s *pv);
void nrf5x_ble_rtc_start(struct nrf5x_ble_private_s *pv);
void nrf5x_ble_rtc_stop(struct nrf5x_ble_private_s *pv);
uint64_t nrf5x_ble_rtc_value_get(struct nrf5x_ble_private_s *pv);
void nrf5x_ble_radio_init(void);
void nrf5x_ble_rtc_init(void);
void nrf5x_ble_timer_init(void);
void nrf5x_ble_ppi_init(void);
void nrf5x_ble_radio_disable(struct nrf5x_ble_private_s *pv);
void nrf5x_ble_config_init(const struct nrf5x_ble_params_s *params);
bool_t nrf5x_ble_pipelined_setup(const struct nrf5x_ble_params_s *params);
void nrf5x_ble_rtc_boundary_set(dev_timer_value_t value, bool_t stop);
void nrf5x_ble_rtc_boundary_clear(void);

void nrf5x_ble_event_address_matched(struct nrf5x_ble_private_s *pv);
void nrf5x_ble_event_packet_ended(struct nrf5x_ble_private_s *pv);
void nrf5x_ble_event_ifs_timeout(struct nrf5x_ble_private_s *pv);
void nrf5x_ble_event_timeout(struct nrf5x_ble_private_s *pv);
void nrf5x_ble_context_start_first(struct nrf5x_ble_private_s *pv);
void nrf5x_ble_ppi_cleanup(struct nrf5x_ble_private_s *pv);
void nrf5x_ble_pipelined_reset(struct nrf5x_ble_private_s *pv);
void nrf5x_ble_addr_get(struct ble_addr_s *addr);

ALWAYS_INLINE
bool_t ble_radio_params_equal(
    const struct nrf5x_ble_params_s *a,
    const struct nrf5x_ble_params_s *b)
{
  return a->access == b->access
    && a->crc_init == b->crc_init
    && a->channel == b->channel
    && a->ifs_timeout == b->ifs_timeout
    && a->mode == b->mode;
}

#if defined(CONFIG_DEVICE_CLOCK)

bool_t nrf5x_ble_clock_request(struct nrf5x_ble_private_s *pv);
void nrf5x_ble_clock_release(struct nrf5x_ble_private_s *pv);
DEV_CLOCK_SINK_CHANGED(nrf5x_ble_clock_changed);

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
  void (*event_opened)(struct nrf5x_ble_context_s *radio);
  void (*event_closed)(struct nrf5x_ble_context_s *radio,
                       enum event_status_e status);
  bool_t (*radio_params)(struct nrf5x_ble_context_s *radio,
                         struct nrf5x_ble_params_s *params);
  struct buffer_s *(*payload_get)(struct nrf5x_ble_context_s *radio);
  void (*ifs_event)(struct nrf5x_ble_context_s *radio, bool_t rx_timeout);
  void (*payload_received)(struct nrf5x_ble_context_s *radio,
                           dev_timer_value_t timestamp,
                           bool_t crc_valid,
                           struct buffer_s *packet);
};

error_t nrf5x_ble_context_init(struct nrf5x_ble_context_s *ctx,
                               struct net_scheduler_s *scheduler,
                               const struct net_layer_handler_s *layer_handler,
                               struct nrf5x_ble_private_s *priv,
                               const struct nrf5x_ble_context_handler_s *handler,
                               void *delegate,
                               const struct net_layer_delegate_vtable_s *delegate_vtable);

void nrf5x_ble_context_cleanup(struct nrf5x_ble_context_s *ctx);

void nrf5x_ble_context_schedule(struct nrf5x_ble_context_s *ctx,
                                dev_timer_value_t event_begin,
                                dev_timer_value_t event_end,
                                dev_timer_delay_t event_max_duration,
                                uint32_t can_miss_score);

#endif

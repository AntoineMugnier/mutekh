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

    Copyright Nicolas Pouillon <nipo@ssji.net> (c) 2014-2015
*/

#include <mutek/kroutine.h>
#include <mutek/printk.h>
#include <mutek/buffer_pool.h>

#include <device/device.h>
#include <device/driver.h>
#include <device/resources.h>
#include <device/class/icu.h>
#include <device/class/clock.h>
#include <device/class/timer.h>
#include <device/class/ble_radio.h>

#include <arch/nrf5x/ppi.h>
#include <arch/nrf5x/radio.h>
#include <arch/nrf5x/timer.h>
#include <arch/nrf5x/rtc.h>
#include <arch/nrf5x/ficr.h>
#include <arch/nrf5x/uart.h>
#include <arch/nrf5x/clock.h>
#include <arch/nrf5x/power.h>

#include <stdlib.h>

#include <ble/protocol/radio.h>
#include <ble/protocol/address.h>

#include "ble_radio_handler.h"

#if defined(CONFIG_DEVICE_CLOCK)
# if defined(CONFIG_ARCH_NRF51)
#  define HFCLK_RAMPUP_US      900
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

/* Must use Timer0 and RTC0 as there are some hardwired PPIs */
#define BLE_RADIO_ADDR NRF_PERIPHERAL_ADDR(NRF5X_RADIO)
#define BLE_TIMER_ADDR NRF_PERIPHERAL_ADDR(NRF5X_TIMER0)
#define BLE_RTC_ADDR NRF_PERIPHERAL_ADDR(NRF5X_RTC0)

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
  PPI_RTC_REQUEST_END_DISABLE = CONFIG_DRIVER_NRF5X_BLE_RADIO_PPI_FIRST,
  PPI_RTC_MATCH_START,
  PPI_END_TIMER_START,
  PPI_ADDRESS_TIMER_STOP,

  PPI_RTC_ENABLE_TXEN = NRF_PPI_RTC0_COMPARE_0_RADIO_TXEN,
  PPI_RTC_ENABLE_RXEN = NRF_PPI_RTC0_COMPARE_0_RADIO_RXEN,
};

#define US_TO_TICKS_CEIL(us) (((us) * 32768 + 32767) / 1000000)

struct ble_radio;

static DEV_IRQ_SRC_PROCESS(ble_radio_irq);
static DEV_IRQ_SRC_PROCESS(ble_timer_irq);
static DEV_IRQ_SRC_PROCESS(ble_rtc_irq);

static void ble_radio_first_start(struct ble_radio *pv);
static void ble_radio_first_schedule(struct ble_radio *pv);
static void ble_radio_clock_release(struct ble_radio *pv);
static bool_t ble_radio_clock_request(struct ble_radio *pv);
static void ble_radio_queue_schedule(struct ble_radio *pv);
static void ble_radio_request_done(struct ble_radio *pv);
static void ble_radio_data_setup(struct ble_radio *pv);
static void ble_radio_request_callback(struct ble_radio *pv,
                                       struct dev_ble_radio_rq_s *rq);
static void ble_radio_end(struct ble_radio *pv);
static void ble_radio_request_timeout(struct ble_radio *pv);
static void ble_radio_pipelined_reset(struct ble_radio *pv);

static void ble_rtc_start(struct ble_radio *pv)
{
#if defined(CONFIG_DEVICE_CLOCK)
  dev_clock_sink_hold(&pv->clock_sink[NRF5X_BLE_RADIO_CLK_SLEEP], 1);
#endif

  nrf_it_disable_mask(BLE_RTC_ADDR, -1);
  nrf_it_enable(BLE_RTC_ADDR, NRF_RTC_OVERFLW);
  nrf_evt_disable_mask(BLE_RTC_ADDR, -1);
  nrf_task_trigger(BLE_RTC_ADDR, NRF_RTC_START);
}

static void ble_rtc_stop(struct ble_radio *pv)
{
#if defined(CONFIG_DEVICE_CLOCK)
  dev_clock_sink_release(&pv->clock_sink[NRF5X_BLE_RADIO_CLK_SLEEP]);
#endif

  nrf_task_trigger(BLE_RTC_ADDR, NRF_RTC_STOP);
  nrf_it_disable_mask(BLE_RTC_ADDR, -1);
  nrf_evt_disable_mask(BLE_RTC_ADDR, -1);
}

static dev_timer_value_t ble_rtc_value_get(struct ble_radio *pv)
{
  uint32_t counter;

  CPU_INTERRUPT_SAVESTATE_DISABLE;
  counter = nrf_reg_get(BLE_RTC_ADDR, NRF_RTC_COUNTER);
  if (!(counter & 0x800000)
      && nrf_event_check(BLE_RTC_ADDR, NRF_RTC_OVERFLW))
    counter += 0x1000000;
  CPU_INTERRUPT_RESTORESTATE;

  pv->last_now = pv->base + counter;
  return pv->last_now;
}

static void ble_radio_init(void)
{
  nrf_reg_set(BLE_RADIO_ADDR, NRF_RADIO_POWER, 0);

  for (uint16_t i = 0; i < 32; ++i)
    asm volatile("");

  nrf_reg_set(BLE_RADIO_ADDR, NRF_RADIO_POWER, 1);

  for (uint16_t i = 0; i < 32; ++i)
    asm volatile("");

  nrf_reg_set(NRF_PERIPHERAL_ADDR(NRF5X_POWER), NRF_POWER_DCDCEN,
              _CONFIG_NRF5X_RADIO_DCDC ? NRF_POWER_DCDCEN_ENABLE : 0);

  nrf_it_disable_mask(BLE_RADIO_ADDR, -1);

  if ((~cpu_mem_read_32(NRF_FICR_OVERRIDEEN)) & (1 << NRF_FICR_OVERRIDE_BLE_1MBIT)) {
    for (uint8_t i = 0; i < 5; ++i)
      nrf_reg_set(BLE_RADIO_ADDR, NRF_RADIO_OVERRIDE(i),
                  cpu_mem_read_32(NRF_FICR_OVERRIDE(BLE_1MBIT, i)));
  }

  nrf_reg_set(BLE_RADIO_ADDR, NRF_RADIO_MODE, NRF_RADIO_MODE_BLE_1MBIT);

  nrf_reg_set(BLE_RADIO_ADDR, NRF_RADIO_PCNF0, 0
              | (8 << NRF_RADIO_PCNF0_LFLEN_OFFSET)
              | (1 << NRF_RADIO_PCNF0_S0LEN_OFFSET));

  nrf_reg_set(BLE_RADIO_ADDR, NRF_RADIO_TXPOWER, 0);
  nrf_reg_set(BLE_RADIO_ADDR, NRF_RADIO_CRCPOLY, 0x100065B);
  nrf_reg_set(BLE_RADIO_ADDR, NRF_RADIO_CRCCNF, 0
              | NRF_RADIO_CRCCNF_SKIPADDR
              | (3 << NRF_RADIO_CRCCNF_LEN_OFFSET));
  nrf_reg_set(BLE_RADIO_ADDR, NRF_RADIO_TIFS, BLE_T_IFS);

  nrf_task_trigger(BLE_RADIO_ADDR, NRF_RADIO_STOP);
  nrf_task_trigger(BLE_RADIO_ADDR, NRF_RADIO_DISABLE);
}

static void rtc_init(void)
{
    nrf_task_trigger(BLE_RTC_ADDR, NRF_RTC_STOP);
    nrf_reg_set(BLE_RTC_ADDR, NRF_RTC_PRESCALER, 0);
    nrf_task_trigger(BLE_RTC_ADDR, NRF_RTC_CLEAR);
}

static void timer_init(void)
{
    nrf_task_trigger(BLE_TIMER_ADDR, NRF_TIMER_STOP);
    nrf_reg_set(BLE_TIMER_ADDR, NRF_TIMER_PRESCALER, 4);
    nrf_reg_set(BLE_TIMER_ADDR, NRF_TIMER_MODE, NRF_TIMER_MODE_TIMER);
    nrf_reg_set(BLE_TIMER_ADDR, NRF_TIMER_BITMODE, NRF_TIMER_BITMODE_16);
}

static void ppi_init(struct ble_radio *pv)
{
  nrf_ppi_setup(PPI_RTC_REQUEST_END_DISABLE,
                  BLE_RTC_ADDR, NRF_RTC_COMPARE(RTC_REQUEST_BOUNDARY),
                  BLE_RADIO_ADDR, NRF_RADIO_DISABLE);

  nrf_ppi_setup(PPI_RTC_MATCH_START,
                  BLE_RTC_ADDR, NRF_RTC_COMPARE(RTC_START),
                  BLE_RADIO_ADDR, NRF_RADIO_START);

  nrf_ppi_setup(PPI_END_TIMER_START,
                  BLE_RADIO_ADDR, NRF_RADIO_END,
                  BLE_TIMER_ADDR, NRF_TIMER_START);

  nrf_ppi_setup(PPI_ADDRESS_TIMER_STOP,
                  BLE_RADIO_ADDR, NRF_RADIO_ADDRESS,
                  BLE_TIMER_ADDR, NRF_TIMER_STOP);
}

static bool_t ble_radio_params_equal(
    const struct ble_radio_params *a,
    const struct ble_radio_params *b)
{
  return a->access == b->access
    && a->crc_init == b->crc_init
    && a->channel == b->channel
    && a->ifs == b->ifs
    && a->mode == b->mode;
}

static
void ble_radio_disable(struct ble_radio *pv)
{
  pv->radio_current.channel = -1;

  nrf_it_disable_mask(BLE_RADIO_ADDR, -1);
  nrf_short_set(BLE_RADIO_ADDR, 0);

  nrf_ppi_disable_mask(0
                         | (1 << PPI_RTC_MATCH_START)
                         | (1 << PPI_RTC_ENABLE_RXEN)
                         | (1 << PPI_RTC_ENABLE_TXEN)
                         | (1 << PPI_RTC_REQUEST_END_DISABLE)
                         | (1 << PPI_END_TIMER_START)
                         | (1 << PPI_ADDRESS_TIMER_STOP)
                         );

  nrf_evt_disable_mask(BLE_RTC_ADDR, 0
                       | (1 << NRF_RTC_COMPARE(RTC_ENABLE))
                       | (1 << NRF_RTC_COMPARE(RTC_START))
                       );

  nrf_task_trigger(BLE_RADIO_ADDR, NRF_RADIO_STOP);
  nrf_task_trigger(BLE_RADIO_ADDR, NRF_RADIO_DISABLE);
  while (nrf_reg_get(BLE_RADIO_ADDR, NRF_RADIO_STATE)
    != NRF_RADIO_STATE_DISABLED);

  nrf_event_clear(BLE_RADIO_ADDR, NRF_RADIO_ADDRESS);
  nrf_event_clear(BLE_RADIO_ADDR, NRF_RADIO_END);
  nrf_reg_set(BLE_RADIO_ADDR, NRF_RADIO_BCC, 16);
}

static
void ble_radio_config_init(
    struct buffer_pool_s *pool,
    const struct ble_radio_params *params)
{
  // Packet available space, except decryption prefix, header and CRC
  size_t packet_max_size = buffer_pool_unit_size(pool) - 1 - 2 - 3;

  nrf_reg_set(BLE_RADIO_ADDR, NRF_RADIO_TXPOWER, params->tx_power / 8);
  nrf_reg_set(BLE_RADIO_ADDR, NRF_RADIO_PCNF1, 0
              | ((packet_max_size - 2) << NRF_RADIO_PCNF1_MAXLEN_OFFSET)
              | (3 << NRF_RADIO_PCNF1_BALEN_OFFSET)
              | NRF_RADIO_PCNF1_ENDIAN_LITTLE
              | NRF_RADIO_PCNF1_WHITEEN_ENABLED);

  nrf_reg_set(BLE_RADIO_ADDR, NRF_RADIO_BASE0, params->access << 8);
  nrf_reg_set(BLE_RADIO_ADDR, NRF_RADIO_PREFIX0, params->access >> 24);
  nrf_reg_set(BLE_RADIO_ADDR, NRF_RADIO_CRCINIT, params->crc_init);
  nrf_reg_set(BLE_RADIO_ADDR, NRF_RADIO_TXADDRESS, 0);
  nrf_reg_set(BLE_RADIO_ADDR, NRF_RADIO_RXADDRESSES, 1 << 0);
}

static
void ble_radio_channel_set(
    const struct ble_radio_params *params)
{
  nrf_reg_set(BLE_RADIO_ADDR, NRF_RADIO_FREQUENCY,
              ble_channel_freq_mhz(params->channel) - 2400);
  nrf_reg_set(BLE_RADIO_ADDR, NRF_RADIO_DATAWHITEIV, params->channel | 0x40);
}

static
void ble_radio_pipelined_setup(struct ble_radio *pv)
{
  ble_radio_channel_set(&pv->radio_next);

#if defined(CONFIG_ARCH_NRF52)
  nrf_reg_set(BLE_RADIO_ADDR, NRF_RADIO_MODECNF0, 0
    | NRF_RADIO_MODECNF0_RU_NORMAL
    | NRF_RADIO_MODECNF0_DTX_B1);
#endif

  switch (pv->radio_next.mode) {
  case MODE_TX:
    nrf_short_set(BLE_RADIO_ADDR, 0
                  | (1 << NRF_RADIO_END_DISABLE)
                  | (1 << NRF_RADIO_DISABLED_TXEN)
                  | (1 << NRF_RADIO_READY_START));

    nrf_task_trigger(BLE_TIMER_ADDR, NRF_TIMER_STOP);
    nrf_task_trigger(BLE_TIMER_ADDR, NRF_TIMER_CLEAR);

    nrf_ppi_disable_mask(0
                          | (1 << PPI_END_TIMER_START)
                          | (1 << PPI_ADDRESS_TIMER_STOP)
                          );

    if (nrf_event_check(BLE_RADIO_ADDR, NRF_RADIO_END)) {
      nrf_task_trigger(BLE_RADIO_ADDR, NRF_RADIO_TXEN);
    }

    break;

  default:
  case MODE_RX:
    nrf_short_set(BLE_RADIO_ADDR, 0
                  | (1 << NRF_RADIO_END_DISABLE)
                  | (1 << NRF_RADIO_DISABLED_RXEN)
                  | (1 << NRF_RADIO_READY_START));

    if (pv->radio_next.ifs) {
      /*
        RX IFS timeout

        Use some timer running at 1MHz.

        Start timer on radio END, stop on radio ADDRESS.  Set timeout in
        CC after T_IFS + T_preamble + T_access_address + Epsilon.

        If timeout trigggers before timer is stopped, this is because of
        an IFS timeout.
      */

      nrf_task_trigger(BLE_TIMER_ADDR, NRF_TIMER_STOP);
      nrf_task_trigger(BLE_TIMER_ADDR, NRF_TIMER_CLEAR);

      nrf_ppi_enable_mask(0
                            | (1 << PPI_END_TIMER_START)
                            | (1 << PPI_ADDRESS_TIMER_STOP)
                            );

      nrf_reg_set(BLE_TIMER_ADDR, NRF_TIMER_CC(TIMER_IFS_TIMEOUT),
                  BLE_T_IFS + 8 + 32 + RADIO_IRQ_LATENCY_US + 20);
      nrf_it_enable(BLE_TIMER_ADDR, NRF_TIMER_COMPARE(TIMER_IFS_TIMEOUT));
      nrf_event_clear(BLE_TIMER_ADDR, NRF_TIMER_COMPARE(TIMER_IFS_TIMEOUT));
    }

    if (nrf_event_check(BLE_RADIO_ADDR, NRF_RADIO_END)) {
      nrf_task_trigger(BLE_RADIO_ADDR, NRF_RADIO_RXEN);

      nrf_reg_set(BLE_TIMER_ADDR, NRF_TIMER_CC(TIMER_IFS_TIMEOUT),
                  BLE_T_IFS + 8 + 32 + RADIO_IRQ_LATENCY_US + 20 
                  + 800);
    }
    break;
  }
}

static void ble_radio_queue_schedule(struct ble_radio *pv)
{
  struct dev_ble_radio_rq_s *first, *next = NULL;

  assert(!cpu_is_interruptible());

  first = dev_ble_radio_rq_s_cast(dev_request_pqueue_head(&pv->queue));
  if (first)
    next = dev_ble_radio_rq_s_cast(dev_request_pqueue_next(&pv->queue, &first->base));

  assert(!first || !next || (first != next));

  if (pv->first == first && pv->next == next)
    return;

  if (pv->first != first) {
    pv->first = first;

    if (first) {
      dev_timer_value_t now = ble_rtc_value_get(pv);

      if (first->not_before)
        pv->start = first->not_before;
      else
        pv->start = now + US_TO_TICKS_CEIL(RADIO_RAMPUP_US + HFCLK_RAMPUP_US) + 6;

      pv->end = first->not_after;

      //printk("\nNext changed: %p %d start %d end %d\n", pv->first,
      //       pv->first->type, (uint32_t)pv->start, (uint32_t)pv->end);
    }
  }

  if (pv->next != next) {
    pv->next = next;

    if (next && next->not_after && next->not_before
        && next->not_before < pv->end && next->type > first->type) {
      pv->end = first->not_before;
      //printk("  preempt end %d\n", (uint32_t)pv->end);
    }
  }
}

static void ble_radio_first_schedule(struct ble_radio *pv)
{
  assert(!cpu_is_interruptible());

  if (pv->current) {
    return;
  }

  ble_radio_queue_schedule(pv);

  if (!pv->first) {
    ble_radio_clock_release(pv);
    return;
  }

  /*
             We are here
                  |     
                  v              | [ Request handling ]
     Time --------+--------------+------------------------->
                  ^              ^
                  |              |
           Request scheduled   Time we should start
           Clock in unknown    worrying about starting
           state               hardware

     Depending on time we have left until TXEN/RXEN, we may or may not
     have time to release clock.
  */

#if defined(CONFIG_DEVICE_CLOCK)
  dev_timer_value_t now = ble_rtc_value_get(pv);

  if (pv->start > now + US_TO_TICKS_CEIL(RADIO_RAMPUP_US + HFCLK_RAMPUP_US) + 4) {
    uint32_t deadline = pv->start - US_TO_TICKS_CEIL(RADIO_RAMPUP_US + HFCLK_RAMPUP_US) - 2;

    nrf_reg_set(BLE_RTC_ADDR, NRF_RTC_CC(RTC_REQUEST_BOUNDARY), deadline);
    nrf_event_clear(BLE_RTC_ADDR, NRF_RTC_COMPARE(RTC_REQUEST_BOUNDARY));
    nrf_it_enable(BLE_RTC_ADDR, NRF_RTC_COMPARE(RTC_REQUEST_BOUNDARY));

    ble_radio_clock_release(pv);
    return;
  }
#endif

  ble_radio_first_start(pv);
}

static void ble_radio_first_start(struct ble_radio *pv)
{
  /*
                            We are here      Will be both trigged
                                 |           by RTC through PPI
                                 |              |         |
                                 v              v         v
     Time --------+--------------+--------------+---------+------------....>
                  ^              ^              ^         ^
                  |              |              |         |
           Request scheduled Clock enable   TXEN/RXEN   START

    Request clock now, it will be available later, when radio is
    enabled.  As clock rampup and enable times are known constants, if
    we are on time now, TXEN/RXEN and START will be too.
   */

  assert(pv->first);
  assert(!pv->current);
  assert(!pv->transmitting);
  assert(nrf_reg_get(BLE_RADIO_ADDR, NRF_RADIO_STATE) == NRF_RADIO_STATE_DISABLED);

  bool_t clock_running = ble_radio_clock_request(pv);

  // First request is now current request

  pv->current = pv->first;
  pv->first_packet = 1;
  pv->current->status = DEVICE_BLE_RADIO_IN_PAST;
  pv->handler = &request_handler[pv->current->type];

  if (pv->handler->startup)
    pv->handler->startup(pv);

  // Ensure timing is correct

  dev_timer_value_t now = ble_rtc_value_get(pv);

  if (clock_running) {
    if (pv->start < now + US_TO_TICKS_CEIL(RADIO_RAMPUP_US)) {
      printk("Bad timing: %d < %d + ramp\n", (uint32_t)pv->start, (uint32_t)now);
      pv->current->status = DEVICE_BLE_RADIO_IN_PAST;
      ble_radio_request_done(pv);
      return;
    }
  } else {
    if (pv->start < now + US_TO_TICKS_CEIL(RADIO_RAMPUP_US + HFCLK_RAMPUP_US)) {
      printk("Bad timing: %d < %d + ramps\n", (uint32_t)pv->start, (uint32_t)now);
      pv->current->status = DEVICE_BLE_RADIO_IN_PAST;
      ble_radio_request_done(pv);
      return;
    }
  }

  pv->radio_current.channel = -1;

  // Maybe handler wants to abort, after all...

  bool_t running = pv->handler->radio_params(pv, &pv->radio_current);
  if (!running) {
    pv->current->status = DEVICE_BLE_RADIO_HANDLER_DONE;
    ble_radio_request_done(pv);
    return;
  }

  // Actual register setup

  if (pv->end) {
    nrf_reg_set(BLE_RTC_ADDR, NRF_RTC_CC(RTC_REQUEST_BOUNDARY), pv->end);
    nrf_event_clear(BLE_RTC_ADDR, NRF_RTC_COMPARE(RTC_REQUEST_BOUNDARY));
    nrf_it_enable(BLE_RTC_ADDR, NRF_RTC_COMPARE(RTC_REQUEST_BOUNDARY));
    nrf_ppi_enable(PPI_RTC_REQUEST_END_DISABLE);
  }

  nrf_ppi_disable_mask(0
                         | (1 << PPI_END_TIMER_START)
                         | (1 << PPI_ADDRESS_TIMER_STOP)
                         );
  nrf_reg_set(BLE_RADIO_ADDR, NRF_RADIO_BCC, 16);
  ble_radio_config_init(pv->current->packet_pool, &pv->radio_current);
  ble_radio_channel_set(&pv->radio_current);
  ble_radio_data_setup(pv);

#if defined(CONFIG_ARCH_NRF52)
  nrf_reg_set(BLE_RADIO_ADDR, NRF_RADIO_MODECNF0, 0
    | NRF_RADIO_MODECNF0_RU_FAST
    | NRF_RADIO_MODECNF0_DTX_B1);
#endif

  nrf_event_clear(BLE_RTC_ADDR, NRF_RTC_COMPARE(RTC_ENABLE));
  nrf_event_clear(BLE_RTC_ADDR, NRF_RTC_COMPARE(RTC_START));

  nrf_reg_set(BLE_RTC_ADDR, NRF_RTC_CC(RTC_ENABLE), pv->start - US_TO_TICKS_CEIL(RADIO_RAMPUP_US));
  nrf_reg_set(BLE_RTC_ADDR, NRF_RTC_CC(RTC_START), pv->start);

  nrf_evt_enable_mask(BLE_RTC_ADDR, 0
                      | (1 << NRF_RTC_COMPARE(RTC_ENABLE))
                      | (1 << NRF_RTC_COMPARE(RTC_START))
                      );

  switch (pv->radio_current.mode) {
  case MODE_TX:
    nrf_ppi_enable_mask(0
                          | (1 << PPI_RTC_ENABLE_TXEN)
                          | (1 << PPI_RTC_MATCH_START));
    break;

  case MODE_RX:
    nrf_ppi_enable_mask(0
                          | (1 << PPI_RTC_ENABLE_RXEN)
                          | (1 << PPI_RTC_MATCH_START));
    break;

  case MODE_RSSI:
    break;
  }
}

#if defined(CONFIG_DEVICE_CLOCK)

static bool_t ble_radio_clock_request(struct ble_radio *pv)
{
  if (!pv->accurate_clock_requested) {
    pv->accurate_clock_requested = 1;
    dev_clock_sink_hold(&pv->clock_sink[NRF5X_BLE_RADIO_CLK_RADIO], 0);
  }

  return pv->accurate_clock_running;
}

/* TODO: Do this later on */
static void ble_radio_clock_release(struct ble_radio *pv)
{
  if (pv->accurate_clock_requested) {
    pv->accurate_clock_requested = 0;
    dev_clock_sink_release(&pv->clock_sink[NRF5X_BLE_RADIO_CLK_RADIO]);
  }
}

static DEV_CLOCK_SINK_CHANGED(nrf5x_ble_radio_clock_changed)
{
  struct ble_radio *pv = ep->dev->drv_pv;
  uint8_t clk = ep - pv->clock_sink;

  if (clk == NRF5X_BLE_RADIO_CLK_SLEEP
      && ep->src->flags & DEV_CLOCK_SRC_EP_RUNNING)
    pv->sleep_acc = *acc;

  if (clk == NRF5X_BLE_RADIO_CLK_RADIO) {
    pv->accurate_clock_running = !!(ep->src->flags & DEV_CLOCK_SRC_EP_RUNNING);
  }
}

#else

static bool_t ble_radio_clock_request(struct ble_radio *pv)
{
  return 1;
}

static void ble_radio_clock_release(struct ble_radio *pv)
{
}

#endif

static DEVICE_BLE_RADIO_REQUEST(nrf5x_ble_radio_request);
static DEVICE_BLE_RADIO_CANCEL(nrf5x_ble_radio_cancel);
static DEVICE_BLE_RADIO_GET_INFO(nrf5x_ble_radio_get_info);

static DEV_TIMER_CONFIG(nrf5x_ble_rtc_config)
{
  struct device_s *dev = accessor->dev;
  struct ble_radio *pv = dev->drv_pv;
  error_t err = 0;

  LOCK_SPIN_IRQ(&dev->lock);

  if (cfg) {
    cfg->freq.num = 32768;
    cfg->freq.denom = 1;
    cfg->acc = pv->sleep_acc;
    cfg->rev = 0;
    cfg->res = 1;
    cfg->cap = DEV_TIMER_CAP_STOPPABLE | DEV_TIMER_CAP_HIGHRES;
    cfg->max = (dev_timer_value_t)-1;
  }

  if (res > 1)
    err = -ERANGE;

  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
}

static DEV_TIMER_GET_VALUE(nrf5x_ble_rtc_get_value)
{
  struct device_s *dev = accessor->dev;
  struct ble_radio *pv = dev->drv_pv;
  error_t err = 0;

  LOCK_SPIN_IRQ(&dev->lock);

  if (value)
    *value = ble_rtc_value_get(pv);

  if (rev)
    err = -EAGAIN;

  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
}


static DEV_INIT(nrf5x_ble_radio_init);
static DEV_CLEANUP(nrf5x_ble_radio_cleanup);
static DEV_USE(nrf5x_ble_radio_use);

#define nrf5x_ble_rtc_request (dev_timer_request_t*)dev_driver_notsup_fcn
#define nrf5x_ble_rtc_cancel (dev_timer_request_t*)dev_driver_notsup_fcn

DRIVER_DECLARE(nrf5x_ble_radio_drv, 0, "nRF5x BLE-Radio", nrf5x_ble_radio,
               DRIVER_BLE_RADIO_METHODS(nrf5x_ble_radio),
               DRIVER_TIMER_METHODS(nrf5x_ble_rtc));

DRIVER_REGISTER(nrf5x_ble_radio_drv);

static DEV_USE(nrf5x_ble_radio_use)
{
  struct device_s *dev = accessor->dev;
  struct ble_radio *pv = dev->drv_pv;
  uint32_t old;

  switch (op) {
  default:
    return 0;

  case DEV_USE_START:
    old = pv->use_count;

    pv->use_count++;

    if (!old)
      ble_rtc_start(pv);
    break;

  case DEV_USE_STOP:
    pv->use_count--;

    if (!pv->use_count)
      ble_rtc_stop(pv);
    break;
  }

  return 0;
}

static DEV_INIT(nrf5x_ble_radio_init)
{
  error_t err = -ENOENT;
  struct ble_radio *pv;

  dev->status = DEVICE_DRIVER_INIT_FAILED;

  pv = malloc(sizeof(*pv));
  if (!pv)
    return -ENOMEM;

  memset(pv, 0, sizeof(*pv));

  __unused__ uintptr_t addr = 0;
  assert(device_res_get_uint(dev, DEV_RES_MEM, 0, &addr, NULL) == 0 &&
         BLE_RADIO_ADDR == addr);

  assert(device_res_get_uint(dev, DEV_RES_MEM, 1, &addr, NULL) == 0 &&
         BLE_TIMER_ADDR == addr);

  assert(device_res_get_uint(dev, DEV_RES_MEM, 2, &addr, NULL) == 0 &&
         BLE_RTC_ADDR == addr);

  device_irq_source_init(dev, &pv->irq_source[NRF5X_BLE_RADIO_IRQ_RADIO], 1, &ble_radio_irq);
  device_irq_source_init(dev, &pv->irq_source[NRF5X_BLE_RADIO_IRQ_TIMER], 1, &ble_timer_irq);
  device_irq_source_init(dev, &pv->irq_source[NRF5X_BLE_RADIO_IRQ_RTC], 1, &ble_rtc_irq);

  err = device_irq_source_link(dev, pv->irq_source, NRF5X_BLE_RADIO_IRQ_COUNT, -1);
  if (err)
    goto free_pv;

#if defined(CONFIG_DEVICE_CLOCK)
  pv->accurate_clock_requested = 0;
  pv->accurate_clock_running = 0;

  dev_clock_sink_init(dev, &pv->clock_sink[NRF5X_BLE_RADIO_CLK_RADIO], &nrf5x_ble_radio_clock_changed);
  dev_clock_sink_init(dev, &pv->clock_sink[NRF5X_BLE_RADIO_CLK_SLEEP], &nrf5x_ble_radio_clock_changed);

  struct dev_clock_link_info_s ckinfo[2];
  if (dev_clock_sink_link(dev, pv->clock_sink, ckinfo, 0, 1))
    goto free_pv;
#endif

  dev_request_pqueue_init(&pv->queue);

  ble_radio_init();
  rtc_init();
  timer_init();
  ppi_init(pv);

  dev->drv = &nrf5x_ble_radio_drv;
  dev->status = DEVICE_DRIVER_INIT_DONE;
  dev->drv_pv = pv;

  return 0;

 free_pv:
  free(pv);

  return err;
}

static DEV_CLEANUP(nrf5x_ble_radio_cleanup)
{
  struct ble_radio *pv = dev->drv_pv;

  nrf_it_disable_mask(BLE_RADIO_ADDR, -1);
  nrf_it_disable_mask(BLE_TIMER_ADDR, -1);
  nrf_it_disable_mask(BLE_RTC_ADDR, -1);

  ble_radio_disable(pv);

  nrf_task_trigger(BLE_TIMER_ADDR, NRF_TIMER_STOP);
  nrf_task_trigger(BLE_RTC_ADDR, NRF_RTC_STOP);

  dev_request_pqueue_destroy(&pv->queue);

#if defined(CONFIG_DEVICE_CLOCK)
  dev_clock_sink_unlink(dev, pv->clock_sink, 2);
#endif

  device_irq_source_unlink(dev, pv->irq_source, NRF5X_BLE_RADIO_IRQ_COUNT);

  mem_free(pv);
}

static DEVICE_BLE_RADIO_REQUEST(nrf5x_ble_radio_request)
{
  struct device_s *dev = accessor->dev;
  struct ble_radio *pv = dev->drv_pv;
  dev_timer_value_t now = ble_rtc_value_get(pv);

  assert(!rq->base.drvdata);

  if (rq->not_before && rq->not_before < now)
    return -ETIMEDOUT;

  LOCK_SPIN_IRQ(&dev->lock);

  rq->base.drvdata = dev;
  dev_ble_radio_pqueue_insert(&pv->queue, &rq->base);

  if (!pv->callbacking)
    ble_radio_first_schedule(pv);

  LOCK_RELEASE_IRQ(&dev->lock);

  return 0;
}

static DEVICE_BLE_RADIO_CANCEL(nrf5x_ble_radio_cancel)
{
  struct device_s *dev = accessor->dev;
  struct ble_radio *pv = dev->drv_pv;
  error_t err;

  LOCK_SPIN_IRQ(&dev->lock);

  if (rq->base.drvdata != dev) {
    err = -ENOENT;
  } else if (rq == pv->current) {
    err = -EBUSY;
  } else {
    dev_ble_radio_pqueue_remove(&pv->queue, &rq->base);
    rq->base.drvdata = NULL;
    err = 0;

    if (rq == pv->next || rq == pv->first)
      ble_radio_first_schedule(pv);

    assert(rq != pv->next && rq != pv->first && rq != pv->current);
  }

  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
}

static DEVICE_BLE_RADIO_GET_INFO(nrf5x_ble_radio_get_info)
{
  info->address.type = (cpu_mem_read_32(NRF_FICR_DEVICEADDRTYPE) & 1)
    ? BLE_ADDR_RANDOM : BLE_ADDR_PUBLIC;

  memcpy(info->address.addr, (void*)NRF_FICR_DEVICEADDR(0), 6);
  if (info->address.type == BLE_ADDR_RANDOM)
    info->address.addr[5] |= 0xc0;

  info->prefix_size = 1;
  info->mtu = CONFIG_BLE_PACKET_SIZE - 1;

  assert(info->prefix_size + info->mtu <= CONFIG_BLE_PACKET_SIZE);

  return 0;
}

static
void ble_radio_request_callback(
  struct ble_radio *pv,
  struct dev_ble_radio_rq_s *rq)
{
  struct device_s *dev = rq->base.drvdata;
  assert(dev);

  assert(!cpu_is_interruptible());
  dev_request_pqueue_remove(&pv->queue, &rq->base);
  rq->base.drvdata = NULL;

  pv->first = pv->next = NULL;

  pv->callbacking = 1;
  lock_release(&dev->lock);
  //  printk("req cb\n");
  kroutine_exec(&rq->base.kr);
  lock_spin(&dev->lock);
  pv->callbacking = 0;
}

static void ble_radio_request_done(struct ble_radio *pv)
{
  struct dev_ble_radio_rq_s *rq = pv->current;

  ble_radio_disable(pv);

  nrf_it_disable(BLE_RTC_ADDR, NRF_RTC_COMPARE(RTC_REQUEST_BOUNDARY));
  nrf_evt_disable(BLE_RTC_ADDR, NRF_RTC_COMPARE(RTC_REQUEST_BOUNDARY));

  assert(pv->current && pv->current->base.drvdata && pv->handler);

  if (pv->handler->cleanup)
    pv->handler->cleanup(pv);

  if (pv->transmitting) {
    buffer_refdec(pv->transmitting);
    pv->transmitting = NULL;
  }

  pv->current = NULL;
  pv->handler = NULL;

  ble_radio_request_callback(pv, rq);
  ble_radio_first_schedule(pv);
}

static void ppi_cleanup(struct ble_radio *pv)
{
  nrf_ppi_disable_mask(0
                         | (1 << PPI_RTC_MATCH_START)
                         | (1 << PPI_RTC_ENABLE_RXEN)
                         | (1 << PPI_RTC_ENABLE_TXEN)
                         | (1 << PPI_RTC_REQUEST_END_DISABLE)
                         );

  nrf_evt_disable_mask(BLE_RTC_ADDR, 0
                       | (1 << NRF_RTC_COMPARE(RTC_ENABLE))
                       | (1 << NRF_RTC_COMPARE(RTC_START))
                       );

  nrf_it_disable(BLE_TIMER_ADDR, NRF_TIMER_COMPARE(TIMER_IFS_TIMEOUT));
}

static
void ble_radio_address_match(struct ble_radio *pv)
{
  if (!pv->current) {
    ble_radio_disable(pv);
    ble_radio_first_schedule(pv);
    return;
  }

  ppi_cleanup(pv);

  pv->address_timestamp = ble_rtc_value_get(pv) - 2;

  if (pv->first_packet) {
    if (pv->current->max_duration) {
      pv->first_packet = 0;
      pv->end = pv->address_timestamp + pv->current->max_duration;

      nrf_reg_set(BLE_RTC_ADDR, NRF_RTC_CC(RTC_REQUEST_BOUNDARY), pv->end);
      nrf_event_clear(BLE_RTC_ADDR, NRF_RTC_COMPARE(RTC_REQUEST_BOUNDARY));
      nrf_it_enable(BLE_RTC_ADDR, NRF_RTC_COMPARE(RTC_REQUEST_BOUNDARY));
      nrf_ppi_enable(PPI_RTC_REQUEST_END_DISABLE);
    } else {
      pv->end = 0;
      nrf_it_disable(BLE_RTC_ADDR, NRF_RTC_COMPARE(RTC_REQUEST_BOUNDARY));
      nrf_ppi_disable(PPI_RTC_REQUEST_END_DISABLE);
      nrf_event_clear(BLE_RTC_ADDR, NRF_RTC_COMPARE(RTC_REQUEST_BOUNDARY));
    }
  }

  if (pv->handler->ifs_event)
    pv->handler->ifs_event(pv, 0);

  bool_t running = pv->handler->radio_params(pv, &pv->radio_next);

  nrf_it_disable_mask(BLE_RADIO_ADDR, 0
                      | (1 << NRF_RADIO_BCMATCH)
                      | (1 << NRF_RADIO_END)
                      );

  if (running)
    ble_radio_pipelined_setup(pv);
  else {
    nrf_short_set(BLE_RADIO_ADDR, 1 << NRF_RADIO_END_DISABLE);

    nrf_ppi_disable_mask(0
                           | (1 << PPI_END_TIMER_START)
                           | (1 << PPI_ADDRESS_TIMER_STOP)
                           );

    nrf_it_disable(BLE_TIMER_ADDR, NRF_TIMER_COMPARE(TIMER_IFS_TIMEOUT));

    pv->radio_current.channel = -1;
  }

  bool_t pipelined_in_time = !nrf_event_check(BLE_RADIO_ADDR, NRF_RADIO_END);
  if (!pipelined_in_time) {
    ble_radio_request_timeout(pv);
    return;
  }

  while (!nrf_event_check(BLE_RADIO_ADDR, NRF_RADIO_BCMATCH)) {
    if (!nrf_reg_get(BLE_RADIO_ADDR, NRF_RADIO_STATE) == NRF_RADIO_STATE_RX) {
      printk("Not in RX any more\n");
      break;
    }
    assert(nrf_reg_get(BLE_RADIO_ADDR, NRF_RADIO_BCC) == 16);
  }

  nrf_event_clear(BLE_RADIO_ADDR, NRF_RADIO_BCMATCH);

  uint8_t len = cpu_mem_read_8((uintptr_t)(pv->transmitting->data + pv->transmitting->begin + 1));
  uint16_t end_irq_bits = ((uint16_t)len + 5) * 8 - RADIO_IRQ_LATENCY_US;

  nrf_reg_set(BLE_RADIO_ADDR, NRF_RADIO_BCC, end_irq_bits);
  nrf_event_clear(BLE_RADIO_ADDR, NRF_RADIO_BCMATCH);

  if (end_irq_bits > 16 + RADIO_IRQ_LATENCY_US * 2 + 4
      && !nrf_event_check(BLE_RADIO_ADDR, NRF_RADIO_END)) {
    nrf_it_enable_mask(BLE_RADIO_ADDR, 0
                       | (1 << NRF_RADIO_BCMATCH)
                       | (1 << NRF_RADIO_END)
                       );
  } else {
    uint32_t some_long_time = 1000;

    while (!nrf_event_check(BLE_RADIO_ADDR, NRF_RADIO_END)) {
      --some_long_time;

      if (!some_long_time) {
        ble_radio_request_timeout(pv);
        return;
      }
    }

    ble_radio_end(pv);
  }
}

static void ble_radio_data_setup(struct ble_radio *pv)
{
  switch (pv->radio_current.mode) {
  case MODE_TX:
    pv->transmitting = pv->handler->payload_get(pv);
    break;
  case MODE_RX:
    pv->transmitting = nrf5x_ble_radio_packet_alloc(pv);
    break;
  case MODE_RSSI:
    break;
  }

  nrf_reg_set(BLE_RADIO_ADDR, NRF_RADIO_PACKETPTR,
              (uintptr_t)pv->transmitting->data + pv->transmitting->begin);

  nrf_it_disable(BLE_RADIO_ADDR, NRF_RADIO_BCMATCH);
  nrf_short_enable(BLE_RADIO_ADDR, NRF_RADIO_ADDRESS_BCSTART);
  nrf_it_enable_mask(BLE_RADIO_ADDR, 0
                     | (1 << NRF_RADIO_ADDRESS)
                     | (1 << NRF_RADIO_END)
                     );
}

static void ble_radio_pipelined_reset(struct ble_radio *pv)
{
  ble_radio_disable(pv);
  assert(nrf_reg_get(BLE_RADIO_ADDR, NRF_RADIO_STATE) == NRF_RADIO_STATE_DISABLED);

  pv->radio_current = pv->radio_next;
  ble_radio_channel_set(&pv->radio_current);

#if defined(CONFIG_ARCH_NRF52)
  nrf_reg_set(BLE_RADIO_ADDR, NRF_RADIO_MODECNF0, 0
    | NRF_RADIO_MODECNF0_RU_FAST
    | NRF_RADIO_MODECNF0_DTX_B1);
#endif

  nrf_short_enable(BLE_RADIO_ADDR, NRF_RADIO_READY_START);

  switch (pv->radio_current.mode) {
  case MODE_TX:
    nrf_task_trigger(BLE_RADIO_ADDR, NRF_RADIO_TXEN);
    break;
  case MODE_RX:
    nrf_task_trigger(BLE_RADIO_ADDR, NRF_RADIO_RXEN);
    break;
  case MODE_RSSI:
    nrf_task_trigger(BLE_RADIO_ADDR, NRF_RADIO_RSSISTART);
    break;
  }
}

static
void ble_radio_end(struct ble_radio *pv)
{
  nrf_reg_set(BLE_RADIO_ADDR, NRF_RADIO_BCC, 16);
  nrf_event_clear(BLE_RADIO_ADDR, NRF_RADIO_END);

  if (!pv->current) {
    nrf_it_disable(BLE_RADIO_ADDR, NRF_RADIO_END);
    ble_radio_first_schedule(pv);
    return;
  }

  bool_t rx = pv->radio_current.mode == MODE_RX;

  uint32_t shorts = nrf_short_get(BLE_RADIO_ADDR);

  if (shorts & (1 << NRF_RADIO_END_DISABLE) && shorts & (1 << NRF_RADIO_READY_START)
      && shorts & ((1 << NRF_RADIO_DISABLED_RXEN) | (1 << NRF_RADIO_DISABLED_TXEN))) {
    pv->radio_current = pv->radio_next;
  }

  if (rx && pv->handler->payload_received) {
    pv->transmitting->end = pv->transmitting->begin
      + __MIN(pv->transmitting->data[pv->transmitting->begin + 1] + 2,
              buffer_size(pv->transmitting) - pv->transmitting->begin);

    pv->handler->payload_received(
        pv,
        nrf_reg_get(BLE_RADIO_ADDR, NRF_RADIO_CRCSTATUS),
        pv->transmitting);
  }

  buffer_refdec(pv->transmitting);
  pv->transmitting = NULL;

  bool_t running = pv->handler->radio_params(pv, &pv->radio_next);
  if (!running) {
    pv->current->status = DEVICE_BLE_RADIO_HANDLER_DONE;
    ble_radio_request_done(pv);
    return;
  }

  if (pv->next && pv->next->not_after && pv->next->not_before
      && pv->next->type > pv->current->type
      && pv->next->not_before < (ble_rtc_value_get(pv)
                                 + US_TO_TICKS_CEIL(RADIO_RAMPUP_US)
                                 + US_TO_TICKS_CEIL(BLE_T_IFS + BLE_PACKET_TIME(20)))) {
    debug(pv, "!P");
    pv->current->status = DEVICE_BLE_RADIO_PREEMPTED;
    ble_radio_request_done(pv);
    return;
  }

  if (!ble_radio_params_equal(&pv->radio_next, &pv->radio_current))
    ble_radio_pipelined_reset(pv);

  nrf_reg_set(BLE_RADIO_ADDR, NRF_RADIO_BCC, 16);
  ble_radio_data_setup(pv);
}

static
void ble_radio_request_timeout(struct ble_radio *pv)
{
  ppi_cleanup(pv);

  if (pv->current) {
    pv->current->status = DEVICE_BLE_RADIO_WINDOW_DONE;
    ble_radio_request_done(pv);
  } else {
    ble_radio_first_schedule(pv);
  }
}

static
void ble_radio_rx_timeout(struct ble_radio *pv)
{
  nrf_task_trigger(BLE_TIMER_ADDR, NRF_TIMER_STOP);
  ble_radio_disable(pv);
  ppi_cleanup(pv);

  nrf_event_clear(BLE_RADIO_ADDR, NRF_RADIO_BCMATCH);
  nrf_it_disable(BLE_RADIO_ADDR, NRF_RADIO_BCMATCH);
  nrf_reg_set(BLE_RADIO_ADDR, NRF_RADIO_BCC, 16);

  if (pv->transmitting) {
    buffer_refdec(pv->transmitting);
    pv->transmitting = NULL;
  }

  if (!pv->current)
    return;

  pv->current->status = DEVICE_BLE_RADIO_IFS_TIMEOUT;
  if (pv->handler->ifs_event)
    pv->handler->ifs_event(pv, 1);

  bool_t running = pv->handler->radio_params(pv, &pv->radio_next);
  if (!running) {
    pv->current->status = DEVICE_BLE_RADIO_HANDLER_DONE;
    ble_radio_request_done(pv);
    return;
  }

  ble_radio_pipelined_reset(pv);
  ble_radio_data_setup(pv);
}

struct buffer_s *nrf5x_ble_radio_packet_alloc(const struct ble_radio *pv)
{
  struct buffer_s *packet;

  assert(pv->current && pv->current->base.drvdata);

  packet = buffer_pool_alloc(pv->current->packet_pool);

  packet->begin = 1;
  packet->end = buffer_size(packet);
  packet->data[packet->begin + 1] = 0;

  return packet;
}

static DEV_IRQ_SRC_PROCESS(ble_radio_irq)
{
  struct device_s *dev = ep->base.dev;
  struct ble_radio *pv = dev->drv_pv;

  if (nrf_event_check(BLE_RADIO_ADDR, NRF_RADIO_ADDRESS)) {
    nrf_event_clear(BLE_RADIO_ADDR, NRF_RADIO_ADDRESS);

    ble_radio_address_match(pv);
  }

  if (nrf_event_check(BLE_RADIO_ADDR, NRF_RADIO_BCMATCH)
      && nrf_it_is_enabled(BLE_RADIO_ADDR, NRF_RADIO_BCMATCH)) {
    nrf_event_clear(BLE_RADIO_ADDR, NRF_RADIO_BCMATCH);
    nrf_it_disable(BLE_RADIO_ADDR, NRF_RADIO_BCMATCH);

    assert(nrf_reg_get(BLE_RADIO_ADDR, NRF_RADIO_BCC) != 16);
    while (!nrf_event_check(BLE_RADIO_ADDR, NRF_RADIO_END))
      ;
    ble_radio_end(pv);
  }

  if (nrf_event_check(BLE_RADIO_ADDR, NRF_RADIO_END)
      && nrf_it_is_enabled(BLE_RADIO_ADDR, NRF_RADIO_END)) {

    //    assert(nrf_reg_get(BLE_RADIO_ADDR, NRF_RADIO_BCC) != 16);
    ble_radio_end(pv);
  }
}

static DEV_IRQ_SRC_PROCESS(ble_timer_irq)
{
  struct device_s *dev = ep->base.dev;
  struct ble_radio *pv = dev->drv_pv;

  if (nrf_event_check(BLE_TIMER_ADDR, NRF_TIMER_COMPARE(TIMER_IFS_TIMEOUT))) {
    nrf_event_clear(BLE_TIMER_ADDR, NRF_TIMER_COMPARE(TIMER_IFS_TIMEOUT));

    if (nrf_event_check(BLE_RADIO_ADDR, NRF_RADIO_ADDRESS)) {
      nrf_event_clear(BLE_RADIO_ADDR, NRF_RADIO_ADDRESS);

      ble_radio_address_match(pv);
    } else {
      ble_radio_rx_timeout(pv);
    }
  }
}

static DEV_IRQ_SRC_PROCESS(ble_rtc_irq)
{
  struct device_s *dev = ep->base.dev;
  struct ble_radio *pv = dev->drv_pv;

  if (nrf_event_check(BLE_RTC_ADDR, NRF_RTC_OVERFLW)) {
    nrf_event_clear(BLE_RTC_ADDR, NRF_RTC_OVERFLW);

    pv->base += 1<<24;
  }

  /* if (nrf_event_check(BLE_RTC_ADDR, NRF_RTC_COMPARE(RTC_START)) */
  /*     && nrf_it_is_enabled(BLE_RTC_ADDR, NRF_RTC_COMPARE(RTC_START))) { */
  /*   nrf_event_clear(BLE_RTC_ADDR, NRF_RTC_COMPARE(RTC_START)); */
  /*   nrf_it_disable(BLE_RTC_ADDR, NRF_RTC_COMPARE(RTC_START)); */

  /*   ble_radio_first_schedule(pv); */
  /* } */

  if (nrf_event_check(BLE_RTC_ADDR, NRF_RTC_COMPARE(RTC_REQUEST_BOUNDARY))) {
    nrf_event_clear(BLE_RTC_ADDR, NRF_RTC_COMPARE(RTC_REQUEST_BOUNDARY));
    nrf_it_disable(BLE_RTC_ADDR, NRF_RTC_COMPARE(RTC_REQUEST_BOUNDARY));

    if (pv->current) {
      if (pv->end)
        ble_radio_request_timeout(pv);
    } else if (pv->first)
      ble_radio_first_start(pv);
  }
}

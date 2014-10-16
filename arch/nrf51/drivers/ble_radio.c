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

#include <arch/nrf51/ppi.h>
#include <arch/nrf51/radio.h>
#include <arch/nrf51/timer.h>
#include <arch/nrf51/rtc.h>
#include <arch/nrf51/ficr.h>
#include <arch/nrf51/uart.h>
#include <arch/nrf51/clock.h>
#include <arch/nrf51/power.h>

#include <stdlib.h>

#include <ble/protocol/radio.h>
#include <ble/protocol/address.h>

#include "ble_radio_handler.h"

#if defined(CONFIG_DEVICE_CLOCK)
# define HFCLK_RAMPUP_US      1000
#endif

#define RADIO_RAMPUP_US      140
#define RADIO_IRQ_LATENCY_US 10
#define CONN_PACKET_TIME     BLE_PACKET_TIME(6 + 6 + 22)
#define BLE_MIN_RQ_TICKS     30

/* Must use Timer0 and RTC0 as there are some hardwired PPIs */
#define BLE_RADIO_ADDR NRF_PERIPHERAL_ADDR(NRF51_RADIO)
#define BLE_TIMER_ADDR NRF_PERIPHERAL_ADDR(NRF51_TIMER0)
#define BLE_RTC_ADDR NRF_PERIPHERAL_ADDR(NRF51_RTC0)

#define RTC_ENABLE             0
#define PPI_RTC_ENABLE_TXEN    NRF51_PPI_RTC0_COMPARE_0_RADIO_TXEN
#define PPI_RTC_ENABLE_RXEN    NRF51_PPI_RTC0_COMPARE_0_RADIO_RXEN

#define RTC_TIMEOUT            1
#define PPI_RTC_TIMEOUT        0

#define RTC_START              2
#define PPI_RTC_MATCH_START    1

#define TIMER_IFS_TIMEOUT      0
#define PPI_END_TIMER_START    2
#define PPI_ADDRESS_TIMER_STOP 3


static inline uint32_t us_to_ticks_ceil(uint32_t us)
{
  // more error
  return (us + 31) / 32;
  // 1.6% error
  return (us + 30) / 31;
}

struct ble_radio;

static DEV_IRQ_EP_PROCESS(ble_radio_irq);
static DEV_IRQ_EP_PROCESS(ble_timer_irq);
static DEV_IRQ_EP_PROCESS(ble_rtc_irq);

static void ble_radio_next_action(struct ble_radio *pv);
static void ble_radio_clock_release(struct ble_radio *pv);
static bool_t ble_radio_clock_request(struct ble_radio *pv);
static void ble_radio_deadlines_check(struct ble_radio *pv);
static void ble_radio_request_done(struct ble_radio *pv);
static void ble_transfer_data_setup(struct ble_radio *pv);
static
void ble_radio_request_callback(
  struct ble_radio *pv,
  struct dev_ble_radio_rq_s *rq);
static void ble_radio_end(struct ble_radio *pv);
static void ble_radio_pipelined_continue(struct ble_radio *pv);

static void ble_rtc_start(struct ble_radio *pv)
{
#if defined(CONFIG_DEVICE_CLOCK)
  dev_clock_sink_hold(&pv->clock_sink[NRF51_BLE_RADIO_CLK_SLEEP], 1);
#endif

  nrf_it_disable_mask(BLE_RTC_ADDR, -1);
  nrf_it_enable(BLE_RTC_ADDR, NRF51_RTC_OVERFLW);
  nrf_evt_disable_mask(BLE_RTC_ADDR, -1);
  nrf_task_trigger(BLE_RTC_ADDR, NRF51_RTC_START);
}

static void ble_rtc_stop(struct ble_radio *pv)
{
#if defined(CONFIG_DEVICE_CLOCK)
  dev_clock_sink_release(&pv->clock_sink[NRF51_BLE_RADIO_CLK_SLEEP]);
#endif

  nrf_task_trigger(BLE_RTC_ADDR, NRF51_RTC_STOP);
  nrf_it_disable_mask(BLE_RTC_ADDR, -1);
  nrf_evt_disable_mask(BLE_RTC_ADDR, -1);
}

static dev_timer_value_t ble_rtc_value_get(struct ble_radio *pv)
{
  uint32_t counter;

  CPU_INTERRUPT_SAVESTATE_DISABLE;
  counter = nrf_reg_get(BLE_RTC_ADDR, NRF51_RTC_COUNTER);
  if (!(counter & 0x800000)
      && nrf_event_check(BLE_RTC_ADDR, NRF51_RTC_OVERFLW))
    counter += 0x1000000;
  CPU_INTERRUPT_RESTORESTATE;

  pv->last_now = pv->base + counter;
  return pv->last_now;
}

static void ble_radio_init(void)
{
  nrf_reg_set(BLE_RADIO_ADDR, NRF51_RADIO_POWER, 0);

  for (uint16_t i = 0; i < 32; ++i)
    asm volatile("");

  nrf_reg_set(BLE_RADIO_ADDR, NRF51_RADIO_POWER, 1);

  for (uint16_t i = 0; i < 32; ++i)
    asm volatile("");

  nrf_reg_set(NRF_PERIPHERAL_ADDR(NRF51_POWER),
              NRF51_POWER_DCDCEN,
#if defined(CONFIG_DRIVER_NRF51_BLE_RADIO_DCDC)
              NRF51_POWER_DCDCEN_ENABLE
#else
              0
#endif
              );

  nrf_it_disable_mask(BLE_RADIO_ADDR, -1);

  for (uint8_t i = 0; i < 5; ++i)
    nrf_reg_set(BLE_RADIO_ADDR, NRF51_RADIO_OVERRIDE(i),
                cpu_mem_read_32(NRF51_FICR_BLE_1MBIT(i)));

  nrf_reg_set(BLE_RADIO_ADDR, NRF51_RADIO_MODE, NRF51_RADIO_MODE_BLE_1MBIT);

  nrf_reg_set(BLE_RADIO_ADDR, NRF51_RADIO_PCNF0, 0
              | (8 << NRF51_RADIO_PCNF0_LFLEN_OFFSET)
              | (1 << NRF51_RADIO_PCNF0_S0LEN_OFFSET));

  nrf_reg_set(BLE_RADIO_ADDR, NRF51_RADIO_RXADDRESSES, 1 << 0);
  nrf_reg_set(BLE_RADIO_ADDR, NRF51_RADIO_CRCPOLY, 0x00065B);
  nrf_reg_set(BLE_RADIO_ADDR, NRF51_RADIO_CRCCNF, 0
              | NRF51_RADIO_CRCCNF_SKIPADDR
              | (3 << NRF51_RADIO_CRCCNF_LEN_OFFSET));
  nrf_reg_set(BLE_RADIO_ADDR, NRF51_RADIO_TIFS, BLE_T_IFS - 1);

  nrf_task_trigger(BLE_RADIO_ADDR, NRF51_RADIO_STOP);
  nrf_task_trigger(BLE_RADIO_ADDR, NRF51_RADIO_DISABLE);
}

static void rtc_init(void)
{
    nrf_task_trigger(BLE_RTC_ADDR, NRF51_RTC_STOP);
    nrf_reg_set(BLE_RTC_ADDR, NRF51_RTC_PRESCALER, 0);
    nrf_task_trigger(BLE_RTC_ADDR, NRF51_RTC_CLEAR);
}

static void timer_init(void)
{
    nrf_task_trigger(BLE_TIMER_ADDR, NRF51_TIMER_STOP);
    nrf_reg_set(BLE_TIMER_ADDR, NRF51_TIMER_PRESCALER, 4);
    nrf_reg_set(BLE_TIMER_ADDR, NRF51_TIMER_MODE, NRF51_TIMER_MODE_TIMER);
    nrf_reg_set(BLE_TIMER_ADDR, NRF51_TIMER_BITMODE, NRF51_TIMER_BITMODE_16);
}

static void ppi_init(void)
{
  nrf51_ppi_setup(NRF51_PPI_ADDR, PPI_RTC_TIMEOUT,
                  BLE_RTC_ADDR, NRF51_RTC_COMPARE(RTC_TIMEOUT),
                  BLE_RADIO_ADDR, NRF51_RADIO_DISABLE);

  nrf51_ppi_setup(NRF51_PPI_ADDR, PPI_RTC_MATCH_START,
                  BLE_RTC_ADDR, NRF51_RTC_COMPARE(RTC_START),
                  BLE_RADIO_ADDR, NRF51_RADIO_START);

  nrf51_ppi_setup(NRF51_PPI_ADDR, PPI_END_TIMER_START,
                  BLE_RADIO_ADDR, NRF51_RADIO_END,
                  BLE_TIMER_ADDR, NRF51_TIMER_START);

  nrf51_ppi_setup(NRF51_PPI_ADDR, PPI_ADDRESS_TIMER_STOP,
                  BLE_RADIO_ADDR, NRF51_RADIO_ADDRESS,
                  BLE_TIMER_ADDR, NRF51_TIMER_STOP);
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

  nrf51_ppi_disable_mask(NRF51_PPI_ADDR, 0
                         | (1 << PPI_RTC_MATCH_START)
                         | (1 << PPI_RTC_ENABLE_RXEN)
                         | (1 << PPI_RTC_ENABLE_TXEN)
                         | (1 << PPI_RTC_TIMEOUT)
                         );

  nrf_evt_disable_mask(BLE_RTC_ADDR, 0
                       | (1 << NRF51_RTC_COMPARE(RTC_ENABLE))
                       | (1 << NRF51_RTC_COMPARE(RTC_START))
                       );

  nrf_task_trigger(BLE_RADIO_ADDR, NRF51_RADIO_STOP);
  nrf_task_trigger(BLE_RADIO_ADDR, NRF51_RADIO_DISABLE);
}

static
void ble_radio_config_set(
    struct buffer_pool_s *pool,
    const struct ble_radio_params *params)
{
  // Packet available space, except decryption prefix, header and CRC
  size_t packet_max_size = buffer_pool_unit_size(pool) - 1 - 2 - 3;

  nrf_reg_set(BLE_RADIO_ADDR, NRF51_RADIO_FREQUENCY,
              ble_channel_freq_mhz(params->channel) - 2400);
  nrf_reg_set(BLE_RADIO_ADDR, NRF51_RADIO_DATAWHITEIV, params->channel | 0x40);
  nrf_reg_set(BLE_RADIO_ADDR, NRF51_RADIO_PCNF1, 0
              | ((packet_max_size - 2) << NRF51_RADIO_PCNF1_MAXLEN_OFFSET)
              | (3 << NRF51_RADIO_PCNF1_BALEN_OFFSET)
              | NRF51_RADIO_PCNF1_ENDIAN_LITTLE
              | NRF51_RADIO_PCNF1_WHITEEN_ENABLED);

  nrf_reg_set(BLE_RADIO_ADDR, NRF51_RADIO_BASE0, params->access << 8);
  nrf_reg_set(BLE_RADIO_ADDR, NRF51_RADIO_PREFIX0, params->access >> 24);
  nrf_reg_set(BLE_RADIO_ADDR, NRF51_RADIO_CRCINIT, params->crc_init);
}

static
void ble_radio_pipeline_setup(struct ble_radio *pv)
{
  ble_radio_config_set(pv->current->packet_pool, &pv->radio_next);

  switch (pv->radio_next.mode) {
  case MODE_TX:
    nrf_short_set(BLE_RADIO_ADDR, 0
                  | (1 << NRF51_RADIO_END_DISABLE)
                  | (1 << NRF51_RADIO_DISABLED_TXEN)
                  | (1 << NRF51_RADIO_READY_START));
    break;

  default:
  case MODE_RX:
    nrf_short_set(BLE_RADIO_ADDR, 0
                  | (1 << NRF51_RADIO_END_DISABLE)
                  | (1 << NRF51_RADIO_DISABLED_RXEN)
                  | (1 << NRF51_RADIO_READY_START));

    if (pv->radio_next.ifs) {
      /*
        RX IFS timeout

        Use some timer running at 1MHz.

        Start timer on radio END, stop on radio ADDRESS.  Set timeout in
        CC after T_IFS + T_preamble + T_access_address + Epsilon.

        If timeout trigggers before timer is stopped, this is because of
        an IFS timeout.
      */

      nrf_task_trigger(BLE_TIMER_ADDR, NRF51_TIMER_STOP);
      nrf_task_trigger(BLE_TIMER_ADDR, NRF51_TIMER_CLEAR);

      nrf51_ppi_enable_mask(NRF51_PPI_ADDR, 0
                            | (1 << PPI_END_TIMER_START)
                            | (1 << PPI_ADDRESS_TIMER_STOP)
                            );

      nrf_reg_set(BLE_TIMER_ADDR, NRF51_TIMER_CC(TIMER_IFS_TIMEOUT),
                  BLE_T_IFS + 40 + 12);
      nrf_it_enable(BLE_TIMER_ADDR, NRF51_TIMER_COMPARE(TIMER_IFS_TIMEOUT));
      nrf_event_clear(BLE_TIMER_ADDR, NRF51_TIMER_COMPARE(TIMER_IFS_TIMEOUT));
    }
    break;
  }

  if (nrf_event_check(BLE_RADIO_ADDR, NRF51_RADIO_END)) {
    pv->current->status = DEVICE_BLE_RADIO_DEADLINE_MISSED;
    ble_radio_request_done(pv);
    return;
  }
}

static void ble_radio_deadlines_check(struct ble_radio *pv)
{
  struct dev_ble_radio_rq_s *first, *next = NULL;

  assert(!cpu_is_interruptible());

  first = dev_ble_radio_rq_s_cast(dev_request_pqueue_head(&pv->queue));
  if (first)
    next = dev_ble_radio_rq_s_cast(dev_request_pqueue_next(&pv->queue, &first->base));

  if (pv->first == first && pv->next == next)
    return;

  if (pv->first != first) {
    pv->first = first;

    if (first) {
      dev_timer_value_t now;

      now = ble_rtc_value_get(pv);

      pv->start = now;
      pv->end = 0;

      if (first->not_before)
        pv->start = first->not_before;

      if (first->max_duration && first->not_after)
        pv->end = __MIN(pv->start + first->max_duration,
                        __MIN(now + first->max_duration,
                              first->not_after));
      else if (first->max_duration)
        pv->end = pv->start + first->max_duration;
      else
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

static void ble_radio_next_action(struct ble_radio *pv)
{
  if (pv->current)
    return;

  ble_radio_deadlines_check(pv);

  if (!pv->first) {
    ble_radio_clock_release(pv);
    return;
  }

  dev_timer_value_t now = ble_rtc_value_get(pv);

#if defined(CONFIG_DEVICE_CLOCK)
  if (pv->start > now + us_to_ticks_ceil(RADIO_RAMPUP_US + HFCLK_RAMPUP_US) + 1) {
    nrf_event_clear(BLE_RTC_ADDR, NRF51_RTC_COMPARE(RTC_START));
    nrf_it_enable(BLE_RTC_ADDR, NRF51_RTC_COMPARE(RTC_START));
    nrf_reg_set(BLE_RTC_ADDR, NRF51_RTC_CC(RTC_START),
                pv->start - us_to_ticks_ceil(RADIO_RAMPUP_US + HFCLK_RAMPUP_US));

    ble_radio_clock_release(pv);
    return;
  }
#endif

  bool_t clock_running = ble_radio_clock_request(pv);

  if (pv->end) {
    // request timeout: use RTC.
    nrf_reg_set(BLE_RTC_ADDR, NRF51_RTC_CC(RTC_TIMEOUT), pv->end);
    nrf_event_clear(BLE_RTC_ADDR, NRF51_RTC_COMPARE(RTC_TIMEOUT));
    nrf_it_enable(BLE_RTC_ADDR, NRF51_RTC_COMPARE(RTC_TIMEOUT));
    nrf51_ppi_enable(NRF51_PPI_ADDR, PPI_RTC_TIMEOUT);
  }

  pv->current = pv->first;
  pv->current->status = DEVICE_BLE_RADIO_IN_PAST;
  pv->handler = &request_handler[pv->current->type];
  if (pv->handler->startup)
    pv->handler->startup(pv);

  pv->radio_current.channel = -1;

  if (pv->transmitting) {
    buffer_refdec(pv->transmitting);
    pv->transmitting = NULL;
  }

  assert(!pv->transmitting);
  assert(nrf_reg_get(BLE_RADIO_ADDR, NRF51_RADIO_STATE)
         == NRF51_RADIO_STATE_DISABLED);

  bool_t running = pv->handler->radio_params(pv, &pv->radio_next);
  if (!running) {
    pv->current->status = DEVICE_BLE_RADIO_HANDLER_DONE;
    ble_radio_request_done(pv);
    return;
  }

  bool_t later = pv->start > now + us_to_ticks_ceil(RADIO_RAMPUP_US);
  if (!clock_running && !later)
    return;

  pv->radio_current = pv->radio_next;
  nrf51_ppi_disable_mask(NRF51_PPI_ADDR, 0
                         | (1 << PPI_END_TIMER_START)
                         | (1 << PPI_ADDRESS_TIMER_STOP)
                         );
  nrf_reg_set(BLE_RADIO_ADDR, NRF51_RADIO_BCC, 16);
  ble_radio_config_set(pv->current->packet_pool, &pv->radio_current);

  if (later) {
    nrf_event_clear(BLE_RTC_ADDR, NRF51_RTC_COMPARE(RTC_ENABLE));
    nrf_event_clear(BLE_RTC_ADDR, NRF51_RTC_COMPARE(RTC_START));

    nrf_reg_set(BLE_RTC_ADDR, NRF51_RTC_CC(RTC_ENABLE),
                pv->start - us_to_ticks_ceil(RADIO_RAMPUP_US));
    nrf_reg_set(BLE_RTC_ADDR, NRF51_RTC_CC(RTC_START), pv->start);

    nrf_evt_enable_mask(BLE_RTC_ADDR, 0
                        | (1 << NRF51_RTC_COMPARE(RTC_ENABLE))
                        | (1 << NRF51_RTC_COMPARE(RTC_START))
                        );

    switch (pv->radio_current.mode) {
    case MODE_TX:
      nrf51_ppi_enable_mask(NRF51_PPI_ADDR, 0
                            | (1 << PPI_RTC_ENABLE_TXEN)
                            | (1 << PPI_RTC_MATCH_START));
      break;

    case MODE_RX:
      nrf51_ppi_enable_mask(NRF51_PPI_ADDR, 0
                            | (1 << PPI_RTC_ENABLE_RXEN)
                            | (1 << PPI_RTC_MATCH_START));
      break;

    case MODE_RSSI:
      break;
    }

  } else {
    switch (pv->radio_current.mode) {
    case MODE_TX:
      nrf_short_enable(BLE_RADIO_ADDR, NRF51_RADIO_READY_START);
      nrf_task_trigger(BLE_RADIO_ADDR, NRF51_RADIO_TXEN);
      break;

    case MODE_RX:
      nrf_short_enable(BLE_RADIO_ADDR, NRF51_RADIO_READY_START);
      nrf_task_trigger(BLE_RADIO_ADDR, NRF51_RADIO_RXEN);
      break;

    case MODE_RSSI:
      break;
    }
  }

  ble_transfer_data_setup(pv);
}

#if defined(CONFIG_DEVICE_CLOCK)

static bool_t ble_radio_clock_request(struct ble_radio *pv)
{
  if (!pv->accurate_clock_requested) {
    dev_clock_sink_hold(&pv->clock_sink[NRF51_BLE_RADIO_CLK_RADIO], 0);
    pv->accurate_clock_requested = 1;
  }

  return pv->accurate_clock_running;
}

/* TODO: Do this later on */
static void ble_radio_clock_release(struct ble_radio *pv)
{
  if (pv->accurate_clock_requested) {
    dev_clock_sink_release(&pv->clock_sink[NRF51_BLE_RADIO_CLK_RADIO]);
    pv->accurate_clock_requested = 0;
  }
}

static DEV_CLOCK_SINK_CHANGED(nrf51_ble_radio_clock_changed)
{
  struct ble_radio *pv = ep->dev->drv_pv;
  uint8_t clk = ep - pv->clock_sink;

  if (clk == NRF51_BLE_RADIO_CLK_SLEEP
      && ep->src->flags & DEV_CLOCK_SRC_EP_RUNNING)
    pv->sleep_acc = *acc;

  if (clk == NRF51_BLE_RADIO_CLK_RADIO) {
    bool_t was_running = pv->accurate_clock_running;

    pv->accurate_clock_running = !!(ep->src->flags & DEV_CLOCK_SRC_EP_RUNNING);

    if (!was_running && pv->accurate_clock_running)
      ble_radio_next_action(pv);
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

static DEVICE_BLE_RADIO_REQUEST(nrf51_ble_radio_request);
static DEVICE_BLE_RADIO_CANCEL(nrf51_ble_radio_cancel);
static DEVICE_BLE_RADIO_GET_INFO(nrf51_ble_radio_get_info);

static DEV_TIMER_CONFIG(nrf51_ble_rtc_config)
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

static DEV_TIMER_GET_VALUE(nrf51_ble_rtc_get_value)
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


static DEV_INIT(nrf51_ble_radio_init);
static DEV_CLEANUP(nrf51_ble_radio_cleanup);
static DEV_USE(nrf51_ble_radio_use);

#define nrf51_ble_rtc_request (dev_timer_request_t*)dev_driver_notsup_fcn
#define nrf51_ble_rtc_cancel (dev_timer_request_t*)dev_driver_notsup_fcn

DRIVER_DECLARE(nrf51_ble_radio_drv, "nRF51 BLE-Radio", nrf51_ble_radio,
               DRIVER_BLE_RADIO_METHODS(nrf51_ble_radio),
               DRIVER_TIMER_METHODS(nrf51_ble_rtc));

DRIVER_REGISTER(nrf51_ble_radio_drv);

static DEV_USE(nrf51_ble_radio_use)
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

static DEV_INIT(nrf51_ble_radio_init)
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
         NRF_PERIPHERAL_ADDR(NRF51_RADIO) == addr);

  assert(device_res_get_uint(dev, DEV_RES_MEM, 1, &addr, NULL) == 0 &&
         NRF_PERIPHERAL_ADDR(NRF51_TIMER0) == addr);

  assert(device_res_get_uint(dev, DEV_RES_MEM, 2, &addr, NULL) == 0 &&
         NRF_PERIPHERAL_ADDR(NRF51_RTC0) == addr);

  device_irq_source_init(dev, &pv->irq_source[NRF51_BLE_RADIO_IRQ_RADIO], 1,
                         &ble_radio_irq, DEV_IRQ_SENSE_HIGH_LEVEL);

  device_irq_source_init(dev, &pv->irq_source[NRF51_BLE_RADIO_IRQ_TIMER], 1,
                         &ble_timer_irq, DEV_IRQ_SENSE_HIGH_LEVEL);

  device_irq_source_init(dev, &pv->irq_source[NRF51_BLE_RADIO_IRQ_RTC], 1,
                         &ble_rtc_irq, DEV_IRQ_SENSE_HIGH_LEVEL);

  err = device_irq_source_link(dev, pv->irq_source, NRF51_BLE_RADIO_IRQ_COUNT, -1);
  if (err)
    goto free_pv;

#if defined(CONFIG_DEVICE_CLOCK)
  pv->accurate_clock_requested = 0;
  pv->accurate_clock_running = 0;

  dev_clock_sink_init(dev, &pv->clock_sink[NRF51_BLE_RADIO_CLK_RADIO], &nrf51_ble_radio_clock_changed);
  dev_clock_sink_init(dev, &pv->clock_sink[NRF51_BLE_RADIO_CLK_SLEEP], &nrf51_ble_radio_clock_changed);

  struct dev_clock_link_info_s ckinfo[2];
  if (dev_clock_sink_link(dev, pv->clock_sink, ckinfo, 0, 1))
    goto free_pv;
#endif

  dev_request_pqueue_init(&pv->queue);

  ble_radio_init();
  rtc_init();
  timer_init();
  ppi_init();

  dev->drv = &nrf51_ble_radio_drv;
  dev->status = DEVICE_DRIVER_INIT_DONE;
  dev->drv_pv = pv;

  return 0;

 free_pv:
  free(pv);

  return err;
}

static DEV_CLEANUP(nrf51_ble_radio_cleanup)
{
  struct ble_radio *pv = dev->drv_pv;

  nrf_it_disable_mask(BLE_RADIO_ADDR, -1);
  nrf_it_disable_mask(BLE_TIMER_ADDR, -1);
  nrf_it_disable_mask(BLE_RTC_ADDR, -1);

  ble_radio_disable(pv);

  nrf_task_trigger(BLE_TIMER_ADDR, NRF51_TIMER_STOP);
  nrf_task_trigger(BLE_RTC_ADDR, NRF51_RTC_STOP);

  dev_request_pqueue_destroy(&pv->queue);

  device_irq_source_unlink(dev, pv->irq_source, NRF51_BLE_RADIO_IRQ_COUNT);

  mem_free(pv);
}

static DEVICE_BLE_RADIO_REQUEST(nrf51_ble_radio_request)
{
  struct device_s *dev = accessor->dev;
  struct ble_radio *pv = dev->drv_pv;
  dev_timer_value_t now = ble_rtc_value_get(pv);

  if (rq->not_before && rq->not_before < now)
    return -ETIMEDOUT;

  LOCK_SPIN_IRQ(&dev->lock);

  rq->base.drvdata = dev;
  dev_ble_radio_pqueue_insert(&pv->queue, &rq->base);

  if (!pv->callbacking)
    ble_radio_next_action(pv);

  LOCK_RELEASE_IRQ(&dev->lock);

  return 0;
}

static DEVICE_BLE_RADIO_CANCEL(nrf51_ble_radio_cancel)
{
  struct device_s *dev = accessor->dev;
  struct ble_radio *pv = dev->drv_pv;
  error_t err = 0;

  LOCK_SPIN_IRQ(&dev->lock);

  if (rq != pv->current) {
    dev_ble_radio_pqueue_remove(&pv->queue, &rq->base);
    ble_radio_next_action(pv);
  }

 out:
  LOCK_RELEASE_IRQ(&dev->lock);
  return err;
}

static DEVICE_BLE_RADIO_GET_INFO(nrf51_ble_radio_get_info)
{
  info->address.type = cpu_mem_read_32(NRF51_FICR_DEVICEADDRTYPE)
    ? BLE_ADDR_RANDOM : BLE_ADDR_PUBLIC;

  memcpy(info->address.addr, (void*)NRF51_FICR_DEVICEADDR(0), 6);
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

  pv->callbacking = 1;
  lock_release(&dev->lock);
  kroutine_exec(&rq->base.kr, 0);
  lock_spin(&dev->lock);
  pv->callbacking = 0;
}

static void ble_radio_request_done(struct ble_radio *pv)
{
  struct dev_ble_radio_rq_s *rq = pv->current;

  ble_radio_disable(pv);

  nrf_it_disable(BLE_RTC_ADDR, NRF51_RTC_COMPARE(RTC_TIMEOUT));
  nrf_evt_disable(BLE_RTC_ADDR, NRF51_RTC_COMPARE(RTC_TIMEOUT));

  assert(pv->current);
  if (pv->handler->cleanup)
    pv->handler->cleanup(pv);

  if (pv->transmitting) {
    buffer_refdec(pv->transmitting);
    pv->transmitting = NULL;
  }

  pv->current = NULL;
  pv->handler = NULL;

  debug(pv, "\n");

  ble_radio_request_callback(pv, rq);
  ble_radio_next_action(pv);
}

static void ppi_cleanup(struct ble_radio *pv)
{
  nrf51_ppi_disable_mask(NRF51_PPI_ADDR, 0
                         | (1 << PPI_RTC_MATCH_START)
                         | (1 << PPI_RTC_ENABLE_RXEN)
                         | (1 << PPI_RTC_ENABLE_TXEN)
                         | (1 << PPI_RTC_TIMEOUT)
                         );

  nrf_evt_disable_mask(BLE_RTC_ADDR, 0
                       | (1 << NRF51_RTC_COMPARE(RTC_ENABLE))
                       | (1 << NRF51_RTC_COMPARE(RTC_START))
                       );

  nrf_it_disable(BLE_TIMER_ADDR, NRF51_TIMER_COMPARE(TIMER_IFS_TIMEOUT));
}

static
void ble_radio_address_match(struct ble_radio *pv)
{
  if (!pv->current) {
    ble_radio_disable(pv);
    ble_radio_next_action(pv);
    return;
  }

  ppi_cleanup(pv);
  nrf_task_trigger(BLE_TIMER_ADDR, NRF51_TIMER_STOP);
  nrf_short_set(BLE_RADIO_ADDR, 1 << NRF51_RADIO_END_DISABLE);

  pv->address_timestamp = ble_rtc_value_get(pv) - 2;

  if (pv->handler->ifs_event)
    pv->handler->ifs_event(pv, 0);

  bool_t running = pv->handler->radio_params(pv, &pv->radio_next);

  if (running)
    ble_radio_pipeline_setup(pv);
  else {
    nrf_short_set(BLE_RADIO_ADDR, 1 << NRF51_RADIO_END_DISABLE);

    nrf51_ppi_disable_mask(NRF51_PPI_ADDR, 0
                           | (1 << PPI_END_TIMER_START)
                           | (1 << PPI_ADDRESS_TIMER_STOP)
                           );

    nrf_it_disable(BLE_TIMER_ADDR, NRF51_TIMER_COMPARE(TIMER_IFS_TIMEOUT));

    pv->radio_current.channel = -1;
  }

  while (!nrf_event_check(BLE_RADIO_ADDR, NRF51_RADIO_BCMATCH)
         && nrf_reg_get(BLE_RADIO_ADDR, NRF51_RADIO_STATE) == NRF51_RADIO_STATE_RX)
      assert(nrf_reg_get(BLE_RADIO_ADDR, NRF51_RADIO_BCC) == 16);
  nrf_event_clear(BLE_RADIO_ADDR, NRF51_RADIO_BCMATCH);

  uint8_t len = pv->transmitting->data[pv->transmitting->begin + 1];
  uint16_t end_irq_bits = ((uint16_t)len + 5) * 8 - RADIO_IRQ_LATENCY_US;

  if (end_irq_bits > 16 + RADIO_IRQ_LATENCY_US * 2 + 4
      && !nrf_event_check(BLE_RADIO_ADDR, NRF51_RADIO_END)) {
    nrf_reg_set(BLE_RADIO_ADDR, NRF51_RADIO_BCC, end_irq_bits);

    return;
  }

  nrf_it_disable(BLE_RADIO_ADDR, NRF51_RADIO_BCMATCH);

  while (!nrf_event_check(BLE_RADIO_ADDR, NRF51_RADIO_END))
    ;

  ble_radio_end(pv);
}

static void ble_transfer_data_setup(struct ble_radio *pv)
{
  switch (pv->radio_current.mode) {
  case MODE_TX:
    pv->transmitting = pv->handler->payload_get(pv);
    break;
  case MODE_RX:
    pv->transmitting = nrf51_ble_radio_packet_alloc(pv);
    break;
  case MODE_RSSI:
    break;
  }

  nrf_short_disable_mask(BLE_RADIO_ADDR, 0
                         | (1 << NRF51_RADIO_DISABLED_RXEN)
                         | (1 << NRF51_RADIO_DISABLED_TXEN)
                         );

  nrf_short_enable_mask(BLE_RADIO_ADDR, 0
                        | (1 << NRF51_RADIO_END_DISABLE)
                        | (1 << NRF51_RADIO_ADDRESS_BCSTART)
                        );

  nrf_it_enable_mask(BLE_RADIO_ADDR, 0
                     | (1 << NRF51_RADIO_ADDRESS)
                     | (1 << NRF51_RADIO_END)
                     );

  nrf_reg_set(BLE_RADIO_ADDR, NRF51_RADIO_PACKETPTR,
              (uintptr_t)pv->transmitting->data + pv->transmitting->begin);
}

static void ble_transfer_transfer_restart(struct ble_radio *pv)
{
  ble_radio_disable(pv);

  pv->radio_current = pv->radio_next;
  nrf51_ppi_disable_mask(NRF51_PPI_ADDR, 0
                         | (1 << PPI_END_TIMER_START)
                         | (1 << PPI_ADDRESS_TIMER_STOP)
                         );
  nrf_reg_set(BLE_RADIO_ADDR, NRF51_RADIO_BCC, 16);

  while (nrf_reg_get(BLE_RADIO_ADDR, NRF51_RADIO_STATE)
         != NRF51_RADIO_STATE_DISABLED)
    ;
  ble_radio_config_set(pv->current->packet_pool, &pv->radio_current);

  switch (pv->radio_current.mode) {
  case MODE_TX:
    nrf_task_trigger(BLE_RADIO_ADDR, NRF51_RADIO_TXEN);
    break;
  case MODE_RX:
    nrf_task_trigger(BLE_RADIO_ADDR, NRF51_RADIO_RXEN);
    break;
  case MODE_RSSI:
    nrf_task_trigger(BLE_RADIO_ADDR, NRF51_RADIO_RSSISTART);
    break;
  }
}

static void ble_radio_pipelined_continue(struct ble_radio *pv)
{
  assert(pv->current);
  assert(pv->transmitting);

  buffer_refdec(pv->transmitting);
  pv->transmitting = NULL;

  bool_t running = pv->handler->radio_params(pv, &pv->radio_next);
  if (!running) {
    pv->current->status = DEVICE_BLE_RADIO_HANDLER_DONE;
    ble_radio_request_done(pv);
    return;
  }

  if (!ble_radio_params_equal(&pv->radio_next, &pv->radio_current))
    ble_transfer_transfer_restart(pv);
  nrf_short_enable(BLE_RADIO_ADDR, NRF51_RADIO_READY_START);

  ble_transfer_data_setup(pv);
}


static
void ble_radio_end(struct ble_radio *pv)
{
  nrf_reg_set(BLE_RADIO_ADDR, NRF51_RADIO_BCC, 16);
  nrf_event_clear(BLE_RADIO_ADDR, NRF51_RADIO_END);

  if (!pv->current) {
    nrf_it_disable(BLE_RADIO_ADDR, NRF51_RADIO_END);
    ble_radio_next_action(pv);
    return;
  }

  bool_t rx = pv->radio_current.mode == MODE_RX;

  if (nrf_short_is_enabled(BLE_RADIO_ADDR, NRF51_RADIO_END_DISABLE)
      && nrf_short_is_enabled(BLE_RADIO_ADDR, NRF51_RADIO_READY_START)
      && (nrf_short_is_enabled(BLE_RADIO_ADDR, NRF51_RADIO_DISABLED_RXEN)
          || nrf_short_is_enabled(BLE_RADIO_ADDR, NRF51_RADIO_DISABLED_TXEN))) {
    pv->radio_current = pv->radio_next;
  }

  if (rx && pv->handler->payload_received) {
    pv->transmitting->end = pv->transmitting->begin
      + __MIN(pv->transmitting->data[pv->transmitting->begin + 1] + 2,
              buffer_size(pv->transmitting) - pv->transmitting->begin);

    pv->handler->payload_received(
        pv,
        nrf_reg_get(BLE_RADIO_ADDR, NRF51_RADIO_CRCSTATUS),
        pv->transmitting);
  }

  ble_radio_pipelined_continue(pv);
}

static
void ble_radio_request_timeout(struct ble_radio *pv)
{
  ppi_cleanup(pv);
  if (pv->current) {
    pv->current->status = DEVICE_BLE_RADIO_WINDOW_DONE;
    ble_radio_request_done(pv);
  }
}

static
void ble_radio_rx_timeout(struct ble_radio *pv)
{
  nrf_task_trigger(BLE_TIMER_ADDR, NRF51_TIMER_STOP);
  ble_radio_disable(pv);
  ppi_cleanup(pv);

  nrf_event_clear(BLE_RADIO_ADDR, NRF51_RADIO_BCMATCH);
  nrf_it_disable(BLE_RADIO_ADDR, NRF51_RADIO_BCMATCH);
  nrf_reg_set(BLE_RADIO_ADDR, NRF51_RADIO_BCC, 16);

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

  ble_transfer_transfer_restart(pv);
  nrf_short_enable(BLE_RADIO_ADDR, NRF51_RADIO_READY_START);

  ble_transfer_data_setup(pv);
}

struct buffer_s *nrf51_ble_radio_packet_alloc(const struct ble_radio *pv)
{
  struct buffer_s *packet;

  assert(pv->current);

  packet = buffer_pool_alloc(pv->current->packet_pool);

  packet->begin = 1;
  packet->end = buffer_size(packet);
  packet->data[packet->begin + 1] = 0;

  return packet;
}

static DEV_IRQ_EP_PROCESS(ble_radio_irq)
{
  struct ble_radio *pv = KROUTINE_CONTAINER(ep, *pv, irq_source[NRF51_BLE_RADIO_IRQ_RADIO]);

  if (nrf_event_check(BLE_RADIO_ADDR, NRF51_RADIO_ADDRESS)) {
    nrf_event_clear(BLE_RADIO_ADDR, NRF51_RADIO_ADDRESS);

    ble_radio_address_match(pv);
  }

  if (nrf_event_check(BLE_RADIO_ADDR, NRF51_RADIO_BCMATCH)
      && nrf_it_is_enabled(BLE_RADIO_ADDR, NRF51_RADIO_BCMATCH)) {
    nrf_event_clear(BLE_RADIO_ADDR, NRF51_RADIO_BCMATCH);
    nrf_it_disable(BLE_RADIO_ADDR, NRF51_RADIO_BCMATCH);

    assert(nrf_reg_get(BLE_RADIO_ADDR, NRF51_RADIO_BCC) != 16);
    ble_radio_end(pv);
  }

  if (nrf_event_check(BLE_RADIO_ADDR, NRF51_RADIO_END)) {
    //    assert(nrf_reg_get(BLE_RADIO_ADDR, NRF51_RADIO_BCC) != 16);
    ble_radio_end(pv);
  }
}

static DEV_IRQ_EP_PROCESS(ble_timer_irq)
{
  struct ble_radio *pv = KROUTINE_CONTAINER(ep, *pv, irq_source[NRF51_BLE_RADIO_IRQ_TIMER]);

  if (nrf_event_check(BLE_TIMER_ADDR, NRF51_TIMER_COMPARE(TIMER_IFS_TIMEOUT))) {
    nrf_event_clear(BLE_TIMER_ADDR, NRF51_TIMER_COMPARE(TIMER_IFS_TIMEOUT));

    if (nrf_event_check(BLE_RADIO_ADDR, NRF51_RADIO_ADDRESS)) {
      nrf_event_clear(BLE_RADIO_ADDR, NRF51_RADIO_ADDRESS);

      ble_radio_address_match(pv);
    } else {
      ble_radio_rx_timeout(pv);
    }
  }
}

static DEV_IRQ_EP_PROCESS(ble_rtc_irq)
{
  struct ble_radio *pv = KROUTINE_CONTAINER(ep, *pv, irq_source[NRF51_BLE_RADIO_IRQ_RTC]);

  if (nrf_event_check(BLE_RTC_ADDR, NRF51_RTC_OVERFLW)) {
    nrf_event_clear(BLE_RTC_ADDR, NRF51_RTC_OVERFLW);

    pv->base += 1<<24;
  }

  if (nrf_event_check(BLE_RTC_ADDR, NRF51_RTC_COMPARE(RTC_START))
      && nrf_it_is_enabled(BLE_RTC_ADDR, NRF51_RTC_COMPARE(RTC_START))) {
    nrf_event_clear(BLE_RTC_ADDR, NRF51_RTC_COMPARE(RTC_START));
    nrf_it_disable(BLE_RTC_ADDR, NRF51_RTC_COMPARE(RTC_START));

    ble_radio_next_action(pv);
  }

  if (nrf_event_check(BLE_RTC_ADDR, NRF51_RTC_COMPARE(RTC_TIMEOUT))) {
    nrf_event_clear(BLE_RTC_ADDR, NRF51_RTC_COMPARE(RTC_TIMEOUT));

    ble_radio_request_timeout(pv);
  }
}

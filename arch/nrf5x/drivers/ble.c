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

#include <hexo/types.h>
#include <hexo/endian.h>
#include <string.h>

#include <mutek/mem_alloc.h>

#include <device/device.h>
#include <device/resources.h>
#include <device/driver.h>
#include <device/class/net.h>

#include <mutek/printk.h>

#include <ble/protocol/radio.h>
#include <ble/net/layer_id.h>

#include <arch/nrf5x/ficr.h>
#include <arch/nrf5x/timer.h>
#include <arch/nrf5x/rtc.h>
#include <arch/nrf5x/radio.h>

#include "ble.h"
#include "ble_debug.h"
#include "ble_trx_gpio.h"

#define dprintk(...) do{}while(0)
//#define dprintk printk

static KROUTINE_EXEC(nrf5x_ble_reschedule_kr);
static KROUTINE_EXEC(nrf5x_ble_closer_kr);

#ifdef NRF5X_BLE_BACKLOG
void nrf5x_ble_backlog(struct nrf5x_ble_context_s *ctx,
                         const char *msg,
                         uint32_t arg)
{
  size_t id = ctx->backlog_cur % NRF5X_BLE_BACKLOG;

  ctx->backlog[id].ts = nrf_reg_get(BLE_RTC_ADDR, NRF_RTC_COUNTER);
  ctx->backlog[id].msg = msg;
  ctx->backlog[id].arg = arg;

  ctx->backlog_cur++;
}

void nrf5x_ble_backlog_dump(struct nrf5x_ble_context_s *ctx)
{
  ssize_t min = __MAX((ssize_t)ctx->backlog_cur - NRF5X_BLE_BACKLOG, 0);
  ssize_t max = __MAX(ctx->backlog_cur - 1, 0);

  for (ssize_t i = min; i < max; ++i) {
    const struct nrf5x_ble_backlog_s *b = ctx->backlog + (i % NRF5X_BLE_BACKLOG);

    printk("%d: ", b->ts);
    printk(b->msg, b->arg);
    printk("\n");
  }
}
#endif

void nrf5x_ble_debug_init(void)
{
#if defined(CONFIG_DRIVER_NRF5X_BLE_DEBUG)
  uint32_t gpios = 0
    | I_NEXT_ACTION
    | I_CLOCK_REQ
    | I_CLOCK_RUN
    | I_LATER
    | I_ENABLE
    | I_TRANSFER
    | I_TX
    | I_PIPELINE
    | I_WAIT
    | I_IRQ
    ;

  while (gpios) {
    uint8_t i = __builtin_ctz(gpios);
    nrf_reg_set(NRF5X_GPIO_ADDR, NRF_GPIO_PIN_CNF(i), 0
                | NRF_GPIO_PIN_CNF_DIR_OUTPUT
                | NRF_GPIO_PIN_CNF_INPUT_DISCONNECT
                | NRF_GPIO_PIN_CNF_DRIVE_S0S1
                );

    gpios &= ~(1 << i);
  }
#endif
}

static DEV_NET_LAYER_CREATE(nrf5x_ble_layer_create)
{
  struct device_s *dev = accessor->dev;
  struct nrf5x_ble_private_s *pv = dev->drv_pv;

  switch (type) {
#if defined(CONFIG_BLE_MASTER)
  case BLE_NET_LAYER_MASTER:
    return nrf5x_ble_master_create(scheduler, pv, params, delegate, delegate_vtable, layer);
#endif
#if defined(CONFIG_BLE_SCANNER)
  case BLE_NET_LAYER_SCANNER:
    return nrf5x_ble_scanner_create(scheduler, pv, params, delegate, delegate_vtable, layer);
#endif
#if defined(CONFIG_BLE_ADVERTISER)
  case BLE_NET_LAYER_ADV:
    return nrf5x_ble_advertiser_create(scheduler, pv, params, delegate, delegate_vtable, layer);
#endif
#if defined(CONFIG_BLE_SLAVE)
  case BLE_NET_LAYER_SLAVE:
    return nrf5x_ble_slave_create(scheduler, pv, params, delegate, delegate_vtable, layer);
#endif
#if defined(CONFIG_BLE_DTM_TX)
  case BLE_NET_LAYER_DTM_TX:
    return nrf5x_ble_dtm_tx_create(scheduler, pv, params, delegate, delegate_vtable, layer);
#endif
  default:
    return -ENOTSUP;
  }
}

void nrf5x_ble_context_init(struct nrf5x_ble_private_s *priv,
                            struct nrf5x_ble_context_s *ctx,
                            const struct nrf5x_ble_context_handler_s *handler)
{
  ctx->handler = handler;
  ctx->pv = priv;
  ctx->scheduled = 0;

  if (priv->context_count++ == 0)
    nrf5x_ble_rtc_start(priv);
}

static DEV_NET_GET_INFO(nrf5x_ble_get_info)
{
  memset(info, 0, sizeof(*info));
  info->implemented_layers = 0
#if defined(CONFIG_BLE_MASTER)
    | (1 << BLE_NET_LAYER_MASTER)
#endif
#if defined(CONFIG_BLE_SCANNER)
    | (1 << BLE_NET_LAYER_SCANNER)
#endif
#if defined(CONFIG_BLE_ADVERTISER)
    | (1 << BLE_NET_LAYER_ADV)
#endif
#if defined(CONFIG_BLE_SLAVE)
    | (1 << BLE_NET_LAYER_SLAVE)
#endif
#if defined(CONFIG_BLE_DTM_TX)
    | (1 << BLE_NET_LAYER_DTM_TX)
#endif
    ;
  info->prefix_size = 1;
  info->mtu = CONFIG_BLE_PACKET_SIZE - 1;
  info->addr.random_addr = cpu_mem_read_32(NRF_FICR_DEVICEADDRTYPE) & 1;

  memrevcpy(info->addr.mac, (void*)NRF_FICR_DEVICEADDR(0), 6);
  if (info->addr.random_addr)
    info->addr.mac[0] |= 0xc0;

  assert(info->prefix_size + info->mtu <= CONFIG_BLE_PACKET_SIZE);

  return 0;
}

static DEV_CLEANUP(nrf5x_ble_cleanup);
static DEV_INIT(nrf5x_ble_init);
#define nrf5x_ble_rtc_request (dev_timer_request_t*)dev_driver_notsup_fcn
#define nrf5x_ble_rtc_cancel (dev_timer_request_t*)dev_driver_notsup_fcn

static DEV_USE(nrf5x_ble_use)
{
  switch (op)
    {
#if defined(CONFIG_DEVICE_CLOCK)
# ifdef CONFIG_DEVICE_CLOCK_VARFREQ
    case DEV_USE_CLOCK_SINK_FREQ_CHANGED: {
      struct dev_clock_notify_s *notif = param;
      struct dev_clock_sink_ep_s *sink = notif->sink;
      struct device_s *dev = sink->dev;
      struct nrf5x_ble_private_s *pv = dev->drv_pv;
      uint_fast8_t sink_id = sink - pv->clock_sink;

      switch (sink_id) {
      case NRF5X_BLE_CLK_SLEEP:
        pv->sleep_freq = notif->freq;
        pv->sca = ble_sca_from_accuracy(&pv->sleep_freq);
        break;

      case NRF5X_BLE_CLK_RADIO:
        pv->hfclk_is_precise = notif->freq.acc_e < 17;
        gpio(I_CLOCK_RUN, 1);
        break;
      }

      return 0;
    }
# endif
#endif

    case DEV_USE_START: {
      struct device_accessor_s *acc = param;
      struct device_s *dev = acc->dev;
      struct nrf5x_ble_private_s *pv = dev->drv_pv;

      if (dev->start_count == 0)
        nrf5x_ble_rtc_start(pv);

      return 0;
    }

    case DEV_USE_STOP: {
      struct device_accessor_s *acc = param;
      struct device_s *dev = acc->dev;
      struct nrf5x_ble_private_s *pv = dev->drv_pv;

      if (dev->start_count == 0)
        nrf5x_ble_rtc_stop(pv);

      return 0;
    }

    default:
      return dev_use_generic(param, op);
    }
}

static DEV_TIMER_CONFIG(nrf5x_ble_rtc_config)
{
  struct device_s *dev = accessor->dev;
  struct nrf5x_ble_private_s *pv = dev->drv_pv;
  error_t err = 0;

  LOCK_SPIN_IRQ(&dev->lock);

  if (cfg) {
    cfg->freq = pv->sleep_freq;
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
  struct nrf5x_ble_private_s *pv = dev->drv_pv;
  error_t err = 0;

  LOCK_SPIN_IRQ(&dev->lock);

  if (value)
    *value = nrf5x_ble_rtc_value_get(pv);

  if (rev)
    err = -EAGAIN;

  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
}

DRIVER_DECLARE(nrf5x_ble_drv, 0, "BLE Network", nrf5x_ble,
               DRIVER_NET_METHODS(nrf5x_ble),
               DRIVER_TIMER_METHODS(nrf5x_ble_rtc));

DRIVER_REGISTER(nrf5x_ble_drv);

static DEV_CLEANUP(nrf5x_ble_cleanup)
{
  struct nrf5x_ble_private_s *pv = dev->drv_pv;

  if (!nrf5x_ble_context_list_isempty(&pv->context_list)
      || !nrf5x_ble_context_list_isempty(&pv->closed_list))
    return -EBUSY;

  nrf_it_disable_mask(BLE_RADIO_ADDR, -1);
  nrf_it_disable_mask(BLE_TIMER_ADDR, -1);
  nrf_it_disable_mask(BLE_RTC_ADDR, -1);

  nrf5x_ble_radio_disable(pv);
  nrf_task_trigger(BLE_TIMER_ADDR, NRF_TIMER_STOP);
  nrf_task_trigger(BLE_RTC_ADDR, NRF_RTC_STOP);

  dev_drv_clock_cleanup(dev, &pv->clock_sink[NRF5X_BLE_CLK_SLEEP]);
  dev_drv_clock_cleanup(dev, &pv->clock_sink[NRF5X_BLE_CLK_RADIO]);

  device_irq_source_unlink(dev, pv->irq_source, NRF5X_BLE_IRQ_COUNT);

  mem_free(pv);

  return 0;
}

static DEV_INIT(nrf5x_ble_init)
{
  struct nrf5x_ble_private_s *pv;
  error_t err;

  pv = mem_alloc(sizeof(*pv), mem_scope_sys);
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

  device_irq_source_init(dev, &pv->irq_source[NRF5X_BLE_IRQ_RADIO], 1, &nrf5x_ble_radio_irq);
  device_irq_source_init(dev, &pv->irq_source[NRF5X_BLE_IRQ_TIMER], 1, &nrf5x_ble_timer_irq);
  device_irq_source_init(dev, &pv->irq_source[NRF5X_BLE_IRQ_RTC], 1, &nrf5x_ble_rtc_irq);

  err = device_irq_source_link(dev, pv->irq_source, NRF5X_BLE_IRQ_COUNT, -1);
  if (err)
    goto err_free_pv;

  err = dev_drv_clock_init(dev, &pv->clock_sink[NRF5X_BLE_CLK_RADIO], NRF5X_BLE_CLK_RADIO,
                           DEV_CLOCK_EP_FREQ_NOTIFY, NULL);
  if (err)
    goto err_free_pv;

  err = dev_drv_clock_init(dev, &pv->clock_sink[NRF5X_BLE_CLK_SLEEP], NRF5X_BLE_CLK_SLEEP,
                           DEV_CLOCK_EP_FREQ_NOTIFY, &pv->sleep_freq);
  if (err)
    goto err_cleanup_sleep_clock;

  pv->sca = ble_sca_from_accuracy(&pv->sleep_freq);

  kroutine_init_sched_switch(&pv->rescheduler, nrf5x_ble_reschedule_kr);
  kroutine_init_sched_switch(&pv->closer, nrf5x_ble_closer_kr);

  nrf5x_ble_radio_init();
  nrf5x_ble_rtc_init();
  nrf5x_ble_timer_init();
  nrf5x_ble_ppi_init();
  nrf5x_ble_debug_init();

  nrf5x_ble_context_list_init(&pv->context_list);
  nrf5x_ble_context_list_init(&pv->closed_list);

  pv->dev = dev;

  dev->drv_pv = pv;

  return 0;

 err_cleanup_sleep_clock:
  dev_drv_clock_cleanup(dev, &pv->clock_sink[NRF5X_BLE_CLK_SLEEP]);

 err_free_pv:
  mem_free(pv);
  return 1;
}

static void nrf5x_ble_event_close(struct nrf5x_ble_private_s *pv,
                                  enum event_status_e status)
{
  struct nrf5x_ble_context_s *ctx = pv->current;

  nrf5x_ble_backlog(ctx, "close %d", status);

  nrf5x_ble_radio_disable(pv);
  nrf5x_ble_rtc_boundary_clear();

  pv->transmitting = NULL;

  dprintk("%s %p %d %d\n", __FUNCTION__, ctx, pv->event_packet_count, status);

  assert(ctx);

  pv->current = NULL;

  if (ctx->scheduled) {
    nrf5x_ble_context_list_remove(&pv->context_list, ctx);
    ctx->scheduled = 0;
  }

  ctx->status = status;
  nrf5x_ble_context_list_pushback(&pv->closed_list, ctx);
  ctx->closing = 1;

  kroutine_exec(&pv->closer);
}

static KROUTINE_EXEC(nrf5x_ble_closer_kr)
{
  struct nrf5x_ble_private_s *pv = KROUTINE_CONTAINER(kr, *pv, closer);
  struct nrf5x_ble_context_s *ctx;
  
  while ((ctx = nrf5x_ble_context_list_pop(&pv->closed_list))) {
    ctx->closing = 0;
    ctx->handler->event_closed(ctx, ctx->status);

    nrf5x_ble_backlog_dump(ctx);
  }

  kroutine_exec(&pv->rescheduler);
}

void nrf5x_ble_context_start_first(struct nrf5x_ble_private_s *pv)
{
  struct nrf5x_ble_context_s *ctx = nrf5x_ble_context_list_pop(&pv->context_list);
  struct nrf5x_ble_context_s *next = nrf5x_ble_context_list_head(&pv->context_list);

  /*
                            We are here      Will be both trigged
                                 |           by RTC through PPI
                                 |              |         |
                                 v              v         v
     Time --------+--------------+--------------+---------+----------->
                  ^              ^              ^         ^
                  |              |              |         |
           Request scheduled Clock enable   TXEN/RXEN   START

    Request clock now, it will be available later, when radio is
    enabled.  As clock rampup and enable times are known constants, if
    we are on time now, TXEN/RXEN and START will be too.
   */

  if (!ctx) {
    nrf5x_ble_radio_disable(pv);
    return;
  }

#ifdef NRF5X_BLE_BACKLOG
  ctx->backlog_cur = 0;
#endif

  assert(!(nrf_reg_get(NRF_PPI_ADDR, NRF_PPI_CHEN) & (1 << PPI_TIMER_IFS_RADIO_START)));

  ctx->scheduled = 0;
  kroutine_exec(&pv->rescheduler);

  pv->current = ctx;
  pv->event_packet_count = 0;

  pv->event_begin = ctx->event_begin;
  pv->event_end = ctx->event_end;
  pv->event_max_duration = ctx->event_max_duration;

  if (next && next->precise_timing
      && (next->event_begin - CLOCK_ENABLE_TK - RADIO_ENABLE_TK) < pv->event_end)
    pv->event_end = next->event_begin - CLOCK_ENABLE_TK - RADIO_ENABLE_TK - 4;

  bool_t clock_running = nrf5x_ble_clock_request(pv);
  gpio(I_LATER, 0);

  //  printk("Frame open\n");

  assert(ctx->handler->event_opened);

  if (!ctx->handler->event_opened(ctx)) {
    pv->current = NULL;
    nrf5x_ble_radio_disable(pv);
    kroutine_exec(&pv->rescheduler);
    return;
  }

  // Ensure timing is correct

  dev_timer_value_t now = nrf5x_ble_rtc_value_get(pv);
  dev_timer_delay_t ramp_time = RADIO_ENABLE_TK
    + (clock_running ? 0 : CLOCK_ENABLE_TK);
  bool_t sync_start = 0;

  if (pv->event_begin < now + ramp_time) {
    dprintk("Bad timing: %d < %d + %d ramp ",
            (uint32_t)pv->event_begin, (uint32_t)now, ramp_time);
    if (ctx->precise_timing || !clock_running) {
      dprintk("aborting\n");
      nrf5x_ble_event_close(pv, EVENT_STATUS_IN_PAST);
      return;
    } else {
      dprintk("starting ASAP\n");
      pv->event_begin = now + ramp_time;
      sync_start = 1;
    }
  }

  // Maybe handler wants to abort, after all...

  bool_t running = ctx->handler->radio_params(ctx, &pv->current_params);
  if (!running) {
    nrf5x_ble_event_close(pv, EVENT_STATUS_HANDLER_DONE);
    return;
  }

  nrf5x_ble_backlog(ctx, "starting sync %d", sync_start);

  // Actual register setup

  if (pv->event_end)
    nrf5x_ble_rtc_boundary_set(pv->event_end, 1);

  nrf_ppi_disable_mask(0
                       | (1 << PPI_END_TIMER_START)
                       | (1 << PPI_ADDRESS_TIMER_STOP)
                       );
  nrf_reg_set(BLE_RADIO_ADDR, NRF_RADIO_BCC, 16);
  nrf5x_ble_config_init(&pv->current_params);

#if 52000 <= CONFIG_NRF5X_MODEL && CONFIG_NRF5X_MODEL <= 52999
  nrf_reg_set(BLE_RADIO_ADDR, NRF_RADIO_MODECNF0, 0
    | NRF_RADIO_MODECNF0_RU_FAST
    | NRF_RADIO_MODECNF0_DTX_B1);
#endif

  pv->pipelining_race = 0;
  nrf_event_clear(BLE_RTC_ADDR, NRF_RTC_COMPARE(RTC_ENABLE));
  nrf_event_clear(BLE_RTC_ADDR, NRF_RTC_COMPARE(RTC_START));

  dev_timer_value_t enable_date = pv->event_begin - RADIO_ENABLE_TK;

  if (!sync_start) {
    nrf_reg_set(BLE_RTC_ADDR, NRF_RTC_CC(RTC_ENABLE), enable_date);
    nrf_reg_set(BLE_RTC_ADDR, NRF_RTC_CC(RTC_START), pv->event_begin);

    nrf_evt_enable_mask(BLE_RTC_ADDR, 0
                        | (1 << NRF_RTC_COMPARE(RTC_ENABLE))
                        | (1 << NRF_RTC_COMPARE(RTC_START))
                        );

    switch (pv->current_params.mode) {
    case MODE_TX:
      nrf5x_ble_trx_gpio_tx();
      nrf_ppi_enable_mask(0
                            | (1 << PPI_RTC_ENABLE_TXEN)
                            | (1 << PPI_RTC_MATCH_START));
      break;

    case MODE_RX:
      nrf5x_ble_trx_gpio_rx();
      nrf_ppi_enable_mask(0
                            | (1 << PPI_RTC_ENABLE_RXEN)
                            | (1 << PPI_RTC_MATCH_START));
      break;
    }
  }

  bool_t enabled = enable_date > nrf5x_ble_rtc_value_get(pv) + 2
    || nrf_reg_get(BLE_RADIO_ADDR, NRF_RADIO_STATE) != NRF_RADIO_STATE_DISABLED;

  nrf5x_ble_backlog(ctx, "enabled %d", enabled);

  if (sync_start || !enabled) {
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

    pv->next_params = pv->current_params;
    nrf5x_ble_pipelined_reset(pv);
  }

  if (nrf5x_ble_data_setup(pv))
    nrf5x_ble_event_close(pv, EVENT_STATUS_RESOURCES_MISSING);
}

static
void nrf5x_ble_context_start_later(struct nrf5x_ble_private_s *pv,
                                   struct nrf5x_ble_context_s *ctx)
{
  dev_timer_value_t deadline = ctx->event_begin - RADIO_ENABLE_TK - CLOCK_ENABLE_TK - RTC_SKEW_TK * 2;
  dev_timer_value_t now = nrf5x_ble_rtc_value_get(pv);

  gpio(I_LATER, I_LATER);

  nrf5x_ble_backlog(ctx, "start later %d", deadline);

  nrf5x_ble_rtc_boundary_set(__MAX(now + RTC_SKEW_TK * 2, deadline), 0);
  nrf5x_ble_clock_release(pv);
}

static
void nrf5x_ble_reschedule(struct nrf5x_ble_private_s *pv)
{
  dprintk("%s\n", __FUNCTION__);

  GCT_FOREACH(nrf5x_ble_context_list, &pv->context_list, c,
              dprintk(" %p %lld %d\n", c, c->event_begin, c->layer.handler->type);
              );

  if (pv->current)
    return;

  assert(!pv->transmitting);
  assert(!pv->current);

  struct nrf5x_ble_context_s *ctx = nrf5x_ble_context_list_head(&pv->context_list);

  if (!ctx) {
    dprintk("%s empty\n", __FUNCTION__);

    nrf5x_ble_clock_release(pv);
    return;
  }

  gpio(I_LATER, I_LATER);
  gpio(I_LATER, 0);
  gpio(I_LATER, I_LATER);

  dprintk("%s %lld %lld ", __FUNCTION__, ctx->event_begin, ctx->event_end);

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

  gpio(I_LATER, 0);

  CPU_INTERRUPT_SAVESTATE_DISABLE;

#if defined(CONFIG_DEVICE_CLOCK)
  dev_timer_value_t now = nrf5x_ble_rtc_value_get(pv);
  dev_timer_value_t ramp = RADIO_ENABLE_TK + CLOCK_ENABLE_TK;

  if (ctx->event_begin > now + ramp + RTC_SKEW_TK * 2) {
    dprintk("later\n");
    nrf5x_ble_context_start_later(pv, ctx);
    goto out;
  }
#endif

  dprintk("now\n");
  nrf5x_ble_context_start_first(pv);

 out:
  CPU_INTERRUPT_RESTORESTATE;
}

static
KROUTINE_EXEC(nrf5x_ble_reschedule_kr)
{
  struct nrf5x_ble_private_s *pv = KROUTINE_CONTAINER(kr, *pv, rescheduler);

  nrf5x_ble_reschedule(pv);
}

static
void _ble_context_unschedule(struct nrf5x_ble_private_s *pv,
    struct nrf5x_ble_context_s *ctx)
{
  if (ctx->closing) {
    ctx->closing = 0;
    nrf5x_ble_context_list_remove(&pv->closed_list, ctx);
    ctx->handler->event_closed(ctx, ctx->status);
    return;
  }

  if (!ctx->scheduled)
    return;

  gpio(I_ENABLE | I_TRANSFER, I_ENABLE | I_TRANSFER);
  gpio(I_ENABLE | I_TRANSFER, 0);

  if (ctx == nrf5x_ble_context_list_head(&pv->context_list))
    kroutine_exec(&pv->rescheduler);

  nrf5x_ble_context_list_remove(&pv->context_list, ctx);
  ctx->scheduled = 0;
}

void nrf5x_ble_context_cleanup(struct nrf5x_ble_context_s *ctx)
{
  struct nrf5x_ble_private_s *pv = ctx->pv;

  if (ctx == pv->current) {
    nrf5x_ble_radio_disable(pv);
    nrf5x_ble_rtc_boundary_clear();

    pv->current = NULL;
  }

  assert(ctx != pv->current);

  _ble_context_unschedule(pv, ctx);

  if (--(pv->context_count) == 0)
    nrf5x_ble_rtc_stop(pv);
}

void nrf5x_ble_context_schedule(struct nrf5x_ble_context_s *ctx,
                                dev_timer_value_t event_begin,
                                dev_timer_value_t event_end,
                                dev_timer_delay_t event_max_duration,
                                bool_t precise_timing,
                                uint32_t importance)
{
  struct nrf5x_ble_private_s *pv = ctx->pv;

  dprintk("%s %p %lld %lld\n", __FUNCTION__, ctx, event_begin, event_end);

  _ble_context_unschedule(pv, ctx);

  ctx->event_begin = event_begin;
  ctx->event_end = event_end;
  ctx->event_max_duration = event_max_duration;
  ctx->importance = importance;
  ctx->precise_timing = precise_timing;

  nrf5x_ble_context_list_insert_sorted(&pv->context_list, ctx);
  ctx->scheduled = 1;

  GCT_FOREACH(nrf5x_ble_context_list, &pv->context_list, c,
              dprintk(" %p %lld %d\n", c, c->event_begin, c->layer.handler->type);
              );

  if (ctx == nrf5x_ble_context_list_head(&pv->context_list))
    kroutine_exec(&pv->rescheduler);
}

void nrf5x_ble_event_address_matched(struct nrf5x_ble_private_s *pv)
{
  struct nrf5x_ble_context_s *current = pv->current;

  nrf_event_clear(BLE_RADIO_ADDR, NRF_RADIO_READY);

  if (!current) {
    nrf5x_ble_radio_disable(pv);
    kroutine_exec(&pv->rescheduler);
    return;
  }

  pv->address_ts = nrf5x_ble_rtc_value_get(pv) - 1;

  if (pv->current_params.rx_rssi)
    pv->rx_rssi = -(int8_t)nrf_reg_get(BLE_RADIO_ADDR, NRF_RADIO_RSSISAMPLE) * 8;

  gpio(I_TRANSFER | I_TX, pv->current_params.mode == MODE_TX ? (I_TRANSFER | I_TX) : I_TRANSFER);

  nrf5x_ble_ppi_cleanup(pv);
  nrf_it_disable_mask(BLE_RADIO_ADDR, 0
                      | (1 << NRF_RADIO_BCMATCH)
                      | (1 << NRF_RADIO_END)
                      );

  current->handler->ifs_event(current, 0);

  bool_t go_on = current->handler->radio_params(current, &pv->next_params);

  pv->pipelining_race = 0;

  if (go_on)
    pv->pipelining_race = nrf5x_ble_pipelined_setup(pv);

  if (!go_on || pv->pipelining_race) {
    nrf_short_set(BLE_RADIO_ADDR, 1 << NRF_RADIO_END_DISABLE);

    nrf_ppi_disable_mask(0
                         | (1 << PPI_END_TIMER_START)
                         | (1 << PPI_ADDRESS_TIMER_STOP)
                         );

    nrf_it_disable(BLE_TIMER_ADDR, NRF_TIMER_COMPARE(TIMER_IFS_TIMEOUT));

    pv->current_params.channel = -1;
  }

  if (pv->event_packet_count == 0) {
    if (current->event_max_duration) {
      pv->event_end = pv->address_ts + current->event_max_duration;
      nrf5x_ble_rtc_boundary_set(pv->event_end, 1);
    } else {
      nrf5x_ble_rtc_boundary_clear();
    }
  }

  pv->event_packet_count++;

  gpio(I_WAIT, I_WAIT);

  while (!nrf_event_check(BLE_RADIO_ADDR, NRF_RADIO_BCMATCH)) {
    uint32_t state = nrf_reg_get(BLE_RADIO_ADDR, NRF_RADIO_STATE);
    if (state != NRF_RADIO_STATE_RX && state != NRF_RADIO_STATE_TX
#if 52000 <= CONFIG_NRF5X_MODEL && CONFIG_NRF5X_MODEL <= 52999
        && state != NRF_RADIO_STATE_RXIDLE && state != NRF_RADIO_STATE_TXIDLE
#endif
        ) {
      printk("Not in RX/TX any more: %d\n", state);
      break;
    }
    assert(nrf_reg_get(BLE_RADIO_ADDR, NRF_RADIO_BCC) == 16);
  }

  nrf_event_clear(BLE_RADIO_ADDR, NRF_RADIO_BCMATCH);
  gpio(I_WAIT, 0);

  uint8_t len = pv->transmitting[1];
  uint16_t end_irq_bits = ((uint16_t)len + 5) * 8;

  nrf_reg_set(BLE_RADIO_ADDR, NRF_RADIO_BCC, end_irq_bits - RADIO_IRQ_LATENCY_US);
  nrf_event_clear(BLE_RADIO_ADDR, NRF_RADIO_BCMATCH);

  nrf_it_enable_mask(BLE_RADIO_ADDR, 0
                     | (1 << NRF_RADIO_BCMATCH)
                     | (1 << NRF_RADIO_END)
                     );

  gpio(I_WAIT, I_WAIT);
  if (end_irq_bits > 16 + RADIO_IRQ_LATENCY_US * 2 + RADIO_RX_CHAIN_DELAY_US
      && !nrf_event_check(BLE_RADIO_ADDR, NRF_RADIO_END)) {
    pv->wait_end = 1;
    return;
  }

  gpio(I_WAIT, 0);
  gpio(I_WAIT, I_WAIT);

  uint32_t some_long_time = 1024;
  while (!nrf_event_check(BLE_RADIO_ADDR, NRF_RADIO_END)) {
    --some_long_time;

    if (!some_long_time) {
      printk("Long time passed\n");
      nrf5x_ble_event_timeout(pv);
      return;
    }
  }
  gpio(I_WAIT, 0);

  nrf5x_ble_event_packet_ended(pv);
}

void nrf5x_ble_event_bcc_end(struct nrf5x_ble_private_s *pv)
{
  if (!pv->wait_end)
    return;
  pv->wait_end = 0;
  gpio(I_WAIT, 0);
  gpio(I_WAIT, I_WAIT);

  assert(nrf_reg_get(BLE_RADIO_ADDR, NRF_RADIO_BCC) != 16);
  while (!nrf_event_check(BLE_RADIO_ADDR, NRF_RADIO_END))
    ;

  gpio(I_WAIT, 0);
  nrf5x_ble_event_packet_ended(pv);
}

void nrf5x_ble_event_packet_ended(struct nrf5x_ble_private_s *pv)
{
  struct nrf5x_ble_context_s *current = pv->current;

  nrf_reg_set(BLE_RADIO_ADDR, NRF_RADIO_BCC, 16);
  nrf_event_clear(BLE_RADIO_ADDR, NRF_RADIO_END);

  gpio(I_TRANSFER, 0);

  if (!current) {
    nrf_it_disable(BLE_RADIO_ADDR, NRF_RADIO_END);
    nrf5x_ble_radio_disable(pv);
    kroutine_exec(&pv->rescheduler);
    return;
  }

  bool_t rx = pv->current_params.mode == MODE_RX;

  if (pv->pipelining)
    pv->current_params = pv->next_params;

  if (rx) {
    current->handler->payload_received(current,
                                       pv->address_ts,
                                       pv->rx_rssi,
                                       nrf_reg_get(BLE_RADIO_ADDR, NRF_RADIO_CRCSTATUS));
  }

  pv->transmitting = NULL;

  if (pv->pipelining_race)
    current->handler->ifs_event(current, 1);

  bool_t go_on = current->handler->radio_params(current, &pv->next_params);
  if (!go_on) {
    nrf5x_ble_event_close(pv, EVENT_STATUS_HANDLER_DONE);
    return;
  }

  if (pv->event_end && pv->event_end < nrf5x_ble_rtc_value_get(pv)) {
    nrf5x_ble_event_close(pv, EVENT_STATUS_WINDOW_DONE);
    return;
  }

  bool_t pipeline = ble_radio_params_equal(&pv->next_params, &pv->current_params);

  if (!pipeline)
    nrf5x_ble_pipelined_reset(pv);

  gpio(I_TX, I_TX);
  gpio(I_TX, 0);

  nrf_reg_set(BLE_RADIO_ADDR, NRF_RADIO_BCC, 16);
  if (nrf5x_ble_data_setup(pv)) {
    nrf5x_ble_event_close(pv, EVENT_STATUS_RESOURCES_MISSING);
    return;
  }

  if (pipeline)
    nrf5x_ble_pipelined_commit(pv);
}

void nrf5x_ble_event_ifs_timeout(struct nrf5x_ble_private_s *pv)
{
  struct nrf5x_ble_context_s *current = pv->current;

  nrf_task_trigger(BLE_TIMER_ADDR, NRF_TIMER_STOP);
  nrf5x_ble_radio_disable(pv);
  nrf5x_ble_ppi_cleanup(pv);

  nrf_event_clear(BLE_RADIO_ADDR, NRF_RADIO_BCMATCH);
  nrf_it_disable(BLE_RADIO_ADDR, NRF_RADIO_BCMATCH);

  pv->transmitting = NULL;

  if (!current) {
    kroutine_exec(&pv->rescheduler);
    return;
  }

  nrf5x_ble_backlog(pv->current, "IF Timeout", 0);

  current->handler->ifs_event(current, 1);

  bool_t running = current->handler->radio_params(current, &pv->next_params);
  if (!running) {
    nrf5x_ble_event_close(pv, EVENT_STATUS_HANDLER_DONE);
    return;
  }

  if (pv->event_end && pv->event_end < nrf5x_ble_rtc_value_get(pv)) {
    nrf5x_ble_event_close(pv, EVENT_STATUS_WINDOW_DONE);
    return;
  }

  nrf5x_ble_pipelined_reset(pv);
  if (nrf5x_ble_data_setup(pv))
    nrf5x_ble_event_close(pv, EVENT_STATUS_RESOURCES_MISSING);
}

void nrf5x_ble_event_timeout(struct nrf5x_ble_private_s *pv)
{
  nrf5x_ble_ppi_cleanup(pv);

  if (pv->current)
    nrf5x_ble_event_close(pv, EVENT_STATUS_WINDOW_DONE);
  else
    kroutine_exec(&pv->rescheduler);
}

static
int32_t nrf5x_ble_context_cmp(const struct nrf5x_ble_context_s *a,
                              const struct nrf5x_ble_context_s *b)
{
  dprintk("%s (%lld %lld %d) (%lld %lld %d)\n",
         __FUNCTION__,
         a->event_begin, a->event_end, a->importance,
         b->event_begin, b->event_end, b->importance);

  if (__MAX(b->event_end, b->event_begin) < a->event_begin)
    return 1;

  if (__MAX(a->event_end, a->event_begin) < b->event_begin)
    return -1;

  if (a->precise_timing != b->precise_timing)
    return a->precise_timing < b->precise_timing ? 1 : -1;

  return a->importance < b->importance ? -1 : 1;
}

void nrf5x_ble_context_list_insert_sorted(nrf5x_ble_context_list_root_t *root,
                                          struct nrf5x_ble_context_s *ctx)
{
  struct nrf5x_ble_context_s *cur;

  dprintk("%s %lld %d\n", __FUNCTION__, ctx->event_begin, ctx->layer.handler->type);

  for (cur = nrf5x_ble_context_list_head(root);
       cur;
       cur = nrf5x_ble_context_list_next(root, cur)) {
    dprintk(" %lld %d\n", cur->event_begin, cur->layer.handler->type);
    if (nrf5x_ble_context_cmp(ctx, cur) > 0)
      continue;
    nrf5x_ble_context_list_insert_prev(root, cur, ctx);
    return;
  }

  nrf5x_ble_context_list_pushback(root, ctx);
}

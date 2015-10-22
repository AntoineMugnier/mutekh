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
#include <device/class/crypto.h>
#include <device/class/net.h>

#include <mutek/printk.h>

#include <ble/net/layer.h>
#include <ble/protocol/radio.h>

#include <arch/nrf5x/ficr.h>
#include <arch/nrf5x/timer.h>
#include <arch/nrf5x/rtc.h>
#include <arch/nrf5x/radio.h>

#include "ble.h"
#include "ble_debug.h"

#define dprintk(...) do{}while(0)
//#define dprintk printk

#if defined(CONFIG_BLE_CENTRAL)
# include "ble_master.h"
#endif

#if defined(CONFIG_BLE_PERIPHERAL)
# include "ble_slave.h"
# include "ble_advertiser.h"
#endif

static KROUTINE_EXEC(nrf5x_ble_reschedule_kr);

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
#if defined(CONFIG_BLE_CENTRAL)
  case BLE_NET_LAYER_MASTER:
    return nrf5x_ble_master_create(scheduler, pv, params, delegate, delegate_vtable, layer);
#endif
#if defined(CONFIG_BLE_PERIPHERAL)
  case BLE_NET_LAYER_ADV:
    return nrf5x_ble_advertiser_create(scheduler, pv, params, delegate, delegate_vtable, layer);
  case BLE_NET_LAYER_SLAVE:
    return nrf5x_ble_slave_create(scheduler, pv, params, delegate, delegate_vtable, layer);
#endif
  default:
    return -ENOTSUP;
  }
}

error_t nrf5x_ble_context_init(struct nrf5x_ble_context_s *ctx,
                               struct net_scheduler_s *scheduler,
                               const struct net_layer_handler_s *layer_handler,
                               struct nrf5x_ble_private_s *priv,
                               const struct nrf5x_ble_context_handler_s *handler,
                               void *delegate,
                               const struct net_layer_delegate_vtable_s *delegate_vtable)
{
  error_t err;

  err = net_layer_init(&ctx->layer, layer_handler, scheduler, delegate, delegate_vtable);
  if (err)
    return err;

  ctx->handler = handler;
  ctx->pv = priv;
  ctx->scheduled = 0;

  if (priv->context_count++ == 0)
    nrf5x_ble_rtc_start(priv);

  return 0;
}

static DEV_NET_GET_INFO(nrf5x_ble_get_info)
{
  memset(info, 0, sizeof(*info));
  info->implemented_layers = 0
#if defined(CONFIG_BLE_CENTRAL)
    | (1 << BLE_NET_LAYER_MASTER)
#endif
#if defined(CONFIG_BLE_PERIPHERAL)
    | (1 << BLE_NET_LAYER_ADV)
    | (1 << BLE_NET_LAYER_SLAVE)
#endif
    ;
  info->prefix_size = 1;
  info->mtu = CONFIG_BLE_PACKET_SIZE - 1;
  info->addr.random_addr = cpu_mem_read_32(NRF_FICR_DEVICEADDRTYPE) & 1;

  memcpy(info->addr.mac, (void*)NRF_FICR_DEVICEADDR(0), 6);
  if (info->addr.random_addr)
    info->addr.mac[5] |= 0xc0;

  assert(info->prefix_size + info->mtu <= CONFIG_BLE_PACKET_SIZE);

  return 0;
}

static DEV_CLEANUP(nrf5x_ble_cleanup);
static DEV_INIT(nrf5x_ble_init);
#define nrf5x_ble_use dev_use_generic
#define nrf5x_ble_rtc_request (dev_timer_request_t*)dev_driver_notsup_fcn
#define nrf5x_ble_rtc_cancel (dev_timer_request_t*)dev_driver_notsup_fcn

static DEV_TIMER_CONFIG(nrf5x_ble_rtc_config)
{
  struct device_s *dev = accessor->dev;
  struct nrf5x_ble_private_s *pv = dev->drv_pv;
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

  nrf_it_disable_mask(BLE_RADIO_ADDR, -1);
  nrf_it_disable_mask(BLE_TIMER_ADDR, -1);
  nrf_it_disable_mask(BLE_RTC_ADDR, -1);

  nrf5x_ble_radio_disable(pv);
  nrf_task_trigger(BLE_TIMER_ADDR, NRF_TIMER_STOP);
  nrf_task_trigger(BLE_RTC_ADDR, NRF_RTC_STOP);

#if defined(CONFIG_DEVICE_CLOCK)
  dev_clock_sink_unlink(dev, pv->clock_sink, 2);
#endif

  device_irq_source_unlink(dev, pv->irq_source, NRF5X_BLE_RADIO_IRQ_COUNT);

#if defined(CONFIG_BLE_CRYPTO)
  device_put_accessor(&pv->crypto);
#endif

  mem_free(pv);
}

static DEV_INIT(nrf5x_ble_init)
{
  struct nrf5x_ble_private_s *pv;
  error_t err;

  dev->status = DEVICE_DRIVER_INIT_FAILED;

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

#if defined(CONFIG_DEVICE_CLOCK)
  pv->accurate_clock_requested = 0;
  pv->accurate_clock_running = 0;

  dev_clock_sink_init(dev, &pv->clock_sink[NRF5X_BLE_CLK_RADIO], &nrf5x_ble_clock_changed);
  dev_clock_sink_init(dev, &pv->clock_sink[NRF5X_BLE_CLK_SLEEP], &nrf5x_ble_clock_changed);

  struct dev_clock_link_info_s ckinfo[2];
  if (dev_clock_sink_link(dev, pv->clock_sink, ckinfo, 0, 1))
    goto err_sink_cleanup;
#endif

  kroutine_init(&pv->rescheduler, nrf5x_ble_reschedule_kr, KROUTINE_INTERRUPTIBLE);

#if defined(CONFIG_BLE_CRYPTO)
  err = device_get_param_dev_accessor(dev, "crypto", &pv->crypto, DRIVER_CLASS_CRYPTO);
  if (err) {
    goto err_sink_unlink;
  }
#endif

  nrf5x_ble_radio_init();
  nrf5x_ble_rtc_init();
  nrf5x_ble_timer_init();
  nrf5x_ble_ppi_init();
  nrf5x_ble_debug_init();

  nrf5x_ble_context_list_init(&pv->context_list);

  pv->dev = dev;

  dev->drv_pv = pv;
  dev->drv = &nrf5x_ble_drv;
  dev->status = DEVICE_DRIVER_INIT_DONE;

  return 0;

 err_sink_unlink:
#if defined(CONFIG_DEVICE_CLOCK)
  dev_clock_sink_unlink(dev, pv->clock_sink, 2);
 err_sink_cleanup:
#endif
 err_free_pv:
  mem_free(pv);
  return 1;
}

static void nrf5x_ble_event_close(struct nrf5x_ble_private_s *pv,
                                  enum event_status_e status)
{
  struct nrf5x_ble_context_s *ctx = pv->current;

  nrf5x_ble_radio_disable(pv);
  nrf5x_ble_rtc_boundary_clear();

  if (pv->transmitting) {
    buffer_refdec(pv->transmitting);
    pv->transmitting = NULL;
  }

  dprintk("%s %p %d %d\n", __FUNCTION__, ctx, pv->event_packet_count, status);

  assert(ctx);

  pv->current = NULL;
  nrf5x_ble_context_list_remove(&pv->context_list, ctx);

  ctx->handler->event_closed(ctx, status);

  net_layer_refdec(&ctx->layer);

  kroutine_exec(&pv->rescheduler);
}

void nrf5x_ble_context_start_first(struct nrf5x_ble_private_s *pv)
{
  struct nrf5x_ble_context_s *ctx = nrf5x_ble_context_list_pop(&pv->context_list);

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

  net_layer_refinc(&ctx->layer);
  ctx->scheduled = 0;
  kroutine_exec(&pv->rescheduler);

  pv->current = ctx;
  pv->event_packet_count = 0;

  pv->event_begin = ctx->event_begin;
  pv->event_end = ctx->event_end;
  pv->event_max_duration = ctx->event_max_duration;

  bool_t clock_running = nrf5x_ble_clock_request(pv);
  gpio(I_LATER, 0);

  //  printk("Frame open type %d\n", ctx->layer.handler->type);

  if (ctx->handler->event_opened)
    ctx->handler->event_opened(ctx);

  // Ensure timing is correct

  dev_timer_value_t now = nrf5x_ble_rtc_value_get(pv);
  dev_timer_delay_t ramp_time = RADIO_ENABLE_TK
    + clock_running ? 0 : CLOCK_ENABLE_TK;
  bool_t sync_start = 0;

  if (pv->event_begin < now + ramp_time) {
    dprintk("Bad timing: %d < %d + %d ramp ",
            (uint32_t)pv->event_begin, (uint32_t)now, ramp_time);
    if (pv->event_end || !clock_running) {
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

  // Actual register setup

  if (pv->event_end)
    nrf5x_ble_rtc_boundary_set(pv->event_end, 1);

  nrf_ppi_disable_mask(0
                         | (1 << PPI_END_TIMER_START)
                         | (1 << PPI_ADDRESS_TIMER_STOP)
                         );
  nrf_reg_set(BLE_RADIO_ADDR, NRF_RADIO_BCC, 16);
  nrf5x_ble_config_init(&pv->current_params);

#if defined(CONFIG_ARCH_NRF52)
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
      nrf_ppi_enable_mask(0
                            | (1 << PPI_RTC_ENABLE_TXEN)
                            | (1 << PPI_RTC_MATCH_START));
      break;

    case MODE_RX:
      nrf_ppi_enable_mask(0
                            | (1 << PPI_RTC_ENABLE_RXEN)
                            | (1 << PPI_RTC_MATCH_START));
      break;
    }
  }

  bool_t enabled = enable_date > nrf5x_ble_rtc_value_get(pv) + 2
    || nrf_reg_get(BLE_RADIO_ADDR, NRF_RADIO_STATE) != NRF_RADIO_STATE_DISABLED;

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

  nrf5x_ble_data_setup(pv);
}

static
void nrf5x_ble_context_start_later(struct nrf5x_ble_private_s *pv,
                                   struct nrf5x_ble_context_s *ctx)
{
  dev_timer_value_t deadline = ctx->event_begin - RADIO_ENABLE_TK - CLOCK_ENABLE_TK - RTC_SKEW * 2;
  dev_timer_value_t now = nrf5x_ble_rtc_value_get(pv);

  gpio(I_LATER, I_LATER);

  nrf5x_ble_rtc_boundary_set(__MAX(now + 4, deadline), 0);
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

#if defined(CONFIG_DEVICE_CLOCK)
  dev_timer_value_t now = nrf5x_ble_rtc_value_get(pv);
  dev_timer_value_t ramp = RADIO_ENABLE_TK + CLOCK_ENABLE_TK;

  if (ctx->event_begin > now + ramp + RTC_SKEW * 2) {
    dprintk("later\n");
    nrf5x_ble_context_start_later(pv, ctx);
    return;
  }
#endif

  dprintk("now\n");
  nrf5x_ble_context_start_first(pv);
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
  if (!ctx->scheduled)
    return;

  gpio(I_ENABLE | I_TRANSFER, I_ENABLE | I_TRANSFER);
  gpio(I_ENABLE | I_TRANSFER, 0);

  if (ctx == nrf5x_ble_context_list_head(&pv->context_list))
    kroutine_exec(&pv->rescheduler);

  nrf5x_ble_context_list_remove(&pv->context_list, ctx);
}

void nrf5x_ble_context_cleanup(struct nrf5x_ble_context_s *ctx)
{
  struct nrf5x_ble_private_s *pv = ctx->pv;

  assert(ctx != pv->current);

  _ble_context_unschedule(pv, ctx);

  nrf5x_ble_reschedule(pv);

  if (--(pv->context_count) == 0)
    nrf5x_ble_rtc_stop(pv);
}

void nrf5x_ble_context_schedule(struct nrf5x_ble_context_s *ctx,
                                dev_timer_value_t event_begin,
                                dev_timer_value_t event_end,
                                dev_timer_delay_t event_max_duration,
                                uint32_t can_miss_score)
{
  struct nrf5x_ble_private_s *pv = ctx->pv;

  dprintk("%s %p %lld %lld\n", __FUNCTION__, ctx, ctx->event_begin, ctx->event_end);

  _ble_context_unschedule(pv, ctx);

  ctx->event_begin = event_begin;
  ctx->event_end = event_end;
  ctx->event_max_duration = event_max_duration;
  ctx->can_miss_score = can_miss_score;

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

  if (!current) {
    nrf5x_ble_radio_disable(pv);
    kroutine_exec(&pv->rescheduler);
    return;
  }

  pv->address_ts = nrf5x_ble_rtc_value_get(pv) - 1;

  gpio(I_TRANSFER | I_TX, pv->current_params.mode == MODE_TX ? (I_TRANSFER | I_TX) : I_TRANSFER);

  nrf5x_ble_ppi_cleanup(pv);
  nrf_it_disable_mask(BLE_RADIO_ADDR, 0
                      | (1 << NRF_RADIO_BCMATCH)
                      | (1 << NRF_RADIO_END)
                      );

  if (current->handler->ifs_event)
    current->handler->ifs_event(current, 0);

  bool_t go_on = current->handler->radio_params(current, &pv->next_params);

  if (go_on)
    pv->pipelining_race = nrf5x_ble_pipelined_setup(&pv->next_params);

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
    if (!nrf_reg_get(BLE_RADIO_ADDR, NRF_RADIO_STATE) == NRF_RADIO_STATE_RX) {
      dprintk("Not in RX any more\n");
      break;
    }
    assert(nrf_reg_get(BLE_RADIO_ADDR, NRF_RADIO_BCC) == 16);
  }

  nrf_event_clear(BLE_RADIO_ADDR, NRF_RADIO_BCMATCH);
  gpio(I_WAIT, 0);

  uint8_t len = pv->transmitting->data[pv->transmitting->begin + 1];
  uint16_t end_irq_bits = ((uint16_t)len + 5) * 8;

  nrf_reg_set(BLE_RADIO_ADDR, NRF_RADIO_BCC, end_irq_bits - RADIO_IRQ_LATENCY_US);
  nrf_event_clear(BLE_RADIO_ADDR, NRF_RADIO_BCMATCH);

  nrf_it_enable_mask(BLE_RADIO_ADDR, 0
                     | (1 << NRF_RADIO_BCMATCH)
                     | (1 << NRF_RADIO_END)
                     );

  gpio(I_WAIT, I_WAIT);
  if (end_irq_bits > 36 + RADIO_IRQ_LATENCY_US * 2 + 4
      && !nrf_event_check(BLE_RADIO_ADDR, NRF_RADIO_END)) {
  } else {
    gpio(I_WAIT, 0);
    gpio(I_WAIT, I_WAIT);
    uint32_t some_long_time = 1024;

    while (!nrf_event_check(BLE_RADIO_ADDR, NRF_RADIO_END)) {
      --some_long_time;

      if (!some_long_time) {
        nrf5x_ble_event_timeout(pv);
        return;
      }
    }
    gpio(I_WAIT, 0);

    nrf5x_ble_event_packet_ended(pv);
  }
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
  uint32_t shorts = nrf_short_get(BLE_RADIO_ADDR);

  if (shorts & (1 << NRF_RADIO_END_DISABLE) && shorts & (1 << NRF_RADIO_READY_START)
      && shorts & ((1 << NRF_RADIO_DISABLED_RXEN) | (1 << NRF_RADIO_DISABLED_TXEN))) {
    pv->current_params = pv->next_params;
  }

  if (rx) {
    pv->transmitting->end = pv->transmitting->begin
      + __MIN(pv->transmitting->data[pv->transmitting->begin + 1] + 2,
              buffer_size(pv->transmitting) - pv->transmitting->begin);

    current->handler->payload_received(current,
                                       pv->address_ts,
                                       nrf_reg_get(BLE_RADIO_ADDR, NRF_RADIO_CRCSTATUS),
                                       pv->transmitting);
  }

  buffer_refdec(pv->transmitting);
  pv->transmitting = NULL;

  gpio(I_PIPELINE, 0);

  if (pv->pipelining_race) {
    nrf5x_ble_event_close(pv, EVENT_STATUS_PIPELINE_FAILED);
    return;
  }

  bool_t go_on = current->handler->radio_params(current, &pv->next_params);
  if (!go_on) {
    nrf5x_ble_event_close(pv, EVENT_STATUS_HANDLER_DONE);
    return;
  }

  if (pv->event_end
      && pv->event_end < (nrf5x_ble_rtc_value_get(pv) + RADIO_ENABLE_TK + PACKET_MIN_TK)) {
    nrf5x_ble_event_close(pv, EVENT_STATUS_PREEMPTED);
    return;
  }

  if (!ble_radio_params_equal(&pv->next_params, &pv->current_params))
    nrf5x_ble_pipelined_reset(pv);

  gpio(I_TX, I_TX);
  gpio(I_TX, 0);

  nrf_reg_set(BLE_RADIO_ADDR, NRF_RADIO_BCC, 16);
  nrf5x_ble_data_setup(pv);
}

void nrf5x_ble_event_ifs_timeout(struct nrf5x_ble_private_s *pv)
{
  struct nrf5x_ble_context_s *current = pv->current;

  nrf_task_trigger(BLE_TIMER_ADDR, NRF_TIMER_STOP);
  nrf5x_ble_radio_disable(pv);
  nrf5x_ble_ppi_cleanup(pv);

  nrf_event_clear(BLE_RADIO_ADDR, NRF_RADIO_BCMATCH);
  nrf_it_disable(BLE_RADIO_ADDR, NRF_RADIO_BCMATCH);
  nrf_reg_set(BLE_RADIO_ADDR, NRF_RADIO_BCC, 16);

  if (pv->transmitting) {
    buffer_refdec(pv->transmitting);
    pv->transmitting = NULL;
  }

  if (!current) {
    kroutine_exec(&pv->rescheduler);
    return;
  }

  if (current->handler->ifs_event)
    current->handler->ifs_event(current, 1);

  bool_t running = current->handler->radio_params(current, &pv->next_params);
  if (!running) {
    nrf5x_ble_event_close(pv, EVENT_STATUS_IFS_TIMEOUT);
    return;
  }

  nrf5x_ble_pipelined_reset(pv);
  nrf5x_ble_data_setup(pv);
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
         a->event_begin, a->event_end, a->can_miss_score,
         b->event_begin, b->event_end, b->can_miss_score);

  if (__MAX(b->event_end, b->event_begin) < a->event_begin)
    return 1;

  if (__MAX(a->event_end, a->event_begin) < b->event_begin)
    return -1;

  return a->can_miss_score < b->can_miss_score ? -1 : 1;
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

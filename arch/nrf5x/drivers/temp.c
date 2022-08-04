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

    Copyright (c) 2021, Nicolas Pouillon <nipo@ssji.net>
*/

#define LOGK_MODULE_ID "ntmp"

#include <hexo/types.h>
#include <hexo/endian.h>
#include <hexo/iospace.h>
#include <hexo/interrupt.h>

#include <mutek/mem_alloc.h>
#include <mutek/printk.h>

#include <device/device.h>
#include <device/resources.h>
#include <device/driver.h>
#include <device/irq.h>

#include <device/class/valio.h>
#include <device/class/timer.h>

#include <device/valio/temperature.h>

#include <arch/nrf5x/temp.h>

#define TEMP_ADDR NRF_PERIPHERAL_ADDR(NRF5X_TEMP)

struct nrf5x_temp_pv_s
{
  dev_request_queue_root_t queue;
  struct dev_irq_src_s irq_ep;
  struct device_timer_s timer;
  struct dev_timer_rq_s timer_rq;
  uint32_t threshold;
  bool_t timer_scheduled;
};

STRUCT_COMPOSE(nrf5x_temp_pv_s, timer_rq);
DRIVER_PV(struct nrf5x_temp_pv_s);

static
void nrf5x_temp_conv_start(struct nrf5x_temp_pv_s *pv)
{
  logk_debug("start");

  nrf_task_trigger(TEMP_ADDR, NRF_TEMP_START);
}

static
void nrf5x_temp_conv_later(struct nrf5x_temp_pv_s *pv)
{
  if (pv->timer_scheduled)
    return;

  pv->timer_rq.deadline = 0;
  error_t err = DEVICE_OP(&pv->timer, request, &pv->timer_rq);

  logk_debug("later %d", err);

  if (!err)
    pv->timer_scheduled = 1;
  else
    nrf5x_temp_conv_start(pv);
}

static DEV_IRQ_SRC_PROCESS(nrf5x_temp_irq)
{
  struct device_s *dev = ep->base.dev;
  struct nrf5x_temp_pv_s *pv = dev->drv_pv;

  logk_debug("irq");
  
  if (!nrf_event_check(TEMP_ADDR, NRF_TEMP_DATARDY))
    return;

  uint32_t temperature = 250 * (int32_t)nrf_reg_get(TEMP_ADDR, NRF_TEMP_TEMP) + 273150;
  nrf_event_clear(TEMP_ADDR, NRF_TEMP_DATARDY);

  logk_debug("temp: %d mK", temperature);

  bool_t waiting = 0;

  GCT_FOREACH(dev_request_queue, &pv->queue, item, {
      struct dev_valio_rq_s *rq = dev_valio_rq_s_cast(item);
      struct valio_temperature_s *temp = rq->data;

      if (rq->type == DEVICE_VALIO_WAIT_EVENT
          && __ABS(temp->temperature - temperature) <= pv->threshold) {
        logk_debug("rq %p not changed %d %d", rq, temp->temperature, temperature);
        waiting = 1;
        GCT_FOREACH_CONTINUE;
      }

      temp->temperature = temperature;

      dev_valio_rq_remove(&pv->queue, rq);
      dev_valio_rq_done(rq);
    });

  if (waiting)
    nrf5x_temp_conv_later(pv);
}

static
KROUTINE_EXEC(nrf5x_temp_timer_done)
{
  struct dev_timer_rq_s *timer_rq = dev_timer_rq_from_kr(kr);
  struct nrf5x_temp_pv_s *pv = nrf5x_temp_pv_s_from_timer_rq(timer_rq);
  struct device_s *dev = pv->timer_rq.pvdata;

  LOCK_SPIN_IRQ_SCOPED(&dev->lock);

  pv->timer_scheduled = 0;

  logk_debug("tick");

  if (!dev_rq_queue_isempty(&pv->queue))
    nrf5x_temp_conv_start(pv);
}

static DEV_VALIO_REQUEST(nrf5x_temp_request)
{
  struct device_s *dev = accessor->dev;
  struct nrf5x_temp_pv_s *pv = dev->drv_pv;
  bool_t had_pending;

  logk_debug("%s", __func__);

  if ((rq->type != DEVICE_VALIO_READ && rq->type != DEVICE_VALIO_WAIT_EVENT)
      || rq->attribute != VALIO_TEMPERATURE_VALUE) {
    rq->error = -EINVAL;
    logk_debug("inval");
    dev_valio_rq_done(rq);
    return;
  }

  LOCK_SPIN_IRQ_SCOPED(&dev->lock);

  had_pending = !dev_rq_queue_isempty(&pv->queue);

  rq->error = 0;
  dev_valio_rq_pushback(&pv->queue, rq);

  if (had_pending) {
    logk_debug("pending");
    return;
  }

  if (rq->type == DEVICE_VALIO_READ)
    nrf5x_temp_conv_start(pv);
  else
    nrf5x_temp_conv_later(pv);
}

static DEV_VALIO_CANCEL(nrf5x_temp_cancel)
{
  struct device_s *dev = accessor->dev;
  struct nrf5x_temp_pv_s *pv = dev->drv_pv;

  logk_trace("%s %p", __func__, rq);

  LOCK_SPIN_IRQ_SCOPED(&dev->lock);

  GCT_FOREACH(dev_request_queue, &pv->queue, item, {
      struct dev_valio_rq_s *rq_item = dev_valio_rq_s_cast(item);
      if (rq == rq_item) {
        dev_valio_rq_remove(&pv->queue, rq);
        return 0;
      }
    });

  return -ENOENT;
}

#define nrf5x_temp_use dev_use_generic

static DEV_INIT(nrf5x_temp_init)
{
  struct nrf5x_temp_pv_s *pv;
  uintptr_t period, threshold;
  error_t err;

  pv = mem_alloc(sizeof(*pv), mem_scope_sys);
  if (!pv)
    return -ENOMEM;

  dev->drv_pv = pv;
  memset(pv, 0, sizeof(*pv));

  err = device_get_param_dev_accessor(dev, "timer", &pv->timer.base, DRIVER_CLASS_TIMER);
  if (err) {
    logk_fatal("Cannot get timer");
    goto free_pv;
  }

  device_irq_source_init(dev, &pv->irq_ep, 1, &nrf5x_temp_irq);
  err = device_irq_source_link(dev, &pv->irq_ep, 1, -1);
  if (err) {
    logk_fatal("Cannot link IRQ");
    goto free_pv;
  }

  err = device_get_param_uint(dev, "period", &period);
  if (err)
    period = 1000;

  err = device_get_param_uint(dev, "threshold", &threshold);
  if (err)
    threshold = 250;
  pv->threshold = threshold;

  dev_rq_queue_init(&pv->queue);

  dev_timer_rq_init(&pv->timer_rq, nrf5x_temp_timer_done);
  dev_timer_init_sec(&pv->timer, &pv->timer_rq.delay, 0, period, 1000);
  pv->timer_rq.pvdata = dev;

  nrf_it_enable(TEMP_ADDR, NRF_TEMP_DATARDY);

  return 0;

 free_timer:
  device_put_accessor(&pv->timer.base);
 free_pv:
  mem_free(pv);

  return err;
}

static DEV_CLEANUP(nrf5x_temp_cleanup)
{
  struct nrf5x_temp_pv_s *pv = dev->drv_pv;

  device_irq_source_unlink(dev, &pv->irq_ep, 1);

  dev_rq_queue_destroy(&pv->queue);
  device_put_accessor(&pv->timer.base);
  dev->drv_pv = NULL;
  mem_free(pv);

  return 0;
}

DRIVER_DECLARE(nrf5x_temp_drv, 0, "nRF5x temp", nrf5x_temp,
               DRIVER_VALIO_METHODS(nrf5x_temp));

DRIVER_REGISTER(nrf5x_temp_drv);

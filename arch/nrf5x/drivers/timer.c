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

    Copyright (c) Nicolas Pouillon <nipo@ssji.net>, 2014
*/

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
#include <device/class/timer.h>
#include <device/class/clock.h>

#include <arch/nrf5x/timer.h>
#include <arch/nrf5x/peripheral.h>
#include <arch/nrf5x/ids.h>

struct nrf5x_timer_context_s
{
  uintptr_t addr;

  struct dev_irq_src_s irq_ep[1];
  dev_request_pqueue_root_t queue;
  dev_timer_value_t base;
  uint8_t width;
  uint8_t div;
  struct dev_freq_accuracy_s acc;

#if defined(CONFIG_DEVICE_CLOCK)
  struct dev_clock_sink_ep_s clock_sink;
#endif
};

#define OVERFLOW 0
#define DEADLINE 1
#define VALUE 2

static void nrf5x_timer_start(struct nrf5x_timer_context_s *pv)
{
#if defined(CONFIG_DEVICE_CLOCK)
  dev_clock_sink_hold(&pv->clock_sink, 1);
#endif
  nrf_task_trigger(pv->addr, NRF_TIMER_START);
  nrf_it_enable(pv->addr, NRF_TIMER_COMPARE(OVERFLOW));
}

static void nrf5x_timer_stop(struct nrf5x_timer_context_s *pv)
{
  nrf_task_trigger(pv->addr, NRF_TIMER_STOP);
  nrf_it_disable_mask(pv->addr, -1);
#if defined(CONFIG_DEVICE_CLOCK)
  dev_clock_sink_release(&pv->clock_sink);
#endif
}

static dev_timer_value_t nrf5x_timer_value_get(
    struct nrf5x_timer_context_s *pv)
{
  uint32_t counter;

  CPU_INTERRUPT_SAVESTATE_DISABLE;
  nrf_task_trigger(pv->addr, NRF_TIMER_CAPTURE(VALUE));
  counter = nrf_reg_get(pv->addr, NRF_TIMER_CC(VALUE));
  if (!(counter >> (pv->width - 1))
      && nrf_event_check(pv->addr, NRF_TIMER_COMPARE(OVERFLOW)))
    counter += (dev_timer_value_t)1 << pv->width;
  CPU_INTERRUPT_RESTORESTATE;

  return pv->base + counter;
}

static void nrf5x_deadline_set(
  struct nrf5x_timer_context_s *pv,
  dev_timer_value_t next_deadline)
{
  nrf_reg_set(pv->addr, NRF_TIMER_CC(DEADLINE), next_deadline);
  nrf_it_enable(pv->addr, NRF_TIMER_COMPARE(DEADLINE));
}

static void nrf5x_deadline_disable(
  struct nrf5x_timer_context_s *pv)
{
  nrf_it_disable(pv->addr, NRF_TIMER_COMPARE(DEADLINE));
}

static DEV_TIMER_REQUEST(nrf5x_timer_request)
{
  struct device_s *dev = accessor->dev;
  struct nrf5x_timer_context_s *pv = dev->drv_pv;
  dev_timer_value_t value;
  error_t err = 0;

  LOCK_SPIN_IRQ(&dev->lock);

  if (dev_request_pqueue_isempty(&pv->queue)) {
    if (!pv->start_count)
      nrf5x_timer_start(pv);

    pv->start_count++;
  }

  value = nrf5x_timer_value_get(pv);

  if (rq->delay)
    rq->deadline = value + rq->delay;

  if (rq->deadline <= value + 256) {
    err = -ETIMEDOUT;
  } else {
    dev_timer_pqueue_insert(&pv->queue, &rq->rq);
    rq->rq.drvdata = pv;

    if (dev_request_pqueue_head(&pv->queue) == &rq->rq)
      nrf5x_deadline_set(pv, rq->deadline);
  }

  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
}

static DEV_TIMER_CANCEL(nrf5x_timer_cancel)
{
  struct device_s *dev = accessor->dev;
  struct nrf5x_timer_context_s *pv = dev->drv_pv;
  error_t err = -ETIMEDOUT;
  struct dev_timer_rq_s *head;

  if (rq->rq.drvdata != pv)
    return err;

  LOCK_SPIN_IRQ(&dev->lock);

  head = dev_timer_rq_s_cast(dev_request_pqueue_head(&pv->queue));

  dev_timer_pqueue_remove(&pv->queue, &rq->rq);
  rq->rq.drvdata = NULL;

  if (rq == head) {
    head = dev_timer_rq_s_cast(dev_request_pqueue_head(&pv->queue));

    if (head)
      nrf5x_deadline_set(pv, head->deadline);
    else
      nrf5x_deadline_disable(pv);
  }

  err = 0;

  if (dev_request_pqueue_isempty(&pv->queue)) {
    pv->start_count--;

    if (!pv->start_count)
      nrf5x_timer_stop(pv);
  }

  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
}

static DEV_USE(nrf5x_timer_use)
{
  struct device_accessor_s *accessor = param;
  struct device_s *dev = accessor->dev;
  struct nrf5x_timer_context_s *pv = dev->drv_pv;

  switch (op) {
  default:
    return 0;

  case DEV_USE_START:
    if (!pv->start_count)
      nrf5x_timer_start(pv);
    break;

  case DEV_USE_STOP:
    if (!pv->start_count)
      nrf5x_timer_stop(pv);
    break;
  }

  return 0;
}

static DEV_TIMER_GET_VALUE(nrf5x_timer_get_value)
{
  struct device_s *dev = accessor->dev;
  struct nrf5x_timer_context_s *pv = dev->drv_pv;
  error_t err = 0;

  LOCK_SPIN_IRQ(&dev->lock);

  if (value)
    *value = nrf5x_timer_value_get(pv);

  if (rev)
    err = -EAGAIN;

  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
}

static DEV_IRQ_SRC_PROCESS(nrf5x_timer_irq)
{
  struct device_s *dev = ep->base.dev;
  struct nrf5x_timer_context_s *pv = dev->drv_pv;
  struct dev_timer_rq_s *rq;

  lock_spin(&dev->lock);

  if (nrf_event_check(pv->addr, NRF_TIMER_COMPARE(OVERFLOW))) {
    nrf_event_clear(pv->addr, NRF_TIMER_COMPARE(OVERFLOW));
    pv->base += 1ULL << pv->width;
  }

  if (nrf_event_check(pv->addr, NRF_TIMER_COMPARE(DEADLINE)))
    nrf_event_clear(pv->addr, NRF_TIMER_COMPARE(DEADLINE));

  while ((rq = dev_timer_rq_s_cast(dev_request_pqueue_head(&pv->queue))))
    {
      if (nrf5x_timer_value_get(pv) < rq->deadline)
        break;

      dev_request_pqueue_pop(&pv->queue);
      rq->rq.drvdata = 0;

      lock_release(&dev->lock);
      kroutine_exec(&rq->rq.kr);
      lock_spin(&dev->lock);
    }

  if (rq)
    nrf5x_deadline_set(pv, rq->deadline);
  else if (!pv->start_count) {
    nrf5x_deadline_disable(pv);
  }

  lock_release(&dev->lock);
}

static DEV_TIMER_CONFIG(nrf5x_timer_config)
{
  struct device_s *dev = accessor->dev;
  struct nrf5x_timer_context_s *pv = dev->drv_pv;
  error_t err = 0;

  LOCK_SPIN_IRQ(&dev->lock);

  if (cfg) {
    cfg->freq.num = 1000000;
    cfg->freq.denom = 1;
    cfg->acc = pv->acc;
    cfg->rev = 0;
    cfg->res = 1;
    cfg->cap = DEV_TIMER_CAP_STOPPABLE
      | DEV_TIMER_CAP_HIGHRES
      | DEV_TIMER_CAP_TICKLESS
      | DEV_TIMER_CAP_KEEPVALUE
      | DEV_TIMER_CAP_REQUEST;
    cfg->max = -1;
  }

  if (res > 1)
    err = -ERANGE;

  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
}

#if defined(CONFIG_DEVICE_CLOCK)
static DEV_CLOCK_SINK_CHANGED(timer_clock_changed)
{
  struct nrf5x_timer_context_s *pv = ep->dev->drv_pv;

  pv->acc = *acc;
}
#endif

static DEV_INIT(nrf5x_timer_init);
static DEV_CLEANUP(nrf5x_timer_cleanup);

DRIVER_DECLARE(nrf5x_timer_drv, 0, "nRF5x Timer", nrf5x_timer,
               DRIVER_TIMER_METHODS(nrf5x_timer));

DRIVER_REGISTER(nrf5x_timer_drv);

static DEV_INIT(nrf5x_timer_init)
{
  struct nrf5x_timer_context_s *pv;

  pv = mem_alloc(sizeof(*pv), mem_scope_sys);
  dev->drv_pv = pv;

  if (!pv)
    return -ENOMEM;

  if (device_res_get_uint(dev, DEV_RES_MEM, 0, &pv->addr, NULL))
    goto free_pv;

  // 1% default
  pv->acc.m = 7;
  pv->acc.e = 20;

  switch (pv->addr) {
  case NRF_PERIPHERAL_ADDR(NRF5X_TIMER0):
    nrf_reg_set(pv->addr, NRF_TIMER_BITMODE, NRF_TIMER_BITMODE_32);
    pv->width = 32;
    break;

  case NRF_PERIPHERAL_ADDR(NRF5X_TIMER1):
  case NRF_PERIPHERAL_ADDR(NRF5X_TIMER2):
    nrf_reg_set(pv->addr, NRF_TIMER_BITMODE, NRF_TIMER_BITMODE_16);
    pv->width = 16;
    break;

  default:
    assert(!"Unsupported timer address");
    goto free_pv;
  }

  device_irq_source_init(dev, pv->irq_ep, 1, &nrf5x_timer_irq);

  if (device_irq_source_link(dev, pv->irq_ep, 1, -1))
    goto free_pv;

#if defined(CONFIG_DEVICE_CLOCK)
  dev_clock_sink_init(dev, &pv->clock_sink, &timer_clock_changed);
  struct dev_clock_link_info_s ckinfo;
  if (dev_clock_sink_link(dev, &pv->clock_sink, &ckinfo, 0, 0))
    goto free_pv;
#endif

  dev_request_pqueue_init(&pv->queue);
  pv->base = 0;
  pv->start_count = 0;

  nrf_reg_set(pv->addr, NRF_TIMER_CC(OVERFLOW), 0);
  nrf_reg_set(pv->addr, NRF_TIMER_MODE, NRF_TIMER_MODE_TIMER);
  nrf_reg_set(pv->addr, NRF_TIMER_PRESCALER, 4);
  nrf_it_disable_mask(pv->addr, -1);

  return 0;

 free_pv:
  mem_free(pv);
  return -1;
}

DEV_CLEANUP(nrf5x_timer_cleanup)
{
  struct nrf5x_timer_context_s *pv = dev->drv_pv;

  if (!dev_request_pqueue_isempty(&pv->queue))
    return -EBUSY;

  dev_request_pqueue_destroy(&pv->queue);
  nrf_it_disable_mask(pv->addr, -1);
#if defined(CONFIG_DEVICE_CLOCK)
  dev_clock_sink_unlink(dev, &pv->clock_sink, 1);
#endif

  device_irq_source_unlink(dev, pv->irq_ep, 1);

  mem_free(pv);

  return 0;
}

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

#include <arch/nrf5x/rtc.h>
#include <arch/nrf5x/ids.h>

struct nrf5x_rtc_context_s
{
  uintptr_t addr;

  struct dev_irq_src_s irq_ep[1];
  dev_request_pqueue_root_t queue;
  size_t start_count;
  dev_timer_value_t base;
  struct dev_freq_accuracy_s acc;

#if defined(CONFIG_DEVICE_CLOCK)
  struct dev_clock_sink_ep_s clock_sink;
#endif
};

static void nrf5x_rtc_start(struct nrf5x_rtc_context_s *pv)
{
  if (!pv->start_count) {
#if defined(CONFIG_DEVICE_CLOCK)
    dev_clock_sink_hold(&pv->clock_sink, 1);
#endif
    nrf_task_trigger(pv->addr, NRF_RTC_START);
    nrf_it_enable(pv->addr, NRF_RTC_OVERFLW);
  }

  pv->start_count++;
}

static void nrf5x_rtc_stop(struct nrf5x_rtc_context_s *pv)
{
  pv->start_count--;

  if (!pv->start_count) {
    nrf_task_trigger(pv->addr, NRF_RTC_STOP);
    nrf_it_disable_mask(pv->addr, -1);
#if defined(CONFIG_DEVICE_CLOCK)
    dev_clock_sink_release(&pv->clock_sink);
#endif
  }
}

static dev_timer_value_t nrf5x_rtc_value_get(
  struct nrf5x_rtc_context_s *pv)
{
  uint32_t counter;

  CPU_INTERRUPT_SAVESTATE_DISABLE;
  counter = nrf_reg_get(pv->addr, NRF_RTC_COUNTER);
  if (!(counter & (1 << 23)) && nrf_event_check(pv->addr, NRF_RTC_OVERFLW))
    counter += (1 << 24);
  CPU_INTERRUPT_RESTORESTATE;

  return pv->base + counter;
}

static void nrf5x_deadline_set(struct nrf5x_rtc_context_s *pv,
                               dev_timer_value_t next_deadline)
{
  nrf_reg_set(pv->addr, NRF_RTC_CC(0), next_deadline);
  nrf_it_enable(pv->addr, NRF_RTC_COMPARE(0));

  if (next_deadline <= nrf5x_rtc_value_get(pv) + 4)
    nrf_it_enable(pv->addr, NRF_RTC_TICK);
}

static void nrf5x_deadline_disable(struct nrf5x_rtc_context_s *pv)
{
  nrf_it_disable(pv->addr, NRF_RTC_COMPARE(0));
  nrf_it_disable(pv->addr, NRF_RTC_TICK);
}

static DEV_TIMER_REQUEST(nrf5x_rtc_request)
{
  struct device_s *dev = accessor->dev;
  struct nrf5x_rtc_context_s *pv = dev->drv_pv;
  dev_timer_value_t value;
  error_t err = 0;

  //  assert(rq->rq.drvdata == NULL);

  LOCK_SPIN_IRQ(&dev->lock);

  if (dev_request_pqueue_isempty(&pv->queue))
    nrf5x_rtc_start(pv);

  value = nrf5x_rtc_value_get(pv);

  if (rq->delay)
    rq->deadline = value + rq->delay;

  if (rq->deadline <= value) {
    err = -ETIMEDOUT;

    if (dev_request_pqueue_isempty(&pv->queue))
      nrf5x_rtc_stop(pv);
  } else {
    dev_timer_pqueue_insert(&pv->queue, &rq->rq);
    rq->rq.drvdata = pv;

    if (dev_request_pqueue_head(&pv->queue) == &rq->rq)
      nrf5x_deadline_set(pv, rq->deadline);
  }

  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
}

static DEV_TIMER_CANCEL(nrf5x_rtc_cancel)
{
  struct device_s *dev = accessor->dev;
  struct nrf5x_rtc_context_s *pv = dev->drv_pv;
  error_t err = -ENOENT;
  struct dev_timer_rq_s *head;


  LOCK_SPIN_IRQ(&dev->lock);

  if (rq->rq.drvdata != pv)
    goto out;

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

  if (dev_request_pqueue_isempty(&pv->queue))
    nrf5x_rtc_stop(pv);

 out:

  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
}

static DEV_USE(nrf5x_rtc_use)
{
  struct device_accessor_s *accessor = param;
  struct device_s *dev = accessor->dev;
  struct nrf5x_rtc_context_s *pv = dev->drv_pv;

  switch (op) {
  default:
    return 0;

  case DEV_USE_START:
    nrf5x_rtc_start(pv);
    break;

  case DEV_USE_STOP:
    nrf5x_rtc_stop(pv);
    break;
  }

  return 0;
}

static DEV_TIMER_GET_VALUE(nrf5x_rtc_get_value)
{
  struct device_s *dev = accessor->dev;
  struct nrf5x_rtc_context_s *pv = dev->drv_pv;
  error_t err = 0;

  LOCK_SPIN_IRQ(&dev->lock);

  if (value)
    *value = nrf5x_rtc_value_get(pv);

  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
}

static DEV_IRQ_SRC_PROCESS(nrf5x_rtc_irq)
{
  struct device_s *dev = ep->base.dev;
  struct nrf5x_rtc_context_s *pv = dev->drv_pv;
  struct dev_timer_rq_s *rq;

  lock_spin(&dev->lock);

  if (nrf_event_check(pv->addr, NRF_RTC_OVERFLW)) {
    nrf_event_clear(pv->addr, NRF_RTC_OVERFLW);
    pv->base += 1ULL << 24;
  }

  if (nrf_event_check(pv->addr, NRF_RTC_TICK)) {
    nrf_event_clear(pv->addr, NRF_RTC_TICK);
    nrf_it_disable(pv->addr, NRF_RTC_TICK);
  }

  if (nrf_event_check(pv->addr, NRF_RTC_COMPARE(0))) {
    nrf_event_clear(pv->addr, NRF_RTC_COMPARE(0));
  }

  while ((rq = dev_timer_rq_s_cast(dev_request_pqueue_head(&pv->queue))))
    {
      if (nrf5x_rtc_value_get(pv) < rq->deadline)
        break;

      dev_request_pqueue_pop(&pv->queue);
      rq->rq.drvdata = 0;

      lock_release(&dev->lock);
      kroutine_exec(&rq->rq.kr);
      lock_spin(&dev->lock);
    }

  if (rq)
    nrf5x_deadline_set(pv, rq->deadline);
  else {
    nrf5x_rtc_stop(pv);
    nrf5x_deadline_disable(pv);
  }

  lock_release(&dev->lock);
}

static DEV_TIMER_CONFIG(nrf5x_rtc_config)
{
  struct device_s *dev = accessor->dev;
  struct nrf5x_rtc_context_s *pv = dev->drv_pv;
  error_t err = 0;

  LOCK_SPIN_IRQ(&dev->lock);

  if (cfg) {
    cfg->freq.num = 32768;
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
static DEV_CLOCK_SINK_CHANGED(rtc_clock_changed)
{
  struct nrf5x_rtc_context_s *pv = ep->dev->drv_pv;

  pv->acc = *acc;
}
#endif

static DEV_INIT(nrf5x_rtc_init);
static DEV_CLEANUP(nrf5x_rtc_cleanup);

DRIVER_DECLARE(nrf5x_rtc_drv, 0, "nRF5x RTC", nrf5x_rtc,
               DRIVER_TIMER_METHODS(nrf5x_rtc));

DRIVER_REGISTER(nrf5x_rtc_drv);

static DEV_INIT(nrf5x_rtc_init)
{
  struct nrf5x_rtc_context_s *pv;

  dev->status = DEVICE_DRIVER_INIT_FAILED;

  pv = mem_alloc(sizeof(*pv), mem_scope_sys);
  dev->drv_pv = pv;

  if (!pv)
    return -ENOMEM;

  if (device_res_get_uint(dev, DEV_RES_MEM, 0, &pv->addr, NULL))
    goto free_pv;

  // 2% default
  pv->acc.m = 0;
  pv->acc.e = 22;

  device_irq_source_init(dev, pv->irq_ep, 1, &nrf5x_rtc_irq);

  if (device_irq_source_link(dev, pv->irq_ep, 1, -1))
    goto free_pv;

#if defined(CONFIG_DEVICE_CLOCK)
  dev_clock_sink_init(dev, &pv->clock_sink, &rtc_clock_changed);
  struct dev_clock_link_info_s ckinfo;
  if (dev_clock_sink_link(dev, &pv->clock_sink, &ckinfo, 0, 0))
    goto free_pv;
#endif

  dev_request_pqueue_init(&pv->queue);
  pv->base = 0;
  pv->start_count = 0;

  nrf_it_disable_mask(pv->addr, -1);
  nrf_evt_disable_mask(pv->addr, -1);

  dev->drv = &nrf5x_rtc_drv;
  dev->status = DEVICE_DRIVER_INIT_DONE;

  return 0;

 free_pv:
  mem_free(pv);
  return -1;
}

DEV_CLEANUP(nrf5x_rtc_cleanup)
{
  struct nrf5x_rtc_context_s *pv = dev->drv_pv;

  if (!dev_request_pqueue_isempty(&pv->queue))
    return -EBUSY;

  dev_request_pqueue_destroy(&pv->queue);

  nrf_it_disable_mask(pv->addr, -1);
  nrf_evt_disable_mask(pv->addr, -1);

  device_irq_source_unlink(dev, pv->irq_ep, 1);
#if defined(CONFIG_DEVICE_CLOCK)
  dev_clock_sink_unlink(dev, &pv->clock_sink, 1);
#endif

  mem_free(pv);

  return 0;
}

#if defined(CONFIG_ARCH_NRF51) && defined(CONFIG_SYSTEMVIEW)
uint64_t systemview_timestamp_get(void);
uint64_t systemview_timestamp_get(void)
{
  return nrf_reg_get(NRF_PERIPHERAL_ADDR(NRF5X_RTC1), NRF_RTC_COUNTER);
}
#endif

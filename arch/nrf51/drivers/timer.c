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

#include <arch/nrf51/timer.h>
#include <arch/nrf51/peripheral.h>
#include <arch/nrf51/ids.h>

struct nrf51_timer_context_s
{
  uintptr_t addr;

  struct dev_irq_ep_s irq_ep[1];
  dev_request_pqueue_root_t queue;
  size_t start_count;
  dev_timer_value_t base;
  uint8_t width;
  uint8_t div;
};

#define OVERFLOW 0
#define DEADLINE 1
#define VALUE 2

static void nrf51_timer_start(struct nrf51_timer_context_s *pv)
{
  if (!pv->start_count) {
    nrf_task_trigger(pv->addr, NRF51_TIMER_START);
    nrf_it_enable(pv->addr, NRF51_TIMER_COMPARE(OVERFLOW));
  }

  pv->start_count++;
}

static void nrf51_timer_stop(struct nrf51_timer_context_s *pv)
{
  pv->start_count--;

  if (!pv->start_count) {
    nrf_task_trigger(pv->addr, NRF51_TIMER_STOP);
    nrf_it_disable_mask(pv->addr, -1);
  }
}

static dev_timer_value_t nrf51_timer_value_get(
    struct nrf51_timer_context_s *pv)
{
  uint32_t counter;

  CPU_INTERRUPT_SAVESTATE_DISABLE;
  nrf_task_trigger(pv->addr, NRF51_TIMER_CAPTURE(VALUE));
  counter = nrf_reg_get(pv->addr, NRF51_TIMER_CC(VALUE));
  if (!(counter >> (pv->width - 1))
      && nrf_event_check(pv->addr, NRF51_TIMER_COMPARE(OVERFLOW)))
    counter += (dev_timer_value_t)1 << pv->width;
  CPU_INTERRUPT_RESTORESTATE;

  return pv->base + counter;
}

static void nrf51_deadline_set(
  struct nrf51_timer_context_s *pv,
  dev_timer_value_t next_deadline)
{
  nrf_reg_set(pv->addr, NRF51_TIMER_CC(DEADLINE), next_deadline);
  nrf_it_enable(pv->addr, NRF51_TIMER_COMPARE(DEADLINE));
}

static void nrf51_deadline_disable(
  struct nrf51_timer_context_s *pv)
{
  nrf_it_disable(pv->addr, NRF51_TIMER_COMPARE(DEADLINE));
}

static DEV_TIMER_REQUEST(nrf51_timer_request)
{
  struct device_s *dev = accessor->dev;
  struct nrf51_timer_context_s *pv = dev->drv_pv;
  dev_timer_value_t value;
  error_t err = 0;

  LOCK_SPIN_IRQ(&dev->lock);

  if (dev_request_pqueue_isempty(&pv->queue))
    nrf51_timer_start(pv);

  value = nrf51_timer_value_get(pv);

  if (rq->delay)
    rq->deadline = value + rq->delay;

  if (rq->deadline <= value + 256) {
    err = -ETIMEDOUT;
  } else {
    dev_timer_pqueue_insert(&pv->queue, &rq->rq);
    rq->rq.drvdata = pv;

    if (dev_request_pqueue_head(&pv->queue) == &rq->rq)
      nrf51_deadline_set(pv, rq->deadline);
  }

  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
}

static DEV_TIMER_CANCEL(nrf51_timer_cancel)
{
  struct device_s *dev = accessor->dev;
  struct nrf51_timer_context_s *pv = dev->drv_pv;
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
      nrf51_deadline_set(pv, head->deadline);
    else
      nrf51_deadline_disable(pv);
  }

  err = 0;

  if (dev_request_pqueue_isempty(&pv->queue))
    nrf51_timer_stop(pv);

  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
}

static DEV_USE(nrf51_timer_use)
{
  struct device_s *dev = accessor->dev;
  struct nrf51_timer_context_s *pv = dev->drv_pv;

  switch (op) {
  default:
    return 0;

  case DEV_USE_START:
    nrf51_timer_start(pv);
    break;

  case DEV_USE_STOP:
    nrf51_timer_stop(pv);
    break;
  }

  return 0;
}

static DEV_TIMER_GET_VALUE(nrf51_timer_get_value)
{
  struct device_s *dev = accessor->dev;
  struct nrf51_timer_context_s *pv = dev->drv_pv;
  error_t err = 0;

  LOCK_SPIN_IRQ(&dev->lock);

  if (value)
    *value = nrf51_timer_value_get(pv);

  if (rev)
    err = -EAGAIN;

  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
}

static DEV_IRQ_EP_PROCESS(nrf51_timer_irq)
{
  struct device_s *dev = ep->dev;
  struct nrf51_timer_context_s *pv = dev->drv_pv;
  struct dev_timer_rq_s *rq;

  lock_spin(&dev->lock);

  if (nrf_event_check(pv->addr, NRF51_TIMER_COMPARE(OVERFLOW))) {
    nrf_event_clear(pv->addr, NRF51_TIMER_COMPARE(OVERFLOW));
    pv->base += 1ULL << pv->width;
  }

  if (nrf_event_check(pv->addr, NRF51_TIMER_COMPARE(DEADLINE)))
    nrf_event_clear(pv->addr, NRF51_TIMER_COMPARE(DEADLINE));

  while ((rq = dev_timer_rq_s_cast(dev_request_pqueue_head(&pv->queue))))
    {
      if (nrf51_timer_value_get(pv) < rq->deadline)
        break;

      dev_request_pqueue_pop(&pv->queue);
      rq->rq.drvdata = 0;

      lock_release(&dev->lock);
      kroutine_exec(&rq->rq.kr, 0);
      lock_spin(&dev->lock);
    }

  if (rq)
    nrf51_deadline_set(pv, rq->deadline);
  else {
    nrf51_timer_stop(pv);
    nrf51_deadline_disable(pv);
  }

  lock_release(&dev->lock);
}

static DEV_TIMER_CONFIG(nrf51_timer_config)
{
  struct device_s *dev = accessor->dev;
  error_t err = 0;

  LOCK_SPIN_IRQ(&dev->lock);

  if (cfg) {
    cfg->freq.num = 1000000;
    cfg->freq.denom = 1;
    cfg->acc.m = 4;
    cfg->acc.e = 5; // 24 ppm
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

static DEV_INIT(nrf51_timer_init);
static DEV_CLEANUP(nrf51_timer_cleanup);

DRIVER_DECLARE(nrf51_timer_drv, "nRF51 Timer", nrf51_timer,
               DRIVER_TIMER_METHODS(nrf51_timer));

DRIVER_REGISTER(nrf51_timer_drv);

static DEV_INIT(nrf51_timer_init)
{
  struct nrf51_timer_context_s *pv;

  dev->status = DEVICE_DRIVER_INIT_FAILED;

  pv = mem_alloc(sizeof(*pv), mem_scope_sys);
  dev->drv_pv = pv;

  if (!pv)
    return -ENOMEM;

  if (device_res_get_uint(dev, DEV_RES_MEM, 0, &pv->addr, NULL))
    goto free_pv;

  switch (pv->addr) {
  case NRF_PERIPHERAL_ADDR(NRF51_TIMER0):
    nrf_reg_set(pv->addr, NRF51_TIMER_BITMODE, NRF51_TIMER_BITMODE_32);
    pv->width = 32;
    break;

  case NRF_PERIPHERAL_ADDR(NRF51_TIMER1):
  case NRF_PERIPHERAL_ADDR(NRF51_TIMER2):
    nrf_reg_set(pv->addr, NRF51_TIMER_BITMODE, NRF51_TIMER_BITMODE_16);
    pv->width = 16;
    break;

  default:
    assert(!"Unsupported timer address");
    goto free_pv;
  }

  device_irq_source_init(dev, pv->irq_ep, 1,
                         &nrf51_timer_irq, DEV_IRQ_SENSE_HIGH_LEVEL);

  if (device_irq_source_link(dev, pv->irq_ep, 1, -1))
    goto free_pv;

  dev_request_pqueue_init(&pv->queue);
  pv->base = 0;
  pv->start_count = 0;

  nrf_reg_set(pv->addr, NRF51_TIMER_CC(OVERFLOW), 0);
  nrf_reg_set(pv->addr, NRF51_TIMER_MODE, NRF51_TIMER_MODE_TIMER);
  nrf_reg_set(pv->addr, NRF51_TIMER_PRESCALER, 4);
  nrf_it_disable_mask(pv->addr, -1);

  dev->drv = &nrf51_timer_drv;
  dev->status = DEVICE_DRIVER_INIT_DONE;

  return 0;

 free_pv:
  mem_free(pv);
  return -1;
}

DEV_CLEANUP(nrf51_timer_cleanup)
{
  struct nrf51_timer_context_s *pv = dev->drv_pv;

  dev_request_pqueue_destroy(&pv->queue);
  nrf_it_disable_mask(pv->addr, -1);

  device_irq_source_unlink(dev, pv->irq_ep, 1);

  mem_free(pv);
}

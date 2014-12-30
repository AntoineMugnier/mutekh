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
    License along with MutekH; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
    02110-1301 USA.

    Copyright (c), Nicolas Pouillon <nipo@ssji.net>, 2015
*/

#include <mutek/mem_alloc.h>
#include <mutek/kroutine.h>

#include <hexo/types.h>
#include <hexo/endian.h>

#include <device/device.h>
#include <device/resources.h>
#include <device/driver.h>
#include <device/irq.h>
#include <device/class/valio.h>
#include <device/valio/adc.h>
#include <device/request.h>

#include <arch/nrf51/peripheral.h>
#include <arch/nrf51/adc.h>
#include <arch/nrf51/ids.h>

#define ADC_ADDR NRF_PERIPHERAL_ADDR(NRF51_ADC)

#define dprintk(...) do{}while(0)

struct nrf51_adc_private_s
{
  struct dev_irq_ep_s irq_ep[1];
  dev_request_queue_root_t queue;
  uint16_t todo;
  uint8_t index;
  bool_t callbacking;
  const uint32_t *config;
};

static void nrf51_adc_sample_next(struct device_s *dev);

static DEV_IRQ_EP_PROCESS(nrf51_adc_irq)
{
  struct device_s *dev = ep->dev;
  struct nrf51_adc_private_s *pv = dev->drv_pv;
  struct dev_valio_rq_s *rq;
  struct valio_adc_group_s *group;

  lock_spin(&dev->lock);

  rq = dev_valio_rq_s_from_base(dev_request_queue_head(&pv->queue));
  if (!rq)
    goto stop;

  group = rq->data;

  if (nrf_event_check(ADC_ADDR, NRF51_ADC_END)) {
    nrf_event_clear(ADC_ADDR, NRF51_ADC_END);

    dprintk("ADC sample %d done: %d\n", pv->index, nrf_reg_get(ADC_ADDR, NRF51_ADC_RESULT));

    group->value[pv->index] = nrf_reg_get(ADC_ADDR, NRF51_ADC_RESULT);
    nrf_task_trigger(ADC_ADDR, NRF51_ADC_STOP);
    pv->index++;

    if (!pv->todo)
      goto callback;

    goto again;
  }

  goto out;

 callback:
  dev_request_queue_pop(&pv->queue);

  rq->base.drvdata = 0;
  rq->error = 0;

  pv->callbacking = 1;
  lock_release(&dev->lock);
  kroutine_exec(&rq->base.kr, 0);
  lock_spin(&dev->lock);
  pv->callbacking = 0;

  rq = dev_valio_rq_s_from_base(dev_request_queue_head(&pv->queue));
  if (!rq)
    goto stop;

 again:
  nrf51_adc_sample_next(dev);
  goto out;

 stop:
  nrf_it_disable(ADC_ADDR, NRF51_ADC_END);
  nrf_reg_set(ADC_ADDR, NRF51_ADC_CONFIG, 0);
  nrf_reg_set(ADC_ADDR, NRF51_ADC_ENABLE, 0);

 out:
  lock_release(&dev->lock);
}

static void nrf51_adc_sample_next(struct device_s *dev)
{
  struct nrf51_adc_private_s *pv = dev->drv_pv;

  uint8_t line = __builtin_ctz(pv->todo);
  pv->todo &= ~(1 << line);

  dprintk("ADC sample %d line %d conf %08x\n", pv->index, line, pv->config[line]);

  nrf_reg_set(ADC_ADDR, NRF51_ADC_CONFIG, 0
              | pv->config[line]
              | NRF51_ADC_CONFIG_PSEL(line)
              );

  nrf_event_clear(ADC_ADDR, NRF51_ADC_END);
  nrf_task_trigger(ADC_ADDR, NRF51_ADC_START);
}

static void nrf51_adc_request_start(struct device_s *dev)
{
  struct nrf51_adc_private_s *pv = dev->drv_pv;
  struct dev_valio_rq_s *rq;
  struct valio_adc_group_s *group;

  rq = dev_valio_rq_s_from_base(dev_request_queue_head(&pv->queue));

  assert(rq);

  group = rq->data;

  pv->todo = group->mask;
  pv->index = 0;

  nrf_reg_set(ADC_ADDR, NRF51_ADC_ENABLE, NRF51_ADC_ENABLE_ENABLED);
  nrf_it_enable(ADC_ADDR, NRF51_ADC_END);

  nrf51_adc_sample_next(dev);
}


static DEV_VALIO_REQUEST(nrf51_adc_request)
{
  struct device_s *dev = accessor->dev;
  struct nrf51_adc_private_s *pv = dev->drv_pv;
  struct valio_adc_group_s *group = req->data;
  bool_t start;

  req->error = 0;

  dprintk("%s %d %d %02x\n", __FUNCTION__, req->type, req->attribute, group->mask);

  if (!group->mask) {
    req->error = -ENOTSUP;
    kroutine_exec(&req->base.kr, 0);
    return;
  }

  if (req->attribute != VALIO_ADC_VALUE || req->type != DEVICE_VALIO_READ) {
    req->error = -ENOTSUP;
    kroutine_exec(&req->base.kr, 0);
    return;
  }

  LOCK_SPIN_IRQ(&dev->lock);

  start = dev_request_queue_isempty(&pv->queue);

  dev_request_queue_pushback(&pv->queue, &req->base);
  req->base.drvdata = dev;

  if (start)
    nrf51_adc_request_start(dev);

  LOCK_RELEASE_IRQ(&dev->lock);
}

static DEV_INIT(nrf51_adc_init);
static DEV_CLEANUP(nrf51_adc_cleanup);
#define nrf51_adc_use dev_use_generic

#define nrf51_adc_use dev_use_generic

DRIVER_DECLARE(nrf51_adc_drv, "nRF51 ADC", nrf51_adc,
               DRIVER_VALIO_METHODS(nrf51_adc));

DRIVER_REGISTER(nrf51_adc_drv);

static DEV_INIT(nrf51_adc_init)
{
  struct nrf51_adc_private_s *pv;
  const void *config;
  error_t err;

  dev->status = DEVICE_DRIVER_INIT_FAILED;

  err = device_get_param_blob(dev, "config", 0, &config);
  if (err)
    goto out;

  pv = mem_alloc(sizeof(*pv), mem_scope_sys);

  if (!pv)
    return -ENOMEM;

  dev->drv_pv = pv;

  memset(pv, 0, sizeof(*pv));

  device_irq_source_init(dev, pv->irq_ep, 1,
                         &nrf51_adc_irq, DEV_IRQ_SENSE_HIGH_LEVEL);

  if (device_irq_source_link(dev, pv->irq_ep, 1, -1))
    goto free_pv;

  __unused__ uintptr_t addr = 0;
  assert(device_res_get_uint(dev, DEV_RES_MEM, 0, &addr, NULL) == 0 &&
         ADC_ADDR == addr);

  dev_request_queue_init(&pv->queue);

  nrf_it_disable_mask(ADC_ADDR, -1);

  pv->config = config;
  dev->drv = &nrf51_adc_drv;
  dev->status = DEVICE_DRIVER_INIT_DONE;

  return 0;

 free_pv:
  mem_free(pv);
 out:
  return -1;
}

static DEV_CLEANUP(nrf51_adc_cleanup)
{
  struct nrf51_adc_private_s *pv = dev->drv_pv;

  dev_request_queue_destroy(&pv->queue);
  nrf_it_disable_mask(ADC_ADDR, -1);
  device_irq_source_unlink(dev, pv->irq_ep, 1);

  mem_free(pv);
}

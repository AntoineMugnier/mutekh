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

#define LOGK_MODULE_ID "nadc"

#include <mutek/mem_alloc.h>
#include <mutek/kroutine.h>
#include <mutek/printk.h>

#include <hexo/types.h>
#include <hexo/endian.h>
#include <hexo/bit.h>

#include <device/device.h>
#include <device/resources.h>
#include <device/driver.h>
#include <device/irq.h>
#include <device/class/valio.h>
#include <device/valio/adc.h>
#include <device/request.h>

#include <arch/nrf5x/peripheral.h>
#include <arch/nrf5x/adc.h>
#include <arch/nrf5x/ids.h>

#define ADC_ADDR NRF_PERIPHERAL_ADDR(NRF5X_ADC)

DRIVER_PV(struct nrf5x_adc_private_s
{
  struct dev_irq_src_s irq_ep[1];
  dev_request_queue_root_t queue;
  const uintptr_t *config;
  uint16_t config_count;
  uint16_t todo;
  uint8_t index;
});

static void nrf5x_adc_sample_next(struct device_s *dev);

static DEV_IRQ_SRC_PROCESS(nrf5x_adc_irq)
{
  struct device_s *dev = ep->base.dev;
  struct nrf5x_adc_private_s *pv = dev->drv_pv;
  struct dev_valio_rq_s *rq;
  struct valio_adc_group_s *group;

  LOCK_SPIN_SCOPED(&dev->lock);

  rq = dev_valio_rq_head(&pv->queue);
  if (rq) {
    group = rq->data;

    if (nrf_event_check(ADC_ADDR, NRF_ADC_END)) {
      nrf_event_clear(ADC_ADDR, NRF_ADC_END);

      logk_trace("ADC sample %d done: %d\n", pv->index, nrf_reg_get(ADC_ADDR, NRF_ADC_RESULT));

      group->value[pv->index] = nrf_reg_get(ADC_ADDR, NRF_ADC_RESULT);
      nrf_task_trigger(ADC_ADDR, NRF_ADC_STOP);
      pv->index++;

      if (!pv->todo) {
        dev_valio_rq_pop(&pv->queue);

        rq->base.drvdata = 0;
        rq->error = 0;

        dev_valio_rq_done(rq);

        rq = dev_valio_rq_head(&pv->queue);
      }

      if (rq) {
        nrf5x_adc_sample_next(dev);
        return;
      }
    }
  }

 stop:
  nrf_it_disable(ADC_ADDR, NRF_ADC_END);
  nrf_reg_set(ADC_ADDR, NRF_ADC_CONFIG, 0);
  nrf_reg_set(ADC_ADDR, NRF_ADC_ENABLE, 0);
}

static void nrf5x_adc_sample_next(struct device_s *dev)
{
  struct nrf5x_adc_private_s *pv = dev->drv_pv;

  uint8_t line = bit_ctz(pv->todo);
  pv->todo &= ~bit(line);

  logk_trace("ADC sample %d line %d conf %08x\n", pv->index, line, pv->config[line]);

  nrf_reg_set(ADC_ADDR, NRF_ADC_CONFIG, 0
              | (line < pv->config_count ? pv->config[line] : 0)
              | NRF_ADC_CONFIG_PSEL(line)
              );

  nrf_event_clear(ADC_ADDR, NRF_ADC_END);
  nrf_task_trigger(ADC_ADDR, NRF_ADC_START);
}

static void nrf5x_adc_request_start(struct device_s *dev)
{
  struct nrf5x_adc_private_s *pv = dev->drv_pv;
  struct dev_valio_rq_s *rq;
  struct valio_adc_group_s *group;

  rq = dev_valio_rq_head(&pv->queue);

  assert(rq);

  group = rq->data;

  pv->todo = group->mask;
  pv->index = 0;

  nrf_reg_set(ADC_ADDR, NRF_ADC_ENABLE, NRF_ADC_ENABLE_ENABLED);
  nrf_it_enable(ADC_ADDR, NRF_ADC_END);

  nrf5x_adc_sample_next(dev);
}

static DEV_VALIO_REQUEST(nrf5x_adc_request)
{
  struct device_s *dev = accessor->dev;
  struct nrf5x_adc_private_s *pv = dev->drv_pv;
  struct valio_adc_group_s *group = rq->data;
  bool_t start;

  rq->error = 0;

  logk_trace("%s %d %d %02x\n", __FUNCTION__, rq->type, rq->attribute, group->mask);

  if (!group->mask) {
    rq->error = -ENOTSUP;
    dev_valio_rq_done(rq);
    return;
  }

  if (rq->attribute != VALIO_ADC_VALUE || rq->type != DEVICE_VALIO_READ) {
    rq->error = -ENOTSUP;
    dev_valio_rq_done(rq);
    return;
  }

  LOCK_SPIN_IRQ_SCOPED(&dev->lock);

  start = dev_rq_queue_isempty(&pv->queue);

  dev_valio_rq_pushback(&pv->queue, rq);
  rq->base.drvdata = dev;

  if (start)
    nrf5x_adc_request_start(dev);
}

static DEV_VALIO_CANCEL(nrf5x_adc_cancel)
{
  struct device_s *dev = accessor->dev;
  struct nrf5x_adc_private_s *pv = dev->drv_pv;
  error_t err = -ENOENT;

  LOCK_SPIN_IRQ_SCOPED(&dev->lock);

  if (rq->base.drvdata == dev) {
    err = 0;
    dev_valio_rq_remove(&pv->queue, rq);
  }

  return err;
}

#define nrf5x_adc_use dev_use_generic

static DEV_INIT(nrf5x_adc_init)
{
  struct nrf5x_adc_private_s *pv;
  uint16_t config_count;
  const uintptr_t *config;
  error_t err;

  err = device_get_param_uint_array(dev, "config", &config_count, &config);
  if (err)
    goto out;

  pv = mem_alloc(sizeof(*pv), mem_scope_sys);
  if (!pv)
    return -ENOMEM;

  dev->drv_pv = pv;

  memset(pv, 0, sizeof(*pv));

  device_irq_source_init(dev, pv->irq_ep, 1, &nrf5x_adc_irq);

  if (device_irq_source_link(dev, pv->irq_ep, 1, -1))
    goto free_pv;

  __unused__ uintptr_t addr = 0;
  assert(device_res_get_uint(dev, DEV_RES_MEM, 0, &addr, NULL) == 0 &&
         ADC_ADDR == addr);

  dev_rq_queue_init(&pv->queue);

  nrf_it_disable_mask(ADC_ADDR, -1);

  pv->config = config;
  pv->config_count = config_count;
  return 0;

 free_pv:
  mem_free(pv);
 out:
  return -1;
}

static DEV_CLEANUP(nrf5x_adc_cleanup)
{
  struct nrf5x_adc_private_s *pv = dev->drv_pv;

  if (!dev_rq_queue_isempty(&pv->queue))
    return -EBUSY;

  dev_rq_queue_destroy(&pv->queue);
  nrf_it_disable_mask(ADC_ADDR, -1);
  device_irq_source_unlink(dev, pv->irq_ep, 1);

  mem_free(pv);

  return 0;
}

DRIVER_DECLARE(nrf5x_adc_drv, 0, "nRF5x ADC", nrf5x_adc,
               DRIVER_VALIO_METHODS(nrf5x_adc));

DRIVER_REGISTER(nrf5x_adc_drv);


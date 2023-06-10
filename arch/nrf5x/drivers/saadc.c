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

#define LOGK_MODULE_ID "sadc"

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
#include <device/class/iomux.h>
#include <device/valio/adc.h>
#include <device/request.h>

#include <arch/nrf5x/peripheral.h>
#include <arch/nrf5x/saadc.h>
#include <arch/nrf5x/ids.h>

#define SAADC_ADDR NRF_PERIPHERAL_ADDR(NRF5X_SAADC)

struct nrf5x_saadc_private_s
{
  struct dev_irq_src_s irq_ep[1];
  dev_request_queue_root_t queue;
  uint_fast8_t channel_count;
};

DRIVER_PV(struct nrf5x_saadc_private_s);

static void nrf5x_saadc_request_start(struct device_s *dev);

static DEV_IRQ_SRC_PROCESS(nrf5x_saadc_irq)
{
  struct device_s *dev = ep->base.dev;
  struct nrf5x_saadc_private_s *pv = dev->drv_pv;
  struct dev_valio_rq_s *rq;

  LOCK_SPIN_SCOPED(&dev->lock);

  rq = dev_valio_rq_head(&pv->queue);
  if (rq && nrf_event_check(SAADC_ADDR, NRF_SAADC_DONE)) {
    nrf_event_clear(SAADC_ADDR, NRF_SAADC_DONE);

    logk_trace("SAADC done");

    nrf_task_trigger(SAADC_ADDR, NRF_SAADC_STOP);
  
    dev_valio_rq_pop(&pv->queue);

    rq->base.drvdata = 0;
    rq->error = 0;

    dev_valio_rq_done(rq);

    rq = dev_valio_rq_head(&pv->queue);

    if (rq) {
      nrf5x_saadc_request_start(dev);
      return;
    }
  }
  nrf_it_disable(SAADC_ADDR, NRF_SAADC_END);
  //  nrf_reg_set(SAADC_ADDR, NRF_SAADC_ENABLE, 0);
}

static void nrf5x_saadc_request_start(struct device_s *dev)
{
  struct nrf5x_saadc_private_s *pv = dev->drv_pv;
  struct dev_valio_rq_s *rq;
  struct valio_adc_group_s *group;

  rq = dev_valio_rq_head(&pv->queue);
  assert(rq);

  group = rq->data;

  //  nrf_reg_set(SAADC_ADDR, NRF_SAADC_ENABLE, NRF_SAADC_ENABLE_ENABLED);
  nrf_event_clear(SAADC_ADDR, NRF_SAADC_END);
  nrf_it_enable(SAADC_ADDR, NRF_SAADC_END);

  nrf_reg_set(SAADC_ADDR, NRF_SAADC_RESOLUTION, NRF_SAADC_RESOLUTION_12BIT);
  nrf_reg_set(SAADC_ADDR, NRF_SAADC_SAMPLERATE, 0
              | NRF_SAADC_SAMPLERATE_CC(80)
              | NRF_SAADC_SAMPLERATE_MODE_TIMER);
  nrf_reg_set(SAADC_ADDR, NRF_SAADC_OVERSAMPLE, NRF_SAADC_OVERSAMPLE_BYPASS);
  nrf_reg_set(SAADC_ADDR, NRF_SAADC_RESULT_PTR, (uintptr_t)group->value);
  nrf_reg_set(SAADC_ADDR, NRF_SAADC_RESULT_MAXCNT, pv->channel_count);
  nrf_task_trigger(SAADC_ADDR, NRF_SAADC_START);
  nrf_task_trigger(SAADC_ADDR, NRF_SAADC_SAMPLE);
}

static DEV_VALIO_REQUEST(nrf5x_saadc_request)
{
  struct device_s *dev = accessor->dev;
  struct nrf5x_saadc_private_s *pv = dev->drv_pv;
  struct valio_adc_group_s *group = rq->data;
  bool_t start;

  rq->error = 0;

  logk_trace("%s %d %d %02x", __FUNCTION__, rq->type, rq->attribute, group->mask);

  if (!group->mask)
    goto notsup;

  if ((group->mask + 1) != (1 << pv->channel_count))
    goto notsup;

  if (rq->attribute != VALIO_ADC_VALUE || rq->type != DEVICE_VALIO_READ)
    goto notsup;

  LOCK_SPIN_IRQ_SCOPED(&dev->lock);

  start = dev_rq_queue_isempty(&pv->queue);
  dev_valio_rq_pushback(&pv->queue, rq);
  rq->base.drvdata = dev;
  if (start)
    nrf5x_saadc_request_start(dev);

  return;

 notsup:
    rq->error = -ENOTSUP;
    dev_valio_rq_done(rq);
}

static DEV_VALIO_CANCEL(nrf5x_saadc_cancel)
{
  struct device_s *dev = accessor->dev;
  struct nrf5x_saadc_private_s *pv = dev->drv_pv;

  LOCK_SPIN_IRQ_SCOPED(&dev->lock);

  if (rq == dev_valio_rq_head(&pv->queue))
    return -EBUSY;
  
  if (rq->base.drvdata == dev) {
    dev_valio_rq_remove(&pv->queue, rq);
    return 0;
  }

  return -ENOENT;
}

#define nrf5x_saadc_use dev_use_generic

static DEV_INIT(nrf5x_saadc_init)
{
  struct nrf5x_saadc_private_s *pv;
  iomux_io_id_t pin[8];
  iomux_config_t config[8];
  size_t pin_count = 0;
  error_t err;

  err = device_iomux_setup(dev, "ch0? ch1? ch2? ch3? ch4? ch5? ch6? ch7?", NULL, pin, config);
  if (err)
    return err;

  for (size_t i = 0; i < 8; ++i)
    logk_debug("ch%d pin %d config %03x", i, pin[i], config[i]);

  while (pin_count < 8) {
    if (pin[pin_count] == IOMUX_INVALID_ID)
      break;
    pin_count++;
  }

  for (size_t i = pin_count; i < 8; ++i) {
    if (pin[pin_count] != IOMUX_INVALID_ID)
      return -EINVAL;
  }

  nrf_reg_set(SAADC_ADDR, NRF_SAADC_ENABLE, NRF_SAADC_ENABLE_ENABLED);

  for (size_t i = 0; i < pin_count; ++i) {
    uint8_t ain = NRF_SAADC_PSEL_NC;

    if (2 <= pin[i] && pin[i] <= 5)
      ain = NRF_SAADC_PSEL_AIN(pin[i] - 2);
    else if (28 <= pin[i] && pin[i] <= 31)
      ain = NRF_SAADC_PSEL_AIN(pin[i] - 28 + 4);
    else
      return -EINVAL;

    logk_debug("Pin %d, ain %d, config %03x", i, ain, config[i]);

    nrf_reg_set(SAADC_ADDR, NRF_SAADC_CH_CONFIG(i), NRF52_SAADC_PIN_CONFIG_EXTRACT(config[i]));
    nrf_reg_set(SAADC_ADDR, NRF_SAADC_CH_PSELP(i), ain);
    nrf_reg_set(SAADC_ADDR, NRF_SAADC_CH_PSELN(i), NRF_SAADC_PSEL_NC);
    nrf_reg_set(SAADC_ADDR, NRF_SAADC_CH_LIMIT(i), 0);
  }

  if (pin_count < 8) {
    uintptr_t vcc_channel;
    if (device_get_param_uint(dev, "vcc", &vcc_channel) == 0) {
      nrf_reg_set(SAADC_ADDR, NRF_SAADC_CH_CONFIG(pin_count), NRF52_SAADC_PIN_CONFIG_EXTRACT(vcc_channel));
      nrf_reg_set(SAADC_ADDR, NRF_SAADC_CH_PSELP(pin_count), NRF_SAADC_PSEL_VDD);
      nrf_reg_set(SAADC_ADDR, NRF_SAADC_CH_PSELN(pin_count), NRF_SAADC_PSEL_NC);
      nrf_reg_set(SAADC_ADDR, NRF_SAADC_CH_LIMIT(pin_count), 0);
      pin_count++;
    }
  }

  pv = mem_alloc(sizeof(*pv), mem_scope_sys);
  if (!pv)
    return -ENOMEM;

  dev->drv_pv = pv;
  memset(pv, 0, sizeof(*pv));

  pv->channel_count = pin_count;

  device_irq_source_init(dev, pv->irq_ep, 1, &nrf5x_saadc_irq);
  if (device_irq_source_link(dev, pv->irq_ep, 1, -1))
    goto free_pv;

  __unused__ uintptr_t addr = 0;
  assert(device_res_get_uint(dev, DEV_RES_MEM, 0, &addr, NULL) == 0 &&
         SAADC_ADDR == addr);

  dev_rq_queue_init(&pv->queue);

  nrf_it_disable_mask(SAADC_ADDR, -1);
  return 0;

 free_pv:
  mem_free(pv);
 out:
  return -1;
}

static DEV_CLEANUP(nrf5x_saadc_cleanup)
{
  struct nrf5x_saadc_private_s *pv = dev->drv_pv;

  if (!dev_rq_queue_isempty(&pv->queue))
    return -EBUSY;

  dev_rq_queue_destroy(&pv->queue);
  nrf_it_disable_mask(SAADC_ADDR, -1);
  device_irq_source_unlink(dev, pv->irq_ep, 1);

  device_iomux_cleanup(dev);
  mem_free(pv);

  return 0;
}

DRIVER_DECLARE(nrf52_saadc_drv, 0, "nRF52 SAADC", nrf5x_saadc,
               DRIVER_VALIO_METHODS(nrf5x_saadc));

DRIVER_REGISTER(nrf52_saadc_drv);


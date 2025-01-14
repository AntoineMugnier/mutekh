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

#include <mutek/mem_alloc.h>
#include <mutek/kroutine.h>

#include <hexo/types.h>
#include <hexo/endian.h>
#include <hexo/bit.h>

#include <device/device.h>
#include <device/resources.h>
#include <device/driver.h>
#include <device/irq.h>
#include <device/class/crypto.h>
#include <device/request.h>

#include <arch/nrf5x/peripheral.h>
#include <arch/nrf5x/rng.h>
#include <arch/nrf5x/ids.h>

#define RNG_ADDR NRF_PERIPHERAL_ADDR(NRF5X_RNG)

DRIVER_PV(struct nrf5x_rng_private_s
{
  struct dev_irq_src_s irq_ep[1];
  dev_request_queue_root_t queue;
});

static DEV_CRYPTO_INFO(nrf5x_rng_info)
{
  if (accessor->number > 0)
    return -ENOENT;

  memset(info, 0, sizeof(*info));

  info->name = "rng";
  info->modes_mask = bit(DEV_CRYPTO_MODE_RANDOM);
  info->cap = 0;
  info->state_size = 0;
  info->block_len = 0;

  return 0;
};

static DEV_IRQ_SRC_PROCESS(nrf5x_rng_irq)
{
  struct device_s *dev = ep->base.dev;
  struct nrf5x_rng_private_s *pv = dev->drv_pv;
  struct dev_crypto_rq_s *rq;

  LOCK_SPIN_SCOPED(&dev->lock);

  rq = dev_crypto_rq_head(&pv->queue);
  if (rq) {
    if (nrf_event_check(RNG_ADDR, NRF_RNG_VALRDY)) {
      nrf_event_clear(RNG_ADDR, NRF_RNG_VALRDY);
      *rq->out = nrf_reg_get(RNG_ADDR, NRF_RNG_VALUE);
      rq->out++;
      rq->len--;

      if (!rq->len) {
        dev_crypto_rq_pop(&pv->queue);
        rq->base.drvdata = 0;
        dev_crypto_rq_done(rq);

        rq = dev_crypto_rq_head(&pv->queue);
      }

      if (rq) {
        nrf_task_trigger(RNG_ADDR, NRF_RNG_START);
        nrf_it_enable(RNG_ADDR, NRF_RNG_VALRDY);

        return;
      }
    }
  }

  nrf_it_disable(RNG_ADDR, NRF_RNG_VALRDY);
  nrf_task_trigger(RNG_ADDR, NRF_RNG_STOP);
}

static void nrf5x_rng_byte_start(struct device_s *dev)
{
  nrf_event_clear(RNG_ADDR, NRF_RNG_VALRDY);
  nrf_it_enable(RNG_ADDR, NRF_RNG_VALRDY);
  nrf_task_trigger(RNG_ADDR, NRF_RNG_START);
}

static DEV_CRYPTO_REQUEST(nrf5x_rng_request)
{
  struct device_s *dev = accessor->dev;
  struct nrf5x_rng_private_s *pv = dev->drv_pv;
  bool_t start;

  if (!rq->len) {
    dev_crypto_rq_done(rq);
    return;
  }

  LOCK_SPIN_IRQ_SCOPED(&dev->lock);

  start = dev_rq_queue_isempty(&pv->queue);

  dev_crypto_rq_pushback(&pv->queue, rq);
  rq->base.drvdata = dev;

  if (start)
    nrf5x_rng_byte_start(dev);
}


#define nrf5x_rng_use dev_use_generic

static DEV_INIT(nrf5x_rng_init)
{
  struct nrf5x_rng_private_s *pv;

  pv = mem_alloc(sizeof(*pv), mem_scope_sys);
  dev->drv_pv = pv;

  if (!pv)
    return -ENOMEM;

  device_irq_source_init(dev, pv->irq_ep, 1, &nrf5x_rng_irq);

  if (device_irq_source_link(dev, pv->irq_ep, 1, -1))
    goto free_pv;

  __unused__ uintptr_t addr = 0;
  assert(device_res_get_uint(dev, DEV_RES_MEM, 0, &addr, NULL) == 0 &&
         RNG_ADDR == addr);

  dev_rq_queue_init(&pv->queue);

  nrf_it_disable_mask(RNG_ADDR, -1);
  nrf_reg_set(RNG_ADDR, NRF_RNG_CONFIG, NRF_RNG_CONFIG_DERCEN_ENABLED);

  return 0;

 free_pv:
  mem_free(pv);
  return -1;
}

static DEV_CLEANUP(nrf5x_rng_cleanup)
{
  struct nrf5x_rng_private_s *pv = dev->drv_pv;

  if (!dev_rq_queue_isempty(&pv->queue))
    return -EBUSY;

  dev_rq_queue_destroy(&pv->queue);
  nrf_it_disable_mask(RNG_ADDR, -1);
  device_irq_source_unlink(dev, pv->irq_ep, 1);

  mem_free(pv);

  return 0;
}

DRIVER_DECLARE(nrf5x_rng_drv, 0, "nRF5x RNG", nrf5x_rng,
               DRIVER_CRYPTO_METHODS(nrf5x_rng));

DRIVER_REGISTER(nrf5x_rng_drv);


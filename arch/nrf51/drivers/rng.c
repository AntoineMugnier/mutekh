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
#include <device/class/crypto.h>
#include <device/request.h>

#include <arch/nrf51/peripheral.h>
#include <arch/nrf51/rng.h>
#include <arch/nrf51/ids.h>

#define RNG_ADDR NRF_PERIPHERAL_ADDR(NRF51_RNG)

struct nrf51_rng_private_s
{
  struct dev_irq_ep_s irq_ep[1];
  dev_request_queue_root_t queue;
  bool_t callbacking;
};

static DEVCRYPTO_INFO(nrf51_rng_info)
{
  if (accessor->number > 0)
    return -ENOENT;

  memset(info, 0, sizeof(*info));

  info->name = "rng";
  info->modes_mask = 1 << DEV_CRYPTO_MODE_RANDOM;
  info->cap = 0;
  info->state_size = 0;
  info->block_len = 0;

  return 0;
};

static DEV_IRQ_EP_PROCESS(nrf51_rng_irq)
{
  struct device_s *dev = ep->dev;
  struct nrf51_rng_private_s *pv = dev->drv_pv;
  struct dev_crypto_rq_s *rq;

  lock_spin(&dev->lock);

  rq = dev_crypto_rq_s_cast(dev_request_queue_head(&pv->queue));
  if (!rq)
    goto stop;

  if (nrf_event_check(RNG_ADDR, NRF51_RNG_VALRDY)) {
    nrf_event_clear(RNG_ADDR, NRF51_RNG_VALRDY);
    *rq->out = nrf_reg_get(RNG_ADDR, NRF51_RNG_VALUE);
    rq->out++;
    rq->len--;

    if (rq->len)
      goto again;
    goto callback;
  }

  goto out;

 callback:
  dev_request_queue_pop(&pv->queue);

  rq->rq.drvdata = 0;

  pv->callbacking = 1;
  lock_release(&dev->lock);
  kroutine_exec(&rq->rq.kr, 0);
  lock_spin(&dev->lock);
  pv->callbacking = 0;

  rq = dev_crypto_rq_s_cast(dev_request_queue_head(&pv->queue));
  if (!rq)
    goto stop;

 again:
  nrf_task_trigger(RNG_ADDR, NRF51_RNG_START);
  nrf_it_enable(RNG_ADDR, NRF51_RNG_VALRDY);

  goto out;

 stop:
  nrf_it_disable(RNG_ADDR, NRF51_RNG_VALRDY);
  nrf_task_trigger(RNG_ADDR, NRF51_RNG_STOP);

 out:
  lock_release(&dev->lock);
}

static void nrf51_rng_byte_start(struct device_s *dev)
{
  nrf_event_clear(RNG_ADDR, NRF51_RNG_VALRDY);
  nrf_it_enable(RNG_ADDR, NRF51_RNG_VALRDY);
  nrf_task_trigger(RNG_ADDR, NRF51_RNG_START);
}

static DEVCRYPTO_REQUEST(nrf51_rng_request)
{
  struct device_s *dev = accessor->dev;
  struct nrf51_rng_private_s *pv = dev->drv_pv;
  bool_t start;

  if (!rq->len) {
    kroutine_exec(&rq->rq.kr, 0);
    return;
  }

  LOCK_SPIN_IRQ(&dev->lock);

  start = dev_request_queue_isempty(&pv->queue);

  dev_request_queue_pushback(&pv->queue, &rq->rq);
  rq->rq.drvdata = dev;

  if (start)
    nrf51_rng_byte_start(dev);

  LOCK_RELEASE_IRQ(&dev->lock);
}

static DEV_INIT(nrf51_rng_init);
static DEV_CLEANUP(nrf51_rng_cleanup);

#define nrf51_rng_use dev_use_generic

DRIVER_DECLARE(nrf51_rng_drv, "nRF51 RNG", nrf51_rng,
               DRIVER_CRYPTO_METHODS(nrf51_rng));

DRIVER_REGISTER(nrf51_rng_drv);

static DEV_INIT(nrf51_rng_init)
{
  struct nrf51_rng_private_s *pv;

  dev->status = DEVICE_DRIVER_INIT_FAILED;

  pv = mem_alloc(sizeof(*pv), mem_scope_sys);
  dev->drv_pv = pv;

  if (!pv)
    return -ENOMEM;

  device_irq_source_init(dev, pv->irq_ep, 1,
                         &nrf51_rng_irq, DEV_IRQ_SENSE_HIGH_LEVEL);

  if (device_irq_source_link(dev, pv->irq_ep, 1, -1))
    goto free_pv;

  __unused__ uintptr_t addr = 0;
  assert(device_res_get_uint(dev, DEV_RES_MEM, 0, &addr, NULL) == 0 &&
         RNG_ADDR == addr);

  dev_request_queue_init(&pv->queue);

  nrf_it_disable_mask(RNG_ADDR, -1);
  nrf_reg_set(RNG_ADDR, NRF51_RNG_CONFIG, NRF51_RNG_CONFIG_DERCEN_ENABLED);

  dev->drv = &nrf51_rng_drv;
  dev->status = DEVICE_DRIVER_INIT_DONE;

  return 0;

 free_pv:
  mem_free(pv);
  return -1;
}

static DEV_CLEANUP(nrf51_rng_cleanup)
{
  struct nrf51_rng_private_s *pv = dev->drv_pv;

  dev_request_queue_destroy(&pv->queue);
  nrf_it_disable_mask(RNG_ADDR, -1);
  device_irq_source_unlink(dev, pv->irq_ep, 1);

  mem_free(pv);
}

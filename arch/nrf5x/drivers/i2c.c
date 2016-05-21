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
#include <device/class/i2c.h>
#include <device/class/iomux.h>

#include <arch/nrf5x/i2c.h>
#include <arch/nrf5x/gpio.h>

#define dprintk(k...) do {} while (0)
//#define dprintk printk

DRIVER_PV(struct nrf5x_i2c_priv_s
{
  uintptr_t addr;
  struct dev_irq_src_s irq_ep[1];
  dev_request_queue_root_t queue;
});

static
void nrf5x_i2c_flit_first(struct nrf5x_i2c_priv_s *pv,
                          struct dev_i2c_rq_s *rq)
{
  struct dev_i2c_transfer_s *tr = &rq->transfer[rq->error_transfer];
  struct dev_i2c_transfer_s *next = tr + 1;

  nrf_reg_set(pv->addr, NRF_I2C_ADDR, rq->saddr);

  dprintk("%s type %d, tr %d/%d byte %d/%d\n",
          __FUNCTION__, tr->type,
          rq->error_transfer, rq->transfer_count,
          rq->error_offset, tr->size);

  if (rq->error_transfer + 1 >= rq->transfer_count)
    next = NULL;

  assert(rq->error_offset < tr->size || next);

  switch (tr->type) {
  case DEV_I2C_WRITE:
    /* dprintk("%s start txd %02x\n", __FUNCTION__, tr->data[rq->error_offset]); */

    nrf_short_set(pv->addr, 0);
    nrf_event_clear(pv->addr, NRF_I2C_TXDSENT);
    nrf_it_set_mask(pv->addr, 0
                    | (1 << NRF_I2C_STOPPED)
                    | (1 << NRF_I2C_ERROR)
                    | (1 << NRF_I2C_TXDSENT));

    CPU_INTERRUPT_SAVESTATE_DISABLE;
    nrf_task_trigger(pv->addr, NRF_I2C_STARTTX);
    nrf_reg_set(pv->addr, NRF_I2C_TXD, tr->data[rq->error_offset]);
    CPU_INTERRUPT_RESTORESTATE;
    break;

  case DEV_I2C_READ:
    if ((ssize_t)rq->error_offset == (ssize_t)tr->size - 1 && !next) {
      /* dprintk("%s rx bb->stop\n", __FUNCTION__); */
      nrf_short_set(pv->addr, 1 << NRF_I2C_BB_STOP);
    } else {
      nrf_short_set(pv->addr, 1 << NRF_I2C_BB_SUSPEND);
    }

    nrf_event_clear(pv->addr, NRF_I2C_RXDRDY);
    nrf_it_set_mask(pv->addr, 0
                    | (1 << NRF_I2C_STOPPED)
                    | (1 << NRF_I2C_ERROR)
                    | (1 << NRF_I2C_RXDRDY));

    nrf_task_trigger(pv->addr, NRF_I2C_STARTRX);
    break;
  }
}

static
bool_t nrf5x_i2c_request_progress(struct nrf5x_i2c_priv_s *pv,
                                  struct dev_i2c_rq_s *rq)
{
  struct dev_i2c_transfer_s *tr = &rq->transfer[rq->error_transfer];
  struct dev_i2c_transfer_s *next = tr + 1;

  if (rq->error_transfer + 1 >= rq->transfer_count)
    next = NULL;

  /* dprintk("%s progress %d%s\n", */
  /*         __FUNCTION__, tr->type, next ? ", next" : ""); */

  if (nrf_event_check(pv->addr, NRF_I2C_ERROR)) {
    nrf_event_clear(pv->addr, NRF_I2C_ERROR);

    uint32_t error = nrf_reg_get(pv->addr, NRF_I2C_ERRORSRC);

    dprintk("%s error %d\n", __FUNCTION__, error);

    nrf_reg_set(pv->addr, NRF_I2C_ERRORSRC, error);

    if (error & NRF_I2C_ERRORSRC_ANACK) {
      rq->error = -EHOSTUNREACH;
      rq->error_offset = 0;
    } else if (error & NRF_I2C_ERRORSRC_DNACK) {
      rq->error = -EIO;
    }

    if (error & (NRF_I2C_ERRORSRC_ANACK | NRF_I2C_ERRORSRC_DNACK)) {
      nrf_short_set(pv->addr, 0);
      nrf_task_trigger(pv->addr, NRF_I2C_STOP);
      return 0;
    }
  }

  if (nrf_event_check(pv->addr, NRF_I2C_TXDSENT)) {
    nrf_event_clear(pv->addr, NRF_I2C_TXDSENT);

    rq->error_offset++;

    if ((ssize_t)rq->error_offset == (ssize_t)tr->size) {
      if (!next) {
        /* dprintk("%s read end\n", __FUNCTION__); */
        nrf_task_trigger(pv->addr, NRF_I2C_STOP);
        return 0;
      }

      goto next;
    }

    dprintk("%s txd sent, -> %02x\n", __FUNCTION__, tr->data[rq->error_offset]);
    nrf_reg_set(pv->addr, NRF_I2C_TXD, tr->data[rq->error_offset]);
  }

  if (nrf_event_check(pv->addr, NRF_I2C_RXDRDY)) {
    nrf_event_clear(pv->addr, NRF_I2C_RXDRDY);
    uint8_t data = nrf_reg_get(pv->addr, NRF_I2C_RXD);

    dprintk("%s rxd ready %02x\n", __FUNCTION__, data);

    tr->data[rq->error_offset++] = data;

    switch ((ssize_t)tr->size - (ssize_t)rq->error_offset) {
    case 1:
      if (next)
        nrf_short_set(pv->addr, 1 << NRF_I2C_BB_SUSPEND);
      else
        nrf_short_set(pv->addr, 1 << NRF_I2C_BB_STOP);
      break;

    case 0:
      nrf_short_set(pv->addr, 0);

      if (next)
        goto next;

      // We had a BB->STOP short on previous byte, let STOP test
      // happen.
      goto stop;

    default:
      nrf_short_set(pv->addr, 1 << NRF_I2C_BB_SUSPEND);
      break;
    }

    uint32_t scl = nrf_reg_get(pv->addr, NRF_I2C_PSELSCL);

    for (uint8_t i = 0; i < 20; ++i) {
      uint32_t in = nrf_reg_get(NRF5X_GPIO_ADDR, NRF_GPIO_IN);
      if (!(in & (1 << scl)))
        break;
      dprintk("\nscl wait %d\n", (in >> scl) & 1);
    }

    nrf_task_trigger(pv->addr, NRF_I2C_RESUME);
  }

 stop:
  if (nrf_event_check(pv->addr, NRF_I2C_STOPPED)) {
    nrf_event_clear(pv->addr, NRF_I2C_STOPPED);

    nrf_short_set(pv->addr, 0);
    nrf_it_set_mask(pv->addr, 0);

    dprintk("%s stopped\n", __FUNCTION__);

    return 1;
  }

  return 0;

 next:
  dprintk("%s next tr\n", __FUNCTION__);
  rq->error_transfer++;
  rq->error_offset = 0;

  nrf5x_i2c_flit_first(pv, rq);
  return 0;
}

static
void nrf5x_i2c_request_do(struct nrf5x_i2c_priv_s *pv,
                          struct dev_i2c_rq_s *rq)
{
  rq->error_transfer = 0;
  rq->error_offset = 0;
  nrf5x_i2c_flit_first(pv, rq);
}

static DEV_IRQ_SRC_PROCESS(nrf5x_i2c_irq)
{
  struct device_s *dev = ep->base.dev;
  struct nrf5x_i2c_priv_s *pv = dev->drv_pv;

  lock_spin(&dev->lock);

  struct dev_i2c_rq_s *rq = dev_i2c_rq_s_cast(dev_request_queue_head(&pv->queue));
  struct dev_i2c_rq_s *done = NULL;

  if (!rq)
    goto out;

  if (nrf5x_i2c_request_progress(pv, rq))
    done = rq;

  if (!done)
    goto out;

  dev_request_queue_remove(&pv->queue, &done->base);

  rq = dev_i2c_rq_s_cast(dev_request_queue_head(&pv->queue));

  if (rq)
    nrf5x_i2c_request_do(pv, rq);

 out:
  lock_release(&dev->lock);

  dprintk("%s rq done %p\n", __FUNCTION__, done);

  if (done)
    kroutine_exec(&done->base.kr);
}

static DEV_I2C_REQUEST(nrf5x_i2c_request)
{
  struct device_s *dev = accessor->dev;
  struct nrf5x_i2c_priv_s *pv = dev->drv_pv;

  if (!req->transfer_count) {
    req->error = -ENOTSUP;
    kroutine_exec(&req->base.kr);
    return;
  }

  LOCK_SPIN_IRQ(&dev->lock);

  req->error = 0;
  req->error_transfer = 0;
  req->error_offset = 0;

  bool_t should_start = dev_request_queue_isempty(&pv->queue);

  dev_request_queue_pushback(&pv->queue, &req->base);

  if (should_start)
    nrf5x_i2c_request_do(pv, req);

  LOCK_RELEASE_IRQ(&dev->lock);
}

static DEV_I2C_CONFIG(nrf5x_i2c_config)
{
  struct device_s *dev = accessor->dev;
  struct nrf5x_i2c_priv_s *pv = dev->drv_pv;
  uint32_t rate = config->bit_rate;

  if (rate == 0)
    return -ENOTSUP;

  dprintk("%s %d\n", __FUNCTION__, rate);

  nrf_reg_set(pv->addr, NRF_I2C_FREQUENCY, NRF_I2C_FREQUENCY_(rate));

  return 0;
}

#define nrf5x_i2c_use dev_use_generic

static void nrf5x_i2c_reset(uintptr_t addr, uint8_t scl, uint8_t sda)
{
  nrf_event_clear(addr, NRF_I2C_ERROR);
  nrf_reg_set(addr, NRF_I2C_ENABLE, NRF_I2C_ENABLE_DISABLED);
  nrf_reg_set(addr, NRF_I2C_POWER, 0);
  for (uint16_t i = 0; i < 32; ++i)
    asm volatile("");

  nrf_reg_set(NRF5X_GPIO_ADDR, NRF_GPIO_OUTSET, 1 << sda);
  nrf_reg_set(NRF5X_GPIO_ADDR, NRF_GPIO_OUTSET, 1 << scl);
  for (uint16_t i = 0; i < 8; ++i)
    asm volatile("");

  uint32_t mask = (1 << scl) | (1 << sda);
  for (uint8_t i = 0; i < 18 && (nrf_reg_get(NRF5X_GPIO_ADDR, NRF_GPIO_IN) & mask) != mask; ++i) {
    nrf_reg_set(NRF5X_GPIO_ADDR, NRF_GPIO_OUTCLR, 1 << scl);
    for (uint16_t j = 0; j < 8; ++j)
      asm volatile("");
    nrf_reg_set(NRF5X_GPIO_ADDR, NRF_GPIO_OUTSET, 1 << scl);
    for (uint16_t j = 0; j < 8; ++j)
      asm volatile("");
  }

  nrf_reg_set(addr, NRF_I2C_POWER, 1);
  nrf_reg_set(addr, NRF_I2C_PSELSCL, scl);
  nrf_reg_set(addr, NRF_I2C_PSELSDA, sda);
  nrf_reg_set(addr, NRF_I2C_ENABLE, NRF_I2C_ENABLE_ENABLED);
}

static DEV_INIT(nrf5x_i2c_init)
{
  struct nrf5x_i2c_priv_s *pv;
  iomux_io_id_t id[2];
  error_t err = -ENOMEM;

  pv = mem_alloc(sizeof(*pv), mem_scope_sys);
  dev->drv_pv = pv;

  if (!pv)
    return -ENOMEM;

  if (device_res_get_uint(dev, DEV_RES_MEM, 0, &pv->addr, NULL)) {
    err = -EDESTADDRREQ;
    goto free_pv;
  }

  if (device_iomux_setup(dev, "_scl _sda", NULL, id, NULL)
      || id[0] == IOMUX_INVALID_ID
      || id[1] == IOMUX_INVALID_ID) {
    err = -ENOTCONN;
    goto free_pv;
  }

  nrf5x_i2c_reset(pv->addr, id[0], id[1]);

  nrf_reg_set(pv->addr, NRF_I2C_FREQUENCY, NRF_I2C_FREQUENCY_(100000));

  dev_request_queue_init(&pv->queue);

  nrf_it_set_mask(pv->addr, 0);

  device_irq_source_init(dev, pv->irq_ep, 1, &nrf5x_i2c_irq);

  if (device_irq_source_link(dev, pv->irq_ep, 1, -1)) {
    err = -EHOSTUNREACH;
    goto free_pv;
  }

  return 0;

 free_pv:
  mem_free(pv);
  return err;
}

static DEV_CLEANUP(nrf5x_i2c_cleanup)
{
  struct nrf5x_i2c_priv_s *pv = dev->drv_pv;

  if (!dev_request_queue_isempty(&pv->queue))
    return -EBUSY;

  nrf_reg_set(pv->addr, NRF_I2C_ENABLE, NRF_I2C_ENABLE_DISABLED);

  nrf_it_set_mask(pv->addr, 0);

  device_irq_source_unlink(dev, pv->irq_ep, 1);

  nrf_reg_set(pv->addr, NRF_I2C_PSELSCL, (uint32_t)-1);
  nrf_reg_set(pv->addr, NRF_I2C_PSELSDA, (uint32_t)-1);

  dev_request_queue_destroy(&pv->queue);

  mem_free(pv);

  return 0;
}

DRIVER_DECLARE(nrf5x_i2c_drv, 0, "nRF5x i2c", nrf5x_i2c,
               DRIVER_I2C_METHODS(nrf5x_i2c));

DRIVER_REGISTER(nrf5x_i2c_drv);


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

    Copyright (c) Nicolas Pouillon <nipo@ssji.net>, 2016
*/

#include <hexo/types.h>
#include <hexo/endian.h>
#include <hexo/iospace.h>
#include <hexo/interrupt.h>
#include <hexo/bit.h>

#include <mutek/mem_alloc.h>
#include <mutek/printk.h>

#include <device/device.h>
#include <device/resources.h>
#include <device/driver.h>
#include <device/irq.h>
#include <device/class/i2c.h>
#include <device/class/iomux.h>

#include <arch/nrf5x/twim.h>
#include <arch/nrf5x/gpio.h>

//#define dprintk(k...) do {} while (0)
#define dprintk printk

struct nrf5x_twim_priv_s
{
  uintptr_t addr;
  struct dev_irq_src_s irq_ep[1];
  struct dev_i2c_ctrl_context_s i2c_ctrl_ctx;
  struct dev_i2c_ctrl_transfer_s *current;
  bool_t started;
  iomux_io_id_t pin[2];
  uint32_t rate;
  uint32_t tmp[4];
};

DRIVER_PV(struct nrf5x_twim_priv_s);

static
void nrf5x_twim_transfer_error(struct nrf5x_twim_priv_s *pv)
{
  struct dev_i2c_ctrl_transfer_s *tr = pv->current;

  uint32_t error = nrf_reg_get(pv->addr, NRF_TWIM_ERRORSRC);

  dprintk("%s %p src %d\n", __FUNCTION__, tr, error);

  nrf_reg_set(pv->addr, NRF_TWIM_ERRORSRC, error);

  if (!tr)
    return;

  if (error & NRF_TWIM_ERRORSRC_ANACK)
    tr->err = -EHOSTUNREACH;
  else if (error & NRF_TWIM_ERRORSRC_DNACK)
    tr->err = -EAGAIN;

  nrf_task_trigger(pv->addr, NRF_TWIM_STOP);
}

static
void nrf5x_twim_tx_setup(struct nrf5x_twim_priv_s *pv)
{
  struct dev_i2c_ctrl_transfer_s *tr = pv->current;
  size_t count;

  assert(tr && tr->type & _DEV_I2C_WRITE_OP);

  if (!tr->size
      && !(tr->type & _DEV_I2C_STOP_CONDITION)) {
    dprintk("%s %p done, err=%d\n", __FUNCTION__, tr, tr->err);
    kroutine_exec(&tr->kr);
    pv->current = NULL;
    return;
  }

  if ((uintptr_t)tr->data < 0x20000000) {
    uintptr_t offset = (uintptr_t)tr->data % sizeof(pv->tmp);
    uint32_t *src = (uint32_t *)((uintptr_t)tr->data - offset);

    for (size_t i = offset / 4; i < sizeof(pv->tmp) / 4; ++i)
      pv->tmp[i] = src[i];

    nrf_reg_set(pv->addr, NRF_TWIM_TXD_PTR, (uintptr_t)pv->buffer_out.byte + offset);
    count = __MIN(count, sizeof(pv->buffer_out) - offset);
  } else {
    nrf_reg_set(pv->addr, NRF_TWIM_TXD_PTR, (uintptr_t)tr->data);
    count = tr->size;
  }
  
  nrf_reg_set(pv->addr, NRF_TWIM_TXD_MAXCOUNT, count);

  tr->data += count;
  tr->size -= count;

  switch (tr->type) {
  case DEV_I2C_WRITE_STOP:
    if (tr->size == 0) {
      dprintk("%s %p stopping after write\n", __FUNCTION__, tr);
      nrf_short_set(pv->addr, bit(NRF_TWIM_LASTTX_STOP));
      break;
    }
  default:
  case DEV_I2C_WRITE_RESTART:
  case DEV_I2C_WRITE:
    dprintk("%s %p suspending after write\n", __FUNCTION__, tr);
    nrf_short_set(pv->addr, bit(NRF_TWIM_LASTTX_SUSPEND));
    break;
  }

  if (pv->started) {
    nrf_task_trigger(pv->addr, NRF_TWIM_RESUME);
  } else {
    nrf_task_trigger(pv->addr, NRF_TWIM_STARTTX);
    pv->started = 1;
  }
}

static
void nrf5x_twim_rx_setup(struct nrf5x_twim_priv_s *pv)
{
  struct dev_i2c_ctrl_transfer_s *tr = pv->current;

  assert(tr && tr->type & _DEV_I2C_READ_OP);

  if (!tr->size
      && !(tr->type & _DEV_I2C_STOP_CONDITION)) {
    dprintk("%s %p done, err=%d\n", __FUNCTION__, tr, tr->err);
    kroutine_exec(&tr->kr);
    pv->current = NULL;
    return;
  }

  nrf_reg_set(pv->addr, NRF_TWIM_TXD_PTR, (uintptr_t)tr->data);
  nrf_reg_set(pv->addr, NRF_TWIM_TXD_MAXCOUNT, tr->size);

  switch (tr->type) {
  default:
  case DEV_I2C_READ_STOP:
    dprintk("%s %p stopping after write\n", __FUNCTION__, tr);
    nrf_short_set(pv->addr, bit(NRF_TWIM_LASTRX_STOP));
    break;
  }

  if (pv->started) {
    nrf_task_trigger(pv->addr, NRF_TWIM_RESUME);
  } else {
    nrf_task_trigger(pv->addr, NRF_TWIM_STARTRX);
    pv->started = 1;
  }
}

static
void nrf5x_twim_transfer_start(struct nrf5x_twim_priv_s *pv)
{
  struct dev_i2c_ctrl_transfer_s *tr = pv->current;

  tr->err = 0;

  nrf_reg_set(pv->addr, NRF_TWIM_ADDR, tr->saddr);

  dprintk("%s %p %s %s%s%s%s saddr %02x, %d bytes\n", __FUNCTION__, tr,
          pv->started ? "continue" : "starting",
          tr->type & _DEV_I2C_READ_OP ? "read," : "",
          tr->type & _DEV_I2C_WRITE_OP ? "write," : "",
          tr->type & _DEV_I2C_STOP_CONDITION ? "stop," : "",
          tr->type & _DEV_I2C_RESTART_CONDITION ? "restart," : "",
          tr->saddr, tr->size);

  nrf_it_set_mask(pv->addr, 0
                  | bit(NRF_TWIM_STOPPED)
                  | bit(NRF_TWIM_SUSPENDED)
                  | bit(NRF_TWIM_ERROR));

  if (tr->type & _DEV_I2C_WRITE_OP) {
    nrf5x_twim_tx_setup(pv);
  } else if (tr->type & _DEV_I2C_READ_OP) {
    nrf5x_twim_rx_setup(pv);
  }
}

static DEV_IRQ_SRC_PROCESS(nrf5x_twim_irq)
{
  struct device_s *dev = ep->base.dev;
  struct nrf5x_twim_priv_s *pv = dev->drv_pv;

  lock_spin(&dev->lock);

  if (nrf_event_check(pv->addr, NRF_TWIM_ERROR)) {
    nrf_event_clear(pv->addr, NRF_TWIM_ERROR);
    nrf5x_twim_transfer_error(pv);
  }

  if (nrf_event_check(pv->addr, NRF_TWIM_SUSPENDED)) {
    nrf_event_clear(pv->addr, NRF_TWIM_SUSPENDED);

    if (tr) {
      if (tr->type & _DEV_I2C_WRITE_OP) {
        nrf5x_twim_tx_setup(pv);
      } else if (tr->type & _DEV_I2C_READ_OP) {
        nrf5x_twim_rx_setup(pv);
      }
    }
  }

  if (nrf_event_check(pv->addr, NRF_TWIM_STOPPED)) {
    struct dev_i2c_ctrl_transfer_s *tr = pv->current;

    dprintk("%s %p stopped\n", __FUNCTION__, tr);

    nrf_event_clear(pv->addr, NRF_TWIM_STOPPED);
    pv->started = 0;
    if (tr && (tr->err
               || ((tr->type & _DEV_I2C_STOP_CONDITION)
                   && !(tr->type & _DEV_I2C_RESTART_CONDITION)))) {
      dprintk("%s %p kroutine_exec(), err=%d\n", __FUNCTION__, tr, tr->err);
      kroutine_exec(&tr->kr);
      pv->current = NULL;
    }
  }

  lock_release(&dev->lock);
}

static DEV_I2C_CTRL_TRANSFER(nrf5x_twim_transfer)
{
  struct device_s *dev = accessor->dev;
  struct nrf5x_twim_priv_s *pv = dev->drv_pv;

  switch (tr->type) {
  case DEV_I2C_READ:
  case DEV_I2C_READ_RESTART:
    tr->err = -ENOTSUP;
    dprintk("%s %p read restart unsupported, kroutine_exec(), err=%d\n", __FUNCTION__, tr, tr->err);
    kroutine_exec(&tr->kr);
    return;

  default:
    break;
  }

  if (tr->size == 0) {
    tr->err = -EINVAL;
    dprintk("%s %p len=0, kroutine_exec(), err=%d\n", __FUNCTION__, tr, tr->err);
    kroutine_exec(&tr->kr);
    return;
  }

  LOCK_SPIN_IRQ(&dev->lock);

  pv->current = tr;

  nrf5x_twim_transfer_start(pv);

  LOCK_RELEASE_IRQ(&dev->lock);
}

static void nrf5x_twim_ip_reset(struct nrf5x_twim_priv_s *pv)
{
  uint8_t scl = pv->pin[0];
  uint8_t sda = pv->pin[1];

  nrf_event_clear(pv->addr, NRF_TWIM_ERROR);
  nrf_reg_set(pv->addr, NRF_TWIM_ENABLE, NRF_TWIM_ENABLE_DISABLED);
  nrf_reg_set(pv->addr, NRF_TWIM_POWER, 0);
  for (uint16_t i = 0; i < 32; ++i)
    asm volatile("");

  nrf_reg_set(NRF5X_GPIO_ADDR, NRF_GPIO_OUTSET, bit(sda));
  nrf_reg_set(NRF5X_GPIO_ADDR, NRF_GPIO_OUTSET, bit(scl));
  for (uint16_t i = 0; i < 8; ++i)
    asm volatile("");

  nrf_reg_set(pv->addr, NRF_TWIM_POWER, 1);
  nrf_reg_set(pv->addr, NRF_TWIM_ENABLE, NRF_TWIM_ENABLE_ENABLED);
  nrf_reg_set(pv->addr, NRF_TWIM_PSELSCL, scl);
  nrf_reg_set(pv->addr, NRF_TWIM_PSELSDA, sda);
  nrf_reg_set(pv->addr, NRF_TWIM_FREQUENCY, NRF_TWIM_FREQUENCY_(pv->rate));
}

static DEV_I2C_CTRL_RESET(nrf5x_twim_reset)
{
  struct device_s *dev = accessor->dev;
  struct nrf5x_twim_priv_s *pv = dev->drv_pv;

  dprintk("%s\n", __FUNCTION__);

  if (pv->current)
    return -EBUSY;

  dprintk("%s\n", __FUNCTION__);

  pv->started = 0;

  nrf5x_twim_ip_reset(pv);

  return 0;
}

#define nrf5x_twim_use dev_use_generic

static DEV_INIT(nrf5x_twim_init)
{
  struct nrf5x_twim_priv_s *pv;
  error_t err = -ENOMEM;

  pv = mem_alloc(sizeof(*pv), mem_scope_sys);

  if (!pv)
    return -ENOMEM;

  memset(pv, 0, sizeof(*pv));
  dev->drv_pv = pv;

  err = device_res_get_uint(dev, DEV_RES_I2C_BITRATE, 0, &pv->rate, NULL);
  if(err)
    goto free_pv;

  if (device_res_get_uint(dev, DEV_RES_MEM, 0, &pv->addr, NULL)) {
    err = -EDESTADDRREQ;
    goto free_pv;
  }

  if (device_iomux_setup(dev, "_scl _sda", NULL, &pv->pin[0], NULL)
      || pv->pin[0] == IOMUX_INVALID_ID
      || pv->pin[1] == IOMUX_INVALID_ID) {
    err = -ENOTCONN;
    goto free_pv;
  }

  nrf5x_twim_ip_reset(pv);

  nrf_it_set_mask(pv->addr, 0);

  device_irq_source_init(dev, pv->irq_ep, 1, &nrf5x_twim_irq);

  err = device_irq_source_link(dev, pv->irq_ep, 1, -1);
  if (err)
    goto free_pv;

  err = dev_i2c_context_init(dev, &pv->i2c_ctrl_ctx);
  if (err)
    goto unlink;

  return 0;

 unlink:
  device_irq_source_unlink(dev, pv->irq_ep, 1);
 free_pv:
  mem_free(pv);
 err_out:
  return err;
}

static DEV_CLEANUP(nrf5x_twim_cleanup)
{
  struct nrf5x_twim_priv_s *pv = dev->drv_pv;

  if (!dev_request_queue_isempty(&pv->i2c_ctrl_ctx.queue))
    return -EBUSY;

  nrf_reg_set(pv->addr, NRF_TWIM_ENABLE, NRF_TWIM_ENABLE_DISABLED);

  nrf_it_set_mask(pv->addr, 0);

  device_irq_source_unlink(dev, pv->irq_ep, 1);

  nrf_reg_set(pv->addr, NRF_TWIM_PSELSCL, (uint32_t)-1);
  nrf_reg_set(pv->addr, NRF_TWIM_PSELSDA, (uint32_t)-1);

  dev_i2c_context_cleanup(&pv->i2c_ctrl_ctx);

  mem_free(pv);

  return 0;
}

DRIVER_DECLARE(nrf5x_twim_drv, 0, "nRF52 twim", nrf5x_twim,
               DRIVER_I2C_CTRL_METHODS(nrf5x_twim));

DRIVER_REGISTER(nrf5x_twim_drv);

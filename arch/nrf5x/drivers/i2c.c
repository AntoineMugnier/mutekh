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
#include <hexo/bit.h>

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

struct nrf5x_i2c_priv_s
{
  uintptr_t addr;
  struct dev_irq_src_s irq_ep[1];
  struct dev_i2c_ctrl_context_s i2c_ctrl_ctx;
  struct dev_i2c_ctrl_transfer_s *current;
  bool_t started;
  iomux_io_id_t pin[2];
  uint32_t rate;
};

DRIVER_PV(struct nrf5x_i2c_priv_s);

static
void nrf5x_i2c_transfer_error(struct nrf5x_i2c_priv_s *pv)
{
  struct dev_i2c_ctrl_transfer_s *tr = pv->current;

  uint32_t error = nrf_reg_get(pv->addr, NRF_I2C_ERRORSRC);

  dprintk("%s %p src %d\n", __FUNCTION__, tr, error);

  nrf_reg_set(pv->addr, NRF_I2C_ERRORSRC, error);

  if (!tr)
    return;

  if (error & NRF_I2C_ERRORSRC_ANACK)
    tr->err = -EHOSTUNREACH;
  else if (error & NRF_I2C_ERRORSRC_DNACK)
    tr->err = -EAGAIN;
  else
    tr->err = -EIO;

  nrf_task_trigger(pv->addr, NRF_I2C_STOP);
}

static
void nrf5x_i2c_tx_continue(struct nrf5x_i2c_priv_s *pv)
{
  struct dev_i2c_ctrl_transfer_s *tr = pv->current;

  if (!tr)
    return;

  assert(!(tr->type & _DEV_I2C_READ_OP));

  tr->data++;
  tr->size--;

  if (tr->size == 0) {
    dprintk("%s %p TX done %x\n", __FUNCTION__, tr, tr->type);

    tr->err = 0;

    switch (tr->type & _DEV_I2C_ENDING_MASK) {
    case _DEV_I2C_STOP:
      return;

    case _DEV_I2C_RESTART:
      pv->started = 0;

    case _DEV_I2C_CONTINUOUS:
      dprintk("%s %p kroutine_exec(), err=%d\n", __FUNCTION__, tr, tr->err);
      kroutine_exec(&tr->kr);
      pv->current = NULL;
      return;
    }
  }

  dprintk("%s %p TX %d bytes left: %02x\n", __FUNCTION__, tr,
          tr->size, tr->data[0]);

  if ((tr->type & _DEV_I2C_STOP) && tr->size == 1) {
    dprintk("%s %p stopping after next byte\n", __FUNCTION__, tr);
    nrf_short_set(pv->addr, bit(NRF_I2C_BB_STOP));
  }

  nrf_reg_set(pv->addr, NRF_I2C_TXD, tr->data[0]);
}

static
void nrf5x_i2c_rx_continue(struct nrf5x_i2c_priv_s *pv)
{
  struct dev_i2c_ctrl_transfer_s *tr = pv->current;

  if (!tr)
    return;

  assert(tr->type & _DEV_I2C_READ_OP);

  uint8_t data = nrf_reg_get(pv->addr, NRF_I2C_RXD);

  dprintk("%s %p RX %02x, %d bytes left\n", __FUNCTION__, tr, data, tr->size - 1);

  tr->data[0] = data;

  tr->data++;
  tr->size--;

  if (tr->size == 0) {
    dprintk("%s %p RX done %x\n", __FUNCTION__, tr, tr->type);

    tr->err = 0;

    switch (tr->type & _DEV_I2C_ENDING_MASK) {
    default:
    case _DEV_I2C_STOP:
      return;

    case _DEV_I2C_CONTINUOUS:
      dprintk("%s %p kroutine_exec(), err=%d\n", __FUNCTION__, tr, tr->err);
      kroutine_exec(&tr->kr);
      pv->current = NULL;
      return;
    }
  }

  if (tr->size == 1) {
    switch (tr->type & _DEV_I2C_ENDING_MASK) {
    default:
    case _DEV_I2C_CONTINUOUS:
      break;

    case _DEV_I2C_STOP:
      dprintk("%s %p stopping after next byte\n", __FUNCTION__, tr);
      nrf_short_set(pv->addr, bit(NRF_I2C_BB_STOP));
      break;
    }
  }

  nrf_task_trigger(pv->addr, NRF_I2C_RESUME);
}

static
void nrf5x_i2c_transfer_start(struct nrf5x_i2c_priv_s *pv)
{
  struct dev_i2c_ctrl_transfer_s *tr = pv->current;

  tr->err = 0;

  nrf_reg_set(pv->addr, NRF_I2C_ADDR, tr->saddr);

  dprintk("%s %p %s %s%s%s%s saddr %02x, %d bytes\n", __FUNCTION__, tr,
          pv->started ? "continue" : "starting",
          tr->type & _DEV_I2C_READ_OP ? "read," : "",
          tr->type & _DEV_I2C_WRITE_OP ? "write," : "",
          tr->type & _DEV_I2C_STOP_CONDITION ? "stop," : "",
          tr->type & _DEV_I2C_RESTART_CONDITION ? "restart," : "",
          tr->saddr, tr->size);

  if ((tr->type & _DEV_I2C_STOP) && tr->size == 1) {
    dprintk("%s %p stopping at end\n", __FUNCTION__, tr);
    nrf_short_set(pv->addr, bit(NRF_I2C_BB_STOP));
  } else if (!(tr->type & _DEV_I2C_READ_OP)) {
    nrf_short_set(pv->addr, 0);
  } else {
    nrf_short_set(pv->addr, bit(NRF_I2C_BB_SUSPEND));
  }

  if (tr->type & _DEV_I2C_READ_OP) {
    nrf_event_clear(pv->addr, NRF_I2C_RXDRDY);
    nrf_it_set_mask(pv->addr, 0
                    | bit(NRF_I2C_STOPPED)
                    | bit(NRF_I2C_ERROR)
                    | bit(NRF_I2C_RXDRDY));
  } else {
    nrf_event_clear(pv->addr, NRF_I2C_TXDSENT);
    nrf_it_set_mask(pv->addr, 0
                    | bit(NRF_I2C_STOPPED)
                    | bit(NRF_I2C_ERROR)
                    | bit(NRF_I2C_TXDSENT));
  }

  if (!pv->started) {
    if (tr->type & _DEV_I2C_READ_OP)
      nrf_task_trigger(pv->addr, NRF_I2C_STARTRX);
    else
      nrf_task_trigger(pv->addr, NRF_I2C_STARTTX);

    pv->started = 1;
  } else if (tr->type & _DEV_I2C_READ_OP) {
    nrf_task_trigger(pv->addr, NRF_I2C_RESUME);
  }

  if (!(tr->type & _DEV_I2C_READ_OP)) {
    dprintk("%s %p TX %02x\n", __FUNCTION__, tr, tr->data[0]);
    nrf_reg_set(pv->addr, NRF_I2C_TXD, tr->data[0]);
  }
}

static DEV_IRQ_SRC_PROCESS(nrf5x_i2c_irq)
{
  struct device_s *dev = ep->base.dev;
  struct nrf5x_i2c_priv_s *pv = dev->drv_pv;

  lock_spin(&dev->lock);

  if (nrf_event_check(pv->addr, NRF_I2C_ERROR)) {
    nrf_event_clear(pv->addr, NRF_I2C_ERROR);
    nrf5x_i2c_transfer_error(pv);
  }

  if (pv && nrf_event_check(pv->addr, NRF_I2C_TXDSENT)) {
    nrf_event_clear(pv->addr, NRF_I2C_TXDSENT);
    nrf5x_i2c_tx_continue(pv);
  }

  if (nrf_event_check(pv->addr, NRF_I2C_RXDRDY)) {
    nrf_event_clear(pv->addr, NRF_I2C_RXDRDY);
    nrf5x_i2c_rx_continue(pv);
  }

  if (nrf_event_check(pv->addr, NRF_I2C_STOPPED)) {
    struct dev_i2c_ctrl_transfer_s *tr = pv->current;

    dprintk("%s %p stopped\n", __FUNCTION__, tr);

    nrf_event_clear(pv->addr, NRF_I2C_STOPPED);
    pv->started = 0;
    if (tr && (tr->err || (tr->type & _DEV_I2C_STOP))) {
      dprintk("%s %p kroutine_exec(), err=%d\n", __FUNCTION__, tr, tr->err);
      kroutine_exec(&tr->kr);
      pv->current = NULL;
    }
  }

  lock_release(&dev->lock);
}

static void nrf5x_i2c_ip_reset(struct nrf5x_i2c_priv_s *pv)
{
  uint8_t scl = pv->pin[0];
  uint8_t sda = pv->pin[1];

  nrf_event_clear(pv->addr, NRF_I2C_ERROR);
  nrf_reg_set(pv->addr, NRF_I2C_ENABLE, NRF_I2C_ENABLE_DISABLED);
  nrf_reg_set(pv->addr, NRF_I2C_POWER, 0);
  for (uint16_t i = 0; i < 32; ++i)
    asm volatile("");

  nrf_reg_set(NRF5X_GPIO_ADDR, NRF_GPIO_OUTSET, bit(sda));
  nrf_reg_set(NRF5X_GPIO_ADDR, NRF_GPIO_OUTSET, bit(scl));
  for (uint16_t i = 0; i < 8; ++i)
    asm volatile("");

  uint32_t mask = bit(scl) | bit(sda);
  for (uint8_t i = 0; i < 18 && (nrf_reg_get(NRF5X_GPIO_ADDR, NRF_GPIO_IN) & mask) != mask; ++i) {
    nrf_reg_set(NRF5X_GPIO_ADDR, NRF_GPIO_OUTCLR, bit(scl));
    for (uint16_t j = 0; j < 8; ++j)
      asm volatile("");
    nrf_reg_set(NRF5X_GPIO_ADDR, NRF_GPIO_OUTSET, bit(scl));
    for (uint16_t j = 0; j < 8; ++j)
      asm volatile("");
  }

  nrf_reg_set(pv->addr, NRF_I2C_POWER, 1);
  nrf_reg_set(pv->addr, NRF_I2C_ENABLE, NRF_I2C_ENABLE_ENABLED);
  nrf_reg_set(pv->addr, NRF_I2C_PSELSCL, scl);
  nrf_reg_set(pv->addr, NRF_I2C_PSELSDA, sda);
  nrf_reg_set(pv->addr, NRF_I2C_FREQUENCY, NRF_I2C_FREQUENCY_(pv->rate));
}

static DEV_I2C_CTRL_TRANSFER(nrf5x_i2c_transfer)
{
  struct device_s *dev = accessor->dev;
  struct nrf5x_i2c_priv_s *pv = dev->drv_pv;

  switch (tr->type) {
  case DEV_I2C_RESET:
    dprintk("%s reset\n", __FUNCTION__);

    LOCK_SPIN_IRQ(&dev->lock);
    pv->started = 0;
    nrf5x_i2c_ip_reset(pv);
    LOCK_RELEASE_IRQ(&dev->lock);

    kroutine_exec(&tr->kr);
    return;

  case DEV_I2C_READ_RESTART:
  case DEV_I2C_READ_RESTART_SYNC:
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

  nrf5x_i2c_transfer_start(pv);

  LOCK_RELEASE_IRQ(&dev->lock);
}

#define nrf5x_i2c_use dev_use_generic

static DEV_INIT(nrf5x_i2c_init)
{
  struct nrf5x_i2c_priv_s *pv;
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

  nrf5x_i2c_ip_reset(pv);

  nrf_it_set_mask(pv->addr, 0);

  device_irq_source_init(dev, pv->irq_ep, 1, &nrf5x_i2c_irq);

  err = device_irq_source_link(dev, pv->irq_ep, 1, -1);
  if (err)
    goto free_pv;

  err = dev_drv_i2c_ctrl_context_init(dev, &pv->i2c_ctrl_ctx);
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

static DEV_CLEANUP(nrf5x_i2c_cleanup)
{
  struct nrf5x_i2c_priv_s *pv = dev->drv_pv;

  if (!dev_request_queue_isempty(&pv->i2c_ctrl_ctx.queue))
    return -EBUSY;

  nrf_reg_set(pv->addr, NRF_I2C_ENABLE, NRF_I2C_ENABLE_DISABLED);

  nrf_it_set_mask(pv->addr, 0);

  device_irq_source_unlink(dev, pv->irq_ep, 1);

  nrf_reg_set(pv->addr, NRF_I2C_PSELSCL, (uint32_t)-1);
  nrf_reg_set(pv->addr, NRF_I2C_PSELSDA, (uint32_t)-1);

  dev_drv_i2c_ctrl_context_cleanup(&pv->i2c_ctrl_ctx);

  mem_free(pv);

  return 0;
}

DRIVER_DECLARE(nrf5x_i2c_drv, 0, "nRF5x i2c", nrf5x_i2c,
               DRIVER_I2C_CTRL_METHODS(nrf5x_i2c));

DRIVER_REGISTER(nrf5x_i2c_drv);

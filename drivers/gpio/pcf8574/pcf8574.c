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

    Copyright (c) 2016, Nicolas Pouillon, <nipo@ssji.net>
*/

#include <hexo/bit.h>
#include <mutek/mem_alloc.h>
#include <mutek/printk.h>
#include <device/driver.h>
#include <device/request.h>
#include <device/class/gpio.h>
#include <device/class/i2c.h>

//#define dprintk printk
#ifndef dprintk
#define dprintk(x...) do {} while (0)
#endif

enum pcf8574_state_e
{
  STATE_IDLE,
  STATE_IO_GET,
  STATE_IO_SET,
  STATE_NOTIFY_READ,
};

struct pcf8574_priv_s
{
  struct dev_i2c_ctrl_transaction_rq_s i2c_txn;
  struct dev_i2c_ctrl_transaction_data_s i2c_transfer[1];
  struct dev_irq_src_s irq_ep;
  struct device_i2c_ctrl_s i2c;
  uint8_t input;
  uint8_t output;
  
  dev_request_queue_root_t queue;
  dev_request_queue_root_t until_queue;

  enum pcf8574_state_e state;
  bool_t evented;
  bool_t no_irq;
};

DRIVER_PV(struct pcf8574_priv_s);

static void pcf8574_handle_next(struct device_s *dev)
{
  struct pcf8574_priv_s *pv = dev->drv_pv;
  struct dev_gpio_rq_s *rq;

  if (pv->evented) {
    dprintk("%s evented, reading\n", __FUNCTION__);
    pv->evented = 0;

    pv->state = STATE_NOTIFY_READ;

    pv->i2c_transfer[0].data = &pv->input;
    pv->i2c_transfer[0].type = DEV_I2C_CTRL_TRANSACTION_READ;
    dev_i2c_transaction_start(&pv->i2c, &pv->i2c_txn);
    return;
  }

  rq = dev_gpio_rq_s_cast(dev_request_queue_head(&pv->queue));

  if (!rq)
    return;

  switch (rq->type) {
  case DEV_GPIO_SET_OUTPUT: {
    uint_fast8_t range = bit_range(rq->io_first, rq->io_last);
    uint_fast8_t set = (rq->output.set_mask[0] << rq->io_first) & range;
    uint_fast8_t clear = (rq->output.clear_mask[0] << rq->io_first) | ~range;

    pv->output = set ^ (pv->output & (set ^ clear));

    dprintk("%s %p set %d %d %02x %02x %02x\n", __FUNCTION__,
           rq, rq->io_first, rq->io_last, set, clear,
           pv->output);

    pv->i2c_transfer[0].data = &pv->output;
    pv->i2c_transfer[0].type = DEV_I2C_CTRL_TRANSACTION_WRITE;

    pv->state = STATE_IO_SET;
    dev_i2c_transaction_start(&pv->i2c, &pv->i2c_txn);
    return;
  }

  case DEV_GPIO_GET_INPUT:
    pv->i2c_transfer[0].data = &pv->input;
    pv->i2c_transfer[0].type = DEV_I2C_CTRL_TRANSACTION_READ;

    pv->state = STATE_IO_GET;
    dev_i2c_transaction_start(&pv->i2c, &pv->i2c_txn);
    return;

  default:
    assert(0);
  }
}

static KROUTINE_EXEC(pcf8574_i2c_done)
{
  struct pcf8574_priv_s *pv
    = KROUTINE_CONTAINER(kr, *pv, i2c_txn.base.base.kr);
  struct device_s *dev = pv->i2c_txn.base.base.pvdata;

  dprintk("%s\n", __FUNCTION__);

  LOCK_SPIN_IRQ(&dev->lock);
  switch(pv->state) {
  case STATE_NOTIFY_READ: {
    uint_fast8_t cur = pv->input;

    GCT_FOREACH(dev_request_queue, &pv->until_queue, brq, {
        struct dev_gpio_rq_s *rq = dev_gpio_rq_s_cast(brq);
        uint_fast8_t range = bit_range(rq->io_first, rq->io_last);
        uint_fast8_t mask = (rq->until.mask[0] << rq->io_first);
        uint_fast8_t data = (rq->until.data[0] << rq->io_first);

        dprintk("%s %p %02x %02x %02x %02x %02x\n",
                __FUNCTION__, rq, range, mask, data, cur,
                range & mask & (data ^ cur));

        if (range & mask & (data ^ cur)) {
          dev_request_queue_remove(&pv->until_queue, &rq->base);
          kroutine_exec(&rq->base.kr);
        }
      });
    // Dont break, notify read is actually an IO get: fallthroug is OK
  }

  case STATE_IO_GET:
    GCT_FOREACH(dev_request_queue, &pv->queue, brq, {
        struct dev_gpio_rq_s *rq = dev_gpio_rq_s_cast(brq);
        if (rq->type == DEV_GPIO_GET_INPUT) {
          uint_fast8_t range = bit_range(rq->io_first, rq->io_last);
          rq->input.data[0] = ((pv->input & range) >> rq->io_first);
          dprintk("%s %p io get done %02x\n", __FUNCTION__, rq, rq->input.data[0]);
          dev_request_queue_remove(&pv->queue, brq);
          kroutine_exec(&rq->base.kr);
        }
      });
    break;

  case STATE_IO_SET: {
    struct dev_gpio_rq_s *rq = dev_gpio_rq_s_cast(dev_request_queue_head(&pv->queue));
    if (rq && rq->type == DEV_GPIO_SET_OUTPUT) {
      dprintk("%s %p io set done\n", __FUNCTION__, rq);
      dev_request_queue_pop(&pv->queue);
      kroutine_exec(&rq->base.kr);
    }
    break;
  }

  default:
    break;
  }

  pv->state = STATE_IDLE;
  pcf8574_handle_next(dev);
  LOCK_RELEASE_IRQ(&dev->lock);
}

static DEV_GPIO_SET_MODE(pcf8574_set_mode)
{
  struct device_s *dev = gpio->dev;
  struct pcf8574_priv_s *pv = dev->drv_pv;

  switch (mode) {
  case DEV_PIN_PUSHPULL:
  case DEV_PIN_OPENDRAIN_PULLUP:
    return 0;
  default:
    return -ENOTSUP;
  }
}

static DEV_GPIO_REQUEST(pcf8574_request)
{
  struct device_s *dev = gpio->dev;
  struct pcf8574_priv_s *pv = dev->drv_pv;

  switch (req->type) {
  case DEV_GPIO_MODE:
    switch (req->mode.mode) {
    case DEV_PIN_PUSHPULL:
    case DEV_PIN_OPENDRAIN_PULLUP:
      req->error = 0;
      break;
    default:
      req->error = -ENOTSUP;
      break;
    }
    kroutine_exec(&req->base.kr);
    return;

  case DEV_GPIO_UNTIL:
    if (pv->no_irq) {
      req->error = -ENOTSUP;
      kroutine_exec(&req->base.kr);
      return;
    }
    break;

  default:
    break;
  }

  req->error = 0;

  LOCK_SPIN_IRQ(&dev->lock);
  switch (req->type) {
  default:
    dprintk("%s request %p\n", __FUNCTION__, req);
    dev_request_queue_pushback(&pv->queue, &req->base);
    break;

  case DEV_GPIO_UNTIL:
    dprintk("%s until %p\n", __FUNCTION__, req);
    uint_fast8_t range = bit_range(req->io_first, req->io_last);
    uint_fast8_t mask = (req->until.mask[0] << req->io_first);
    uint_fast8_t data = (req->until.data[0] << req->io_first);

    if (range & mask & (data ^ pv->input)) {
      req->error = 0;
      kroutine_exec(&req->base.kr);
      goto noop;
    } else {
      dev_request_queue_pushback(&pv->until_queue, &req->base);
    }
    break;
  }

  pcf8574_handle_next(dev);
 noop:
  LOCK_RELEASE_IRQ(&dev->lock);
}

static DEV_IRQ_SRC_PROCESS(pcf8574_irq)
{
  struct device_s *dev = ep->base.dev;
  struct pcf8574_priv_s *pv = dev->drv_pv;

  lock_spin(&dev->lock);
  dprintk("%s\n", __FUNCTION__);
  pv->evented = 1;
  pcf8574_handle_next(dev);
  lock_release(&dev->lock);
}

#define pcf8574_set_output (dev_gpio_set_output_t*)dev_driver_notsup_fcn
#define pcf8574_get_input (dev_gpio_get_input_t*)dev_driver_notsup_fcn
#define pcf8574_use dev_use_generic

static DEV_INIT(pcf8574_init)
{
  struct pcf8574_priv_s *pv;
  error_t err;

  pv = mem_alloc(sizeof(*pv), mem_scope_sys);
  if (!pv)
    return -ENOMEM;

  memset(pv, 0, sizeof(*pv));

  err = dev_drv_i2c_transaction_init(dev, &pv->i2c_txn, &pv->i2c);
  if (err)
    goto err_pv;

  device_irq_source_init(dev, &pv->irq_ep, 1, &pcf8574_irq);
  // Allow to perform without IRQ
  err = device_irq_source_link(dev, &pv->irq_ep, 1, -1);
  pv->no_irq = !!err;

  pv->i2c_txn.base.base.pvdata = dev;
  pv->i2c_txn.transfer = pv->i2c_transfer;
  pv->i2c_txn.transfer_count = 1;
  pv->i2c_transfer[0].size = 1;

  kroutine_init_deferred(&pv->i2c_txn.base.base.kr, pcf8574_i2c_done);
  
  dev_request_queue_init(&pv->queue);
  dev_request_queue_init(&pv->until_queue);

  dev->drv_pv = pv;

  return 0;

 err_pv:
  mem_free(pv);

  return err;
}

static DEV_CLEANUP(pcf8574_cleanup)
{
  struct pcf8574_priv_s *pv = dev->drv_pv;

  if (!dev_request_queue_isempty(&pv->queue))
    return -EBUSY;

  if (!dev_request_queue_isempty(&pv->until_queue))
    return -EBUSY;

  dev_drv_i2c_transaction_cleanup(&pv->i2c, &pv->i2c_txn);
  dev_request_queue_destroy(&pv->queue);
  dev_request_queue_destroy(&pv->until_queue);
  mem_free(pv);

  return 0;
}

DRIVER_DECLARE(pcf8574_drv, 0, "PCF8574 GPIO", pcf8574,
               DRIVER_GPIO_METHODS(pcf8574));

DRIVER_REGISTER(pcf8574_drv);

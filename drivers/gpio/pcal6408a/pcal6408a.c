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

#define LOGK_MODULE_ID "6408"

#include <hexo/bit.h>
#include <mutek/mem_alloc.h>
#include <mutek/printk.h>
#include <device/driver.h>
#include <device/request.h>
#include <device/class/gpio.h>
#include <device/class/i2c.h>

#include "pcal6408a_io.o.h"

enum pcal6408a_state_e
{
  STATE_INIT,
  STATE_IDLE,
  STATE_IO_GET,
  STATE_IO_SET,
  STATE_NOTIFY_UPDATE,
  STATE_NOTIFY_READ,
};

struct pcal6408a_priv_s
{
  struct device_i2c_ctrl_s i2c;
  struct dev_irq_src_s irq_ep;
  struct dev_i2c_ctrl_bytecode_rq_s i2c_rq;
  
  dev_request_queue_root_t queue;
  dev_request_queue_root_t until_queue;

  enum pcal6408a_state_e state;
  bool_t until_mask_dirty;
  bool_t until_evented;
};

DRIVER_PV(struct pcal6408a_priv_s);

static void pcal6408a_handle_next(struct device_s *dev)
{
  struct pcal6408a_priv_s *pv = dev->drv_pv;
  struct dev_gpio_rq_s *rq;

  if (pv->state != STATE_IDLE)
    return;

  if (pv->until_evented) {
    logk_debug("%s evented, reading", __FUNCTION__);
    pv->until_evented = 0;
    pv->state = STATE_NOTIFY_READ;
    dev_i2c_bytecode_start(&pv->i2c, &pv->i2c_rq, &pcal6408a_get_input, 0);
    return;
  }

  if (pv->until_mask_dirty) {
    uint_fast8_t mask = 0;
    uint_fast8_t value = 0;

    logk_debug("%s util mask dirty", __FUNCTION__);

    GCT_FOREACH(dev_request_queue, &pv->until_queue, brq, {
        struct dev_gpio_rq_s *rq = dev_gpio_rq_s_cast(brq);
        uint_fast8_t range = bit_range(rq->io_first, rq->io_last);
        mask |= (rq->until.mask[0] << rq->io_first) & range;
        value |= (rq->until.data[0] << rq->io_first) & range;
      });

    pv->state = STATE_NOTIFY_UPDATE;
    pv->until_mask_dirty = 0;
    dev_i2c_bytecode_start(&pv->i2c, &pv->i2c_rq, &pcal6408a_until,
                           PCAL6408A_UNTIL_BCARGS(mask));

    return;
  }

  rq = dev_gpio_rq_head(&pv->queue);

  if (!rq)
    return;

  logk_debug("%s handling %p %d", __FUNCTION__, rq, rq->type);

  switch (rq->type) {
  case DEV_GPIO_MODE: {
    uint8_t mask = (rq->mode.mask[0] << rq->io_first) & bit_range(rq->io_first, rq->io_last);
    enum dev_pin_driving_e mode = rq->mode.mode;

    if (mode & (DEV_PIN_DRIVE_UP_ | DEV_PIN_DRIVE_DOWN_))
      bc_set_reg(&pv->i2c_rq.vm, PCAL6408A_SET_MODE_BCIN_INPUT_DIRECTION,
                 bc_get_reg(&pv->i2c_rq.vm, PCAL6408A_SET_MODE_BCIN_INPUT_DIRECTION)
                 & ~mask);
    else
      bc_set_reg(&pv->i2c_rq.vm, PCAL6408A_SET_MODE_BCIN_INPUT_DIRECTION,
                 bc_get_reg(&pv->i2c_rq.vm, PCAL6408A_SET_MODE_BCIN_INPUT_DIRECTION)
                 | mask);

    if (mode & (DEV_PIN_RESISTOR_UP_ | DEV_PIN_RESISTOR_DOWN_)) {
      bc_set_reg(&pv->i2c_rq.vm, PCAL6408A_SET_MODE_BCIN_PULL_ENABLE,
                 bc_get_reg(&pv->i2c_rq.vm, PCAL6408A_SET_MODE_BCIN_PULL_ENABLE)
                 | mask);

      if (mode & DEV_PIN_RESISTOR_UP_)
        bc_set_reg(&pv->i2c_rq.vm, PCAL6408A_SET_MODE_BCIN_PULL_UP,
                 bc_get_reg(&pv->i2c_rq.vm, PCAL6408A_SET_MODE_BCIN_PULL_UP)
                 | mask);
      else
        bc_set_reg(&pv->i2c_rq.vm, PCAL6408A_SET_MODE_BCIN_PULL_UP,
                 bc_get_reg(&pv->i2c_rq.vm, PCAL6408A_SET_MODE_BCIN_PULL_UP)
                 & ~mask);
    }

    pv->state = STATE_IO_SET;
    dev_i2c_bytecode_start(&pv->i2c, &pv->i2c_rq, &pcal6408a_set_mode, 0);
    return;
  }

  case DEV_GPIO_SET_OUTPUT: {
    uint_fast8_t range = bit_range(rq->io_first, rq->io_last);
    uint_fast8_t set = (rq->output.set_mask[0] << rq->io_first) & range;
    uint_fast8_t clear = (rq->output.clear_mask[0] << rq->io_first) | ~range;

    bc_set_reg(&pv->i2c_rq.vm, PCAL6408A_SET_OUTPUT_BCIN_OUTPUT_PORT,
               set ^ (bc_get_reg(&pv->i2c_rq.vm, PCAL6408A_SET_OUTPUT_BCIN_OUTPUT_PORT)
                      & (set ^ clear)));

    pv->state = STATE_IO_SET;
    dev_i2c_bytecode_start(&pv->i2c, &pv->i2c_rq, &pcal6408a_set_output, 0);
    return;
  }

  case DEV_GPIO_GET_INPUT:
    pv->state = STATE_IO_GET;
    dev_i2c_bytecode_start(&pv->i2c, &pv->i2c_rq, &pcal6408a_get_input, 0);
    return;

  default:
    assert(0);
  }
}

static KROUTINE_EXEC(pcal6408a_i2c_done)
{
  struct pcal6408a_priv_s *pv
    = KROUTINE_CONTAINER(kr, *pv, i2c_rq.base.base.kr);
  struct device_s *dev = pv->i2c_rq.pvdata;

  logk_debug("%s", __FUNCTION__);

  LOCK_SPIN_IRQ_SCOPED(&dev->lock);

  switch(pv->state) {
  case STATE_NOTIFY_READ: {
    uint_fast8_t cur = bc_get_reg(&pv->i2c_rq.vm, 0);

    GCT_FOREACH(dev_request_queue, &pv->until_queue, brq, {
        struct dev_gpio_rq_s *rq = dev_gpio_rq_s_cast(brq);
        uint_fast8_t range = bit_range(rq->io_first, rq->io_last);
        uint_fast8_t mask = (rq->until.mask[0] << rq->io_first);
        uint_fast8_t data = (rq->until.data[0] << rq->io_first);

        logk_debug("%s %p %02x %02x %02x %02x %02x",
                __FUNCTION__, rq, range, mask, data, cur,
                range & mask & (data ^ cur));

        if (range & mask & (data ^ cur)) {
          dev_gpio_rq_remove(&pv->until_queue, rq);
          dev_gpio_rq_done(rq);
        }
      });
    // Dont break, notify read is actually an IO get: fallthroug is OK
  }

  case STATE_IO_GET:
    GCT_FOREACH(dev_request_queue, &pv->queue, brq, {
        struct dev_gpio_rq_s *rq = dev_gpio_rq_s_cast(brq);
        if (rq->type == DEV_GPIO_GET_INPUT) {
          uint_fast8_t range = bit_range(rq->io_first, rq->io_last);
          rq->input.data[0] = ((bc_get_reg(&pv->i2c_rq.vm, 0) & range) >> rq->io_first);
          logk_debug("%s %p io get done %02x", __FUNCTION__, rq, rq->input.data[0]);
          dev_gpio_rq_remove(&pv->queue, rq);
          dev_gpio_rq_done(rq);
        }
      });
    break;

  case STATE_IO_SET: {
    struct dev_gpio_rq_s *rq = dev_gpio_rq_head(&pv->queue);
    if (rq && (rq->type == DEV_GPIO_SET_OUTPUT || rq->type == DEV_GPIO_MODE)) {
      logk_debug("%s %p io set done", __FUNCTION__, rq);
      dev_gpio_rq_pop(&pv->queue);
      dev_gpio_rq_done(rq);
    }
    break;
  }

  default:
    break;
  }

  pv->state = STATE_IDLE;
  pcal6408a_handle_next(dev);
}

static DEV_GPIO_REQUEST(pcal6408a_request)
{
  struct device_s *dev = gpio->dev;
  struct pcal6408a_priv_s *pv = dev->drv_pv;

  rq->error = 0;

  LOCK_SPIN_IRQ_SCOPED(&dev->lock);
  switch (rq->type) {
  default:
    logk_debug("%s request %p", __FUNCTION__, rq);
    dev_gpio_rq_pushback(&pv->queue, rq);
    break;

  case DEV_GPIO_UNTIL:
    logk_debug("%s until %p", __FUNCTION__, rq);
    dev_gpio_rq_pushback(&pv->until_queue, rq);
    pv->until_mask_dirty = 1;
    break;
  }

  pcal6408a_handle_next(dev);
}

static DEV_IRQ_SRC_PROCESS(pcal6408a_irq)
{
  struct device_s *dev = ep->base.dev;
  struct pcal6408a_priv_s *pv = dev->drv_pv;

  lock_spin(&dev->lock);
  logk_debug("%s", __FUNCTION__);
  pv->until_evented = 1;
  pcal6408a_handle_next(dev);
  lock_release(&dev->lock);
}

#define pcal6408a_cancel (dev_gpio_cancel_t*)&dev_driver_notsup_fcn
#define pcal6408a_set_mode (dev_gpio_set_mode_t*)dev_driver_notsup_fcn
#define pcal6408a_set_output (dev_gpio_set_output_t*)dev_driver_notsup_fcn
#define pcal6408a_get_input (dev_gpio_get_input_t*)dev_driver_notsup_fcn
#define pcal6408a_use dev_use_generic

static DEV_INIT(pcal6408a_init)
{
  struct pcal6408a_priv_s *pv;
  error_t err;

  pv = mem_alloc(sizeof(*pv), mem_scope_sys);
  if (!pv)
    return -ENOMEM;

  memset(pv, 0, sizeof(*pv));

  err = dev_drv_i2c_bytecode_init(dev, &pv->i2c_rq, &pcal6408a_bytecode,
                                  &pv->i2c, NULL, NULL);
  if (err)
    goto err_pv;

  device_irq_source_init(dev, &pv->irq_ep, 1, &pcal6408a_irq);
  err = device_irq_source_link(dev, &pv->irq_ep, 1, -1);
  if (err)
    goto err_i2c;

  dev_i2c_ctrl_rq_init(&pv->i2c_rq.base, pcal6408a_i2c_done);
  pv->i2c_rq.pvdata = dev;

  dev_rq_queue_init(&pv->queue);
  dev_rq_queue_init(&pv->until_queue);

  dev->drv_pv = pv;

  pv->state = STATE_INIT;
  dev_i2c_bytecode_start(&pv->i2c, &pv->i2c_rq, &pcal6408a_reset, 0);

  return 0;

 err_i2c:
  dev_drv_i2c_bytecode_cleanup(&pv->i2c, &pv->i2c_rq);
 err_pv:
  mem_free(pv);

  return err;
}

static DEV_CLEANUP(pcal6408a_cleanup)
{
  struct pcal6408a_priv_s *pv = dev->drv_pv;

  if (!dev_rq_queue_isempty(&pv->queue))
    return -EBUSY;

  dev_drv_i2c_bytecode_cleanup(&pv->i2c, &pv->i2c_rq);
  dev_rq_queue_destroy(&pv->queue);
  dev_rq_queue_destroy(&pv->until_queue);
  mem_free(pv);

  return 0;
}

DRIVER_DECLARE(pcal6408a_drv, 0, "PCAL6408A GPIO", pcal6408a,
               DRIVER_GPIO_METHODS(pcal6408a));

DRIVER_REGISTER(pcal6408a_drv);

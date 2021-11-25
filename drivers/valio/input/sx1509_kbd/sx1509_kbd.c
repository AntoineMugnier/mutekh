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

#include <device/resources.h>
#include <device/device.h>
#include <device/driver.h>
#include <device/irq.h>

#include <device/class/timer.h>
#include <device/class/icu.h>
#include <device/class/valio.h>
#include <device/valio/keyboard.h>

#include "sx1509_kbd.h"
#include "sx1509_kbd_i2c.o.h"

#define dprintk(x...) do{}while(0)
//#define dprintk printk

DRIVER_PV(struct sx1509_kbd_context_s);

static DEV_IRQ_SRC_PROCESS(sx1509_kbd_irq)
{
  struct device_s *dev = ep->base.dev;
  struct sx1509_kbd_context_s *pv = dev->drv_pv;

  switch (pv->state) {
  case STATE_IDLE:
    dev_i2c_bytecode_start(&pv->i2c, &pv->i2c_rq, &sx1509_kbd_read,
                           SX1509_KBD_READ_BCARGS());
    pv->state = STATE_READING;
    break;

  default:
    pv->state = STATE_IRQ_PENDING;
    break;
  }
}

static DEV_VALIO_REQUEST(sx1509_kbd_request)
{
  struct device_s *dev = accessor->dev;
  struct sx1509_kbd_context_s *pv = dev->drv_pv;

  dprintk("%s\n", __FUNCTION__);

  if (rq->attribute != VALIO_KEYBOARD_MAP) {
    rq->error = -EINVAL;
    dev_valio_rq_done(req);
    return;
  }

  switch (rq->type) {
  default:
    rq->error = -ENOTSUP;
    dev_valio_rq_done(req);
    return;

  case DEVICE_VALIO_WAIT_EVENT: {
    LOCK_SPIN_IRQ_SCOPED(&dev->lock);
    rq->error = 0;
    dev_valio_rq_pushback(&pv->queue, req);
    return;
  }

  case DEVICE_VALIO_READ:
    endian_le64_na_store(rq->data, pv->value_last);
    rq->error = 0;
    dev_valio_rq_done(req);
    return;
  }
}

static DEV_VALIO_CANCEL(sx1509_kbd_cancel)
{
  struct device_s *dev = accessor->dev;
  struct sx1509_kbd_context_s *pv = dev->drv_pv;

  LOCK_SPIN_IRQ_SCOPED(&dev->lock);

  GCT_FOREACH(dev_request_queue, &pv->queue, item,
              if (item == &rq->base) {
                dev_valio_rq_remove(&pv->queue, req);

                if (dev_rq_queue_isempty(&pv->queue))
                  device_sleep_schedule(dev);

                return 0;
              });

  return -ENOENT;
}

static DEV_USE(sx1509_kbd_use)
{
  switch (op) {
  case DEV_USE_SLEEP: {
    struct device_s *dev = param;
    struct sx1509_kbd_context_s *pv = dev->drv_pv;

    if (dev_rq_queue_isempty(&pv->queue)
        && pv->state == STATE_IDLE) {
      dev_i2c_bytecode_start(&pv->i2c, &pv->i2c_rq, &sx1509_kbd_disable,
                             SX1509_KBD_DISABLE_BCARGS());
      pv->state = STATE_DISABLING;
    }

    return 0;
  }

  default:
    return dev_use_generic(param, op);
  }
}

static void sx1509_kbd_update(struct sx1509_kbd_context_s *pv, uint16_t rowcol)
{
  if (pv->last_rowcol >= rowcol) {
    pv->value_cur = pv->value_next;
    pv->value_next = 0;
  }
    
  if (rowcol == 0) {
    pv->value_cur = 0;
    pv->value_next = 0;
  } else {
    uint32_t row = bit_ctz((rowcol >> 8) & bit_mask(0, pv->rows));
    uint32_t cols = rowcol & bit_mask(0, pv->cols);

    while (cols) {
      uint32_t col = bit_ctz(cols);

      BIT_SET(pv->value_next, row * pv->cols + col);

      BIT_CLEAR(cols, col);
    }
  }

  if (pv->value_last != pv->value_cur) {
    dprintk("Keyboard state: %04x %016llx\n", rowcol, pv->value_cur);
    pv->value_last = pv->value_cur;

    struct dev_valio_rq_s *vrq;
    while ((vrq = dev_valio_rq_pop(&pv->queue)))
      {
        endian_le64_na_store(vrq->data, pv->value_last);
        dev_valio_rq_done(vrq);
      }

    device_sleep_schedule(pv->i2c_rq.pvdata);
  }

  pv->last_rowcol = rowcol;

  if (pv->value_cur) {
    DEVICE_OP(pv->timer, cancel, &pv->timer_rq);
    DEVICE_OP(pv->timer, request, &pv->timer_rq);
  }
}

static KROUTINE_EXEC(sx1509_kbd_timeout)
{
  struct sx1509_kbd_context_s *pv = KROUTINE_CONTAINER(kr, *pv, timer_rq.base.kr);

  sx1509_kbd_update(pv, 0);
}

static KROUTINE_EXEC(sx1509_kbd_i2c_done)
{
  struct sx1509_kbd_context_s *pv = KROUTINE_CONTAINER(kr, *pv, i2c_rq.base.base.kr);

  switch (pv->state) {
  case STATE_IRQ_PENDING:
  case STATE_READING:
    sx1509_kbd_update(pv, ~bc_get_reg(&pv->i2c_rq.vm, 0));
    break;

  default:
    break;
  }

  switch (pv->state) {
  case STATE_IRQ_PENDING:
    dev_i2c_bytecode_start(&pv->i2c, &pv->i2c_rq, &sx1509_kbd_read,
                           SX1509_KBD_READ_BCARGS());
    pv->state = STATE_READING;
    break;

  case STATE_READING:
  default:
    pv->state = STATE_IDLE;
    break;

  case STATE_DISABLING:
    if (!dev_rq_queue_isempty(&pv->queue)) {
      dev_i2c_bytecode_start(&pv->i2c, &pv->i2c_rq, &sx1509_kbd_reset,
                             SX1509_KBD_DISABLE_BCARGS());
      pv->state = STATE_INIT;
      break;
    }
    
    pv->state = STATE_DISABLED;
    break;
  }
}

static DEV_INIT(sx1509_kbd_init)
{
  struct sx1509_kbd_context_s *pv;
  error_t err;
  uintptr_t tmp;
  dev_timer_delay_t init_delay;

  pv = mem_alloc(sizeof(*pv), mem_scope_sys);
  if (!pv)
    return -ENOMEM;

  memset(pv, 0, sizeof(*pv));

  dev->drv_pv = pv;

  err = device_get_param_uint(dev, "rows", &tmp);
  if (err)
    goto free_pv;
  pv->rows = tmp;

  err = device_get_param_uint(dev, "cols", &tmp);
  if (err)
    goto free_pv;
  pv->cols = tmp;

  dev_rq_queue_init(&pv->queue);

  device_irq_source_init(dev, &pv->irq_ep, 1, &sx1509_kbd_irq);
  err = device_irq_source_link(dev, &pv->irq_ep, 1, -1);
  if (err)
    goto free_pv;

  pv->state = STATE_INIT;
  err = dev_drv_i2c_bytecode_init(dev, &pv->i2c_rq, &sx1509_kbd_bytecode,
                                  &pv->i2c, NULL, &pv->timer);
  if (err)
    goto free_pv;

  dev_timer_init_sec(pv->timer, &init_delay, 0, 1, 100);

  pv->i2c_rq.pvdata = dev;

  dev_i2c_ctrl_rq_init(&pv->i2c_rq.base, &sx1509_kbd_i2c_done);
  dev_i2c_bytecode_start(&pv->i2c, &pv->i2c_rq, &sx1509_kbd_reset,
                         SX1509_KBD_RESET_BCARGS(pv->rows, pv->cols, init_delay));

  dev_timer_rq_init(&pv->timer_rq, &sx1509_kbd_timeout);
  dev_timer_init_sec(pv->timer, &pv->timer_rq.delay, 0, 1, 50);

  return 0;

 free_pv:
  mem_free(pv);
  return err;
}

static DEV_CLEANUP(sx1509_kbd_cleanup)
{
  struct sx1509_kbd_context_s *pv = dev->drv_pv;

  if (!dev_rq_queue_isempty(&pv->queue))
    return -EBUSY;

  device_irq_source_unlink(dev, &pv->irq_ep, 1);
  dev_rq_queue_destroy(&pv->queue);

  mem_free(pv);

  return 0;
}

DRIVER_DECLARE(sx1509_kbd_drv, 0, "SX1509 keyboard", sx1509_kbd,
               DRIVER_VALIO_METHODS(sx1509_kbd));

DRIVER_REGISTER(sx1509_kbd_drv);


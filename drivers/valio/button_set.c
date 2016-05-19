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

    Copyright (c) Nicolas Pouillon <nipo@ssji.net> 2015
*/

#include <hexo/types.h>
#include <hexo/endian.h>
#include <hexo/iospace.h>
#include <hexo/interrupt.h>

#include <mutek/mem_alloc.h>
#include <mutek/printk.h>

#include <device/resources.h>
#include <device/device.h>
#include <device/driver.h>
#include <device/irq.h>

#include <device/class/timer.h>
#include <device/class/icu.h>
#include <device/class/gpio.h>
#include <device/class/valio.h>
#include <device/valio/keyboard.h>

#define dprintk(x...) do{}while(0)
//#define dprintk printk

struct bs_context_s
{
  dev_request_queue_root_t queue;
  struct dev_irq_src_s irq_ep;

  struct device_gpio_s gpio;
  uint64_t mask;
  uint8_t *last_read_state, *cur_state;

  uint8_t first, last, range_id, count, state_size;
  bool_t active_low;
  bool_t changed;
};

static void bs_notif_disable(struct device_s *dev)
{
  struct bs_context_s *pv = dev->drv_pv;

  if (!(dev->start_count & 1))
    return;

  dprintk("%s\n", __FUNCTION__);

  dev->start_count &= ~1;

  DEVICE_OP(&pv->gpio, input_irq_range,
            pv->first, pv->last, (const uint8_t *)&pv->mask,
            0, pv->range_id);
}

static void bs_notif_enable(struct device_s *dev)
{
  struct bs_context_s *pv = dev->drv_pv;

  if (dev->start_count & 1)
    return;

  dprintk("%s\n", __FUNCTION__);

  dev->start_count |= 1;

  DEVICE_OP(&pv->gpio, input_irq_range,
            pv->first, pv->last, (const uint8_t *)&pv->mask,
            DEV_IRQ_SENSE_LOW_LEVEL | DEV_IRQ_SENSE_HIGH_LEVEL, pv->range_id);
}

static uint32_t bs_value_get(struct bs_context_s *pv)
{
  uint64_t state;

  DEVICE_OP(&pv->gpio, get_input, pv->first, pv->last, (uint8_t *)&state);

  if (pv->active_low)
    state = ~state;

  return state & pv->mask;
}

static bool_t bs_read_or_update(struct bs_context_s *pv,
                                struct dev_valio_rq_s *rq)
{
  uint64_t set = bs_value_get(pv);
  uint64_t mask = pv->mask;
  uint8_t button = 0;

  memset(pv->cur_state, 0, pv->state_size);

  while (mask) {
    uint8_t bit = __builtin_ctzll(mask);
    uint64_t m = 1ULL << bit;

    if (set & m)
      pv->cur_state[button / 8] |= 1 << (button & 7);

    mask &= ~m;
    button++;
  }

  pv->changed = 0;

  if (rq->type == DEVICE_VALIO_WAIT_EVENT
      && !memcmp(pv->cur_state, pv->last_read_state, pv->state_size))
    return 0;

  memcpy(pv->last_read_state, pv->cur_state, pv->state_size);
  memcpy(rq->data, pv->cur_state, pv->state_size);
  rq->error = 0;

  return 1;
}

static DEV_IRQ_SRC_PROCESS(bs_irq)
{
  struct device_s *dev = ep->base.dev;
  struct bs_context_s *pv = dev->drv_pv;
  struct dev_valio_rq_s *rq;

  dprintk("%s\n", __FUNCTION__);

  LOCK_SPIN_IRQ(&dev->lock);
  rq = dev_valio_rq_s_cast(dev_request_queue_head(&pv->queue));

  pv->changed = 1;

  while (rq) {
    if (bs_read_or_update(pv, rq) == 1) {
      dev_request_queue_remove(&pv->queue, &rq->base);

      kroutine_exec(&rq->base.kr);

      rq = dev_valio_rq_s_cast(dev_request_queue_head(&pv->queue));
    }

    if (!rq || rq->type != DEVICE_VALIO_READ)
      break;
  }

  if (!rq)
    device_sleep_schedule(dev);

  LOCK_RELEASE_IRQ(&dev->lock);
}

static DEV_VALIO_REQUEST(button_set_request)
{
  struct device_s *dev = accessor->dev;
  struct bs_context_s *pv = dev->drv_pv;
  bool_t queued = 0;
  req->error = 0;

  dprintk("%s\n", __FUNCTION__);

  if (req->attribute != VALIO_KEYBOARD_MAP) {
    req->error = -EINVAL;
    goto done;
  }

  switch (req->type) {
  default:
    req->error = -ENOTSUP;
    goto done;

  case DEVICE_VALIO_WAIT_EVENT:
    LOCK_SPIN_IRQ(&dev->lock);
    if (!pv->changed) {
      dev_request_queue_pushback(&pv->queue, &req->base);
      bs_notif_enable(dev);
      queued = 1;
    }
    LOCK_RELEASE_IRQ(&dev->lock);
    if (queued)
      return;

  case DEVICE_VALIO_READ:
    bs_read_or_update(pv, req);
    req->error = 0;
    goto done;
  }

 done:
  kroutine_exec(&req->base.kr);
}

static DEV_VALIO_CANCEL(button_set_cancel)
{
  struct device_s *dev = accessor->dev;
  struct bs_context_s *pv = dev->drv_pv;
  error_t err = -ENOENT;

  LOCK_SPIN_IRQ(&dev->lock);

  GCT_FOREACH(dev_request_queue, &pv->queue, item,
              if (item == &req->base) {
                err = 0;
                GCT_FOREACH_BREAK;
              });

  if (err == 0) {
    dev_request_queue_remove(&pv->queue, &req->base);

    device_sleep_schedule(dev);
  }

  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
}

static DEV_USE(button_set_use)
{
  switch (op) {
  case DEV_USE_SLEEP: {
    struct device_s *dev = param;
    struct bs_context_s *pv = dev->drv_pv;

    if (dev_request_queue_isempty(&pv->queue))
      bs_notif_disable(dev);

    return 0;
  }

  default:
    return dev_use_generic(param, op);
  }
}

static DEV_INIT(button_set_init)
{
  struct bs_context_s *pv;
  error_t err;
  const struct dev_resource_s *r;
  uintptr_t first, last, tmp;
  const void* mask;


  pv = mem_alloc(sizeof(*pv), mem_scope_sys);
  if (!pv)
    return -ENOMEM;

  memset(pv, 0, sizeof(*pv));

  dev->drv_pv = pv;

  err = device_get_param_dev_accessor(dev, "gpio", &pv->gpio.base, DRIVER_CLASS_GPIO);
  if (err)
    goto free_pv;

  r = device_res_get(dev, DEV_RES_IRQ, 0);
  if (!r) {
    err = -ENOENT;
    goto put_gpio;
  }

  pv->range_id = r->u.irq.sink_id;

  err = device_res_get_io(dev, 0, &first, &last);
  if (err)
    goto put_gpio;

  pv->first = first;
  pv->last = last;

  err = device_get_param_blob(dev, "mask", 0, &mask);
  if (err)
    return err;

  pv->mask = endian_le64_na_load(mask) & ((1ULL << (last - first + 1)) - 1);
  pv->count = __builtin_popcountll(pv->mask);
  pv->state_size = (pv->count + 7) / 8;

  pv->last_read_state = mem_alloc(pv->state_size * 2, mem_scope_sys);
  if (!pv->last_read_state) {
    err = -ENOMEM;
    goto put_gpio;
  }
  memset(pv->last_read_state, 0, pv->state_size * 2);
  pv->cur_state = pv->last_read_state + pv->state_size;

  dev_request_queue_init(&pv->queue);

  device_irq_source_init(dev, &pv->irq_ep, 1, &bs_irq);

  err = device_irq_source_link(dev, &pv->irq_ep, 1, -1);
  if (err)
    goto put_gpio;

  err = device_get_param_uint(dev, "active", &tmp);
  pv->active_low = !err && !tmp;

  DEVICE_OP(&pv->gpio, set_mode, pv->first, pv->last,
            (const uint8_t *)&pv->mask,
            pv->active_low ? DEV_PIN_INPUT_PULLUP : DEV_PIN_INPUT_PULLDOWN);


  return 0;

 put_gpio:
  device_put_accessor(&pv->gpio.base);
 free_pv:
  mem_free(pv);
  return err;
}

static DEV_CLEANUP(button_set_cleanup)
{
  struct bs_context_s *pv = dev->drv_pv;

  if (!dev_request_queue_isempty(&pv->queue))
    return -EBUSY;

  device_put_accessor(&pv->gpio.base);

  device_irq_source_unlink(dev, &pv->irq_ep, 1);
  dev_request_queue_destroy(&pv->queue);

  mem_free(pv->last_read_state);
  mem_free(pv);

  return 0;
}

DRIVER_DECLARE(button_set_drv, 0, "Button-set keyboard", button_set,
               DRIVER_VALIO_METHODS(button_set));

DRIVER_REGISTER(button_set_drv);


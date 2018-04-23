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

#define LOGK_MODULE_ID "btns"

#include <hexo/types.h>
#include <hexo/endian.h>
#include <hexo/iospace.h>
#include <hexo/bit.h>

#include <mutek/mem_alloc.h>
#include <mutek/printk.h>

#include <device/resources.h>
#include <device/device.h>
#include <device/driver.h>

#include <device/class/timer.h>
#include <device/class/icu.h>
#include <device/class/gpio.h>
#include <device/class/valio.h>
#include <device/valio/keyboard.h>

struct bs_context_s
{
  dev_request_queue_root_t queue;
  struct device_gpio_s gpio;
  struct dev_gpio_rq_s gpio_rq;

  uint8_t mask[8];
  uint8_t cur[8];

  bool_t active_high;
  bool_t busy;
  uint8_t state_size;
};

DRIVER_PV(struct bs_context_s);

static void bs_state_read(struct bs_context_s *pv,
                          struct dev_valio_rq_s *rq)
{
  uint64_t mask = endian_le64_na_load(pv->mask);
  uint64_t value = endian_le64_na_load(pv->cur);
  uint8_t button = 0;

  memset(rq->data, 0, pv->state_size);

  while (mask) {
    uint8_t b = bit_ctz64(mask);

    if (bit_get(value, b) == pv->active_high)
      ((uint8_t *)rq->data)[button / 8] |= 1 << (button & 7);

    BIT_CLEAR(mask, b);
    button++;
  }

  logk_trace("%s %llx %llx %P", __FUNCTION__, mask, value,
             rq->data, pv->state_size);
}

static void bs_gpio_read(struct device_s *dev)
{
  struct bs_context_s *pv = dev->drv_pv;

  pv->gpio_rq.type = DEV_GPIO_GET_INPUT;
  pv->gpio_rq.input.data = pv->cur;

  pv->busy = 1;
  DEVICE_OP(&pv->gpio, request, &pv->gpio_rq);
}

static void bs_gpio_wait(struct device_s *dev)
{
  struct bs_context_s *pv = dev->drv_pv;

  pv->gpio_rq.type = DEV_GPIO_UNTIL;
  pv->gpio_rq.until.mask = pv->mask;
  pv->gpio_rq.until.data = pv->cur;

  pv->busy = 1;
  DEVICE_OP(&pv->gpio, request, &pv->gpio_rq);
}

static KROUTINE_EXEC(bs_gpio_done)
{
  struct bs_context_s *pv = KROUTINE_CONTAINER(kr, *pv, gpio_rq.base.kr);
  struct device_s *dev = pv->gpio_rq.base.pvdata;

  LOCK_SPIN_IRQ(&dev->lock);

  pv->busy = 0;

  switch (pv->gpio_rq.type) {
  default:
  case DEV_GPIO_MODE:
    if (dev_request_queue_isempty(&pv->queue))
      break;

  case DEV_GPIO_GET_INPUT:
    bs_gpio_wait(dev);
    GCT_FOREACH(dev_request_queue, &pv->queue, brq, {
        struct dev_valio_rq_s *rq = dev_valio_rq_s_cast(brq);
        bs_state_read(pv, rq);
        dev_request_queue_remove(&pv->queue, &rq->base);
        kroutine_exec(&rq->base.kr);
      });
    break;

  case DEV_GPIO_UNTIL:
    bs_gpio_read(dev);
    break;
  }
  LOCK_RELEASE_IRQ(&dev->lock);
}

static DEV_VALIO_REQUEST(button_set_request)
{
  struct device_s *dev = accessor->dev;
  struct bs_context_s *pv = dev->drv_pv;
  bool_t was_empty = 0;

  logk_trace("%s", __FUNCTION__);

  if (req->attribute != VALIO_KEYBOARD_MAP) {
    req->error = -EINVAL;
    goto done;
  }

  switch (req->type) {
  default:
    req->error = -ENOTSUP;
    goto done;

  case DEVICE_VALIO_WAIT_EVENT:
    req->error = 0;
    LOCK_SPIN_IRQ(&dev->lock);
    was_empty = dev_request_queue_isempty(&pv->queue);
    dev_request_queue_pushback(&pv->queue, &req->base);
    if (was_empty && !pv->busy)
      bs_gpio_wait(dev);
    LOCK_RELEASE_IRQ(&dev->lock);
    return;

  case DEVICE_VALIO_READ:
    bs_state_read(pv, req);
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

#define button_set_use dev_use_generic

static DEV_INIT(button_set_init)
{
  struct bs_context_s *pv;
  error_t err;
  uintptr_t tmp;
  gpio_id_t id;
  gpio_width_t width;

  pv = mem_alloc(sizeof(*pv), mem_scope_sys);
  if (!pv)
    return -ENOMEM;

  memset(pv, 0, sizeof(*pv));

  dev->drv_pv = pv;

  err = device_get_param_dev_accessor(dev, "gpio", &pv->gpio.base, DRIVER_CLASS_GPIO);
  if (err) {
    logk_error("Cannot get GPIO accessor");
    goto free_pv;
  }

  err = device_gpio_get_setup(&pv->gpio, dev, "pins", &id, &width);
  if (err) {
    logk_error("No 'pins' param");
    goto put_gpio;
  }

  logk_trace("Keyboard pins: %d/%d", id, width);

  err = device_get_param_uint(dev, "mask", &tmp);
  if (err)
    tmp = (uintptr_t)dev_gpio_mask1;

  uint64_t mask = (*(uint64_t *)tmp) & bit_mask(0, width);
  endian_le64_na_store(pv->mask, mask);
  pv->state_size = (bit_popc64(mask) + 7) / 8;

  dev_request_queue_init(&pv->queue);

  err = device_get_param_uint(dev, "active", &tmp);
  pv->active_high = err || tmp;

  pv->gpio_rq.base.pvdata = dev;
  pv->gpio_rq.io_first = id;
  pv->gpio_rq.io_last = id + width - 1;
  kroutine_init_deferred(&pv->gpio_rq.base.kr, bs_gpio_done);
  pv->gpio_rq.type = DEV_GPIO_MODE;
  pv->gpio_rq.mode.mask = pv->mask;
  pv->gpio_rq.mode.mode = pv->active_high ? DEV_PIN_INPUT_PULLDOWN : DEV_PIN_INPUT_PULLUP;
  pv->busy = 1;

  memset(pv->cur, pv->active_high ? 0 : 0xff, sizeof(pv->cur));

  DEVICE_OP(&pv->gpio, request, &pv->gpio_rq);

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

  dev_request_queue_destroy(&pv->queue);

  mem_free(pv);

  return 0;
}

DRIVER_DECLARE(button_set_drv, 0, "Button-set keyboard", button_set,
               DRIVER_VALIO_METHODS(button_set));

DRIVER_REGISTER(button_set_drv);


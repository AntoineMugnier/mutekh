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

    Copyright (c) Nicolas Pouillon <nipo@ssji.net> 2021
*/

#define LOGK_MODULE_ID "qdgp"

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
#include <device/valio/position.h>

struct qdec_pv_s
{
  dev_request_queue_root_t queue;
  struct device_gpio_s gpio;
  struct dev_gpio_rq_s gpio_rq;

  uint8_t cur[8];

  uint8_t last_state;
  int32_t position;
};

DRIVER_PV(struct qdec_pv_s);

static void qdec_gpio_read(struct device_s *dev)
{
  struct qdec_pv_s *pv = dev->drv_pv;

  pv->gpio_rq.type = DEV_GPIO_GET_INPUT;
  pv->gpio_rq.input.data = pv->cur;
  
  DEVICE_OP(&pv->gpio, request, &pv->gpio_rq);
}

static void qdec_gpio_wait(struct device_s *dev)
{
  struct qdec_pv_s *pv = dev->drv_pv;

  pv->gpio_rq.type = DEV_GPIO_UNTIL;
  pv->gpio_rq.until.data = pv->cur;
  pv->gpio_rq.until.mask = dev_gpio_mask1;

  DEVICE_OP(&pv->gpio, request, &pv->gpio_rq);
}

static
void qdec_update(struct device_s *dev)
{
  struct qdec_pv_s *pv = dev->drv_pv;

  uint8_t last_state = pv->last_state;
  uint8_t state = pv->cur[0] & 3;
  uint8_t idx = ((last_state << 2) | state) & 0xf;
#define LUT_ENTRY(before, after, disp, err) ((uint64_t)(((disp) & 3) | ((err) << 2)) << (((before << 2) | (after)) * 4))
  static const uint64_t lut = 0ULL
    // No move
    | LUT_ENTRY(0, 0, 2, 0)
    | LUT_ENTRY(1, 1, 2, 0)
    | LUT_ENTRY(2, 2, 2, 0)
    | LUT_ENTRY(3, 3, 2, 0)
    // Decrements
    | LUT_ENTRY(0, 1, 1, 0)
    | LUT_ENTRY(1, 3, 1, 0)
    | LUT_ENTRY(3, 2, 1, 0)
    | LUT_ENTRY(2, 0, 1, 0)
    // Increments
    | LUT_ENTRY(1, 0, 3, 0)
    | LUT_ENTRY(3, 1, 3, 0)
    | LUT_ENTRY(2, 3, 3, 0)
    | LUT_ENTRY(0, 2, 3, 0)
    // Double moves, errors
    | LUT_ENTRY(2, 1, 2, 1)
    | LUT_ENTRY(2, 1, 2, 1)
    | LUT_ENTRY(0, 3, 2, 1)
    | LUT_ENTRY(3, 0, 2, 1)
    ;
  uint8_t lu = (lut >> (idx * 4)) & 0xf;
  bool_t err = !!(lu & 4);
  int32_t delta = (int32_t)(lu & 3) - 2;
  pv->position += delta;
  pv->last_state = state;
  (void)err;

  uint32_t pos = pv->position / 2;
  
  logk_trace("%d->%d, idx %d entry %x delta %d, err %d, position %d, pos %d",
             last_state, state, idx, lu,
             delta, err, pv->position, pos);
    
  GCT_FOREACH(dev_request_queue, &pv->queue, item, {
      struct dev_valio_rq_s *rq = dev_valio_rq_s_cast(item);
      int32_t *rq_value = rq->data;

      if (rq->type == DEVICE_VALIO_WAIT_EVENT
          && *rq_value == pos) {
        logk_debug("rq %p not changed", rq);
        GCT_FOREACH_CONTINUE;
      }

      *rq_value = pos;

      dev_valio_rq_remove(&pv->queue, rq);
      dev_valio_rq_done(rq);
    });
}

static KROUTINE_EXEC(qdec_gpio_done)
{
  struct qdec_pv_s *pv = KROUTINE_CONTAINER(kr, *pv, gpio_rq.base.kr);
  struct device_s *dev = pv->gpio_rq.pvdata;

  LOCK_SPIN_IRQ_SCOPED(&dev->lock);

  switch (pv->gpio_rq.type) {
  default:
  case DEV_GPIO_MODE:
    // Pin init done, read input once
    qdec_gpio_read(dev);
    break;

  case DEV_GPIO_GET_INPUT:
    // Init done
    qdec_gpio_wait(dev);
    qdec_update(dev);
    break;

  case DEV_GPIO_UNTIL:
    qdec_gpio_read(dev);
    break;
  }
}

static DEV_VALIO_REQUEST(qdec_request)
{
  struct device_s *dev = accessor->dev;
  struct qdec_pv_s *pv = dev->drv_pv;

  logk_debug("%s", __func__);

  LOCK_SPIN_IRQ_SCOPED(&dev->lock);

  if (rq->attribute != VALIO_POSITION_VALUE)
    goto inval;

  int32_t *rq_ptr = (int32_t*)rq->data;
  int32_t pos = pv->position / 2;

  switch (rq->type) {
  case DEVICE_VALIO_WAIT_EVENT:
    if (*rq_ptr == pos) {
      logk_debug("wait %d = %d queued", *rq_ptr, pos);
      rq->error = 0;
      dev_valio_rq_pushback(&pv->queue, rq);
      return;
    }

    // fallthrough
  case DEVICE_VALIO_READ:
    *rq_ptr = pos;
    goto done;

  case DEVICE_VALIO_WRITE:
    pv->position = *rq_ptr * 2;

  done:
    rq->error = 0;
    dev_valio_rq_done(rq);
    return;

  default:
  inval:
    rq->error = -EINVAL;
    logk_debug("inval");
    dev_valio_rq_done(rq);
    return;
  }
}

static DEV_VALIO_CANCEL(qdec_cancel)
{
  struct device_s *dev = accessor->dev;
  struct qdec_pv_s *pv = dev->drv_pv;

  logk_trace("%s %p", __func__, rq);

  LOCK_SPIN_IRQ_SCOPED(&dev->lock);

  GCT_FOREACH(dev_request_queue, &pv->queue, item, {
      struct dev_valio_rq_s *rq_item = dev_valio_rq_s_cast(item);
      if (rq == rq_item) {
        dev_valio_rq_remove(&pv->queue, rq);
        return 0;
      }
    });

  return -ENOENT;
}

#define qdec_use dev_use_generic

static DEV_INIT(qdec_init)
{
  struct qdec_pv_s *pv;
  error_t err;
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

  if (width != 2) {
    logk_error("Width is supposed to be 2");
    goto put_gpio;
  }

  logk_trace("QDEC pins: %d/%d", id, width);

  dev_rq_queue_init(&pv->queue);

  dev_gpio_rq_init(&pv->gpio_rq, qdec_gpio_done);

  pv->gpio_rq.pvdata = dev;
  pv->gpio_rq.io_first = id;
  pv->gpio_rq.io_last = id + 1;
  pv->gpio_rq.mode.mode = DEV_PIN_INPUT;
  pv->gpio_rq.mode.mask = dev_gpio_mask1;
  pv->gpio_rq.type = DEV_GPIO_MODE;
  DEVICE_OP(&pv->gpio, request, &pv->gpio_rq);
  
  return 0;

 put_gpio:
  device_put_accessor(&pv->gpio.base);
 free_pv:
  mem_free(pv);
  return err;
}

static DEV_CLEANUP(qdec_cleanup)
{
  struct qdec_pv_s *pv = dev->drv_pv;

  DEVICE_OP(&pv->gpio, cancel, &pv->gpio_rq);

  GCT_FOREACH(dev_request_queue, &pv->queue, item, {
      struct dev_valio_rq_s *rq = dev_valio_rq_s_cast(item);
      rq->error = -EIO;
      dev_valio_rq_remove(&pv->queue, rq);
      dev_valio_rq_done(rq);
    });

  device_put_accessor(&pv->gpio.base);
  dev_rq_queue_destroy(&pv->queue);

  mem_free(pv);

  return 0;
}

DRIVER_DECLARE(qdec_gpio_drv, 0, "Quadrature decoder", qdec,
               DRIVER_VALIO_METHODS(qdec));

DRIVER_REGISTER(qdec_gpio_drv);


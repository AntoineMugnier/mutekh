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

struct mxk_range_s
{
  uint64_t mask;
  uint8_t first, last, count;
};

struct mxk_context_s
{
  dev_request_queue_root_t queue;
  struct dev_irq_src_s irq_ep;

  struct device_timer_s timer;
  struct dev_timer_rq_s rescan_timer_rq;
  struct dev_timer_rq_s row_timer_rq;
  struct device_gpio_s gpio;
  struct mxk_range_s rows, columns;

  uint8_t *last_state;
  uint8_t *cur_state;

  dev_timer_delay_t inter_row_delay;
  dev_timer_delay_t refresh_delay;

  uint64_t scan_column_mask;
  uint64_t rows_before_scan;

  uint16_t refresh_max;
  uint16_t refresh_count;

  uint8_t scan_column;
  uint8_t state_size;
  uint8_t range_id;

  bool_t scan_running : 1;
  bool_t one_pressed : 1;
};

static void mxk_notif_disable(struct mxk_context_s *pv)
{
  DEVICE_OP(&pv->gpio, input_irq_range,
            pv->rows.first, pv->rows.last, (const uint8_t *)&pv->rows.mask,
            0, pv->range_id);
}

static void mxk_notif_enable(struct mxk_context_s *pv)
{
  DEVICE_OP(&pv->gpio, input_irq_range,
            pv->rows.first, pv->rows.last, (const uint8_t *)&pv->rows.mask,
            DEV_IRQ_SENSE_LOW_LEVEL | DEV_IRQ_SENSE_HIGH_LEVEL, pv->range_id);
}

static uint32_t mxk_rows_get(struct mxk_context_s *pv)
{
  uint64_t state;

  DEVICE_OP(&pv->gpio, get_input, pv->rows.first, pv->rows.last,
            (uint8_t *)&state);

  return state & pv->rows.mask;
}

static void mxk_columns_set(struct mxk_context_s *pv, uint64_t val)
{
  uint64_t set_mask = endian_le64(val & pv->columns.mask);
  uint64_t clear_mask = endian_le64(val | ~pv->columns.mask);

  dprintk("%p %s %d-%d %llx %llx\n", pv, __FUNCTION__,
         pv->columns.first, pv->columns.last,
         clear_mask, set_mask);

  DEVICE_OP(&pv->gpio, set_output, pv->columns.first, pv->columns.last,
            (uint8_t*)&clear_mask, (uint8_t*)&set_mask);
}

static void mxk_scan_start(struct mxk_context_s *pv);
static void mxk_cols_setup(struct mxk_context_s *pv);
static void mxk_rows_scan(struct mxk_context_s *pv);
static void mxk_scan_done(struct mxk_context_s *pv);

static KROUTINE_EXEC(mxk_scan_routine)
{
  struct mxk_context_s *pv = KROUTINE_CONTAINER(kr, *pv, row_timer_rq.rq.kr);

  if (pv->scan_column < pv->columns.count)
    mxk_rows_scan(pv);
  else
    mxk_scan_done(pv);
}

static void mxk_next_later(struct mxk_context_s *pv)
{
  error_t err;

  assert(pv->scan_running);

  if (pv->row_timer_rq.rq.drvdata)
    DEVICE_OP(&pv->timer, cancel, &pv->row_timer_rq);

  pv->row_timer_rq.deadline += pv->inter_row_delay;

  err = DEVICE_OP(&pv->timer, request, &pv->row_timer_rq);
  if (err)
    kroutine_exec(&pv->row_timer_rq.rq.kr);
}

static void mxk_scan_start(struct mxk_context_s *pv)
{
  struct device_s *dev = pv->row_timer_rq.rq.pvdata;
  bool_t running;

  assert(!cpu_is_interruptible());

  dprintk("%p %s %s\n", pv, __FUNCTION__,
         pv->scan_running ? "running" : "ok");

  LOCK_SPIN_IRQ(&dev->lock);
  running = pv->scan_running;
  if (running)
    goto out;

  pv->scan_running = 1;
  pv->one_pressed = 0;
  pv->scan_column_mask = pv->columns.mask;
  pv->scan_column = 0;
  pv->rows_before_scan = mxk_rows_get(pv);

  DEVICE_OP(&pv->timer, get_value, &pv->row_timer_rq.deadline, 0);
  pv->rescan_timer_rq.deadline = pv->row_timer_rq.deadline + pv->refresh_delay;
 out:
  LOCK_RELEASE_IRQ(&dev->lock);

  if (running)
    return;

  mxk_notif_disable(pv);
  memset(pv->cur_state, 0, pv->state_size);
  mxk_cols_setup(pv);
  mxk_next_later(pv);
}

static KROUTINE_EXEC(mxk_restart_routine)
{
  struct mxk_context_s *pv = KROUTINE_CONTAINER(kr, *pv, rescan_timer_rq.rq.kr);

  mxk_scan_start(pv);
}

static void mxk_rescan_later(struct mxk_context_s *pv)
{
  dprintk("%p %s %d %d %d\n", pv, __FUNCTION__,
         pv->scan_running, pv->one_pressed,
         !!pv->rescan_timer_rq.rq.drvdata);

  assert(!cpu_is_interruptible());

  if (pv->rescan_timer_rq.rq.drvdata)
    DEVICE_OP(&pv->timer, cancel, &pv->rescan_timer_rq);
  DEVICE_OP(&pv->timer, request, &pv->rescan_timer_rq);
}

static void mxk_cols_setup(struct mxk_context_s *pv)
{
  uint8_t column = __builtin_ctzll(pv->scan_column_mask);
  uint64_t single = 1ULL << column;

  dprintk("%p %s column %d: %llx\n", pv, __FUNCTION__, column, single);

  mxk_columns_set(pv, single);

  pv->scan_column_mask &= ~single;
}

static void mxk_rows_scan(struct mxk_context_s *pv)
{
  uint64_t rows = mxk_rows_get(pv);
  uint64_t mask = pv->rows.mask;

  dprintk("%p %s rows: %llx\n", pv, __FUNCTION__, rows);

  for (uint8_t r = 0; r < pv->rows.count; ++r) {
    uint8_t row = __builtin_ctzll(mask);
    uint64_t m = (1ULL << row);

    mask &= ~m;

    bool_t pressed = !!(rows & m);
    if (pressed) {
      pv->one_pressed = 1;
      dev_valio_keyboard_key_set(pv->cur_state, pv->columns.count, r, pv->scan_column, 1);
    }
  }

  pv->scan_column++;

  if (pv->scan_column_mask) {
    mxk_cols_setup(pv);
  } else {
    dprintk("%p %s\n", pv, __FUNCTION__);
    mxk_columns_set(pv, pv->columns.mask);
  }

  mxk_next_later(pv);
}

static void mxk_scan_done(struct mxk_context_s *pv)
{
  struct device_s *dev = pv->row_timer_rq.rq.pvdata;
  struct dev_valio_rq_s *rq;
  bool_t changed;
  bool_t start = 0;

  LOCK_SPIN_IRQ(&dev->lock);
  mxk_notif_enable(pv);

  pv->scan_running = 0;
  changed = pv->rows_before_scan != mxk_rows_get(pv);

  if (changed) {
    dprintk("%p %s but changed while scanning\n", pv, __FUNCTION__);

    start = 1;
    goto out;
  }

  rq = dev_valio_rq_s_cast(dev_request_queue_head(&pv->queue));

  dprintk("%p %s done %p %d\n", pv, __FUNCTION__, rq, pv->one_pressed);

  if (!rq)
    goto out;

  if (pv->one_pressed && pv->refresh_count)
    mxk_rescan_later(pv);

  if (rq->type == DEVICE_VALIO_WAIT_UPDATE) {
    if (!memcmp(pv->last_state, pv->cur_state, pv->state_size)) {
      if (pv->refresh_count)
        pv->refresh_count--;
      goto out;
    } else {
      pv->refresh_count = pv->refresh_max;
    }
  }

  memcpy(rq->data, pv->cur_state, pv->state_size);
  memcpy(pv->last_state, pv->cur_state, pv->state_size);

  rq->error = 0;

  dev_request_queue_remove(&pv->queue, &rq->base);

  lock_release(&dev->lock);
  kroutine_exec(&rq->base.kr);
  lock_spin(&dev->lock);

  rq = dev_valio_rq_s_cast(dev_request_queue_head(&pv->queue));

  if (rq && rq->type == DEVICE_VALIO_READ)
    start = 1;

  if (!rq)
    device_stop(&pv->timer);

 out:
  LOCK_RELEASE_IRQ(&dev->lock);

  if (start)
    mxk_scan_start(pv);
}

static DEV_IRQ_SRC_PROCESS(mxk_irq)
{
  struct device_s *dev = ep->base.dev;
  struct mxk_context_s *pv = dev->drv_pv;

  dprintk("%p %s %d %d %d\n", pv, __FUNCTION__,
         pv->scan_running, pv->one_pressed,
         !!pv->row_timer_rq.rq.drvdata);

  if (!dev_request_queue_isempty(&pv->queue))
    mxk_scan_start(pv);
}

static DEV_VALIO_REQUEST(matrix_keyboard_request)
{
  struct device_s *dev = accessor->dev;
  struct mxk_context_s *pv = dev->drv_pv;
  req->error = 0;

  if (req->attribute != VALIO_KEYBOARD_MAP) {
    req->error = -EINVAL;
    goto out;
  }
 
  switch (req->type) {
  default:
    req->error = -ENOTSUP;
    break;

  case DEVICE_VALIO_READ:
  case DEVICE_VALIO_WAIT_UPDATE:
    LOCK_SPIN_IRQ(&dev->lock);
    if (dev_request_queue_isempty(&pv->queue))
      device_start(&pv->timer);

    dev_request_queue_pushback(&pv->queue, &req->base);
    pv->refresh_count = pv->refresh_max;
    LOCK_RELEASE_IRQ(&dev->lock);

    if (req->type == DEVICE_VALIO_READ)
      mxk_scan_start(pv);
    else
      mxk_notif_enable(pv);

    break;
  }

 out:
  if (req->error)
    kroutine_exec(&req->base.kr);
}


static DEV_INIT(matrix_keyboard_init);
static DEV_CLEANUP(matrix_keyboard_cleanup);

#define matrix_keyboard_use dev_use_generic

DRIVER_DECLARE(matrix_keyboard_drv, 0, "Matrix-keyboard", matrix_keyboard,
               DRIVER_VALIO_METHODS(matrix_keyboard));

DRIVER_REGISTER(matrix_keyboard_drv);

static error_t mxk_range_init(struct device_s *dev,
                                          uint8_t id,
                                          struct mxk_range_s *range)
{
  error_t err;
  uintptr_t first, last;
  const void* mask;

  err = device_get_param_io(dev, id, &first, &last);
  if (err)
    return err;

  range->first = first;
  range->last = last;

  err = device_get_param_blob(dev, "mask", id, &mask);
  if (err)
    return err;

  range->mask = endian_le64_na_load(mask) & ((1ULL << (last - first + 1)) - 1);
  range->count = __builtin_popcountll(range->mask);

  dprintk(" %d %s: io %d-%d mask %llx\n", range->count,
         id ? "columns" : "rows",
         first, last, range->mask);

  return range->count ? 0 : -EINVAL;
}

static DEV_INIT(matrix_keyboard_init)
{
  struct mxk_context_s *pv;
  error_t err;
  uintptr_t tmp;
  const struct dev_resource_s *r;


  pv = mem_alloc(sizeof(*pv), mem_scope_sys);
  if (!pv)
    return -ENOMEM;

  memset(pv, 0, sizeof(*pv));

  dev->drv_pv = pv;

  r = device_res_get(dev, DEV_RES_IRQ, 0);
  if (!r) {
    err = -ENOENT;
    goto free_pv;
  }

  pv->range_id = r->u.irq.sink_id;

  err = device_get_param_dev_accessor(dev, "timer", &pv->timer, DRIVER_CLASS_TIMER);
  if (err)
    goto free_pv;

  err = device_get_accessor_by_path(&pv->gpio, NULL, r->u.irq.icu, DRIVER_CLASS_GPIO);
  if (err)
    goto put_timer;

  err = mxk_range_init(dev, 0, &pv->rows);
  if (err)
    goto put_gpio;

  err = mxk_range_init(dev, 1, &pv->columns);
  if (err)
    goto put_gpio;

  err = device_get_param_uint(dev, "row_delay", &tmp);
  if (err)
    goto put_gpio;
  dev_timer_init_sec(&pv->timer, &pv->inter_row_delay, 0, tmp, 1000);

  err = device_get_param_uint(dev, "refresh_period", &tmp);
  if (err)
    goto put_gpio;
  dev_timer_init_sec(&pv->timer, &pv->refresh_delay, 0, tmp, 1000);

  err = device_get_param_uint(dev, "refresh_max", &tmp);
  if (err)
    goto put_gpio;
  pv->refresh_max = tmp;

  dev_request_queue_init(&pv->queue);

  device_irq_source_init(dev, &pv->irq_ep, 1, &mxk_irq /*, DEV_IRQ_SENSE_ANY_EDGE */);

  err = device_irq_source_link(dev, &pv->irq_ep, 1, -1);
  if (err)
    goto put_gpio;

  pv->state_size = (pv->columns.count * pv->rows.count + 7) / 8;

  pv->last_state = mem_alloc(pv->state_size * 2, mem_scope_sys);
  if (!pv->last_state) {
    err = -ENOMEM;
    goto put_gpio;
  }
  pv->cur_state = pv->last_state + pv->state_size;

  mxk_columns_set(pv, -1LL);

  DEVICE_OP(&pv->gpio, set_mode,
            pv->columns.first, pv->columns.last,
            (const uint8_t *)&pv->columns.mask, DEV_PIN_OPENSOURCE_PULLDOWN);

  DEVICE_OP(&pv->gpio, set_mode,
            pv->rows.first, pv->rows.last,
            (const uint8_t *)&pv->rows.mask, DEV_PIN_INPUT_PULLDOWN);

  pv->rescan_timer_rq.rq.pvdata = dev;
  pv->row_timer_rq.rq.pvdata = dev;
  kroutine_init_interruptible(&pv->row_timer_rq.rq.kr, mxk_scan_routine);
  kroutine_init_interruptible(&pv->rescan_timer_rq.rq.kr, mxk_restart_routine);

  return 0;

 put_gpio:
  device_put_accessor(&pv->gpio);
 put_timer:
  device_put_accessor(&pv->timer);
 free_pv:
  mem_free(pv);
  return err;
}

static DEV_CLEANUP(matrix_keyboard_cleanup)
{
  struct mxk_context_s *pv = dev->drv_pv;

  device_put_accessor(&pv->timer);
  device_put_accessor(&pv->gpio);

  device_irq_source_unlink(dev, &pv->irq_ep, 1);
  dev_request_queue_destroy(&pv->queue);

  mem_free(pv->last_state);
  mem_free(pv);
}


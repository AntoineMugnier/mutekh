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

#define LOGK_MODULE_ID "hd44"

#include <hexo/types.h>
#include <hexo/endian.h>
#include <hexo/bit.h>

#include <mutek/mem_alloc.h>
#include <mutek/printk.h>
#include <mutek/bytecode.h>

#include <device/device.h>
#include <device/resources.h>
#include <device/driver.h>
#include <device/irq.h>
#include <device/class/char.h>
#include <device/class/gpio.h>
#include <device/class/timer.h>
#include <device/clock.h>

#include "hd44780_io.o.h"
#include "hd44780_defs.h"

enum hd44780_state_e
{
  HD44780_IDLE,
  HD44780_RUNNING,
  HD44780_WAIT_TIMER,
  HD44780_WAIT_GPIO,
  HD44780_WAIT_CHAR,
};

enum hd44780_gpio_e
{
  HD44780_GPIO_E,
  HD44780_GPIO_RS,
  HD44780_GPIO_D,
  HD44780_GPIO_RW,
  HD44780_GPIO_COUNT,
};

struct hd44780_ctx_s
{
  struct device_gpio_s gpio;
  struct device_timer_s timer;
  struct dev_timer_rq_s timer_rq;
  struct dev_gpio_rq_s gpio_rq;
  struct bc_context_s vm;
  struct kroutine_s vm_runner;
  
  dev_request_queue_root_t queue;
  dev_timer_delay_t cycle;

  enum hd44780_state_e state;
  uint8_t reg;
  
  gpio_id_t gpio_id[HD44780_GPIO_COUNT];
};

DRIVER_PV(struct hd44780_ctx_s);

static KROUTINE_EXEC(hd44780_runner)
{
  struct hd44780_ctx_s *pv = KROUTINE_CONTAINER(kr, *pv, vm_runner);
  struct device_s *dev = pv->gpio_rq.pvdata;
  uint16_t op;
  struct dev_char_rq_s *rq;

  for (;;) {
    bool_t run = 1;

    LOCK_SPIN_IRQ(&dev->lock);
    switch (pv->state) {
    case HD44780_WAIT_CHAR:
    again:
      rq = dev_char_rq_head(&pv->queue);
      if (!rq)
        goto busy;

      if (rq->size == 0) {
        dev_char_rq_pop(&pv->queue);
        rq->base.drvdata = NULL;
        dev_char_rq_done(rq);
        goto again;
      }

      logk_trace("%s refill %02x", __func__, rq->data[0]);
      pv->vm.v[pv->reg] = *rq->data++;
      rq->size--;
      pv->state = HD44780_IDLE;
      goto idle;

    case HD44780_IDLE:
    idle:
      pv->state = HD44780_RUNNING;
      run = 1;
      break;

    default:
    busy:
      run = 0;
      break;
    }
    LOCK_RELEASE_IRQ(&dev->lock);
    
    if (!run)
      break;

    logk_trace("%s run", __func__);

    op = bc_run(&pv->vm);

    logk_trace("%s op = %04x", __func__, op);

    if (!(op & 0x8000)) {
      assert(!op);
      break;
    }

    switch (bit_get_mask(op, 12, 2)) {
    case 0: // Delay
      pv->timer_rq.delay = pv->cycle * bit_get_mask(op, 0, 12);
      logk_trace("%s delay %d %d",
              __func__, bit_get_mask(op, 0, 12),
              pv->timer_rq.delay);
      pv->timer_rq.deadline = 0;
      if (DEVICE_OP(&pv->timer, request, &pv->timer_rq) == 0) {
        pv->state = HD44780_WAIT_TIMER;
        return;
      }
      pv->state = HD44780_IDLE;
      continue;

    case 1: // Data
      logk_trace("%s data r%d: %02x", __func__, bit_get_mask(op, 0, 4),
              pv->vm.v[bit_get_mask(op, 0, 4)]
              & (_CONFIG_DRIVER_HD44780_4BIT ? 0xf : 0xff));
      pv->gpio_rq.io_first = pv->gpio_id[HD44780_GPIO_D];
      pv->gpio_rq.io_last = pv->gpio_id[HD44780_GPIO_D]
        + (_CONFIG_DRIVER_HD44780_4BIT ? 4 : 8) - 1;
      pv->reg = pv->vm.v[bit_get_mask(op, 0, 4)];
      pv->state = HD44780_WAIT_GPIO;
      DEVICE_OP(&pv->gpio, request, &pv->gpio_rq);
      return;
      
    case 2: // E/RS
      logk_trace("%s %d %d", __func__, bit_get_mask(op, 4, 2), bit_get(op, 0));
      if (pv->gpio_id[bit_get_mask(op, 4, 2)] == GPIO_INVALID_ID) {
        pv->state = HD44780_IDLE;
        continue;
      }
      pv->gpio_rq.io_first = pv->gpio_rq.io_last = pv->gpio_id[bit_get_mask(op, 4, 2)];
      pv->reg = bit_get(op, 0);
      pv->state = HD44780_WAIT_GPIO;
      DEVICE_OP(&pv->gpio, request, &pv->gpio_rq);
      return;
      
    case 3: // Next
      logk_trace("%s next r%d", __func__, bit_get_mask(op, 0, 4));
      pv->reg = bit_get_mask(op, 0, 4);
      pv->state = HD44780_WAIT_CHAR;
      continue;
    }
  }
}

static KROUTINE_EXEC(hd44780_gpio_done)
{
  struct hd44780_ctx_s *pv = KROUTINE_CONTAINER(kr, *pv, gpio_rq.base.kr);
  struct device_s *dev = pv->gpio_rq.pvdata;

  logk_trace("%s", __func__);

  LOCK_SPIN_IRQ_SCOPED(&dev->lock);

  assert(pv->state == HD44780_WAIT_GPIO);
  pv->state = HD44780_IDLE;
  kroutine_exec(&pv->vm_runner);
}

static KROUTINE_EXEC(hd44780_timer_done)
{
  struct hd44780_ctx_s *pv = KROUTINE_CONTAINER(kr, *pv, timer_rq.base.kr);
  struct device_s *dev = pv->timer_rq.pvdata;

  logk_trace("%s", __func__);

  LOCK_SPIN_IRQ_SCOPED(&dev->lock);

  assert(pv->state == HD44780_WAIT_TIMER);
  pv->state = HD44780_IDLE;
  kroutine_exec(&pv->vm_runner);
}

static DEV_CHAR_REQUEST(hd4780_request)
{
  struct device_s *dev = accessor->dev;
  struct hd44780_ctx_s *pv = dev->drv_pv;

  switch (rq->type) {
  case DEV_CHAR_WRITE_PARTIAL_FLUSH:
  case DEV_CHAR_WRITE_FLUSH:
  case DEV_CHAR_WRITE_PARTIAL:
  case DEV_CHAR_WRITE:
    break;

  default:
    rq->error = -ENOTSUP;
    dev_char_rq_done(rq);
    return;
  }

  rq->error = 0;

  assert(rq->size);

  rq->base.drvdata = dev;

  LOCK_SPIN_IRQ_SCOPED(&dev->lock);

  dev_char_rq_pushback(&pv->queue, rq);
  if (pv->state == HD44780_WAIT_CHAR)
    kroutine_exec(&pv->vm_runner);
}

static DEV_CHAR_CANCEL(hd4780_cancel)
{
  struct device_s *dev = accessor->dev;
  struct hd44780_ctx_s *pv = dev->drv_pv;
  
  if (rq->base.drvdata != dev)
    return -EINVAL;

  LOCK_SPIN_IRQ_SCOPED(&dev->lock);

  dev_char_rq_remove(&pv->queue, rq);

  if (dev_rq_queue_isempty(&pv->queue))
    device_sleep_schedule(dev);

  return 0;
}

#define hd4780_use dev_use_generic

static DEV_INIT(hd4780_init)
{
  struct hd44780_ctx_s *pv;
  uintptr_t tmp;
  error_t err;
    
  pv = mem_alloc(sizeof(*pv), mem_scope_sys);
  if (!pv)
    return -ENOMEM;

  memset(pv, 0, sizeof(*pv));
  dev->drv_pv = pv;

  err = device_get_param_dev_accessor(dev, "gpio", &pv->gpio.base, DRIVER_CLASS_GPIO);
  if (err)
    goto err_pv;

  err = device_get_param_dev_accessor(dev, "timer", &pv->timer.base, DRIVER_CLASS_TIMER);
  if (err)
    goto err_pv;

  err = device_get_param_uint(dev, "freq", &tmp);
  if (err)
    tmp = 270000;

  dev_timer_init_sec_ceil(&pv->timer, &pv->cycle, 0, 5, tmp);
  if (!pv->cycle)
    pv->cycle = 1;

  gpio_width_t pin_wmap[HD44780_GPIO_COUNT];

  err = device_res_gpio_map(dev,
#if defined(CONFIG_DRIVER_HD44780_4BIT)
                            "e:1 rs:1 d:4 rw?:1",
#else
                            "e:1 rs:1 d:8 rw?:1",
#endif
                            pv->gpio_id, pin_wmap);
  if (err)
    goto err_gpio;

  size_t count = pv->gpio_id[HD44780_GPIO_RW] == GPIO_INVALID_ID ? 3 : 4;

  err = device_gpio_map_set_mode(&pv->gpio, pv->gpio_id, pin_wmap, count,
                                 DEV_PIN_PUSHPULL,
                                 DEV_PIN_PUSHPULL,
                                 DEV_PIN_PUSHPULL,
                                 DEV_PIN_PUSHPULL);
  if (err)
    goto err_gpio;
    
  dev_rq_queue_init(&pv->queue);

  pv->gpio_rq.pvdata = dev;
  pv->timer_rq.pvdata = dev;
  pv->gpio_rq.output.set_mask = &pv->reg;
  pv->gpio_rq.output.clear_mask = &pv->reg;
  pv->gpio_rq.type = DEV_GPIO_SET_OUTPUT;

  dev_gpio_rq_init(&pv->gpio_rq, &hd44780_gpio_done);
  dev_timer_rq_init(&pv->timer_rq, &hd44780_timer_done);
  kroutine_init_deferred(&pv->vm_runner, &hd44780_runner);

  bc_init(&pv->vm, &hd44780_io_bytecode);
  bc_set_pc(&pv->vm, &hd44780_start);

  err = device_get_param_uint(dev, "rows", &tmp);
  if (err)
    tmp = 2;
    
  bc_set_reg(&pv->vm, 12, tmp == 2 ? CMD_FUNCTION_2LINES : CMD_FUNCTION_1LINE);
    
  kroutine_exec(&pv->vm_runner);

  return 0;
    
 err_gpio:
  device_put_accessor(&pv->gpio.base);

 err_pv:
  mem_free(pv);

  return err;
}

static DEV_CLEANUP(hd4780_cleanup)
{
  struct hd44780_ctx_s *pv = dev->drv_pv;

  if (!dev_rq_queue_isempty(&pv->queue)
      || !(pv->state == HD44780_IDLE || pv->state == HD44780_WAIT_CHAR))
    return -EBUSY;

  dev_rq_queue_destroy(&pv->queue);
  device_put_accessor(&pv->timer.base);
  device_put_accessor(&pv->gpio.base);

  mem_free(pv);

  return 0;
}

DRIVER_DECLARE(hd44780_drv, 0, "HD44780 LCD", hd4780,
               DRIVER_CHAR_METHODS(hd4780));

DRIVER_REGISTER(hd44780_drv);

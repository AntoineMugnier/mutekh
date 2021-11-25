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

#include <hexo/types.h>
#include <hexo/endian.h>
#include <hexo/bit.h>

#include <mutek/mem_alloc.h>
#include <mutek/printk.h>
#include <mutek/bytecode.h>

#include <device/device.h>
#include <device/resources.h>
#include <device/driver.h>
#include <device/class/valio.h>
#include <device/valio/keyboard.h>
#include <device/class/gpio.h>
#include <device/class/timer.h>
#include <device/clock.h>

#include "matrix_keyboard_io.o.h"

//#define dprintk printk
#define dprintk(...) do{}while(0)

enum matrix_keyboard_gpio_id_e
{
  GPIO_ROWS,
  GPIO_COLUMNS,
  GPIO_ID_COUNT,
};

enum matrix_keyboard_state_e
{
  MATRIX_KEYBOARD_IDLE,
  MATRIX_KEYBOARD_RUNNING,
  MATRIX_KEYBOARD_WAIT_TIMER,
  MATRIX_KEYBOARD_WAIT_COLUMNS,
  MATRIX_KEYBOARD_WAIT_ROWS,
  MATRIX_KEYBOARD_WAIT_NOTIFY,
  MATRIX_KEYBOARD_WAIT_CHANGE,
};

struct matrix_keyboard_ctx_s
{
  struct device_gpio_s gpio;
  struct device_timer_s timer;
  struct dev_timer_rq_s timer_rq;
  struct dev_gpio_rq_s gpio_rq;

  struct bc_context_s vm;
  struct kroutine_s vm_runner;

  dev_request_queue_root_t queue;
  dev_timer_delay_t row_delay;
  dev_timer_delay_t refresh_delay;
  size_t refresh_max;

  enum matrix_keyboard_state_e state;

  uint8_t *column;
  uint8_t *row;
  uint8_t *last_state;
  uint8_t *cur_state;

  union {
    struct {
      uint8_t set_mask[4];
      uint8_t clear_mask[4];
    };
    uint8_t input[8];
  } gpio_data;
  uint32_t rows_mask, columns_mask;
  uint8_t column_count, row_count, state_size;
  uint8_t reg;
  
  gpio_id_t pin_id[GPIO_ID_COUNT];
  gpio_width_t pin_width[GPIO_ID_COUNT];
};

DRIVER_PV(struct matrix_keyboard_ctx_s);

static KROUTINE_EXEC(matrix_keyboard_runner)
{
  struct matrix_keyboard_ctx_s *pv = KROUTINE_CONTAINER(kr, *pv, vm_runner);
  struct device_s *dev = pv->gpio_rq.pvdata;
  uint16_t op;
  bool_t run = 0;

  //dprintk("%s\n", __FUNCTION__);

  LOCK_SPIN_IRQ(&dev->lock);
  switch (pv->state) {
  case MATRIX_KEYBOARD_IDLE:
    pv->state = MATRIX_KEYBOARD_RUNNING;
    run = 1;
    break;

  default:
    run = 0;
    break;
  }
  LOCK_RELEASE_IRQ(&dev->lock);

  if (!run)
    return;

  for (;;) {
    //dprintk("%s run\n", __FUNCTION__);
    op = bc_run(&pv->vm);
    //dprintk("%s op = %04x\n", __FUNCTION__, op);

    if (!(op & 0x8000)) {
      assert(!op);
      break;
    }

    switch (bit_get_mask(op, 12, 3)) {
    case 0: // Wait
      switch (bit_get_mask(op, 8, 1)) {
      case 0: // Row
        pv->timer_rq.delay = pv->row_delay;
        break;
      case 1: // Refresh
        pv->timer_rq.delay = pv->refresh_delay;
        break;
      }

      dprintk("%s delay %d %d\n", __FUNCTION__,
              (uint32_t)bit_get_mask(op, 0, 12), (uint32_t)pv->timer_rq.delay);
      pv->timer_rq.deadline = 0;

      if (DEVICE_OP(&pv->timer, request, &pv->timer_rq) == 0) {
        pv->state = MATRIX_KEYBOARD_WAIT_TIMER;
        return;
      }
      continue;

    case 1: // IO
      switch (bit_get_mask(op, 8, 3)) {
      case 0: { // column strobe
        uint8_t no = bc_get_reg(&pv->vm, bit_get_mask(op, 0, 4));
        gpio_id_t io = pv->column[no];
        dprintk("%s column_strobe %d, io %d\n", __FUNCTION__, no, io);
        assert(no < pv->column_count);

        /*
          Strobe x => clear column x, let other float
          Strobe 2         v
          Column IDs:    432  10
          Columns mask:  1110011
          GPIO action:   sscuuss
          set_mask:      1100011 == columns_mask & ~io_bit
          clear_mask:    1101111 == ~io_bit
         */
        endian_le32_na_store(pv->gpio_data.set_mask, pv->columns_mask & ~bit(io));
        endian_le32_na_store(pv->gpio_data.clear_mask, ~bit(io));
        pv->gpio_rq.io_first = pv->pin_id[GPIO_COLUMNS];
        pv->gpio_rq.io_last = pv->pin_id[GPIO_COLUMNS] + pv->pin_width[GPIO_COLUMNS] - 1;
        pv->gpio_rq.output.set_mask = pv->gpio_data.set_mask;
        pv->gpio_rq.output.clear_mask = pv->gpio_data.clear_mask;
        pv->gpio_rq.type = DEV_GPIO_SET_OUTPUT;
        pv->state = MATRIX_KEYBOARD_WAIT_COLUMNS;
        DEVICE_OP(&pv->gpio, request, &pv->gpio_rq);
        return;
      }

      case 1: // columns release
        dprintk("%s columns_release\n", __FUNCTION__);
        /*
          Column IDs:    432  10
          Columns mask:  1110011
          GPIO action:   cccuucc
          set_mask:      0000000 == 0
          clear_mask:    0001100 == ~columns_mask
         */
        endian_le32_na_store(pv->gpio_data.set_mask, 0);
        endian_le32_na_store(pv->gpio_data.clear_mask, ~pv->columns_mask);
        pv->gpio_rq.io_first = pv->pin_id[GPIO_COLUMNS];
        pv->gpio_rq.io_last = pv->pin_id[GPIO_COLUMNS] + pv->pin_width[GPIO_COLUMNS] - 1;
        pv->gpio_rq.output.set_mask = pv->gpio_data.set_mask;
        pv->gpio_rq.output.clear_mask = pv->gpio_data.clear_mask;
        pv->gpio_rq.type = DEV_GPIO_SET_OUTPUT;
        pv->state = MATRIX_KEYBOARD_WAIT_COLUMNS;
        DEVICE_OP(&pv->gpio, request, &pv->gpio_rq);
        return;

      case 2: // rows_get
        dprintk("%s rows_get\n", __FUNCTION__);
        pv->gpio_rq.io_first = pv->pin_id[GPIO_ROWS];
        pv->gpio_rq.io_last = pv->pin_id[GPIO_ROWS] + pv->pin_width[GPIO_ROWS] - 1;
        pv->gpio_rq.input.data = pv->gpio_data.input;
        pv->reg = bit_get_mask(op, 0, 4);
        pv->gpio_rq.type = DEV_GPIO_GET_INPUT;
        pv->state = MATRIX_KEYBOARD_WAIT_ROWS;
        DEVICE_OP(&pv->gpio, request, &pv->gpio_rq);
        return;

      case 3: // wait_change
        endian_le32_na_store(pv->gpio_data.input,
                             ~bc_get_reg(&pv->vm, bit_get_mask(op, 0, 4)));
        endian_le32_na_store(pv->gpio_data.input + 4, pv->rows_mask);
        pv->gpio_rq.io_first = pv->pin_id[GPIO_ROWS];
        pv->gpio_rq.io_last = pv->pin_id[GPIO_ROWS] + pv->pin_width[GPIO_ROWS] - 1;
        pv->gpio_rq.until.data = pv->gpio_data.input;
        pv->gpio_rq.until.mask = pv->gpio_data.input + 4;
        pv->gpio_rq.type = DEV_GPIO_UNTIL;
        pv->state = MATRIX_KEYBOARD_WAIT_CHANGE;
        DEVICE_OP(&pv->gpio, request, &pv->gpio_rq);
        return;
      }
      break;

    case 2: // Reporting
      switch (bit_get_mask(op, 8, 2)) {
      case 0: // pressed_reset
        dprintk("%s pressed reset\n", __FUNCTION__);
        memset(pv->cur_state, 0, pv->state_size);
        break;

      case 1: { // pressed_set
        size_t key = bc_get_reg(&pv->vm, bit_get_mask(op, 0, 4));
        dprintk("%s pressed set %d\n", __FUNCTION__, key);
        pv->cur_state[key / 8] |= bit(key % 8);
        break;
      }

      case 2: { // pressed_done
        dprintk("%s pressed state changed: %P\n", __FUNCTION__, pv->cur_state, pv->state_size);

        LOCK_SPIN_IRQ_SCOPED(&dev->lock);
        if (memcmp(pv->cur_state, pv->last_state, pv->state_size)) {
          memcpy(pv->last_state, pv->cur_state, pv->state_size);
          struct dev_valio_rq_s *rq = dev_valio_rq_pop(&pv->queue);
          if (rq) {
            memcpy(rq->data, pv->last_state, pv->state_size);
            rq->error = 0;
            dev_valio_rq_done(rq);
          }
        }

        break;
      }
      }
    }

  }
}

static KROUTINE_EXEC(matrix_keyboard_gpio_done)
{
  struct matrix_keyboard_ctx_s *pv = KROUTINE_CONTAINER(kr, *pv, gpio_rq.base.kr);
  struct device_s *dev = pv->gpio_rq.pvdata;

  dprintk("%s\n", __FUNCTION__);

  LOCK_SPIN_IRQ_SCOPED(&dev->lock);
  assert(pv->state == MATRIX_KEYBOARD_WAIT_COLUMNS
         || pv->state == MATRIX_KEYBOARD_WAIT_ROWS
         || pv->state == MATRIX_KEYBOARD_WAIT_CHANGE
         || pv->state == MATRIX_KEYBOARD_WAIT_NOTIFY);

  if (pv->state == MATRIX_KEYBOARD_WAIT_ROWS) {
    uint32_t data = endian_le32_na_load(pv->gpio_data.input);
    uint32_t reg = 0;

    for (size_t i = 0; i < pv->row_count; ++i) {
      if (bit_get(data, pv->row[i]) == 0)
        reg |= bit(i);
    }

    dprintk("%s rows: %x\n", __FUNCTION__, reg);

    bc_set_reg(&pv->vm, pv->reg, reg);
  }

  pv->state = MATRIX_KEYBOARD_IDLE;
  kroutine_exec(&pv->vm_runner);
}

static KROUTINE_EXEC(matrix_keyboard_timer_done)
{
  struct matrix_keyboard_ctx_s *pv = KROUTINE_CONTAINER(kr, *pv, timer_rq.base.kr);
  struct device_s *dev = pv->timer_rq.pvdata;

  dprintk("%s\n", __FUNCTION__);

  LOCK_SPIN_IRQ_SCOPED(&dev->lock);
  assert(pv->state == MATRIX_KEYBOARD_WAIT_TIMER);

  pv->state = MATRIX_KEYBOARD_IDLE;
  kroutine_exec(&pv->vm_runner);
}

static DEV_VALIO_REQUEST(matrix_keyboard_request)
{
  struct device_s *dev = accessor->dev;
  struct matrix_keyboard_ctx_s *pv = dev->drv_pv;

  if (rq->attribute != VALIO_KEYBOARD_MAP) {
    rq->error = -EINVAL;
    dev_valio_rq_done(req);
    return;
  }
 
  switch (rq->type) {
  default:
    rq->error = -ENOTSUP;
    dev_valio_rq_done(req);
    break;

  case DEVICE_VALIO_READ:
    memcpy(rq->data, pv->last_state, pv->state_size);
    dev_valio_rq_done(req);
    rq->error = 0;
    break;

  case DEVICE_VALIO_WAIT_EVENT: {
    LOCK_SPIN_IRQ_SCOPED(&dev->lock);
    dev_valio_rq_pushback(&pv->queue, req);

    if (pv->state == MATRIX_KEYBOARD_IDLE)
      kroutine_exec(&pv->vm_runner);

    break;
  }
  }
}

#define matrix_keyboard_use dev_use_generic
#define matrix_keyboard_cancel (dev_valio_cancel_t*)&dev_driver_notsup_fcn

static DEV_INIT(matrix_keyboard_init)
{
  struct matrix_keyboard_ctx_s *pv;
  error_t err;
  uintptr_t tmp;
  uintptr_t rows_mask;
  uintptr_t columns_mask;
  size_t column_count, row_count, state_size;
  gpio_id_t pin_id[2];
  gpio_width_t pin_width[2];

  err = device_res_gpio_map(dev, "rows columns", pin_id, pin_width);
  if (err)
    goto err_gpio;

  err = device_get_param_uint(dev, "rows_mask", &rows_mask);
  if (err)
    rows_mask = bit_mask(0, pin_width[0]);

  err = device_get_param_uint(dev, "columns_mask", &columns_mask);
  if (err)
    columns_mask = bit_mask(0, pin_width[1]);

  column_count = bit_popc64(columns_mask);
  row_count = bit_popc64(rows_mask);

  assert(column_count && row_count);

  state_size = (column_count * row_count + 7) / 8;

  pv = mem_alloc(sizeof(*pv) + column_count + row_count + state_size * 2, mem_scope_sys);
  if (!pv)
    return -ENOMEM;

  memset(pv, 0, sizeof(*pv));

  dev->drv_pv = pv;
  pv->column = (void *)(pv + 1);
  pv->row = pv->column + column_count;
  pv->last_state = pv->row + row_count;
  pv->cur_state = pv->last_state + state_size;
  pv->rows_mask = rows_mask;
  pv->columns_mask = columns_mask;
  pv->row_count = row_count;
  pv->column_count = column_count;
  pv->state_size = state_size;

  memcpy(pv->pin_id, pin_id, 2 * sizeof(gpio_id_t));
  memcpy(pv->pin_width, pin_width, 2 * sizeof(gpio_width_t));

  for (uint8_t i = 0; i < pv->row_count; ++i) {
    uint8_t id = bit_ctz(rows_mask);
    pv->row[i] = id;
    rows_mask &= ~bit(id);
  }

  assert(!rows_mask);

  for (uint8_t i = 0; i < pv->column_count; ++i) {
    uint8_t id = bit_ctz(columns_mask);
    pv->column[i] = id;
    columns_mask &= ~bit(id);
  }

  assert(!columns_mask);

  err = device_get_param_dev_accessor(dev, "timer", &pv->timer.base, DRIVER_CLASS_TIMER);
  if (err)
    goto free_pv;

  err = device_get_param_dev_accessor(dev, "gpio", &pv->gpio.base, DRIVER_CLASS_GPIO);
  if (err)
    goto put_timer;

  err = device_get_param_uint(dev, "row_delay", &tmp);
  if (err)
    tmp = 2;
  dev_timer_init_sec(&pv->timer, &pv->row_delay, 0, tmp, 1000);

  assert(pv->row_delay);

  err = device_get_param_uint(dev, "refresh_period", &tmp);
  if (err)
    tmp = 33;
  dev_timer_init_sec(&pv->timer, &pv->refresh_delay, 0, tmp, 1000);

  assert(pv->refresh_delay);

  err = device_get_param_uint(dev, "refresh_max", &tmp);
  if (err)
    tmp = 10;
  pv->refresh_max = tmp;

  dev_rq_queue_init(&pv->queue);

  uint8_t mask[8] = {};

  endian_le32_na_store(mask, pv->rows_mask);
  DEVICE_OP(&pv->gpio, set_mode,
            pv->pin_id[GPIO_ROWS], pv->pin_id[GPIO_ROWS] + pv->pin_width[GPIO_ROWS] - 1,
            mask, DEV_PIN_INPUT_PULLUP);

  endian_le32_na_store(mask, pv->columns_mask);
  DEVICE_OP(&pv->gpio, set_mode,
            pv->pin_id[GPIO_COLUMNS], pv->pin_id[GPIO_COLUMNS] + pv->pin_width[GPIO_COLUMNS] - 1,
            mask, DEV_PIN_OPENDRAIN);

  pv->gpio_rq.pvdata = dev;
  pv->timer_rq.pvdata = dev;

  dev_gpio_rq_init(&pv->gpio_rq, matrix_keyboard_gpio_done);
  dev_timer_rq_init(&pv->timer_rq, matrix_keyboard_timer_done);
  kroutine_init_deferred(&pv->vm_runner, matrix_keyboard_runner);

  bc_init(&pv->vm, &matrix_keyboard_io_bytecode);
  bc_set_pc(&pv->vm, &matrix_keyboard_start);
  bc_set_reg(&pv->vm, 7, pv->column_count);
  bc_set_reg(&pv->vm, 8, pv->row_count);
  bc_set_reg(&pv->vm, 9, pv->refresh_max);
  pv->state = MATRIX_KEYBOARD_IDLE;
  kroutine_exec(&pv->vm_runner);

  return 0;

 put_gpio:
  device_put_accessor(&pv->gpio.base);
 put_timer:
  device_put_accessor(&pv->timer.base);
 free_pv:
  mem_free(pv);
 err_gpio:
  return err;
}

static DEV_CLEANUP(matrix_keyboard_cleanup)
{
  struct matrix_keyboard_ctx_s *pv = dev->drv_pv;

  if (!dev_rq_queue_isempty(&pv->queue)
      || !(pv->state == MATRIX_KEYBOARD_IDLE))
    return -EBUSY;

  dev_rq_queue_destroy(&pv->queue);
  device_put_accessor(&pv->timer.base);
  device_put_accessor(&pv->gpio.base);

  mem_free(pv->last_state);
  mem_free(pv);

  return 0;
}

DRIVER_DECLARE(matrix_keyboard_drv, 0, "Matrix-keyboard", matrix_keyboard,
               DRIVER_VALIO_METHODS(matrix_keyboard));

DRIVER_REGISTER(matrix_keyboard_drv);


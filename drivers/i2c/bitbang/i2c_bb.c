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
#include <device/class/i2c.h>
#include <device/class/gpio.h>
#include <device/class/timer.h>
#include <device/clock.h>

#include <enums.h>

#include "i2c_bb_io.o.h"

//#define dprintk printk
#ifndef dprintk
# define dprintk(...) do{}while(0)
#endif

#define HAS_ASYNC defined(CONFIG_DRIVER_I2C_BITBANG_ASYNC)
#define HAS_DYNAMIC (defined(CONFIG_DRIVER_I2C_BITBANG_SYNC) \
                     && defined(CONFIG_DRIVER_I2C_BITBANG_ASYNC))
#define HAS_SYNC defined(CONFIG_DRIVER_I2C_BITBANG_SYNC)

#if HAS_DYNAMIC
# define HAS_GPIO_SYNC(y, x) DEVICE_HAS_OP(y, x)
#elif HAS_ASYNC
# define HAS_GPIO_SYNC(y, x) 0
#else
# define HAS_GPIO_SYNC(y, x) 1
#endif

enum i2c_bb_gpio_id_e
{
  GPIO_SCL,
  GPIO_SDA,
  GPIO_ID_COUNT,
};

enum i2c_bb_state_e
{
  I2C_BB_IDLE,
  I2C_BB_RUNNING,
  I2C_BB_WAIT_TIMER,
#if HAS_ASYNC
  I2C_BB_WAIT_GPIO_OUT,
  I2C_BB_WAIT_GPIO_IN,
#endif
  I2C_BB_WAIT_RQ,
};

struct i2c_bb_ctx_s
{
  struct device_gpio_s gpio;

#if HAS_ASYNC
  struct dev_gpio_rq_s gpio_rq;
  uint8_t gpio_input[8];
#endif

  struct device_timer_s timer;
  struct dev_timer_rq_s timer_rq;

  struct bc_context_s vm;
  struct kroutine_s vm_runner;

  struct dev_i2c_ctrl_context_s i2c_ctrl_ctx;
  struct dev_i2c_ctrl_transfer_s *current;

  enum i2c_bb_state_e state;

  uint8_t reg;
  
  gpio_id_t pin_id[GPIO_ID_COUNT];
};

DRIVER_PV(struct i2c_bb_ctx_s);

static KROUTINE_EXEC(i2c_bb_runner)
{
  struct i2c_bb_ctx_s *pv = KROUTINE_CONTAINER(kr, *pv, vm_runner);
  struct device_s *dev = pv->timer_rq.pvdata;
  uint16_t op;
  bool_t run = 0;
  error_t err;

  //dprintk("%s\n", __FUNCTION__);

  LOCK_SPIN_IRQ(&dev->lock);
  switch (pv->state) {
  case I2C_BB_IDLE:
    pv->state = I2C_BB_RUNNING;
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

    switch (bit_get_mask(op, 13, 2)) {
    case 0: // Wait
      dprintk("%s delay %d\n", __FUNCTION__,
              (uint32_t)pv->timer_rq.delay);

      err = DEVICE_OP(&pv->timer, request, &pv->timer_rq);

      if (err == 0) {
        LOCK_SPIN_IRQ_SCOPED(&dev->lock);

        pv->state = I2C_BB_WAIT_TIMER;
        return;
      }

      continue;

    case 1: // Txn
      switch (bit_get_mask(op, 8, 3)) {
      case 0: { // txn_next_get
        bool_t cont = 0;

        LOCK_SPIN_IRQ_SCOPED(&dev->lock);

        cont = !!pv->current;
        if (!cont)
          pv->state = I2C_BB_WAIT_RQ;

        if (cont) {
          dprintk("%s wait rq OK\n", __FUNCTION__);
          continue;
        }

        dprintk("%s wait rq\n", __FUNCTION__);
        return;
      }

      case 1: // txn_type_get
        dprintk("%s type get\n", __FUNCTION__);
        bc_set_reg(&pv->vm, bit_get_mask(op, 0, 4), pv->current->type);
        continue;

      case 2: // txn_saddr_get
        dprintk("%s saddr get\n", __FUNCTION__);
        bc_set_reg(&pv->vm, bit_get_mask(op, 0, 4), pv->current->saddr);
        continue;

      case 3: // txn_byte_pop
        dprintk("%s byte pop\n", __FUNCTION__);
        if (!pv->current->size) {
          dprintk(" -> fail\n");
        } else {
          uint8_t byte = pv->current->data[0];
          dprintk(" -> %02x\n", byte);
          bc_set_reg(&pv->vm, bit_get_mask(op, 0, 4), byte);
          pv->current->data++;
          pv->current->size--;
          bc_skip(&pv->vm);
        }
        continue;

      case 4: // txn_byte_push
        dprintk("%s byte push\n", __FUNCTION__,
                bc_get_reg(&pv->vm, bit_get_mask(op, 0, 4)));

        assert(pv->current->size);

        pv->current->data[0] = bc_get_reg(&pv->vm, bit_get_mask(op, 0, 4));
        pv->current->data++;
        pv->current->size--;
        continue;

      case 5: // txn_byte_is_last
        dprintk("%s byte is last: %d %s\n", __FUNCTION__,
                pv->current->size, pv->current->size == 1 ? "yes" : "no");
        bc_set_reg(&pv->vm, bit_get_mask(op, 0, 4), pv->current->size == 1);
        continue;

      case 6: // txn_byte_is_done
        dprintk("%s byte is done: %d %s\n", __FUNCTION__,
                pv->current->size, pv->current->size == 0 ? "yes" : "no");
        bc_set_reg(&pv->vm, bit_get_mask(op, 0, 4), pv->current->size == 0);
        continue;

      case 7: { // txn_done
        dprintk("%s txn done %d\n", __FUNCTION__,
                bit_get_mask(op, 0, 8));

        LOCK_SPIN_IRQ_SCOPED(&dev->lock);

        struct dev_i2c_ctrl_transfer_s *rq = pv->current;

        pv->current = NULL;

        rq->err = -bit_get_mask(op, 0, 8);
        kroutine_exec(&rq->kr);

        continue;
      }
      }

    case 2: { // IO
      size_t io = bit_get(op, 5);

      switch (bit_get_mask(op, 6, 2)) {
      case 0: { // set
        bool_t imm = bit_get(op, 4);
        bool_t value = imm
          ? bit_get_mask(op, 0, 1)
          : (bc_get_reg(&pv->vm, bit_get_mask(op, 0, 4)) & 1);
        dprintk("%s %s_set %s %d\n", __FUNCTION__,
                io ? "sda" : "scl",
                imm ? "imm" : "reg",
                value);

        if (HAS_GPIO_SYNC(&pv->gpio, set_output)) {
#if HAS_SYNC
          // Sync
          dev_gpio_out(&pv->gpio, pv->pin_id[io], value);
          continue;
#endif
        } else {
#if HAS_ASYNC
          // Async
          pv->gpio_rq.io_first = pv->pin_id[io];
          pv->gpio_rq.io_last = pv->pin_id[io];
          pv->gpio_rq.output.set_mask
            = pv->gpio_rq.output.clear_mask
            = value ? dev_gpio_mask1 : dev_gpio_mask0;
          pv->gpio_rq.type = DEV_GPIO_SET_OUTPUT;
          DEVICE_OP(&pv->gpio, request, &pv->gpio_rq);

          LOCK_SPIN_IRQ_SCOPED(&dev->lock);

          pv->state = I2C_BB_WAIT_GPIO_OUT;

          return;
#endif
        }
        break;
      }

      case 1: { // get
        pv->reg = bit_get_mask(op, 0, 4);

        dprintk("%s get %s -> r%d\n", __FUNCTION__,
                io ? "sda" : "scl",
                pv->reg);

        if (HAS_GPIO_SYNC(&pv->gpio, get_input)) {
#if HAS_SYNC
          // Sync
          bc_set_reg(&pv->vm, pv->reg,
                     dev_gpio_input(&pv->gpio, pv->pin_id[io], NULL) & 1);

          dprintk("%s data %d\n", __FUNCTION__,
                  bc_get_reg(&pv->vm, pv->reg));

          continue;
#endif
        } else {
#if HAS_ASYNC
          // Async
          pv->gpio_rq.io_first = pv->pin_id[io];
          pv->gpio_rq.io_last = pv->pin_id[io];
          pv->gpio_rq.type = DEV_GPIO_GET_INPUT;
          pv->gpio_rq.input.data = pv->gpio_input;
          DEVICE_OP(&pv->gpio, request, &pv->gpio_rq);

          LOCK_SPIN_IRQ_SCOPED(&dev->lock);

          pv->state = I2C_BB_WAIT_GPIO_IN;

          return;
#endif
        }
        break;
      }

      case 2: { // mode
        enum dev_pin_driving_e mode = bit_get_mask(op, 0, 5);

        dprintk("%s mode %s %N\n", __FUNCTION__,
                io ? "sda" : "scl", mode, ENUM_DESC_DEV_PIN_DRIVING_E);

        if (HAS_GPIO_SYNC(&pv->gpio, set_mode)) {
#if HAS_SYNC
            dev_gpio_mode(&pv->gpio, pv->pin_id[io], mode);
            continue;
#endif
          } else {
#if HAS_ASYNC
            // Async
            pv->gpio_rq.io_first = pv->pin_id[io];
            pv->gpio_rq.io_last = pv->pin_id[io];
            pv->gpio_rq.type = DEV_GPIO_MODE;
            pv->gpio_rq.mode.mask = dev_gpio_mask1;
            pv->gpio_rq.mode.mode = mode;
            DEVICE_OP(&pv->gpio, request, &pv->gpio_rq);

            LOCK_SPIN_IRQ_SCOPED(&dev->lock);

            pv->state = I2C_BB_WAIT_GPIO_OUT;
            return;
#endif
          }
        break;
      }
      }
    }

    case 3: { // Yield
      LOCK_SPIN_IRQ_SCOPED(&dev->lock);
      pv->state = I2C_BB_IDLE;
      kroutine_exec(&pv->vm_runner);
      return;
    }
    }
  }
}

#if HAS_ASYNC
static KROUTINE_EXEC(i2c_bb_gpio_done)
{
  struct i2c_bb_ctx_s *pv = KROUTINE_CONTAINER(kr, *pv, gpio_rq.base.kr);
  struct device_s *dev = pv->gpio_rq.pvdata;

  dprintk("%s\n", __FUNCTION__);

  LOCK_SPIN_IRQ_SCOPED(&dev->lock);
  assert(pv->state == I2C_BB_WAIT_GPIO_IN
         || pv->state == I2C_BB_WAIT_GPIO_OUT);

  if (pv->state == I2C_BB_WAIT_GPIO_IN) {
    bool_t data = pv->gpio_input[0] & 1;
    dprintk("%s data: %d\n", __FUNCTION__, data);
    bc_set_reg(&pv->vm, pv->reg, data);
  }

  pv->state = I2C_BB_IDLE;
  kroutine_exec(&pv->vm_runner);
}
#endif

static KROUTINE_EXEC(i2c_bb_timer_done)
{
  struct i2c_bb_ctx_s *pv = KROUTINE_CONTAINER(kr, *pv, timer_rq.base.kr);
  struct device_s *dev = pv->timer_rq.pvdata;

  dprintk("%s\n", __FUNCTION__);

  LOCK_SPIN_IRQ_SCOPED(&dev->lock);
  assert(pv->state == I2C_BB_WAIT_TIMER);

  pv->state = I2C_BB_IDLE;
  kroutine_exec(&pv->vm_runner);
}

static DEV_I2C_CTRL_TRANSFER(i2c_bb_transfer)
{
  struct device_s *dev = accessor->dev;
  struct i2c_bb_ctx_s *pv = dev->drv_pv;

  LOCK_SPIN_IRQ_SCOPED(&dev->lock);
  assert(pv->current == NULL);

  pv->current = tr;

  if (pv->state == I2C_BB_WAIT_RQ) {
    dprintk("%s wait rq OK\n", __FUNCTION__);
    pv->state = I2C_BB_IDLE;
    kroutine_exec(&pv->vm_runner);
  }
}

#define i2c_bb_use dev_use_generic

static DEV_INIT(i2c_bb_init)
{
  struct i2c_bb_ctx_s *pv;
  error_t err;
  uintptr_t bitrate;

  pv = mem_alloc(sizeof(*pv), mem_scope_sys);
  if (!pv)
    return -ENOMEM;

  memset(pv, 0, sizeof(*pv));

  dev->drv_pv = pv;

  err = device_get_param_dev_accessor(dev, "timer", &pv->timer.base, DRIVER_CLASS_TIMER);
  if (err)
    goto free_pv;

  err = device_get_param_dev_accessor(dev, "gpio", &pv->gpio.base, DRIVER_CLASS_GPIO);
  if (err)
    goto put_timer;

  err = device_res_gpio_map(dev, "scl sda", pv->pin_id, NULL);
  if (err)
    goto err_gpio;

  err = dev_drv_i2c_ctrl_context_init(dev, &pv->i2c_ctrl_ctx);
  if (err)
    goto err_i2c;

  err = device_get_param_uint(dev, "bitrate", &bitrate);
  if (err)
    bitrate = 10000;
  dev_timer_init_sec(&pv->timer, &pv->timer_rq.delay, 0, 1, bitrate * 2);

  pv->current = NULL;

#if HAS_ASYNC
  pv->gpio_rq.pvdata = dev;
  dev_gpio_rq_init(&pv->gpio_rq, i2c_bb_gpio_done);
#endif

  pv->timer_rq.pvdata = dev;
  dev_timer_rq_init(&pv->timer_rq, i2c_bb_timer_done);
  kroutine_init_deferred(&pv->vm_runner, i2c_bb_runner);

  bc_init(&pv->vm, &i2c_bb_io_bytecode);
  bc_set_pc(&pv->vm, &i2c_bb_start);
  pv->state = I2C_BB_IDLE;
  kroutine_exec(&pv->vm_runner);

  return 0;

 err_i2c:
 err_gpio:
 put_gpio:
  device_put_accessor(&pv->gpio.base);
 put_timer:
  device_put_accessor(&pv->timer.base);
 free_pv:
  mem_free(pv);
  return err;
}

static DEV_CLEANUP(i2c_bb_cleanup)
{
  struct i2c_bb_ctx_s *pv = dev->drv_pv;

  if (pv->current)
    return -EBUSY;

  device_put_accessor(&pv->timer.base);
  device_put_accessor(&pv->gpio.base);
  dev_drv_i2c_ctrl_context_cleanup(&pv->i2c_ctrl_ctx);

  mem_free(pv);

  return 0;
}

DRIVER_DECLARE(i2c_bitbang_drv, 0, "I2C Bitbang", i2c_bb,
               DRIVER_I2C_CTRL_METHODS(i2c_bb));

DRIVER_REGISTER(i2c_bitbang_drv);


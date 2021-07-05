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
    License along with MutekH; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
    02110-1301 USA.

    Copyright Alexandre Becoulet <alexandre.becoulet@free.fr> (c) 2013
    Copyright Vincent DEFILIPPI <vincentdefilippi@gmail.com> (c) 2016
*/

#include <device/device.h>
#include <device/driver.h>
#include <device/resources.h>
#include <device/class/i2c.h>
#include <device/class/timer.h>
#include <device/request.h>

#include <mutek/bytecode.h>
#include <mutek/printk.h>

#include <assert.h>

#if defined(CONFIG_MUTEK_CONTEXT_SCHED)
# include <mutek/scheduler.h>
#endif
#include <hexo/lock.h>
#include <hexo/bit.h>
#include <hexo/interrupt.h>
#include <hexo/endian.h>

#ifdef CONFIG_DEVICE_I2C_REQUEST

enum device_i2c_ret_e {
  DEVICE_I2C_IDLE,
  DEVICE_I2C_CONTINUE,
  DEVICE_I2C_WAIT,
  DEVICE_I2C_RESET,
};

static enum device_i2c_ret_e device_i2c_ctrl_sched(struct dev_i2c_ctrl_context_s *q);
static void device_i2c_ctrl_run(struct dev_i2c_ctrl_context_s *q);
static enum device_i2c_ret_e device_i2c_ctrl_end(struct dev_i2c_ctrl_context_s *q,
                                                 struct dev_i2c_ctrl_rq_s *rq);
static enum device_i2c_ret_e device_i2c_ctrl_transfer(struct dev_i2c_ctrl_context_s *q,
                                                      struct dev_i2c_ctrl_rq_s *rq,
                                                      uint8_t *data, uint16_t size,
                                                      enum dev_i2c_op_e type);

static inline bool_t access_is_conditional(uint16_t op)
{
  return (bit_get(op, 12));
}

static inline bool_t access_is_read_reg(uint16_t op)
{
  return (bit_get(op, 13) && bit_get(op, 8));
}

static inline bool_t op_has_delay(uint16_t op)
{
  return bit_get_mask(op, 4, 1);
}


static KROUTINE_EXEC(device_i2c_ctrl_transfer_end)
{
  struct dev_i2c_ctrl_transfer_s  *tr = KROUTINE_CONTAINER(kr, *tr, kr);
  struct dev_i2c_ctrl_rq_s   *rq = tr->pvdata;
  struct dev_i2c_ctrl_context_s     *q = device_i2c_ctrl_context(rq->ctrl);

  lock_spin_irq(&q->lock);

# ifdef CONFIG_DEVICE_I2C_BYTECODE
  if (rq->bytecode)
    {
      struct dev_i2c_ctrl_bytecode_rq_s   *bcrq = dev_i2c_ctrl_bytecode_rq_s_cast(rq);

      if (tr->type == DEV_I2C_RESET)
        {
          assert(q->tr_in_progress == 0);
          device_i2c_ctrl_end(q, rq);
          device_i2c_ctrl_run(q);
          return;
        }

      if (tr->err != 0)
        {
          if (!access_is_conditional(q->op)
              || (tr->err != -EHOSTUNREACH && tr->err != -EAGAIN))
            {
              rq->error = tr->err;
              if (device_i2c_ctrl_end(q, rq) == DEVICE_I2C_RESET)
                {
                  q->tr_in_progress = 0;
                  lock_release_irq(&q->lock);
                  return;
                }
            }
          q->tr_in_progress = 0;
          device_i2c_ctrl_run(q);
          return;
        }

      if ((tr->type & _DEV_I2C_ENDING_MASK) == _DEV_I2C_STOP)
        q->tr_in_progress = 0;

      if (access_is_conditional(q->op))
        bc_skip(&bcrq->vm);

      device_i2c_ctrl_run(q);
    }
# endif

# ifdef CONFIG_DEVICE_I2C_TRANSACTION
  if (!rq->bytecode)
  {
    struct dev_i2c_ctrl_transaction_rq_s   *trq = dev_i2c_ctrl_transaction_rq_s_cast(rq);

    if (tr->err == 0 && trq->transfer_index + 1 != trq->transfer_count)
      trq->transfer_index++;
    else
    {
      rq->error = tr->err;
      device_i2c_ctrl_end(q, rq);
    }

    device_i2c_ctrl_run(q);
  }
# endif
}

static enum device_i2c_ret_e
device_i2c_ctrl_transfer(struct dev_i2c_ctrl_context_s *q,
                         struct dev_i2c_ctrl_rq_s *rq,
                         uint8_t *data, uint16_t size,
                         enum dev_i2c_op_e type)
{
  struct dev_i2c_ctrl_transfer_s  *tr = &q->transfer;

  tr->data = data;
  tr->size = size;
  tr->saddr = rq->saddr;
  tr->type = type;
  tr->pvdata = rq;
  kroutine_init_deferred(&tr->kr, &device_i2c_ctrl_transfer_end);

  DEVICE_OP(rq->ctrl, transfer, tr);

  return DEVICE_I2C_WAIT;
}


static enum device_i2c_ret_e
device_i2c_ctrl_end(struct dev_i2c_ctrl_context_s *q,
                    struct dev_i2c_ctrl_rq_s *rq)
{
  assert(rq->enqueued);
# ifdef CONFIG_DEVICE_I2C_BYTECODE_TIMER
  assert(q->timeout == NULL);
# endif

  if (rq == q->current)
    {
#ifdef CONFIG_DEVICE_I2C_BYTECODE
      if (rq->bytecode && q->tr_in_progress)
        {
          device_i2c_ctrl_transfer(q, rq, NULL, 0, DEV_I2C_RESET);
          return DEVICE_I2C_RESET;
        }
#endif
      q->current = NULL;
    }
  else
    {
      assert(q->current == NULL);
      __dev_rq_queue_remove(&q->queue, &rq->base);
    }

  rq->enqueued = 0;

# ifdef CONFIG_DEVICE_I2C_BYTECODE_TIMER
  if (device_check_accessor(&q->timer.base)
#  ifdef CONFIG_DEVICE_SPI_TRANSACTION
      && rq->bytecode
#  endif
      )
    device_stop(&q->timer.base);
# endif

  kroutine_exec(&rq->base.kr);

  return DEVICE_I2C_CONTINUE;
}

#  ifdef CONFIG_DEVICE_I2C_BYTECODE_TIMER
static KROUTINE_EXEC(device_i2c_ctrl_timeout)
{
  struct dev_timer_rq_s *trq = KROUTINE_CONTAINER(kr, *trq, base.kr);
  struct dev_i2c_ctrl_context_s *q = trq->pvdata;

  lock_spin_irq(&q->lock);

  assert(q->timeout != NULL);
  q->timeout = NULL;

  device_i2c_ctrl_run(q);
}

static enum device_i2c_ret_e
device_i2c_ctrl_delay(struct dev_i2c_ctrl_context_s *q,
                      struct dev_i2c_ctrl_bytecode_rq_s *rq)
{
  struct dev_timer_rq_s *trq = &q->timer_rq;
  error_t err = -ETIMEDOUT;

  assert(q->timeout == NULL);

  /* enqueue new timer request */
  if (device_check_accessor(&q->timer.base))
    {
      trq->deadline = rq->sleep_before;
      trq->delay = 0;
      trq->rev = 0;
      dev_timer_rq_init(trq, device_i2c_ctrl_timeout);
      trq->pvdata = q;

      err = DEVICE_OP(&q->timer, request, trq);

      switch (err)
        {
        case 0:      /* wait for timeout interrupt */
          q->timeout = rq;
          return DEVICE_I2C_WAIT;

        case -ETIMEDOUT:
          return DEVICE_I2C_CONTINUE;

        default:
          break;
        }
    }

  rq->error = err;
  return device_i2c_ctrl_end(q, &rq->base);
}
#  endif

static enum device_i2c_ret_e
device_i2c_ctrl_sched(struct dev_i2c_ctrl_context_s *q)
{
  struct dev_i2c_ctrl_rq_s *rq = NULL;
  assert(q->current == NULL);

  /* find next candidate request in queue */
#  ifdef CONFIG_DEVICE_I2C_BYTECODE_TIMER
  assert(q->timeout == NULL);

  if (device_check_accessor(&q->timer.base))
    {
      dev_timer_value_t t;
      DEVICE_OP(&q->timer, get_value, &t, 0);

      GCT_FOREACH_NOLOCK(dev_request_queue, &q->queue, item, {
          struct dev_i2c_ctrl_rq_s * const rqitem
            = dev_i2c_ctrl_rq_s_cast(item);
          struct dev_i2c_ctrl_bytecode_rq_s * const bcrq
            = dev_i2c_ctrl_bytecode_rq_s_cast(rqitem);
          /* FIXME use dev_timer_check_timeout */
          if (!rqitem->bytecode || bcrq->sleep_before <= t)
            {
              __dev_rq_queue_remove(&q->queue, item);
              rq = rqitem;
              goto found;
            }

          if (rq == NULL || bcrq->sleep_before
              < dev_i2c_ctrl_bytecode_rq_s_cast(rq)->sleep_before)
            rq = rqitem;
      });

      if (rq == NULL)
        return DEVICE_I2C_IDLE;

      return device_i2c_ctrl_delay(q, dev_i2c_ctrl_bytecode_rq_s_cast(rq));
    }
#  endif

  rq = dev_i2c_ctrl_rq_s_cast(__dev_rq_queue_pop(&q->queue));
  if (rq == NULL)
    return DEVICE_I2C_IDLE;

found:
  q->current = rq;

  return DEVICE_I2C_CONTINUE;
}

# ifdef CONFIG_DEVICE_I2C_BYTECODE
static
enum device_i2c_ret_e
device_i2c_bytecode_exec(struct dev_i2c_ctrl_context_s *q,
                         struct dev_i2c_ctrl_bytecode_rq_s *rq)
{
  error_t err;
  uint16_t op;

  __unused__ enum dev_i2c_op_e type;

  lock_release_irq(&q->lock);

  for (err = 0; err == 0; )
    {
      op = bc_run(&rq->vm);
      q->op = op;
      if (!(op & 0x8000))       /* bytecode end */
        {
          if (op)
            err = -EINVAL;
          break;
        }

      if (bit_get(op, 14))
        {
          /* Conditionnal operations without the sync flag are forbidden. */
          assert(!access_is_conditional(op) || bit_get(op, 11));

          type = bit_get_mask(op, 8, 4);

          if (q->tr_in_progress)
            {
              /* Change of direction during continuous transfer is forbidden. */
              assert(!(((q->last_type == DEV_I2C_WRITE_CONTINUOUS ||
                    q->last_type == DEV_I2C_WRITE_CONTINUOUS_SYNC) && (type & _DEV_I2C_READ_OP)) ||
                  ((q->last_type == DEV_I2C_READ_CONTINUOUS ||
                    q->last_type == DEV_I2C_READ_CONTINUOUS_SYNC) && !(type & _DEV_I2C_READ_OP))));
            }
          q->last_type = type;

          uint8_t   *data;
          uint8_t   reg = (op >> 4) & 0xf;
          uint16_t  size = op & 0xf;
          if (bit_get(op, 13))
            {
              /* data from/to registers */
              size++;
              assert(reg * 4 + size <= 16 * 4);
              data = bc_get_bytepack(&rq->vm, reg);
            }
          else
            {
              /* data from/to memory */
              size = (uint16_t)bc_get_reg(&rq->vm, size);
              data = (uint8_t *)bc_get_reg(&rq->vm, reg);
            }

          q->tr_in_progress = 1;
          lock_spin_irq(&q->lock);
          return device_i2c_ctrl_transfer(q, &rq->base, data, size, type);
        }

      if (op & 0x0800)          /* gpio */
        {
          err = -ENOTSUP;
#  ifdef CONFIG_DEVICE_I2C_BYTECODE_GPIO
          if (!device_check_accessor(&rq->gpio.base))
            continue;

          uint_fast8_t i = (op >> 5) & 0xf;
          gpio_id_t id = rq->gpio_map[i];
          gpio_width_t w = rq->gpio_wmap[i];
          uint8_t value[8];

          if (op & 0x0200)  /* gpioget */
            {
              err = DEVICE_OP(&rq->gpio, get_input, id,
                              id + w - 1, value);
              bc_set_reg(&rq->vm, op & 0xf,
                         endian_le32_na_load(value) & ((1 << w) - 1));
            }
          else if (op & 0x0400) /* gpioset */
            {
              endian_le32_na_store(value, bc_get_reg(&rq->vm, op & 0xf));
              err = DEVICE_OP(&rq->gpio, set_output, id,
                              id + w - 1, value, value);
            }
          else              /* gpiomode */
            {
              err = DEVICE_OP(&rq->gpio, set_mode, id,
                              id + w - 1, dev_gpio_mask1, op & 0x1f);
            }
#  endif
          continue;
        }

      switch ((op & 0x0700) >> 8)
        {
        case 0: // (no)delay, wait
        case 1: // yield
#  ifdef CONFIG_DEVICE_I2C_BYTECODE_TIMER
          if (op_has_delay(op))
            {
              if (!device_check_accessor(&q->timer.base))
                {
                  err = -ETIMEDOUT;
                  continue;
                }
              dev_timer_value_t t = 0;
              DEVICE_OP(&q->timer, get_value, &t, 0);
              rq->sleep_before = t + bc_get_reg(&rq->vm, op & 0xf);
            }
#  endif
          switch (bit_get_mask(op, 7, 2))
            {
            case 2:    /* yield */
            case 3:    /* yieldc */
              assert(q->tr_in_progress == 0);
              lock_spin_irq(&q->lock);
              rq->wakeup_able = bit_get(op, 7);
              if (rq->wakeup_able && rq->wakeup)
                {
                  rq->wakeup = 0;
                  bc_skip(&rq->vm);
                  lock_release_irq(&q->lock);
                  continue;
                }
              __dev_rq_queue_pushback(&q->queue, &rq->base.base);
              q->current = NULL;
              return DEVICE_I2C_CONTINUE;

            case 1:  /* wait */
              lock_spin_irq(&q->lock);
#  ifdef CONFIG_DEVICE_I2C_BYTECODE_TIMER
              return device_i2c_ctrl_delay(q, rq);
#  else
              return DEVICE_I2C_CONTINUE;
#  endif

#  ifdef CONFIG_DEVICE_I2C_BYTECODE_TIMER
            case 0:    /* delay */
              if (!op_has_delay(op))
                rq->sleep_before = 0;
              continue;
#  endif
            default:
              continue;
            }
          continue;

        case 7: // address
          if (!bit_get(op, 4))  /* addr_get */
            bc_set_reg(&rq->vm, op & 0xf, rq->base.saddr);
          else              /* addr_set */
            rq->base.saddr = bc_get_reg(&rq->vm, op & 0xf);
          continue;
        }

      err = -EINVAL;  /* invalid op */
    }

  lock_spin_irq(&q->lock);

  rq->error = err;
  return device_i2c_ctrl_end(q, &rq->base);
}
# endif

# ifdef CONFIG_DEVICE_I2C_TRANSACTION
static enum device_i2c_ret_e
device_i2c_transaction_exec(struct dev_i2c_ctrl_context_s *q,
                            struct dev_i2c_ctrl_transaction_rq_s *rq)
{
  struct dev_i2c_ctrl_transaction_data_s *tr;

  tr = &rq->transfer[rq->transfer_index];
  uint8_t             *data = tr->data;
  uint16_t            size = tr->size;
  enum dev_i2c_op_e   type;

  switch (tr->type)
    {
      case DEV_I2C_CTRL_TRANSACTION_READ:
        type = _DEV_I2C_READ_OP;
        break;

      case DEV_I2C_CTRL_TRANSACTION_WRITE:
        type = 0;
        break;

    default:
      UNREACHABLE();
    }

  if (rq->transfer_index + 1 == rq->transfer_count)
    type |= _DEV_I2C_STOP | _DEV_I2C_SYNC;
  else if (tr->type != rq->transfer[rq->transfer_index + 1].type)
    type |= _DEV_I2C_RESTART;
  else
    type |= _DEV_I2C_CONTINUOUS;

  return device_i2c_ctrl_transfer(q, &rq->base, data, size, type);
}
# endif

static
void device_i2c_ctrl_run(struct dev_i2c_ctrl_context_s *q)
{
  enum device_i2c_ret_e r;
  do {
    r = DEVICE_I2C_IDLE;
    struct dev_i2c_ctrl_rq_s *rq = q->current;
    if (rq != NULL)
      {
        if (rq->bytecode)
          {
#ifdef CONFIG_DEVICE_I2C_BYTECODE
            r = device_i2c_bytecode_exec(q, dev_i2c_ctrl_bytecode_rq_s_cast(rq));
#endif
          }
        else
          {
#ifdef CONFIG_DEVICE_I2C_TRANSACTION
            r = device_i2c_transaction_exec(q, dev_i2c_ctrl_transaction_rq_s_cast(rq));
#endif
          }
      }
    else if (!dev_rq_queue_isempty(&q->queue))
      r = device_i2c_ctrl_sched(q);
  } while (r == DEVICE_I2C_CONTINUE);

  lock_release_irq(&q->lock);
}

static
KROUTINE_EXEC(device_i2c_ctrl_resume)
{
  struct dev_i2c_ctrl_context_s *q = KROUTINE_CONTAINER(kr, *q, kr);

  lock_spin_irq(&q->lock);

  struct dev_i2c_ctrl_rq_s *rq = q->current;
  assert(rq != NULL);

  device_i2c_ctrl_run(q);
}

static bool_t device_i2c_ctrl_entry(struct dev_i2c_ctrl_context_s *q,
                                    struct dev_i2c_ctrl_rq_s *rq)
{
  if (q->current != NULL)
    return 1;

#ifdef CONFIG_DEVICE_I2C_BYTECODE_TIMER
  if (q->timeout != NULL)
    {
      if (DEVICE_OP(&q->timer, cancel, &q->timer_rq))
        return 1;
      q->timeout = NULL;
    }
#endif

  q->current = rq;
  kroutine_init_deferred(&q->kr, device_i2c_ctrl_resume);
  kroutine_exec(&q->kr);

  return 0;
}


# ifdef CONFIG_DEVICE_I2C_TRANSACTION
void dev_i2c_transaction_start(struct device_i2c_ctrl_s *ctrl,
                               struct dev_i2c_ctrl_transaction_rq_s *rq)
{
  rq->base.ctrl = ctrl;
  struct dev_i2c_ctrl_context_s *q = device_i2c_ctrl_context(ctrl);

  lock_spin_irq(&q->lock);

  assert(!rq->base.enqueued);

  rq->error = 0;
  rq->base.enqueued = 1;
  rq->base.bytecode = 0;
  rq->transfer_index = 0;

  if (device_i2c_ctrl_entry(q, &rq->base))
    __dev_rq_queue_pushback(&q->queue, &rq->base.base);

  lock_release_irq(&q->lock);
}
# endif

# ifdef CONFIG_DEVICE_I2C_BYTECODE

error_t dev_i2c_bytecode_start_va(struct device_i2c_ctrl_s *ctrl,
                               struct dev_i2c_ctrl_bytecode_rq_s *rq,
                               const void *pc, uint16_t mask, va_list ap)

{
  error_t err = -EBUSY;

  rq->base.ctrl = ctrl;
  struct dev_i2c_ctrl_context_s *q = device_i2c_ctrl_context(ctrl);

  lock_spin_irq(&q->lock);

  if (!rq->base.enqueued)
    {
      err = 0;

#  ifdef CONFIG_DEVICE_I2C_BYTECODE_TIMER
      if (device_check_accessor(&q->timer.base) &&
          (rq->error = device_start(&q->timer.base)))
        {
          kroutine_exec(&rq->base.base.kr);
          goto err;
        }
      rq->sleep_before = 0;
#  endif

      if (pc != NULL)
        bc_set_pc(&rq->vm, pc);

      bc_set_regs_va(&rq->vm, mask, ap);

      rq->error = 0;
      rq->base.enqueued = 1;
      rq->base.bytecode = 1;
      rq->wakeup = 0;
      rq->wakeup_able = 0;

      if (device_i2c_ctrl_entry(q, &rq->base))
        __dev_rq_queue_pushback(&q->queue, &rq->base.base);
    }

err:
  lock_release_irq(&q->lock);

  return err;
}

error_t dev_i2c_bytecode_start(struct device_i2c_ctrl_s *ctrl,
                               struct dev_i2c_ctrl_bytecode_rq_s *rq,
                               const void *pc, uint16_t mask, ...)
{
  va_list ap;
  va_start(ap, mask);
  error_t err = dev_i2c_bytecode_start_va(ctrl, rq, pc, mask, ap);
  va_end(ap);

  return err;
}

error_t device_i2c_bytecode_wakeup(struct device_i2c_ctrl_s *ctrl,
                                   struct dev_i2c_ctrl_bytecode_rq_s *rq)
{
  struct dev_i2c_ctrl_context_s *q = device_i2c_ctrl_context(ctrl);
  error_t err = 0;

  lock_spin_irq(&q->lock);

  if (!rq->base.enqueued)
    {
      err = -EBUSY;
    }
  else if (q->current != &rq->base && rq->wakeup_able)
    {
#  ifdef CONFIG_DEVICE_I2C_BYTECODE_TIMER
      rq->sleep_before = 0;
#  endif
      bc_skip(&rq->vm);
      rq->wakeup_able = 0;
      if (!device_i2c_ctrl_entry(q, &rq->base))
        __dev_rq_queue_remove(&q->queue, &rq->base.base);
    }
  else
    {
      rq->wakeup = 1;
    }

  lock_release_irq(&q->lock);
  return err;
}
# endif

#ifdef CONFIG_DEVICE_I2C_REQUEST
error_t dev_drv_i2c_ctrl_context_init_(struct device_s *dev, struct dev_i2c_ctrl_context_s *q)
{
  __unused__ error_t err;

# ifdef CONFIG_DEVICE_I2C_BYTECODE_TIMER
  if (device_get_param_dev_accessor(dev, "timer", &q->timer.base, DRIVER_CLASS_TIMER))
    device_init_accessor(&q->timer.base);

  q->timeout = NULL;
# endif

  q->current = NULL;
  dev_rq_queue_init(&q->queue);
  lock_init_irq(&q->lock);
  memset(&q->transfer, 0, sizeof(q->transfer));
#ifdef CONFIG_DEVICE_I2C_BYTECODE
  q->tr_in_progress = 0;
#endif
  return 0;
}

void dev_drv_i2c_ctrl_context_cleanup_(struct dev_i2c_ctrl_context_s *q)
{
  lock_destroy_irq(&q->lock);
  dev_rq_queue_destroy(&q->queue);
# ifdef CONFIG_DEVICE_I2C_BYTECODE_TIMER
  device_put_accessor(&q->timer.base);
# endif
}
#endif

static
error_t dev_drv_i2c_init(struct device_s *dev,
                         struct dev_i2c_ctrl_rq_s *rq,
                         struct device_i2c_ctrl_s *ctrl)
{
  struct dev_resource_s *r;

  r = device_res_get(dev, DEV_RES_I2C_ADDR, 0);
  if (r == NULL)
      return -ENOENT;

  if (device_get_accessor_by_path(&ctrl->base, &dev->node,
        r->u.i2c_addr.ctrl, DRIVER_CLASS_I2C_CTRL))
    return -ENOENT;

  rq->saddr = r->u.i2c_addr.addr;

  return 0;
}

# ifdef CONFIG_DEVICE_I2C_BYTECODE
error_t dev_drv_i2c_bytecode_init(struct device_s *dev,
                                  struct dev_i2c_ctrl_bytecode_rq_s *rq,
                                  const struct bc_descriptor_s *desc,
                                  struct device_i2c_ctrl_s *ctrl,
                                  struct device_gpio_s **gpio,
                                  struct device_timer_s **timer)
{
  dev_i2c_bytecode_init(rq);

  error_t err = dev_drv_i2c_init(dev, &rq->base, ctrl);
  if (err)
    return err;


  if (timer != NULL)
    {
# ifdef CONFIG_DEVICE_I2C_BYTECODE_TIMER
      struct dev_i2c_ctrl_context_s *q = device_i2c_ctrl_context(ctrl);
      err = -ENOENT;
      if (!device_check_accessor(&q->timer.base))
        goto err;
      *timer = &q->timer;
# else
      err = -ENOTSUP;
      goto err;
# endif
    }

  if (gpio != NULL)
    {
# ifdef CONFIG_DEVICE_I2C_BYTECODE_GPIO
      err = -ENOENT;
      if (device_get_param_dev_accessor(dev, "gpio", &rq->gpio.base, DRIVER_CLASS_GPIO))
        goto err;
      *gpio = &rq->gpio;
# else
      err = -ENOTSUP;
      goto err;
# endif
    }

  bc_init(&rq->vm, desc);

  return 0;
 err:
  device_put_accessor(&ctrl->base);
  return err;
}

void dev_drv_i2c_bytecode_cleanup(struct device_i2c_ctrl_s *ctrl,
                                  struct dev_i2c_ctrl_bytecode_rq_s *rq)
{
#  ifdef CONFIG_DEVICE_I2C_BYTECODE_GPIO
  device_put_accessor(&rq->gpio.base);
#  endif
  device_put_accessor(&ctrl->base);
}

# endif


# ifdef CONFIG_DEVICE_I2C_TRANSACTION
error_t dev_drv_i2c_transaction_init(struct device_s *dev,
                                    struct dev_i2c_ctrl_transaction_rq_s *rq,
                                    struct device_i2c_ctrl_s *ctrl)
{
  dev_i2c_transaction_init(rq);

  return dev_drv_i2c_init(dev, &rq->base, ctrl);
}

void dev_drv_i2c_transaction_cleanup(struct device_i2c_ctrl_s *ctrl,
                                     struct dev_i2c_ctrl_transaction_rq_s *rq)
{
  device_put_accessor(&ctrl->base);
}
# endif

#endif /* CONFIG_DEVICE_I2C_REQUEST */


#if defined(CONFIG_MUTEK_CONTEXT_SCHED)

# ifdef CONFIG_DEVICE_I2C_TRANSACTION
void
dev_i2c_wait_transaction(struct device_i2c_ctrl_s *ctrl,
                         struct dev_i2c_ctrl_transaction_rq_s *rq)
{
  struct dev_request_status_s st;

  dev_request_sched_init(&rq->base.base, &st);
  dev_i2c_transaction_start(ctrl, rq);
  dev_request_sched_wait(&st);
}
# endif

# ifdef CONFIG_DEVICE_I2C_BYTECODE
error_t
dev_i2c_wait_bytecode(struct device_i2c_ctrl_s *ctrl,
                      struct dev_i2c_ctrl_bytecode_rq_s *rq,
                      const void *pc, uint16_t mask, ...)
{
  struct dev_request_status_s st;

  dev_request_sched_init(&rq->base.base, &st);
  va_list ap;
  va_start(ap, mask);
  error_t err = dev_i2c_bytecode_start_va(ctrl, rq, pc, mask, ap);
  va_end(ap);
  if (!err)
    dev_request_sched_wait(&st);
  return err;
}
# endif

#endif


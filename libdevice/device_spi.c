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

*/

#include <device/device.h>
#include <device/driver.h>
#include <device/resources.h>
#include <device/class/spi.h>
#include <device/class/timer.h>
#include <device/request.h>

#include <mutek/bytecode.h>

#ifdef CONFIG_MUTEK_CONTEXT_SCHED
# include <mutek/scheduler.h>
#endif
#include <hexo/lock.h>
#include <hexo/interrupt.h>
#include <hexo/endian.h>

const char dev_spi_ckmode_e[] = ENUM_DESC_DEV_SPI_CKMODE_E;
const char dev_spi_polarity_e[] = ENUM_DESC_DEV_SPI_POLARITY_E;
const char dev_spi_bit_order_e[] = ENUM_DESC_DEV_SPI_BIT_ORDER_E;
const char dev_spi_cs_policy_e[] = ENUM_DESC_DEV_SPI_CS_POLICY_E;

#ifdef CONFIG_DEVICE_SPI_REQUEST

enum device_spi_ret_e {
  DEVICE_SPI_IDLE,
  DEVICE_SPI_CONTINUE,
  DEVICE_SPI_WAIT,
};

static enum device_spi_ret_e device_spi_ctrl_sched(struct dev_spi_ctrl_queue_s *q);
static void device_spi_ctrl_run(struct dev_spi_ctrl_queue_s *q);
static enum device_spi_ret_e device_spi_ctrl_end(struct dev_spi_ctrl_queue_s *q,
                                                 struct dev_spi_ctrl_base_rq_s *rq, error_t err);

static error_t device_spi_ctrl_select(struct dev_spi_ctrl_base_rq_s *rq,
                                      enum dev_spi_cs_policy_e pc)
{
  if (rq->cs_ctrl)
    return DEVICE_OP(rq->ctrl, select, pc, rq->cs_polarity, rq->cs_id);

# ifdef CONFIG_DEVICE_GPIO
  if (rq->cs_gpio)
    {
      const uint8_t *value = NULL;
      switch (pc)
        {
        case DEV_SPI_CS_ASSERT:
          value = rq->cs_polarity == DEV_SPI_ACTIVE_HIGH ?
            dev_gpio_mask1 : dev_gpio_mask0;
          break;

        case DEV_SPI_CS_TRANSFER:
        case DEV_SPI_CS_DEASSERT:
        case DEV_SPI_CS_RELEASE:
          value = rq->cs_polarity == DEV_SPI_ACTIVE_HIGH ?
            dev_gpio_mask0 : dev_gpio_mask1;
          break;
        }

      return DEVICE_OP(&rq->gpio, set_output, rq->cs_id,
                       rq->cs_id, value, value);
    }
# endif

  return pc == DEV_SPI_CS_RELEASE ? 0 : -ENOTSUP;
}

static KROUTINE_EXEC(device_spi_ctrl_transfer_end)
{
  struct dev_spi_ctrl_transfer_s *tr = KROUTINE_CONTAINER(kr, *tr, kr);
  struct dev_spi_ctrl_base_rq_s *rq = tr->pvdata;
  struct dev_spi_ctrl_queue_s *q = DEVICE_OP(rq->ctrl, queue);

  lock_spin_irq(&q->lock);

# ifdef CONFIG_DEVICE_GPIO
  if (rq->cs_gpio && rq->cs_policy == DEV_SPI_CS_TRANSFER)
    device_spi_ctrl_select(rq, DEV_SPI_CS_DEASSERT);
# endif

# ifdef CONFIG_DEVICE_SPI_BYTECODE
  if (tr->err == 0 && rq->bytecode)
    return device_spi_ctrl_run(q);
# endif

  device_spi_ctrl_end(q, rq, tr->err);
}

static enum device_spi_ret_e
device_spi_ctrl_transfer(struct dev_spi_ctrl_queue_s *q,
                         struct dev_spi_ctrl_base_rq_s *rq)
{
  struct dev_spi_ctrl_transfer_s *tr = &q->transfer;
  error_t err;

# ifdef CONFIG_DEVICE_GPIO
  if (rq->cs_gpio && rq->cs_policy == DEV_SPI_CS_TRANSFER &&
      (err = device_spi_ctrl_select(rq, DEV_SPI_CS_ASSERT)))
    goto err;
# endif

  if (q->config != &rq->config)
    {
      if ((err = DEVICE_OP(rq->ctrl, config, &rq->config)))
        goto err;
      q->config = &rq->config;
    }

  tr->pvdata = rq;
  kroutine_init_deferred(&tr->kr, &device_spi_ctrl_transfer_end);

  DEVICE_OP(rq->ctrl, transfer, tr);

  return DEVICE_SPI_WAIT;
 err:
  return device_spi_ctrl_end(q, rq, err);
}

static enum device_spi_ret_e
device_spi_ctrl_end(struct dev_spi_ctrl_queue_s *q,
                    struct dev_spi_ctrl_base_rq_s *rq, error_t err)
{
  assert(rq->enqueued);
# ifdef CONFIG_DEVICE_SPI_BYTECODE_TIMER
  assert(q->timeout == NULL);
# endif

  if (rq == q->current)
    {
      if (
# ifdef CONFIG_DEVICE_GPIO
          rq->cs_gpio ||
# endif
          rq->cs_ctrl)
        device_spi_ctrl_select(rq, DEV_SPI_CS_DEASSERT);
      q->config = NULL;
      q->current = NULL;
    }
  else
    {
      assert(q->current == NULL);
      dev_request_queue_remove(&q->queue, &rq->base);
    }

  rq->err = err;
  rq->enqueued = 0;

# ifdef CONFIG_DEVICE_SPI_BYTECODE_TIMER
  if (device_check_accessor(&q->timer.base))
    device_stop(&q->timer.base);
# endif

  kroutine_exec(&rq->base.kr);

  return DEVICE_SPI_CONTINUE;
}

# ifdef CONFIG_DEVICE_SPI_BYTECODE_TIMER
static KROUTINE_EXEC(device_spi_ctrl_timeout)
{
  struct dev_timer_rq_s *trq = KROUTINE_CONTAINER(kr, *trq, rq.kr);
  struct dev_spi_ctrl_queue_s *q = trq->rq.pvdata;

  lock_spin_irq(&q->lock);

  assert(q->timeout != NULL);
  q->timeout = NULL;

  device_spi_ctrl_run(q);
}

static enum device_spi_ret_e
device_spi_ctrl_delay(struct dev_spi_ctrl_queue_s *q,
                      struct dev_spi_ctrl_bytecode_rq_s *rq)
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
      kroutine_init_deferred(&trq->rq.kr, device_spi_ctrl_timeout);
      trq->rq.pvdata = q;

      err = DEVICE_OP(&q->timer, request, trq);

      switch (err)
        {
        case 0:      /* wait for timeout interrupt */
          q->timeout = rq;
          return DEVICE_SPI_WAIT;

        case -ETIMEDOUT:
          return DEVICE_SPI_CONTINUE;

        default:
          break;
        }
    }

  return device_spi_ctrl_end(q, &rq->base, err);
}
# endif

static enum device_spi_ret_e
device_spi_ctrl_sched(struct dev_spi_ctrl_queue_s *q)
{
  error_t err;
  struct dev_spi_ctrl_base_rq_s *rq = NULL;
  assert(q->current == NULL);

  /* find next candidate request in queue */
# ifdef CONFIG_DEVICE_SPI_BYTECODE_TIMER
  assert(q->timeout == NULL);

  if (device_check_accessor(&q->timer.base))
    {
      dev_timer_value_t t;
      DEVICE_OP(&q->timer, get_value, &t, 0);

      GCT_FOREACH_NOLOCK(dev_request_queue, &q->queue, item, {
          struct dev_spi_ctrl_base_rq_s * const rqitem
            = dev_spi_ctrl_base_rq_s_cast(item);
          struct dev_spi_ctrl_bytecode_rq_s * const bcrq
            = dev_spi_ctrl_bytecode_rq_s_cast(rqitem);

          /* FIXME use dev_timer_check_timeout */
          if (!rqitem->bytecode || bcrq->sleep_before <= t)
            {
              dev_request_queue_remove(&q->queue, item);
              rq = rqitem;
              goto found;
            }

          if (rq == NULL || bcrq->sleep_before
              < dev_spi_ctrl_bytecode_rq_s_cast(rq)->sleep_before)
            rq = rqitem;
      });

      if (rq == NULL)
        return DEVICE_SPI_IDLE;

      return device_spi_ctrl_delay(q, dev_spi_ctrl_bytecode_rq_s_cast(rq));
    }
# endif

  rq = dev_spi_ctrl_base_rq_s_cast(dev_request_queue_pop(&q->queue));
  if (rq == NULL)
    return DEVICE_SPI_IDLE;

found:
  q->current = rq;

  if ((err = device_spi_ctrl_select(rq, rq->cs_policy)))
    return device_spi_ctrl_end(q, rq, err);

  return DEVICE_SPI_CONTINUE;
}

# ifdef CONFIG_DEVICE_SPI_BYTECODE
static enum device_spi_ret_e
device_spi_bytecode_exec(struct dev_spi_ctrl_queue_s *q,
                         struct dev_spi_ctrl_bytecode_rq_s *rq)
{
  error_t err;
  uint16_t op;

  lock_release_irq(&q->lock);

  for (err = 0; err == 0; )
    {
      struct dev_spi_ctrl_transfer_s *tr = &q->transfer;

      op = bc_run(&rq->vm, -1);

      if (!(op & 0x8000))       /* bytecode end */
        {
          if (op)
            err = -EINVAL;
          break;
        }

      switch (op & 0x7000)
        {
        case 0x0000:
          switch (op & 0x0c00)
            {
            case 0x0000:
#  ifdef CONFIG_DEVICE_SPI_BYTECODE_TIMER
              if (op & 0x0080)
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
              switch (op & 0x0300)
                {
                case 0x0000:    /* yield */
                  lock_spin_irq(&q->lock);
                  rq->wakeup_able = !(op & 0x0020);
                  if (rq->wakeup_able && rq->wakeup)
                    {
                      rq->wakeup = 0;
                      bc_skip(&rq->vm);
                      lock_release_irq(&q->lock);
                      continue;
                    }
                  if (
#  ifdef CONFIG_DEVICE_GPIO
                      rq->base.cs_gpio ||
#  endif
                      rq->base.cs_ctrl)
                    device_spi_ctrl_select(&rq->base, DEV_SPI_CS_DEASSERT);
                  dev_request_queue_pushback(&q->queue, &rq->base.base);
                  q->current = NULL;
                  return DEVICE_SPI_CONTINUE;

                case 0x0200: {  /* wait, setcs */
                  uint8_t csp = (op >> 4) & 3;
                  if ((err = device_spi_ctrl_select(&rq->base, csp)))
                    continue;
                  if (op & 0x0040) /* setcs */
                    {
                      rq->base.cs_policy = csp;
                      continue;
                    }
                  lock_spin_irq(&q->lock);
#  ifdef CONFIG_DEVICE_SPI_BYTECODE_TIMER
                  return device_spi_ctrl_delay(q, rq);
#  else
                  return DEVICE_SPI_CONTINUE;
#  endif
                }

#  ifdef CONFIG_DEVICE_SPI_BYTECODE_TIMER
                case 0x0300:    /* delay */
                  if (!(op & 0x0080))
                    rq->sleep_before = 0;
                  continue;
#  endif

                default:
                  continue;
                }

            case 0x0400:
              if (op & 0x0080)  /* brate */
                {
                  rq->base.config.bit_rate = bc_get_reg(&rq->vm, op & 0xf);
                  q->config = NULL;
                }
              else              /* width */
                {
                  rq->base.config.word_width = op & 0x001f;
                  rq->base.config.bit_order = (op >> 5) & 1;
                  q->config = NULL;
                }
              continue;
            case 0x0800:        /* swp, swpl */
            case 0x0c00: {
              uint_fast8_t l = ((op >> 8) & 7) + 1;
              uint_fast8_t src = op & 0xf;
              uint_fast8_t dst = (op >> 4) & 0xf;
              void *addr = src + l >= 16 ? NULL : &rq->vm.v[src];
              q->padding_word = bc_get_reg(&rq->vm, 14);
              tr->data.in_width = sizeof(rq->vm.v[0]);
              tr->data.in = addr;
              tr->data.count = l;
              if (dst + l >= 16)
                {
                  tr->data.out_width = 0;
                  tr->data.out = &q->padding_word;
                }
              else
                {
                  tr->data.out_width = sizeof(rq->vm.v[0]);
                  tr->data.out = &rq->vm.v[dst];
                }
              lock_spin_irq(&q->lock);
              return device_spi_ctrl_transfer(q, &rq->base);
            }
            }
          continue;

        case 0x1000: {
          void *addr = (void*)bc_get_reg(&rq->vm, (op >> 4) & 0xf);
          void *addr2 = (void*)bc_get_reg(&rq->vm, 1 ^ ((op >> 4) & 0xf));
          size_t count = bc_get_reg(&rq->vm, op & 0xf);
          uint_fast8_t width = (op >> 8) & 3;
          q->padding_word = bc_get_reg(&rq->vm, 14);
          if (count == 0)
            continue;
          tr->data.count = count;
          lock_spin_irq(&q->lock);
          switch (op & 0x0c00)
            {
            case 0x0000:  /* pad */
              tr->data.in_width = 0;
              tr->data.out_width = 0;
              tr->data.in = NULL;
              tr->data.out = &q->padding_word;
              return device_spi_ctrl_transfer(q, &rq->base);
            case 0x0400:  /* rdm */
              tr->data.in_width = width + 1;
              tr->data.out_width = 0;
              tr->data.in = addr;
              tr->data.out = &q->padding_word;
              return device_spi_ctrl_transfer(q, &rq->base);
            case 0x0800:  /* wrm */
              tr->data.in_width = 0;
              tr->data.out_width = width + 1;
              tr->data.in = NULL;
              tr->data.out = addr;
              return device_spi_ctrl_transfer(q, &rq->base);
            case 0x0c00:  /* swpm */
              tr->data.in_width = width + 1;
              tr->data.out_width = width + 1;
              tr->data.in = addr;
              tr->data.out = addr2;
              return device_spi_ctrl_transfer(q, &rq->base);
            default:
              UNREACHABLE();
            }
        }

#  ifdef CONFIG_DEVICE_SPI_BYTECODE_GPIO
        case 0x4000:            /* gpio* */
        case 0x2000:
        case 0x3000: {
          err = 0;
          if (!device_check_accessor(&rq->base.gpio.base))
            err = -ENOTSUP;
          else
            {
              gpio_id_t id = rq->gpio_map[(op >> 4) & 0xff];
              gpio_width_t w = rq->gpio_wmap[(op >> 4) & 0xff];
              uint8_t value[8];

              if (op & 0x1000)  /* gpioget */
                {
                  err = DEVICE_OP(&rq->base.gpio, get_input, id, id + w - 1, value);
                  bc_set_reg(&rq->vm, op & 0xf, endian_le32_na_load(value) & ((1 << w) - 1));
                }
              else if (op & 0x4000) /* gpiomode */
                {
                  err = DEVICE_OP(&rq->base.gpio, set_mode, id, id + w - 1, dev_gpio_mask1, op & 0xf);
                }
              else              /* gpioset */
                {
                  endian_le32_na_store(value, bc_get_reg(&rq->vm, op & 0xf));
                  err = DEVICE_OP(&rq->base.gpio, set_output, id, id + w - 1, value, value);
                }
            }
          continue;
        }
#  endif

        }

      err = -EINVAL;            /* invalid op */
    }

  lock_spin_irq(&q->lock);
  return device_spi_ctrl_end(q, &rq->base, err);
}
# endif

static void device_spi_ctrl_run(struct dev_spi_ctrl_queue_s *q)
{
  enum device_spi_ret_e r;

  do {
    r = DEVICE_SPI_IDLE;
    struct dev_spi_ctrl_base_rq_s *rq = q->current;
    if (rq != NULL)
      {
        if (rq->bytecode)
          {
#ifdef CONFIG_DEVICE_SPI_BYTECODE
            r = device_spi_bytecode_exec(q, dev_spi_ctrl_bytecode_rq_s_cast(rq));
#endif
          }
        else
          {
#ifdef CONFIG_DEVICE_SPI_TRANSACTION
            struct dev_spi_ctrl_transaction_rq_s *trq = dev_spi_ctrl_transaction_rq_s_cast(rq);
            memcpy(&q->transfer.data, &trq->data, sizeof(trq->data));
            r = device_spi_ctrl_transfer(q, rq);
#endif
          }
      }
    else if (!dev_request_queue_isempty(&q->queue))
      r = device_spi_ctrl_sched(q);
  } while (r == DEVICE_SPI_CONTINUE);

  lock_release_irq(&q->lock);
}

static KROUTINE_EXEC(device_spi_ctrl_resume)
{
  struct dev_spi_ctrl_queue_s *q = KROUTINE_CONTAINER(kr, *q, kr);
  error_t err;

  lock_spin_irq(&q->lock);

  struct dev_spi_ctrl_base_rq_s *rq = q->current;
  assert(rq != NULL);

  if ((err = device_spi_ctrl_select(rq, rq->cs_policy)))
    device_spi_ctrl_end(q, rq, err);

  device_spi_ctrl_run(q);
}

static bool_t device_spi_ctrl_entry(struct dev_spi_ctrl_queue_s *q,
                                    struct dev_spi_ctrl_base_rq_s *rq)
{
  if (q->current != NULL)
    return 1;

#ifdef CONFIG_DEVICE_SPI_BYTECODE_TIMER
  if (q->timeout != NULL)
    {
      if (DEVICE_OP(&q->timer, cancel, &q->timer_rq))
        return 1;
      q->timeout = NULL;
    }
#endif

  q->current = rq;
  kroutine_init_deferred(&q->kr, device_spi_ctrl_resume);
  kroutine_exec(&q->kr);

  return 0;
}

# ifdef CONFIG_DEVICE_SPI_TRANSACTION
void dev_spi_transaction_start(struct device_spi_ctrl_s *ctrl,
                               struct dev_spi_ctrl_transaction_rq_s *rq)
{
#  ifdef CONFIG_DEVICE_GPIO
  assert(!rq->base.cs_gpio || device_check_accessor(&rq->base.gpio.base));
#  endif

  rq->base.ctrl = ctrl;
  struct dev_spi_ctrl_queue_s *q = DEVICE_OP(ctrl, queue);

  lock_spin_irq(&q->lock);

  assert(!rq->base.enqueued);

  rq->base.err = 0;
  rq->base.enqueued = 1;
  rq->base.bytecode = 0;

  if (device_spi_ctrl_entry(q, &rq->base))
    dev_request_queue_pushback(&q->queue, &rq->base.base);

  lock_release_irq(&q->lock);
}
# endif

# ifdef CONFIG_DEVICE_SPI_BYTECODE
error_t dev_spi_bytecode_start(struct device_spi_ctrl_s *ctrl,
                               struct dev_spi_ctrl_bytecode_rq_s *rq,
                               const void *pc, uint16_t mask, ...)
{
  error_t err = -EBUSY;

#  ifdef CONFIG_DEVICE_GPIO
  assert(!rq->base.cs_gpio || device_check_accessor(&rq->base.gpio.base));
#  endif

  rq->base.ctrl = ctrl;
  struct dev_spi_ctrl_queue_s *q = DEVICE_OP(ctrl, queue);

  lock_spin_irq(&q->lock);

  if (!rq->base.enqueued)
    {
      err = 0;

#  ifdef CONFIG_DEVICE_SPI_BYTECODE_TIMER
      if (device_check_accessor(&q->timer.base) &&
          (rq->base.err = device_start(&q->timer.base)))
        {
          kroutine_exec(&rq->base.base.kr);
          goto err;
        }
#  endif

#  ifdef CONFIG_DEVICE_SPI_BYTECODE_TIMER
      rq->sleep_before = 0;
#  endif

      if (pc != NULL)
        bc_set_pc(&rq->vm, pc);

      va_list ap;
      va_start(ap, mask);
      bc_set_regs_va(&rq->vm, mask, ap);
      va_end(ap);

      rq->base.err = 0;
      rq->base.enqueued = 1;
      rq->base.bytecode = 1;
      rq->wakeup = 0;
      rq->wakeup_able = 0;

      if (device_spi_ctrl_entry(q, &rq->base))
        dev_request_queue_pushback(&q->queue, &rq->base.base);
    }

 err:
  lock_release_irq(&q->lock);

  return err;
}

error_t device_spi_bytecode_wakeup(struct device_spi_ctrl_s *ctrl,
                                   struct dev_spi_ctrl_bytecode_rq_s *rq)
{
  struct dev_spi_ctrl_queue_s *q = DEVICE_OP(ctrl, queue);
  error_t err = 0;

  lock_spin_irq(&q->lock);

  if (!rq->base.enqueued)
    {
      err = -EBUSY;
    }
  else if (q->current != &rq->base && rq->wakeup_able)
    {
#  ifdef CONFIG_DEVICE_SPI_BYTECODE_TIMER
      rq->sleep_before = 0;
#  endif
      bc_skip(&rq->vm);
      if (!device_spi_ctrl_entry(q, &rq->base))
        dev_request_queue_remove(&q->queue, &rq->base.base);
    }
  else
    {
      rq->wakeup = 1;
    }

  lock_release_irq(&q->lock);
  return err;
}
# endif

error_t dev_spi_queue_init(struct device_s *dev, struct dev_spi_ctrl_queue_s *q)
{
  __unused__ error_t err;

# ifdef CONFIG_DEVICE_SPI_BYTECODE_TIMER
  if (device_get_param_dev_accessor(dev, "timer", &q->timer.base, DRIVER_CLASS_TIMER))
    device_init_accessor(&q->timer.base);

  q->timeout = NULL;
# endif

  q->config = NULL;
  q->current = NULL;
  dev_request_queue_init(&q->queue);
  lock_init_irq(&q->lock);
  memset(&q->transfer, 0, sizeof(q->transfer));
  return 0;
}

void dev_spi_queue_cleanup(struct dev_spi_ctrl_queue_s *q)
{
  lock_destroy_irq(&q->lock);
  dev_request_queue_destroy(&q->queue);
# ifdef CONFIG_DEVICE_SPI_BYTECODE_TIMER
  device_put_accessor(&q->timer.base);
# endif
}

static error_t dev_drv_spi_init(struct device_s *dev,
                                struct dev_spi_ctrl_base_rq_s *rq,
                                struct device_spi_ctrl_s *ctrl)
{
  uintptr_t x;
  if (!device_get_param_uint(dev, "spi-cs-id", &x))
    {
      rq->cs_ctrl = 1;
      rq->cs_id = x;
    }
  else if (!device_get_param_uint(dev, "gpio-cs-id", &x))
    {
# ifdef CONFIG_DEVICE_GPIO
      rq->cs_gpio = 1;
      rq->cs_id = x;
# else
      return -ENOTSUP;
# endif
    }

  if (device_get_param_dev_accessor(dev, "spi", &ctrl->base, DRIVER_CLASS_SPI_CTRL))
    return -ENOENT;

  return 0;
}

static error_t dev_drv_spi_gpio_init(struct device_s *dev,
                                     struct dev_spi_ctrl_base_rq_s *rq,
                                     struct device_gpio_s **gpio)
{
# ifdef CONFIG_DEVICE_GPIO
  if (gpio != NULL || rq->cs_gpio)
    {
      if (device_get_param_dev_accessor(dev, "gpio", &rq->gpio.base, DRIVER_CLASS_GPIO))
        return -ENOENT;
      if (rq->cs_gpio)
        dev_gpio_mode(&rq->gpio, rq->cs_id, DEV_PIN_PUSHPULL);
    }
# endif

  if (gpio != NULL)
# ifdef CONFIG_DEVICE_GPIO
    *gpio = &rq->gpio;
# else
    return -ENOTSUP;
# endif

  return 0;
}

# ifdef CONFIG_DEVICE_SPI_BYTECODE
error_t dev_drv_spi_bytecode_init(struct device_s *dev,
                                  struct dev_spi_ctrl_bytecode_rq_s *rq,
                                  const struct bc_descriptor_s *desc,
                                  struct device_spi_ctrl_s *ctrl,
                                  struct device_gpio_s **gpio,
                                  struct device_timer_s **timer)
{
  dev_spi_bytecode_init(rq);

  error_t err = dev_drv_spi_init(dev, &rq->base, ctrl);
  if (err)
    return err;

  if (timer != NULL)
    {
# ifdef CONFIG_DEVICE_SPI_BYTECODE_TIMER
      struct dev_spi_ctrl_queue_s *q = DEVICE_OP(ctrl, queue);
      err = -ENOENT;
      if (!device_check_accessor(&q->timer.base))
        goto err;
      *timer = &q->timer;
# else
      err = -ENOTSUP;
      goto err;
# endif
    }

  err = dev_drv_spi_gpio_init(dev, &rq->base, gpio);
  if (err)
    goto err;

  bc_init(&rq->vm, desc);

  return 0;
 err:
  device_put_accessor(&ctrl->base);
  return err;
}

void dev_drv_spi_bytecode_cleanup(struct device_spi_ctrl_s *ctrl,
                                  struct dev_spi_ctrl_bytecode_rq_s *rq)
{
#  ifdef CONFIG_DEVICE_GPIO
  device_put_accessor(&rq->base.gpio.base);
#  endif
  device_put_accessor(&ctrl->base);
}
# endif

# ifdef CONFIG_DEVICE_SPI_TRANSACTION

error_t dev_drv_spi_transaction_init(struct device_s *dev,
                                     struct dev_spi_ctrl_transaction_rq_s *rq,
                                     struct device_spi_ctrl_s *ctrl,
                                     struct device_gpio_s **gpio)
{
  dev_spi_transaction_init(rq);

  error_t err = dev_drv_spi_init(dev, &rq->base, ctrl);
  if (err)
    return err;

  err = dev_drv_spi_gpio_init(dev, &rq->base, gpio);
  if (err)
    goto err;

  return 0;
 err:
  device_put_accessor(&ctrl->base);
  return err;
}

void dev_drv_spi_transaction_cleanup(struct device_spi_ctrl_s *ctrl,
                                     struct dev_spi_ctrl_transaction_rq_s *rq)
{
#  ifdef CONFIG_DEVICE_GPIO
  device_put_accessor(&rq->base.gpio.base);
#  endif
  device_put_accessor(&ctrl->base);
}
# endif

#endif  /* CONFIG_DEVICE_SPI_REQUEST */


#if defined(CONFIG_MUTEK_CONTEXT_SCHED)

static KROUTINE_EXEC(dev_request_spi_wait_done)
{
  struct dev_spi_ctrl_transfer_s *tr = KROUTINE_CONTAINER(kr, *tr, kr);
  struct dev_request_status_s *status = tr->pvdata;

  LOCK_SPIN_IRQ(&status->lock);
  if (status->ctx != NULL)
    sched_context_start(status->ctx);
  status->done = 1;
  LOCK_RELEASE_IRQ(&status->lock);
}

error_t dev_spi_wait_transfer(struct device_spi_ctrl_s *accessor,
                              struct dev_spi_ctrl_transfer_s * tr)
{
  struct dev_request_status_s status;

  status.done = 0;
  lock_init(&status.lock);
  status.ctx = NULL;
  tr->pvdata = &status;
  kroutine_init_immediate(&tr->kr, &dev_request_spi_wait_done);

  DEVICE_OP(accessor, transfer, tr);

  CPU_INTERRUPT_SAVESTATE_DISABLE;

  lock_spin(&status.lock);

  if (!status.done)
    {
      status.ctx = sched_get_current();
      sched_stop_unlock(&status.lock);
    }
  else
    lock_release(&status.lock);

  CPU_INTERRUPT_RESTORESTATE;

  lock_destroy(&status.lock);

  return tr->err;
}

#endif


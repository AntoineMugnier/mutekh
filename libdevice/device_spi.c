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

static enum device_spi_ret_e device_spi_ctrl_exec(struct dev_spi_ctrl_queue_s *q);
static enum device_spi_ret_e device_spi_ctrl_sched(struct dev_spi_ctrl_queue_s *q);
static void device_spi_ctrl_run(struct dev_spi_ctrl_queue_s *q);
static enum device_spi_ret_e device_spi_ctrl_end(struct dev_spi_ctrl_rq_s *rq, error_t err);

static enum device_spi_ret_e
device_spi_ctrl_transfer(struct dev_spi_ctrl_rq_s *rq,
                         uint_fast8_t in_width, uint_fast8_t out_width,
                         void *in, const void *out, size_t count);

static error_t device_spi_ctrl_select(struct dev_spi_ctrl_rq_s *rq,
                                      enum dev_spi_cs_policy_e pc)
{
  if (rq->cs_ctrl)
    return DEVICE_OP(&rq->accessor, select, pc, rq->cs_polarity, rq->cs_id);

#ifdef CONFIG_DEVICE_GPIO
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
#endif

  return pc == DEV_SPI_CS_RELEASE ? 0 : -ENOTSUP;
}

static KROUTINE_EXEC(device_spi_ctrl_transfer_end)
{
  struct dev_spi_ctrl_transfer_s *tr = KROUTINE_CONTAINER(kr, *tr, kr);
  struct dev_spi_ctrl_rq_s *rq = tr->pvdata;
  struct dev_spi_ctrl_queue_s *q = rq->queue;

  lock_spin_irq(&q->lock);

#ifdef CONFIG_DEVICE_GPIO
  if (rq->cs_gpio && rq->cs_policy == DEV_SPI_CS_TRANSFER)
    device_spi_ctrl_select(rq, DEV_SPI_CS_DEASSERT);
#endif

  if (tr->err != 0)
    device_spi_ctrl_end(rq, tr->err);

  device_spi_ctrl_run(q);
}

static enum device_spi_ret_e
device_spi_ctrl_transfer(struct dev_spi_ctrl_rq_s *rq,
                         uint_fast8_t in_width, uint_fast8_t out_width,
                         void *in, const void *out, size_t count)
{
  struct dev_spi_ctrl_queue_s *q = rq->queue;
  struct dev_spi_ctrl_transfer_s *tr = &q->transfer;
  error_t err;

#ifdef CONFIG_DEVICE_GPIO
  if (rq->cs_gpio && rq->cs_policy == DEV_SPI_CS_TRANSFER &&
      (err = device_spi_ctrl_select(rq, DEV_SPI_CS_ASSERT)))
    goto err;
#endif

  if (q->config != &rq->config)
    {
      if ((err = DEVICE_OP(&rq->accessor, config, &rq->config)))
        goto err;
      q->config = &rq->config;
    }

  tr->count = count;
  tr->in = in;
  tr->out = out;
  tr->in_width = in_width;
  tr->out_width = out_width;
  tr->pvdata = rq;
  kroutine_init_deferred(&tr->kr, &device_spi_ctrl_transfer_end);

  DEVICE_OP(&rq->accessor, transfer, tr);

  lock_spin_irq(&q->lock);
  return DEVICE_SPI_WAIT;
 err:
  lock_spin_irq(&q->lock);
  return device_spi_ctrl_end(rq, err);
}

static enum device_spi_ret_e
device_spi_ctrl_end(struct dev_spi_ctrl_rq_s *rq, error_t err)
{
  struct dev_spi_ctrl_queue_s *q = rq->queue;

#ifdef CONFIG_DEVICE_SPI_REQUEST_TIMER
  assert(q->timeout == NULL);
#endif

  if (rq == q->current)
    {
      if (
#ifdef CONFIG_DEVICE_GPIO
          rq->cs_gpio ||
#endif
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

#ifdef CONFIG_DEVICE_SPI_REQUEST_TIMER
  if (device_check_accessor(&q->timer))
    device_stop(&q->timer);
#endif

  lock_release_irq(&q->lock);
  kroutine_exec(&rq->base.kr);
  lock_spin_irq(&q->lock);

  return DEVICE_SPI_CONTINUE;
}

#ifdef CONFIG_DEVICE_SPI_REQUEST_TIMER
static KROUTINE_EXEC(device_spi_ctrl_timeout)
{
  struct dev_timer_rq_s *trq = KROUTINE_CONTAINER(kr, *trq, rq.kr);
  struct dev_spi_ctrl_queue_s *q = trq->rq.pvdata;

  lock_spin_irq(&q->lock);

  q->timeout = NULL;

  device_spi_ctrl_run(q);
}

static enum device_spi_ret_e
device_spi_ctrl_delay(struct dev_spi_ctrl_rq_s *rq)
{
  struct dev_spi_ctrl_queue_s *q = rq->queue;
  struct dev_timer_rq_s *trq = &q->timer_rq;
  error_t err = -ETIMEDOUT;

  /* cancel old timer request */
  if (q->timeout != NULL && DEVICE_OP(&q->timer, cancel, &q->timer_rq))
    return DEVICE_SPI_IDLE;
  q->timeout = NULL;

  /* enqueue new timer request */
  if (device_check_accessor(&q->timer))
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

  return device_spi_ctrl_end(rq, err);
}
#endif

static enum device_spi_ret_e
device_spi_ctrl_sched(struct dev_spi_ctrl_queue_s *q)
{
  error_t err;
  struct dev_spi_ctrl_rq_s *rq = NULL;
  assert(q->current == NULL);

  /* find next candidate request in queue */
#ifdef CONFIG_DEVICE_SPI_REQUEST_TIMER
  if (device_check_accessor(&q->timer))
    {
      dev_timer_value_t t;
      DEVICE_OP(&q->timer, get_value, &t, 0);

      GCT_FOREACH_NOLOCK(dev_request_queue, &q->queue, item, {
          struct dev_spi_ctrl_rq_s * const rqitem = dev_spi_ctrl_rq_s_cast(item);

          /* FIXME use dev_timer_check_timeout */
          if (rqitem->sleep_before <= t)
            {
              /* cancel old timer request */
              if (q->timeout != NULL && DEVICE_OP(&q->timer, cancel, &q->timer_rq))
                return DEVICE_SPI_IDLE;

              q->timeout = NULL;
              dev_request_queue_remove(&q->queue, item);
              rq = rqitem;
              goto found;
            }

          if (rq == NULL || rqitem->sleep_before < rq->sleep_before)
            rq = rqitem;
      });

      if (rq == NULL || q->timeout == rq)
        return DEVICE_SPI_IDLE;

      return device_spi_ctrl_delay(rq);
    }
#endif

  rq = dev_spi_ctrl_rq_s_cast(dev_request_queue_pop(&q->queue));
  if (rq == NULL)
    return DEVICE_SPI_IDLE;

found:
  q->current = rq;

  if ((err = device_spi_ctrl_select(rq, rq->cs_policy)))
    return device_spi_ctrl_end(rq, err);

  return DEVICE_SPI_CONTINUE;
}

static enum device_spi_ret_e
device_spi_ctrl_exec(struct dev_spi_ctrl_queue_s *q)
{
  struct dev_spi_ctrl_rq_s *rq;
  error_t err;
  uint16_t op;

  rq = q->current;
  assert(rq != NULL);

  lock_release_irq(&q->lock);

  for (err = 0; err == 0; )
    {
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
#ifdef CONFIG_DEVICE_SPI_REQUEST_TIMER
              if (op & 0x0080)
                {
                  if (!device_check_accessor(&q->timer))
                    {
                      err = -ETIMEDOUT;
                      continue;
                    }
                  dev_timer_value_t t = 0;
                  DEVICE_OP(&q->timer, get_value, &t, 0);
                  rq->sleep_before = t + bc_get_reg(&rq->vm, op & 0xf);
                }
#endif
              switch (op & 0x0300)
                {
                case 0x0000:    /* yield */
                  rq->wakeup_able = !(op & 0x0020);
                  if (rq->wakeup_able && rq->wakeup)
                    {
                      rq->wakeup = 0;
                      bc_skip(&rq->vm);
                      continue;
                    }
                  if (
#ifdef CONFIG_DEVICE_GPIO
                      rq->cs_gpio ||
#endif
                      rq->cs_ctrl)
                    device_spi_ctrl_select(rq, DEV_SPI_CS_DEASSERT);
                  lock_spin_irq(&q->lock);
                  dev_request_queue_pushback(&q->queue, &rq->base);
                  q->current = NULL;
                  return DEVICE_SPI_CONTINUE;

                case 0x0200: {  /* wait, setcs */
                  uint8_t csp = (op >> 4) & 3;
                  if ((err = device_spi_ctrl_select(rq, csp)))
                    continue;
                  if (op & 0x0040) /* setcs */
                    {
                      rq->cs_policy = csp;
                      continue;
                    }
                  lock_spin_irq(&q->lock);
#ifdef CONFIG_DEVICE_SPI_REQUEST_TIMER
                  return device_spi_ctrl_delay(rq);
#else
                  return DEVICE_SPI_CONTINUE;
#endif
                }

#ifdef CONFIG_DEVICE_SPI_REQUEST_TIMER
                case 0x0300:    /* delay */
                  if (!(op & 0x0080))
                    rq->sleep_before = 0;
                  continue;
#endif

                default:
                  continue;
                }

            case 0x0400:
              if (op & 0x0080)  /* brate */
                {
                  rq->config.bit_rate = bc_get_reg(&rq->vm, op & 0xf);
                  q->config = NULL;
                }
              else              /* width */
                {
                  rq->config.word_width = op & 0x001f;
                  rq->config.bit_order = (op >> 5) & 1;
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
              if (dst + l >= 16)
                return device_spi_ctrl_transfer(rq, sizeof(rq->vm.v[0]), 0,
                                                addr, &q->padding_word, l);
              else
                return device_spi_ctrl_transfer(rq, sizeof(rq->vm.v[0]),
                         sizeof(rq->vm.v[0]), addr, &rq->vm.v[dst], l);
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
          switch (op & 0x0c00)
            {
            case 0x0000:  /* pad */
              return device_spi_ctrl_transfer(rq, 0, 0, NULL, &q->padding_word, count);
            case 0x0400:  /* rdm */
              return device_spi_ctrl_transfer(rq, width+1, 0, addr, &q->padding_word, count);
            case 0x0800:  /* wrm */
              return device_spi_ctrl_transfer(rq, 0, width+1, NULL, addr, count);
            case 0x0c00:  /* swpm */
              return device_spi_ctrl_transfer(rq, width+1, width+1, addr, addr2, count);
            }
          continue;
        }

#ifdef CONFIG_DEVICE_SPI_REQUEST_GPIO
        case 0x4000:            /* gpio* */
        case 0x2000:
        case 0x3000: {
          err = 0;
          if (!device_check_accessor(&rq->gpio))
            err = -ENOTSUP;
          else
            {
              gpio_id_t id = rq->gpio_map[(op >> 4) & 0xff];
              gpio_width_t w = rq->gpio_wmap[(op >> 4) & 0xff];
              uint8_t value[8];

              if (op & 0x1000)  /* gpioget */
                {
                  err = DEVICE_OP(&rq->gpio, get_input, id, id + w - 1, value);
                  bc_set_reg(&rq->vm, op & 0xf, endian_le32_na_load(value) & ((1 << w) - 1));
                }
              else if (op & 0x4000) /* gpiomode */
                {
                  err = DEVICE_OP(&rq->gpio, set_mode, id, id + w - 1, dev_gpio_mask1, op & 0xf);
                }
              else              /* gpioset */
                {
                  endian_le32_na_store(value, bc_get_reg(&rq->vm, op & 0xf));
                  err = DEVICE_OP(&rq->gpio, set_output, id, id + w - 1, value, value);
                }
            }
          continue;
        }
#endif

        }

      err = -EINVAL;            /* invalid op */
    }

  lock_spin_irq(&q->lock);
  return device_spi_ctrl_end(rq, err);
}

static void device_spi_ctrl_run(struct dev_spi_ctrl_queue_s *q)
{
  enum device_spi_ret_e r;

  do {
    r = DEVICE_SPI_IDLE;
    if (q->current != NULL)
      r = device_spi_ctrl_exec(q);
    else if (!dev_request_queue_isempty(&q->queue))
      r = device_spi_ctrl_sched(q);
  } while (r == DEVICE_SPI_CONTINUE);

  lock_release_irq(&q->lock);
}

void
dev_spi_rq_start(struct dev_spi_ctrl_rq_s *rq)
{
  struct dev_spi_ctrl_queue_s *q = rq->queue;
  error_t err = 0;

#ifdef CONFIG_DEVICE_GPIO
  assert(!rq->cs_gpio || device_check_accessor(&rq->gpio));
#endif

  if (
#ifdef CONFIG_DEVICE_GPIO
      rq->cs_gpio ||
#endif
      rq->cs_ctrl)
    rq->cs_policy = DEV_SPI_CS_TRANSFER;
  else
    rq->cs_policy = DEV_SPI_CS_RELEASE;

  lock_spin_irq(&q->lock);

#ifdef CONFIG_DEVICE_SPI_REQUEST_TIMER
  if (device_check_accessor(&q->timer))
    err = device_start(&q->timer);
#endif

  if (!err)
    {
#ifdef CONFIG_DEVICE_SPI_REQUEST_TIMER
      rq->sleep_before = 0;
#endif
      rq->err = 0;
      rq->enqueued = 1;
      rq->wakeup = 0;
      rq->wakeup_able = 0;

      dev_request_queue_pushback(&q->queue, &rq->base);

      if (q->current == NULL)
        device_spi_ctrl_run(q);
      else
        lock_release_irq(&q->lock);
    }
  else
    {
      lock_release_irq(&q->lock);
      rq->err = err;
      kroutine_exec(&rq->base.kr);
    }
}

error_t device_spi_request_wakeup(struct dev_spi_ctrl_rq_s *rq)
{
  struct dev_spi_ctrl_queue_s *q = rq->queue;
  error_t err = 0;

  lock_spin_irq(&q->lock);

  if (!rq->enqueued)
    {
      err = -EBUSY;
    }
  else if (q->current != rq && rq->wakeup_able)
    {
#ifdef CONFIG_DEVICE_SPI_REQUEST_TIMER
      rq->sleep_before = 0;
#endif
      bc_skip(&rq->vm);
      if (q->current == NULL)
        {
          device_spi_ctrl_run(q);
          return 0;
        }
    }
  else
    {
      rq->wakeup = 1;
    }

  lock_release_irq(&q->lock);
  return err;
}

error_t dev_spi_queue_init(struct device_s *dev, struct dev_spi_ctrl_queue_s *q)
{
  __unused__ error_t err;

#ifdef CONFIG_DEVICE_SPI_REQUEST_TIMER
  if (device_get_param_dev_accessor(dev, "timer", &q->timer, DRIVER_CLASS_TIMER))
    device_init_accessor(&q->timer);

  q->timeout = NULL;
#endif

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
#ifdef CONFIG_DEVICE_SPI_REQUEST_TIMER
  device_put_accessor(&q->timer);
#endif
}

error_t dev_spi_request_init(struct device_s *slave,
                             struct dev_spi_ctrl_rq_s *rq,
                             bool_t use_gpio, bool_t use_timer)
{
  uintptr_t x;

  memset(rq, 0, sizeof(*rq));

#ifndef CONFIG_DEVICE_SPI_REQUEST_TIMER
  if (use_timer)
    return -ENOTSUP;
#endif
#ifndef CONFIG_DEVICE_SPI_REQUEST_GPIO
  if (use_gpio)
    return -ENOTSUP;
#endif

  if (device_get_param_dev_accessor(slave, "spi", &rq->accessor, DRIVER_CLASS_SPI_CTRL))
    return -ENOENT;

  rq->queue = DEVICE_OP(&rq->accessor, queue);

#ifdef CONFIG_DEVICE_SPI_REQUEST_TIMER
  if (use_timer && !device_check_accessor(&rq->queue->timer))
    goto err;
#endif

  if (!device_get_param_uint(slave, "spi-cs-id", &x))
    {
      rq->cs_ctrl = 1;
      rq->cs_id = x;
    }
  else if (!device_get_param_uint(slave, "gpio-cs-id", &x))
    {
#ifdef CONFIG_DEVICE_GPIO
      rq->cs_gpio = 1;
      rq->cs_id = x;
#else
      goto err;
#endif
    }

#ifdef CONFIG_DEVICE_GPIO
  if (use_gpio || rq->cs_gpio)
    {
      if (device_get_param_dev_accessor(slave, "gpio", &rq->gpio, DRIVER_CLASS_GPIO))
        goto err;
      if (rq->cs_gpio)
        DEVICE_OP(&rq->gpio, set_mode, rq->cs_id, rq->cs_id, dev_gpio_mask1, DEV_PIN_PUSHPULL);
    }
#endif

  return 0;
 err:
  device_put_accessor(&rq->accessor);
  return -ENOTSUP;
}

void dev_spi_request_cleanup(struct dev_spi_ctrl_rq_s *rq)
{
  device_put_accessor(&rq->accessor);
#ifdef CONFIG_DEVICE_GPIO
  device_put_accessor(&rq->gpio);
#endif
}

#endif


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


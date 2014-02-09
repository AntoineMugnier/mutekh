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
#include <device/class/gpio.h>

#include <mutek/bytecode.h>

#ifdef CONFIG_MUTEK_SCHEDULER
# include <mutek/scheduler.h>
#endif
#include <hexo/lock.h>
#include <hexo/interrupt.h>

#ifdef CONFIG_DEVICE_SPI_REQUEST

static void device_spi_ctrl_exec(struct dev_spi_ctrl_queue_s *q, dev_timer_value_t t);
static void device_spi_ctrl_sched(struct dev_spi_ctrl_queue_s *q, dev_timer_value_t t);
static void device_spi_ctrl_end(struct dev_spi_ctrl_request_s *rq, error_t err);

static error_t device_spi_ctrl_select(struct dev_spi_ctrl_request_s *rq,
                                      enum dev_spi_cs_policy_e pc)
{
  if (rq->cs_ctrl)
    return DEVICE_OP(rq->scdev, select, pc, rq->cs_polarity, rq->cs_id);

  if (rq->cs_gpio)
    {
      const uint8_t *value = NULL;
      switch (pc)
        {
        case DEV_SPI_CS_ASSERT:
          value = rq->cs_polarity == DEV_SPI_CS_ACTIVE_HIGH ?
            dev_gpio_mask1 : dev_gpio_mask0;
          break;

        case DEV_SPI_CS_TRANSFER:
        case DEV_SPI_CS_DEASSERT:
        case DEV_SPI_CS_RELEASE:
          value = rq->cs_polarity == DEV_SPI_CS_ACTIVE_HIGH ?
            dev_gpio_mask0 : dev_gpio_mask1;
          break;
        }

      return DEVICE_OP(&rq->gpio, set_output, rq->cs_id, rq->cs_id, value, value);
    }

  return pc == DEV_SPI_CS_RELEASE ? 0 : -ENOTSUP;
}

static KROUTINE_EXEC(device_spi_ctrl_transfer_end)
{
  struct dev_spi_ctrl_transfer_s *tr = (void*)kr;
  struct dev_spi_ctrl_request_s *rq = tr->pvdata;
  struct dev_spi_ctrl_queue_s *q = rq->queue;

  lock_spin_irq(&q->lock);

  if (rq->cs_gpio && rq->cs_policy == DEV_SPI_CS_TRANSFER)
    device_spi_ctrl_select(rq, DEV_SPI_CS_DEASSERT);

  if (tr->err != 0)
    return device_spi_ctrl_end(rq, tr->err);

  dev_timer_value_t t = 0;
  if (device_check_accessor(&q->timer))
    DEVICE_OP(&q->timer, get_value, &t);

  bc_skip(&rq->vm);

  return device_spi_ctrl_exec(q, t);
}

static void device_spi_ctrl_transfer(struct dev_spi_ctrl_request_s *rq, dev_timer_value_t t,
                                     uint_fast8_t in_width, uint_fast8_t out_width,
                                     void *in, const void *out, size_t count)
{
  struct dev_spi_ctrl_queue_s *q = rq->queue;
  struct dev_spi_ctrl_transfer_s *tr = &q->transfer;
  error_t err;

  if (rq->cs_gpio && rq->cs_policy == DEV_SPI_CS_TRANSFER &&
      (err = device_spi_ctrl_select(rq, DEV_SPI_CS_ASSERT)))
    {
      lock_spin_irq(&q->lock);
      return device_spi_ctrl_end(rq, err);
    }

  if (q->config != &rq->config)
    {
      if ((err = DEVICE_OP(rq->scdev, config, &rq->config)))
        {
          lock_spin_irq(&q->lock);
          return device_spi_ctrl_end(rq, err);
        }
      q->config = &rq->config;
    }

  tr->count = count;
  tr->in = in;
  tr->out = out;
  tr->in_width = in_width;
  tr->out_width = out_width;
  tr->pvdata = rq;
  kroutine_init(&tr->kr, &device_spi_ctrl_transfer_end, KROUTINE_IMMEDIATE);

  return DEVICE_OP(rq->scdev, transfer, tr);
}

#if 0
static KROUTINE_EXEC(device_spi_ctrl_next_kr)
{
  struct dev_spi_ctrl_transfer_s *tr = (void*)kr;
  struct dev_spi_ctrl_queue_s *q = tr->pvdata;

  lock_spin_irq(&q->lock);

  if (q->current != NULL)
    {
      lock_release_irq(&q->lock);
      return;
    }

  dev_timer_value_t t = 0;
  if (device_check_accessor(&q->timer))
    DEVICE_OP(&q->timer, get_value, &t);

  return device_spi_ctrl_sched(q, t);
}

static void device_spi_ctrl_next(struct dev_spi_ctrl_queue_s *q)
{
  /* start next rq from interruptible context */
  struct dev_spi_ctrl_transfer_s *tr = &q->transfer;
  kroutine_init(&tr->kr, &device_spi_ctrl_next_kr,
                rq->priority ? KROUTINE_IMMEDIATE : KROUTINE_SCHED_SWITCH);
  tr->pvdata = q;

  lock_release_irq(&q->lock);
  kroutine_exec(&tr->kr, cpu_is_interruptible());
}
#endif

static void device_spi_ctrl_end(struct dev_spi_ctrl_request_s *rq, error_t err)
{
  struct dev_spi_ctrl_queue_s *q = rq->queue;

  assert(q->timeout == NULL);

  if (rq == q->current)
    {
      if (rq->cs_gpio || rq->cs_ctrl)
        device_spi_ctrl_select(rq, DEV_SPI_CS_DEASSERT);
      q->config = NULL;
      q->current = NULL;
    }
  else
    {
      assert(q->current == NULL);
      dev_spi_ctrl_queue_remove(&q->queue, rq);
    }

  rq->err = err;

  if (device_check_accessor(&q->timer))
    DEVICE_OP(&q->timer, start_stop, 0);

  lock_release_irq(&q->lock);

  kroutine_exec(&rq->kr, cpu_is_interruptible());

  lock_spin_irq(&q->lock);

  if (q->current == NULL && q->timeout == NULL     /* no rq started from kroutine ? */
      && !dev_spi_ctrl_queue_isempty(&q->queue))   /* rq to start */
    {
      dev_timer_value_t t = 0;
      if (device_check_accessor(&q->timer))
        DEVICE_OP(&q->timer, get_value, &t);
      return device_spi_ctrl_sched(q, t);
    }

  lock_release_irq(&q->lock);
}

static KROUTINE_EXEC(device_spi_ctrl_timeout)
{
  struct dev_timer_rq_s *trq = (void*)kr;
  struct dev_spi_ctrl_request_s *tm, *rq = trq->pvdata;
  struct dev_spi_ctrl_queue_s *q = rq->queue;

  lock_spin_irq(&q->lock);

  rq = trq->pvdata;
  tm = q->timeout;
  q->timeout = NULL;

  if (tm != rq)
    {
      lock_release_irq(&q->lock);
      return;
    }

  dev_timer_value_t t = 0;
  DEVICE_OP(&q->timer, get_value, &t);

  if (q->current != NULL)
    return device_spi_ctrl_exec(q, t);
  return device_spi_ctrl_sched(q, t);
}

static void device_spi_ctrl_delay(struct dev_spi_ctrl_request_s *rq)
{
  struct dev_spi_ctrl_queue_s *q = rq->queue;
  struct dev_timer_rq_s *trq = &q->timer_rq;
  error_t err = -ETIMEDOUT;

  /* cancel old timer request */
  if (q->timeout != NULL && DEVICE_OP(&q->timer, cancel, &q->timer_rq))
    {
      lock_release_irq(&q->lock);
      return;
    }
  q->timeout = NULL;

  /* enqueue new timer request */
  if (device_check_accessor(&q->timer))
    {
      trq->deadline = rq->sleep_before;
      trq->delay = 0;
      kroutine_init(&trq->kr, device_spi_ctrl_timeout, KROUTINE_IMMEDIATE);
      trq->pvdata = rq;

      err = DEVICE_OP(&q->timer, request, trq);
    }

  switch (err)
    {
    case 0:      /* wait for timeout interrupt */
      q->timeout = rq;
      lock_release_irq(&q->lock);
      return;

    case ETIMEDOUT: {    /* update time and retry */
      dev_timer_value_t t = 0;

      if (device_check_accessor(&q->timer) &&
          DEVICE_OP(&q->timer, get_value, &t) == 0)
        {
          if (q->current != NULL)
            return device_spi_ctrl_exec(q, t);
          return device_spi_ctrl_sched(q, t);
        }

      err = -ETIMEDOUT;
    }

    default:
      return device_spi_ctrl_end(rq, err);
    }
}

static void device_spi_ctrl_sched(struct dev_spi_ctrl_queue_s *q, dev_timer_value_t t)
{
  error_t err;
  struct dev_spi_ctrl_request_s *rq = NULL;
  assert(q->current == NULL);

  /* find next candidate request in queue */
  CONTAINER_FOREACH_NOLOCK(dev_spi_ctrl_queue, CLIST, &q->queue, {

      /* FIXME use dev_timer_check_timeout */
      if (item->sleep_before <= t)
        {
          /* cancel old timer request */
          if (q->timeout != NULL && DEVICE_OP(&q->timer, cancel, &q->timer_rq))
            {
              lock_release_irq(&q->lock);
              return;
            }
          q->timeout = NULL;

          dev_spi_ctrl_queue_remove(&q->queue, item);
          q->current = item;

          if ((err = device_spi_ctrl_select(item, item->cs_policy)))
            return device_spi_ctrl_end(rq, err);

          return device_spi_ctrl_exec(q, t);
        }

      if (rq == NULL || item->sleep_before < rq->sleep_before)
        rq = item;
    });

  if (rq != NULL && q->timeout != rq)
    return device_spi_ctrl_delay(rq);

  lock_release_irq(&q->lock);
}

static void device_spi_ctrl_exec(struct dev_spi_ctrl_queue_s *q, dev_timer_value_t t)
{
  struct dev_spi_ctrl_request_s *rq;
  error_t err = 0;
  uint16_t op;

  rq = q->current;
  assert(rq != NULL);

  lock_release_irq(&q->lock);

  for (;; bc_skip(&rq->vm))
    {
      op = bc_run(&rq->vm, -1);

      if (!(op & 0x8000))
        break;

      switch (op & 0x7000)
        {
        case 0x0000:
          switch (op & 0x0c00)
            {
            case 0x0000:
              if (op & 0x0080)
                rq->sleep_before = t + dev_timer_delay_shift(q->delay_shift, bc_get_reg(&rq->vm, op & 0xf));
              switch (op & 0x0300)
                {
                case 0x0000:    /* yield */
                  if (rq->cs_gpio || rq->cs_ctrl)
                    device_spi_ctrl_select(rq, DEV_SPI_CS_DEASSERT);
                  lock_spin_irq(&q->lock);
                  dev_spi_ctrl_queue_pushback(&q->queue, rq);
                  q->current = NULL;
                  bc_skip(&rq->vm);
                  return device_spi_ctrl_sched(q, t);

                case 0x0200: {  /* wait, setcs */
                  uint8_t csp = (op >> 4) & 3;
                  if ((err = device_spi_ctrl_select(rq, csp)))
                    {
                      lock_spin_irq(&q->lock);
                      return device_spi_ctrl_end(rq, err);
                    }
                  if (op & 0x0040) /* setcs */
                    {
                      rq->cs_policy = csp;
                      continue;
                    }
                  lock_spin_irq(&q->lock);
                  bc_skip(&rq->vm);
                  return device_spi_ctrl_delay(rq);
                }

                case 0x0300:    /* delay */
                  if (!(op & 0x0080))
                    rq->sleep_before = 0;
                  continue;

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
            case 0x0800:        /* swp */
            case 0x0c00: {
              uint_fast8_t l = ((op >> 8) & 7) + 1;
              uint_fast8_t src = op & 0xf;
              uint_fast8_t dst = (op >> 4) & 0xf;
              if (src + l >= 16 || dst + l >= 16)
                continue;
              return device_spi_ctrl_transfer(rq, t, sizeof(rq->vm.v[0]), sizeof(rq->vm.v[0]),
                                              &rq->vm.v[src], &rq->vm.v[dst], l);
            }
            }
          continue;

        case 0x1000: {
          void *addr = (void*)bc_get_reg(&rq->vm, (op >> 4) & 0xf);
          void *addr2 = (void*)bc_get_reg(&rq->vm, 1 ^ ((op >> 4) & 0xf));
          size_t count = bc_get_reg(&rq->vm, op & 0xf);
          uint_fast8_t width = (op >> 8) & 3;
          uint32_t dummy = bc_get_reg(&rq->vm, 14);
          if (count == 0)
            continue;
          switch (op & 0x0c00)
            {
            case 0x0000:  /* pad */
              return device_spi_ctrl_transfer(rq, t, 0, 0, NULL, &dummy, count);
            case 0x0400:  /* rdm */
              return device_spi_ctrl_transfer(rq, t, width+1, 0, addr, &dummy, count);
            case 0x0800:  /* wrm */
              return device_spi_ctrl_transfer(rq, t, 0, width+1, NULL, addr, count);
            case 0x0c00:  /* swpm */
              return device_spi_ctrl_transfer(rq, t, width+1, width+1, addr, addr2, count);
            }
          continue;
        }

        case 0x4000:            /* iomode */
        case 0x2000:            /* ioset */
        case 0x3000: {          /* ioget */
          err = 0;
          if (!device_check_accessor(&rq->gpio))
            err = -ENOTSUP;
          else
            {
              uint_fast8_t id = rq->gpio_id + ((op >> 4) & 0xff);
              uint8_t value[8];

              if (op & 0x1000)
                {
                  err = DEVICE_OP(&rq->gpio, get_input, id, id, value);
                  bc_set_reg(&rq->vm, op & 0xf, value[0]);
                }
              else if (op & 0x4000)
                {
                  err = DEVICE_OP(&rq->gpio, set_mode, id, id, dev_gpio_mask1, op & 0xf);
                }
              else
                {
                  value[0] = bc_get_reg(&rq->vm, op & 0xf);
                  err = DEVICE_OP(&rq->gpio, set_output, id, id, value, value);
                }
            }
          if (!err)
            continue;
          lock_spin_irq(&q->lock);
          return device_spi_ctrl_end(rq, err);
        }

        }

      break;
    }

  lock_spin_irq(&q->lock);
  return device_spi_ctrl_end(rq, op ? -EINVAL : 0);
}

void
dev_spi_request_start(struct device_spi_ctrl_s *scdev,
                      struct dev_spi_ctrl_queue_s *q,
                      struct dev_spi_ctrl_request_s *rq)
{
  error_t err = 0;

  assert(!rq->cs_gpio || device_check_accessor(&rq->gpio));

  lock_spin_irq(&q->lock);

  dev_timer_value_t t = 0;

  if (device_check_accessor(&q->timer))
    {
      err = DEVICE_OP(&q->timer, start_stop, 1);
      DEVICE_OP(&q->timer, get_value, &t);
    }

  if (!err)
    {
      rq->queue = q;
      rq->sleep_before = 0;
      rq->err = 0;
      rq->scdev = scdev;

      if (rq->priority)
        dev_spi_ctrl_queue_push(&q->queue, rq);
      else
        dev_spi_ctrl_queue_pushback(&q->queue, rq);

      if (q->current == NULL)
        return device_spi_ctrl_sched(q, t);
    }

  lock_release_irq(&q->lock);

  if (err)
    {
      rq->err = err;
      kroutine_exec(&rq->kr, cpu_is_interruptible());
      return;
    }
}

error_t dev_spi_queue_init(struct device_s *dev, struct dev_spi_ctrl_queue_s *q)
{
  if (device_get_param_dev_accessor(dev, "spi-timer", &q->timer, DRIVER_CLASS_TIMER) ||
      dev_timer_shift_sec(&q->timer, &q->delay_shift, 1, 1000000))
    device_init_accessor(&q->timer);

  q->config = NULL;
  q->current = NULL;
  q->timeout = NULL;
  dev_spi_ctrl_queue_init(&q->queue);
  lock_init_irq(&q->lock);
  return 0;
}

void dev_spi_queue_cleanup(struct dev_spi_ctrl_queue_s *q)
{
  lock_destroy_irq(&q->lock);
  dev_spi_ctrl_queue_destroy(&q->queue);
  device_put_accessor(&q->timer);
}

error_t dev_spi_request_init(struct device_s *slave,
                             struct dev_spi_ctrl_request_s *rq,
                             bool_t require_gpio)
{
  uintptr_t x;

  memset(rq, 0, sizeof(*rq));

  if (!device_get_param_uint(slave, "cs-id", &x))
    {
      rq->cs_ctrl = 1;
      rq->cs_id = x;      
    }

  if (device_get_param_dev_accessor(slave, "gpio", &rq->gpio, DRIVER_CLASS_GPIO))
    {
      if (require_gpio)
        return -ENOENT;
    }
  else
    {
      if (!device_get_param_uint(slave, "gpio-id", &x))
        rq->gpio_id = x;
      else if (require_gpio)
        goto err_gpio;

      if (!rq->cs_ctrl && !device_get_param_uint(slave, "cs-gpio-id", &x))
        {
          rq->cs_gpio = 1;
          rq->cs_id = x;

          if (DEVICE_OP(&rq->gpio, set_mode, x, x, dev_gpio_mask1, DEV_GPIO_OUTPUT))
            goto err_gpio;
        }
    }

  if (rq->cs_gpio || rq->cs_ctrl)
    rq->cs_policy = DEV_SPI_CS_TRANSFER;
  else
    rq->cs_policy = DEV_SPI_CS_RELEASE;

  return 0;

 err_gpio:
  device_put_accessor(&rq->gpio);
  return -ENOENT;
}

void dev_spi_request_cleanup(struct dev_spi_ctrl_request_s *rq)
{
  device_put_accessor(&rq->gpio);
}

#endif


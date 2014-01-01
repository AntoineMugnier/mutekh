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
#include <device/class/spi.h>
#include <device/class/timer.h>
#include <device/class/gpio.h>
#include <device/driver.h>

#include <mutek/bytecode.h>

#ifdef CONFIG_MUTEK_SCHEDULER
# include <mutek/scheduler.h>
#endif
#include <hexo/lock.h>
#include <hexo/interrupt.h>

#ifdef CONFIG_DEVICE_SPI_REQUEST

static void device_spi_ctrl_exec(struct dev_spi_ctrl_queue_s *q, dev_timer_value_t t, bool_t in_thread);
static void device_spi_ctrl_sched(struct dev_spi_ctrl_queue_s *q, dev_timer_value_t t, bool_t in_thread);
static void device_spi_ctrl_wait(struct dev_spi_ctrl_request_s *rq, dev_timer_value_t t, bool_t in_thread, error_t err);

static void device_spi_ctrl_transfer_end(struct dev_spi_ctrl_request_s *rq,
                                         struct dev_spi_ctrl_transfer_s *tr, bool_t in_thread)
{
  struct dev_spi_ctrl_queue_s *q = rq->queue;
  error_t err = tr->err;
  dev_timer_value_t t = 0;

  if (err == 0)
    {
      if (device_check_accessor(&q->timer))
        err = DEVICE_OP(&q->timer, get_value, &t);
    }

  if (err == 0)
    {
      bc_skip(&rq->vm);
      return device_spi_ctrl_exec(q, t, 0);
    }

  assert(err != -EAGAIN);
  return device_spi_ctrl_wait(rq, 0, 0, err);
}

static DEVSPI_CTRL_TRANSFER_CALLBACK(device_spi_ctrl_transfer_callback)
{
  if (!nested)
    return device_spi_ctrl_transfer_end(tr->pvdata, tr, 0);
}

static void device_spi_ctrl_transfer(struct dev_spi_ctrl_request_s *rq,
                                     dev_timer_value_t t, bool_t in_thread,
                                     uint_fast8_t width, void *in, const void *out, size_t count,
                                     enum dev_spi_cs_policy_e cs)
{
  error_t err;

  if ((err = DEVICE_OP(rq->scdev, select, cs, rq->cs_id)))
    return device_spi_ctrl_wait(rq, t, in_thread, err);

  struct dev_spi_ctrl_queue_s *q = rq->queue;
  struct dev_spi_ctrl_transfer_s *tr = &q->transfer;

  if (q->config != &rq->config)
    {
      if ((err = DEVICE_OP(rq->scdev, config, &rq->config)))
        return device_spi_ctrl_wait(rq, t, in_thread, err);
      q->config = &rq->config;
    }

  tr->count = count;
  tr->in = in;
  tr->out = out;
  tr->in_width = width;
  tr->out_width = width;
  tr->callback = device_spi_ctrl_transfer_callback;
  tr->pvdata = rq;

  err = DEVICE_OP(rq->scdev, transfer, tr, in_thread);

  if (err == 1)
    return device_spi_ctrl_transfer_end(rq, tr, in_thread);

  assert(err <= 0);
#ifndef CONFIG_DEVICE_IRQ
  /* when interrupts are not available, -EAGAIN can not be
     returned by the device transfer function. */
  assert(err != -EAGAIN);
#endif

  return device_spi_ctrl_wait(rq, t, in_thread, err);
}

/* This function wait for the next interrupt if called with err equal
   to 0. It also handles errors and may terminate the request. */
static void device_spi_ctrl_wait(struct dev_spi_ctrl_request_s *rq, dev_timer_value_t t,
                                 bool_t in_thread, error_t err)
{
  struct dev_spi_ctrl_queue_s *q = rq->queue;

  if (!in_thread)
    {
      if (!err)
        return;      /* return, waiting for next interrupt */

      LOCK_SPIN_IRQ(&q->lock);

      /* not possible to process outside the thread associated with
         the request, restore interrupted thread so that we can
         continue or handle the error. */
      rq->err = err;
#ifdef CONFIG_MUTEK_SCHEDULER
      if (rq->sched_ctx != NULL)
        sched_context_start(rq->sched_ctx);
#endif

      LOCK_RELEASE_IRQ(&q->lock);
      return;
    }

  /* -EAGAIN is reserved for switching back to the request thread and
     can not be used if we are already running from this thread. */
  assert(err != -EAGAIN);

  LOCK_SPIN_IRQ(&q->lock);

  /* sleep if no error, waiting for next interrupt */
  while (err == 0)
    {
#ifdef CONFIG_MUTEK_SCHEDULER
      if (rq->sched_ctx != NULL)
        sched_stop_unlock(&q->lock);
      else
#endif
        {
#ifdef CONFIG_DEVICE_IRQ
          lock_release(&q->lock);
# ifdef CONFIG_CPU_WAIT_IRQ
          cpu_interrupt_wait();
# endif
#else
          /* when interrupts are not available, the callback should
             have been called from the device transfer function. */
          abort();
#endif
        }
      lock_spin(&q->lock);
      err = rq->err;
    }

  /* terminate request */
  if (err && err != -EAGAIN)
    {
      rq->err = err == -EEOF ? 0 : err;
      q->slaves_mask ^= 1ULL << rq->slave_id;
      q->config = NULL;
      q->current = NULL;
      DEVICE_OP(rq->scdev, select, DEV_SPI_CS_SELECT_NONE, rq->cs_id);
      if (device_check_accessor(&q->timer) && !q->slaves_mask)
        DEVICE_OP(&q->timer, start_stop, 0);
    }

  LOCK_RELEASE_IRQ(&q->lock);

  /* resume request processing */
  if (err == -EAGAIN)
    {
      rq->err = 0;
      return device_spi_ctrl_exec(q, t, 1);
    }

  /* resume execution of next request in queue from current thread but force
     switching to the thread of the resumed request for long operations. */
  return device_spi_ctrl_sched(q, t, 0);
}

static DEVTIMER_CALLBACK(device_spi_ctrl_timeout)
{
  if (nested)
    return 0;

  struct dev_spi_ctrl_queue_s *q = rq->pvdata;
  dev_timer_value_t t = rq->deadline;

  device_spi_ctrl_sched(q, t, 0);

  return 0;
}

static void device_spi_ctrl_sched(struct dev_spi_ctrl_queue_s *q, dev_timer_value_t t, bool_t in_thread)
{
  struct dev_timer_rq_s *trq = &q->timer_rq;
  error_t err = 0;

  LOCK_SPIN_IRQ(&q->lock);

  struct dev_spi_ctrl_request_s *rq = q->current;

  if (rq != NULL)   /* wait if a request is already being processed */
    goto wait;

  while (1)
    {
      /* find next candidate request in queue */
      rq = NULL;
      CONTAINER_FOREACH_NOLOCK(dev_spi_ctrl_queue, CLIST, &q->queue, {
          /* FIXME use dev_timer_check_timeout */
          if (item->sleep_before <= t)
            {
              dev_spi_ctrl_queue_remove(&q->queue, item);
              q->current = item;
              item->noyield = 0;
              LOCK_RELEASE_IRQ_X(&q->lock);
              return device_spi_ctrl_exec(q, t, in_thread);
            }

          if (item->noyield)
            {
              rq = item;
              goto wait;
            }

          if (rq == NULL || item->sleep_before < rq->sleep_before)
            rq = item;
      });

      /* no candidate request in queue */
      if (rq == NULL)
        {
          LOCK_RELEASE_IRQ_X(&q->lock);
          return;
        }

      /* cancel old timer request, if any */
      if (q->timeout)
        {
          DEVICE_SAFE_OP(&q->timer, cancel, &q->timer_rq);
          q->timeout = 0;
        }

      /* rely on timer */
      err = -ETIMEDOUT;
      if (device_check_accessor(&q->timer))
        {
          trq->deadline = rq->sleep_before;
          trq->delay = 0;
          trq->callback = &device_spi_ctrl_timeout;
          trq->pvdata = q;

          err = DEVICE_SAFE_OP(&q->timer, request, trq);
        }

      switch (err)
        {
        case 0:      /* wait for timeout interrupt */
          q->timeout = 1;
          LOCK_RELEASE_IRQ_X(&q->lock);
          return;

        case ETIMEDOUT:    /* update time and retry */
          if (device_check_accessor(&q->timer) &&
              DEVICE_OP(&q->timer, get_value, &t) == 0)
            continue;
          err = -ETIMEDOUT;

        default:
        wait:
          LOCK_RELEASE_IRQ_X(&q->lock);
          return device_spi_ctrl_wait(rq, t, in_thread, err);
        }
    }

  LOCK_RELEASE_IRQ(&q->lock);
}

static void device_spi_ctrl_exec(struct dev_spi_ctrl_queue_s *q, dev_timer_value_t t, bool_t in_thread)
{
  struct dev_spi_ctrl_request_s *rq = q->current;
  error_t err = 0;
  uint16_t op;

  for (;; bc_skip(&rq->vm))
    {
      op = bc_run(&rq->vm, -1);

      if (!(op & 0x8000))
        break;

      if (!(op & 0x1000))
        {
          if (!(op & 0x0400))  /* yield, delay* */
            {
              uintptr_t d = bc_get_reg(&rq->vm, op & 0xf);
              rq->sleep_before = t + (dev_timer_value_t)d * rq->delay_unit;
              switch (op & 0x0300)
                {
                case 0x0000: /* yield */
                  LOCK_SPIN_IRQ(&q->lock);
                  dev_spi_ctrl_queue_pushback(&q->queue, rq);
                  q->current = NULL;
                  LOCK_RELEASE_IRQ(&q->lock);
                  return device_spi_ctrl_sched(q, t, in_thread);

                case 0x0200: /* delay* */
                  if ((err = DEVICE_OP(rq->scdev, select, (op >> 4) & 3, rq->cs_id)))
                    return device_spi_ctrl_wait(rq, t, in_thread, err);
                  if (d == 0)
                    continue;
                  LOCK_SPIN_IRQ(&q->lock);
                  dev_spi_ctrl_queue_push(&q->queue, rq);
                  rq->noyield = 1;
                  q->current = NULL;
                  LOCK_RELEASE_IRQ(&q->lock);
                  return device_spi_ctrl_sched(q, t, in_thread);
                }
            }
          else
            {
              switch (op & 0x0300)
                {
                case 0x0000: /* width */
                  rq->config.word_width = op & 0x001f;
                  rq->config.bit_order = (op >> 5) & 1;
                  q->config = NULL;
                  continue;
                case 0x0100: /* brate */
                  rq->config.bit_rate = bc_get_reg(&rq->vm, op & 0xf);
                  q->config = NULL;
                  continue;
                case 0x0200: /* swp */
                  return device_spi_ctrl_transfer(rq, t, in_thread,
                                              sizeof(rq->vm.v[0]), &rq->vm.v[op & 0xf],
                                              &rq->vm.v[(op >> 4) & 0xf],
                                              1, DEV_SPI_CS_SELECT_ON_NEXT_TRANSFER);
                case 0x0300: /* gpio */
                  continue;
                }
            }
        }
      else
        {
          void *addr = (void*)bc_get_reg(&rq->vm, (op >> 4) & 0xf);
          void *addr2 = (void*)bc_get_reg(&rq->vm, 1 ^ ((op >> 4) & 0xf));
          size_t count = bc_get_reg(&rq->vm, op & 0xf);
          uint_fast8_t width = (op >> 8) & 3;
          uint32_t dummy;
          if (count == 0)
            continue;
          switch (op & 0xfc00)
            {
            case 0x9000:  /* pad padu */
              return device_spi_ctrl_transfer(rq, t, in_thread, 0, NULL, NULL,
                                          count, (op >> 4) & 3);
            case 0x9400:  /* rdm */
              return device_spi_ctrl_transfer(rq, t, in_thread, width+1, addr, &dummy,
                                          count, DEV_SPI_CS_SELECT_ON_NEXT_TRANSFER);
            case 0x9800:  /* wrm */
              return device_spi_ctrl_transfer(rq, t, in_thread, width+1, &dummy, addr,
                                          count, DEV_SPI_CS_SELECT_ON_NEXT_TRANSFER);
            case 0x9c00:  /* swpm */
              return device_spi_ctrl_transfer(rq, t, in_thread, width+1, addr, addr2,
                                          count, DEV_SPI_CS_SELECT_ON_NEXT_TRANSFER);
            }
        }

      break;
    }

  return device_spi_ctrl_wait(rq, t, in_thread, op ? -EINVAL : -EEOF);
}

error_t
dev_spi_request_start(struct device_spi_ctrl_s *scdev,
                      struct dev_spi_ctrl_request_s *rq)
{
  struct dev_spi_ctrl_queue_s *q = DEVICE_OP(scdev, queue);

  error_t err = 0;

  if (rq->slave_id > 8 * sizeof(q->slaves_mask))
    return -ERANGE;

  dev_timer_value_t t = 0;

  LOCK_SPIN_IRQ(&q->lock);

  if (q->slaves_mask & (1 << rq->slave_id))
    {
      err = -EBUSY;
    }
  else
    {
      if (device_check_accessor(&q->timer))
        {
          if (!q->slaves_mask)
            err = DEVICE_OP(&q->timer, start_stop, 1);

          DEVICE_OP(&q->timer, get_value, &t);
        }

      if (!err)
        {
          q->slaves_mask |= 1 << rq->slave_id;

          rq->queue = q;
          rq->noyield = 0;
          rq->sleep_before = 0;
          rq->err = 0;
          rq->scdev = scdev;
#ifdef CONFIG_MUTEK_SCHEDULER
          rq->sched_ctx = sched_get_current();   /* FIXME handle lock mode */
#endif

          dev_spi_ctrl_queue_pushback(&q->queue, rq);
        }
    }

  LOCK_RELEASE_IRQ(&q->lock);

  if (!err)
    {
      device_spi_ctrl_sched(q, t, 1);
      err = rq->err;
    }

  return err;
}

#endif


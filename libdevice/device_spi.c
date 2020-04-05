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
#include <mutek/printk.h>

#ifdef CONFIG_MUTEK_CONTEXT_SCHED
# include <mutek/scheduler.h>
#endif
#include <hexo/lock.h>
#include <hexo/interrupt.h>
#include <hexo/endian.h>

const char dev_spi_ckmode_e[] = ENUM_DESC_DEV_SPI_CKMODE_E;
const char dev_spi_polarity_e[] = ENUM_DESC_DEV_SPI_POLARITY_E;
const char dev_spi_bit_order_e[] = ENUM_DESC_DEV_SPI_BIT_ORDER_E;
const char dev_spi_cs_op_e[] = ENUM_DESC_DEV_SPI_CS_OP_E;

#ifdef CONFIG_DEVICE_SPI_REQUEST

enum device_spi_ret_e {
  DEVICE_SPI_IDLE,
  DEVICE_SPI_CONTINUE,
  DEVICE_SPI_WAIT,
};

enum device_spi_wakeup_e
{
  DEVICE_SPI_WAKEUP_NONE  = 0b00,
  DEVICE_SPI_WAKEUP_YIELDC = 0b01,
  DEVICE_SPI_WAKEUP_SLEEP = 0b10,
};

static enum device_spi_ret_e device_spi_ctrl_sched(struct dev_spi_ctrl_context_s *q);
static void device_spi_ctrl_run(struct dev_spi_ctrl_context_s *q);
static enum device_spi_ret_e device_spi_ctrl_end(struct dev_spi_ctrl_context_s *q,
                                                 struct dev_spi_ctrl_rq_s *rq, error_t err);

static KROUTINE_EXEC(device_spi_ctrl_transfer_end)
{
  struct dev_spi_ctrl_transfer_s *tr = KROUTINE_CONTAINER(kr, *tr, kr);
  struct dev_spi_ctrl_rq_s *rq = tr->pvdata;
  struct dev_spi_ctrl_context_s *q = device_spi_ctrl_context(rq->ctrl);

  lock_spin_irq(&q->lock);

# ifdef CONFIG_DEVICE_SPI_GPIO_CS
  struct dev_spi_cs_config_s *cs_cfg = &rq->cs_cfg;
  if (rq->cs_gpio)
    dev_gpio_out(&rq->gpio, cs_cfg->id, rq->cs_state ^ cs_cfg->polarity ^ 1);
# endif

  if (tr->err
# ifdef CONFIG_DEVICE_SPI_TRANSACTION
      || !rq->bytecode
# endif
      )
    device_spi_ctrl_end(q, rq, tr->err);

  device_spi_ctrl_run(q);
}

static enum device_spi_ret_e
device_spi_ctrl_transfer(struct dev_spi_ctrl_context_s *q,
                         struct dev_spi_ctrl_rq_s *rq,
                         enum dev_spi_cs_op_e csop)
{
  struct dev_spi_ctrl_transfer_s *tr = &q->transfer;
  error_t err;

  struct dev_spi_ctrl_config_s *rcfg = rq->config;
  if (rcfg)
    {
      const struct dev_spi_ctrl_config_s *qcfg = q->config;
      if (rcfg->dirty)
        {
          rcfg->dirty = 0;
          qcfg = NULL;
        }

      if (qcfg != rcfg)
        {
          if ((err = DEVICE_OP(rq->ctrl, config, rq->config)))
            goto err;
          q->config = rcfg;
        }
    }

  tr->cs_op = DEV_SPI_CS_NOP_NOP;
  if (csop != DEV_SPI_CS_NOP_NOP)
    {
      __unused__ struct dev_spi_cs_config_s *cs_cfg = &rq->cs_cfg;
      rq->cs_state = (csop == DEV_SPI_CS_SET_NOP);
      if (0)
        ;
# ifdef CONFIG_DEVICE_SPI_GPIO_CS
      if (rq->cs_gpio)
        dev_gpio_out(&rq->gpio, cs_cfg->id, cs_cfg->polarity ^ (csop == DEV_SPI_CS_CLR_NOP));
# endif
# ifdef CONFIG_DEVICE_SPI_CTRL_CS
      else if (rq->cs_ctrl)
        {
          tr->cs_op = csop;
          tr->cs_cfg = *cs_cfg;
        }
# endif
      else
        {
          /* this request requires a chip select configuration */
          logk_error("chip select config missing, %x", csop);
          err = -ENOENT;
          goto err;
        }
    }

  tr->pvdata = rq;
  kroutine_init_deferred(&tr->kr, &device_spi_ctrl_transfer_end);

  DEVICE_OP(rq->ctrl, transfer, tr);

  return DEVICE_SPI_WAIT;
 err:
  return device_spi_ctrl_end(q, rq, err);
}

static enum device_spi_ret_e
device_spi_ctrl_end(struct dev_spi_ctrl_context_s *q,
                    struct dev_spi_ctrl_rq_s *rq, error_t err)
{
  assert(rq->enqueued);
# ifdef CONFIG_DEVICE_SPI_BYTECODE_TIMER
  assert(q->timeout == NULL);
# endif

  if (rq == q->current)
    {
      q->config = NULL;
      q->current = NULL;
    }
  else
    {
      assert(q->current == NULL);
      __dev_rq_queue_remove(&q->queue, &rq->base);
    }

  rq->error = err;
  rq->enqueued = 0;
# ifdef CONFIG_LIBC_ASSERT
  rq->cs_state = 0;
# endif

# ifdef CONFIG_DEVICE_SPI_GPIO_CS
  if (rq->cs_gpio)
    dev_gpio_out(&rq->gpio, rq->cs_cfg.id, rq->cs_cfg.polarity ^ 1);
# endif

# ifdef CONFIG_DEVICE_SPI_BYTECODE_TIMER
  if (device_check_accessor(&q->timer.base)
#  ifdef CONFIG_DEVICE_SPI_TRANSACTION
      && rq->bytecode
#  endif
      )
    device_stop(&q->timer.base);
# endif

  kroutine_exec(&rq->base.kr);

  return DEVICE_SPI_CONTINUE;
}

# ifdef CONFIG_DEVICE_SPI_BYTECODE_TIMER
static KROUTINE_EXEC(device_spi_ctrl_timeout)
{
  struct dev_timer_rq_s *trq = KROUTINE_CONTAINER(kr, *trq, base.kr);
  struct dev_spi_ctrl_context_s *q = trq->pvdata;

  lock_spin_irq(&q->lock);

  assert(q->timeout != NULL);
  q->timeout = NULL;

  device_spi_ctrl_run(q);
}

static enum device_spi_ret_e
device_spi_ctrl_delay(struct dev_spi_ctrl_context_s *q,
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
      dev_timer_rq_init(trq, device_spi_ctrl_timeout);
      trq->pvdata = q;

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
device_spi_ctrl_sched(struct dev_spi_ctrl_context_s *q)
{
  struct dev_spi_ctrl_rq_s *rq = NULL;
  assert(q->current == NULL);

  /* find next candidate request in queue */
# ifdef CONFIG_DEVICE_SPI_BYTECODE_TIMER
  assert(q->timeout == NULL);

  if (device_check_accessor(&q->timer.base))
    {
      dev_timer_value_t t;
      DEVICE_OP(&q->timer, get_value, &t, 0);

      GCT_FOREACH_NOLOCK(dev_request_queue, &q->queue, item, {
          struct dev_spi_ctrl_rq_s * const rqitem
            = dev_spi_ctrl_rq_s_cast(item);
          struct dev_spi_ctrl_bytecode_rq_s * const bcrq
            = dev_spi_ctrl_bytecode_rq_s_cast(rqitem);

          /* FIXME use dev_timer_check_timeout */
          if (
#  ifdef CONFIG_DEVICE_SPI_TRANSACTION
              !rqitem->bytecode ||
#  endif
              bcrq->sleep_before <= t)
            {
              __dev_rq_queue_remove(&q->queue, item);
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

  rq = dev_spi_ctrl_rq_s_cast(__dev_rq_queue_pop(&q->queue));
  if (rq == NULL)
    return DEVICE_SPI_IDLE;

found:
  q->current = rq;

  return DEVICE_SPI_CONTINUE;
}

# ifdef CONFIG_DEVICE_SPI_BYTECODE
static enum device_spi_ret_e
device_spi_bytecode_exec(struct dev_spi_ctrl_context_s *q,
                         struct dev_spi_ctrl_bytecode_rq_s *rq)
{
  error_t err;
  uint16_t op;

  lock_release_irq(&q->lock);

  for (err = 0; err == 0; )
    {
      struct dev_spi_ctrl_transfer_s *tr = &q->transfer;

      op = bc_run(&rq->vm);

      if (!(op & 0x8000))       /* bytecode end */
        {
          if (op)
            err = -EINVAL;
          assert(!rq->base.cs_state);
          break;
        }

#  ifdef CONFIG_DEVICE_SPI_BYTECODE_TIMER
      dev_timer_value_t t = 0;
#  endif

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

                  DEVICE_OP(&q->timer, get_value, &t, 0);
                }

              switch (op & 0x00c0)
                {
                case 0x0080: /* delay */
                  rq->sleep_before = t + bc_get_reg(&rq->vm, op & 0xf);
                  break;

                case 0x0040: /* deadline */
                  rq->sleep_before = *(dev_timer_value_t*)bc_get_reg(&rq->vm, op & 0xf);
                  break;
                }
#  endif

              switch (op & 0x0300)
                {
                case 0x0000: {  /* yield / sleep */
                  lock_spin_irq(&q->lock);
                  assert(!rq->base.cs_state);
                  enum device_spi_wakeup_e w = (op & 0x0030) >> 4;
                  switch (w)
                     {
                     case DEVICE_SPI_WAKEUP_YIELDC:
                       if (!rq->wakeup)
                         {
                         case DEVICE_SPI_WAKEUP_NONE:
                           __dev_rq_queue_pushback(&q->queue, &rq->base.base);
                           break;
                         }
                      bc_skip(&rq->vm);
                      goto yieldc_done;

                     case DEVICE_SPI_WAKEUP_SLEEP:
                       if (!rq->wakeup)
                         break;
                     yieldc_done:
                       rq->wakeup = 0;
                       lock_release_irq(&q->lock);
                       continue;

                     default:
                       UNREACHABLE();
                     }
                  rq->wakeup_able = w;
                  q->current = NULL;
                  return DEVICE_SPI_CONTINUE;
                }

                case 0x0200: {  /* wait */
                  lock_spin_irq(&q->lock);
#  ifdef CONFIG_DEVICE_SPI_BYTECODE_TIMER
                  return device_spi_ctrl_delay(q, rq);
#  else
                  return DEVICE_SPI_CONTINUE;
#  endif
                }

#  ifdef CONFIG_DEVICE_SPI_BYTECODE_TIMER
                case 0x0300: {
                  dev_timer_value_t *r = (void*)bc_get_reg(&rq->vm, op & 0xf);
                  switch (op & 0x00f0)
                    {
                    case 0x0000: /* nodelay */
                      rq->sleep_before = 0;
                      break;

                    case 0x00c0: /* timestamp */
                      *r = t;
                      break;

                    case 0x00e0: /* elapsed */
                      if (t < rq->sleep_before)
                        bc_skip(&rq->vm);
                      break;

                    case 0x00f0: /* elapsed_r */
                      if (t < *r)
                        bc_skip(&rq->vm);
                      break;
                    }
                  continue;
                }
#  endif

                default:
                  continue;
                }

            case 0x0400: {
              struct dev_spi_ctrl_config_s *rcfg = rq->base.config;
              rcfg->dirty = 1;
              if (op & 0x0080)  /* brate */
                {
                  rcfg->bit_rate1k = bc_get_reg(&rq->vm, op & 0xf) >> 10;
                }
              else              /* width */
                {
                  rcfg->word_width = (op & 0x001f) + 1;
                  rcfg->bit_order = (op >> 5) & 1;
                }
              continue;
            }

#  ifdef CONFIG_DEVICE_SPI_BYTECODE_GPIO
            case 0x0800: /* gpio* */
            case 0x0c00: {
              if (!device_check_accessor(&rq->base.gpio.base))
                {
                  err = -ENOENT;
                  continue;
                }
              uint_fast8_t i = (op >> 5) & 0xf;
              gpio_id_t id = rq->gpio_map[i];
              if (id == GPIO_INVALID_ID)
                continue;

              gpio_width_t w = rq->gpio_wmap[i];
              uint8_t value[8];

              if (op & 0x0200)  /* gpioget */
                {
                  err = DEVICE_OP(&rq->base.gpio, get_input, id, id + w - 1, value);
                  bc_set_reg(&rq->vm, op & 0xf, endian_le32_na_load(value) & ((1 << w) - 1));
                }
              else if (op & 0x0400) /* gpioset */
                {
                  endian_le32_na_store(value, bc_get_reg(&rq->vm, op & 0xf));
                  err = DEVICE_OP(&rq->base.gpio, set_output, id, id + w - 1, value, value);
                }
              else              /* gpiomode */
                {
                  err = DEVICE_OP(&rq->base.gpio, set_mode, id,
                                  id + w - 1, dev_gpio_mask1, op & 0x1f);
                }
              continue;
            }
#  endif
            default:
              break;
            }

        case 0x4000: {
          uint8_t ra = (op >> 4) & 0xf;
          void *addr = (void*)bc_get_reg(&rq->vm, ra);
          void *addr2 = (void*)bc_get_reg(&rq->vm, 1 ^ ra);
          size_t count = bc_get_reg(&rq->vm, op & 0xf);
          enum dev_spi_cs_op_e csop = (op >> 8) & 3;
          tr->data.count = count;
          switch (op & 0x0c00)
            {
            case 0x0000:  /* spi_pad */
              if (op & 0x0080)
                tr->data.count = 0; /* spi_cs */
              tr->data.in_width = 0;
              tr->data.out_width = 0;
              tr->data.in = NULL;
              tr->data.out = bc_get_bytepack(&rq->vm, ra);
              break;
            case 0x0400:  /* rdm */
              tr->data.in_width = 1;
              tr->data.out_width = 0;
              tr->data.in = addr;
              memset(tr->data.in, 0xff, count);
              tr->data.out = addr;
              break;
            case 0x0800:  /* wrm */
              tr->data.in_width = 0;
              tr->data.out_width = 1;
              tr->data.in = NULL;
              tr->data.out = addr;
              break;
            case 0x0c00: /* swpm */
              tr->data.in_width = 1;
              tr->data.out_width = 1;
              tr->data.in = addr;
              tr->data.out = addr2;
              break;
            default:
              UNREACHABLE();
            }
          lock_spin_irq(&q->lock);
          return device_spi_ctrl_transfer(q, &rq->base, csop);
        }

        case 0x2000:           /* wr */
        case 0x6000:
          tr->data.count = ((op >> 8) & 0xf) + 1;
          tr->data.out = bc_get_bytepack(&rq->vm, op & 0xf);
          tr->data.in = NULL;
          goto reg_tr;
        case 0x3000:            /* swp */
        case 0x7000:
          tr->data.count = ((op >> 8) & 0xf) + 1;
          tr->data.in = bc_get_bytepack(&rq->vm, (op >> 4) & 0xf);
          tr->data.out = bc_get_bytepack(&rq->vm, op & 0xf);
          goto reg_tr;
        case 0x1000:            /* rd */
        case 0x5000: {
          uint_fast8_t l = ((op >> 8) & 0xf) + 1;
          tr->data.count = l;
          tr->data.out = tr->data.in = bc_get_bytepack(&rq->vm, (op >> 4) & 0xf);
          memset(tr->data.in, 0xff, l);
         reg_tr:
          tr->data.in_width = 1;
          tr->data.out_width = 1;
          lock_spin_irq(&q->lock);
          return device_spi_ctrl_transfer(q, &rq->base, (op >> 14) & 3);
        }

        }

      err = -EINVAL;            /* invalid op */
    }

  lock_spin_irq(&q->lock);
  return device_spi_ctrl_end(q, &rq->base, err);
}
# endif

static void device_spi_ctrl_run(struct dev_spi_ctrl_context_s *q)
{
  enum device_spi_ret_e r;

  do {
    r = DEVICE_SPI_IDLE;
    struct dev_spi_ctrl_rq_s *rq = q->current;
    if (rq != NULL)
      {
# if defined(CONFIG_DEVICE_SPI_TRANSACTION) && defined(CONFIG_DEVICE_SPI_BYTECODE)
        if (rq->bytecode)
# endif
# ifdef CONFIG_DEVICE_SPI_BYTECODE
          {
            r = device_spi_bytecode_exec(q, dev_spi_ctrl_bytecode_rq_s_cast(rq));
          }
# endif
# if defined(CONFIG_DEVICE_SPI_TRANSACTION) && defined(CONFIG_DEVICE_SPI_BYTECODE)
        else
# endif
# ifdef CONFIG_DEVICE_SPI_TRANSACTION
          {
            struct dev_spi_ctrl_transaction_rq_s *trq = dev_spi_ctrl_transaction_rq_s_cast(rq);
            q->transfer.data = trq->data;
            r = device_spi_ctrl_transfer(q, rq, trq->cs_op);
          }
# endif
      }
    else if (!dev_rq_queue_isempty(&q->queue))
      r = device_spi_ctrl_sched(q);
  } while (r == DEVICE_SPI_CONTINUE);

  lock_release_irq(&q->lock);
}

static KROUTINE_EXEC(device_spi_ctrl_resume)
{
  struct dev_spi_ctrl_context_s *q = KROUTINE_CONTAINER(kr, *q, kr);

  lock_spin_irq(&q->lock);

  assert(q->current != NULL);

  device_spi_ctrl_run(q);
}

static bool_t device_spi_ctrl_entry(struct dev_spi_ctrl_context_s *q,
                                    struct dev_spi_ctrl_rq_s *rq)
{
  if (q->current != NULL)
    return 1;

# ifdef CONFIG_DEVICE_SPI_BYTECODE_TIMER
  if (q->timeout != NULL)
    {
      if (DEVICE_OP(&q->timer, cancel, &q->timer_rq))
        return 1;
      q->timeout = NULL;
    }
# endif

  q->current = rq;
  kroutine_init_deferred(&q->kr, device_spi_ctrl_resume);
  kroutine_exec(&q->kr);

  return 0;
}

# ifdef CONFIG_DEVICE_SPI_TRANSACTION
void dev_spi_transaction_start(struct device_spi_ctrl_s *ctrl,
                               struct dev_spi_ctrl_transaction_rq_s *rq)
{
#  ifdef CONFIG_DEVICE_SPI_GPIO_CS
  assert(!rq->base.cs_gpio || device_check_accessor(&rq->base.gpio.base));
#  endif

  rq->base.ctrl = ctrl;
  struct dev_spi_ctrl_context_s *q = device_spi_ctrl_context(ctrl);

  lock_spin_irq(&q->lock);

  assert(!rq->base.enqueued);

  rq->error = 0;
  rq->base.enqueued = 1;
#  ifdef CONFIG_DEVICE_SPI_BYTECODE
  rq->base.bytecode = 0;
#  endif

  if (device_spi_ctrl_entry(q, &rq->base))
    __dev_rq_queue_pushback(&q->queue, &rq->base.base);

  lock_release_irq(&q->lock);
}
# endif

# ifdef CONFIG_DEVICE_SPI_BYTECODE
error_t dev_spi_bytecode_start_va(struct device_spi_ctrl_s *ctrl,
                                  struct dev_spi_ctrl_bytecode_rq_s *rq,
                                  const void *pc, uint16_t mask, va_list ap)
{
  error_t err = -EBUSY;

#  ifdef CONFIG_DEVICE_SPI_GPIO_CS
  assert(!rq->base.cs_gpio || device_check_accessor(&rq->base.gpio.base));
#  endif

  rq->base.ctrl = ctrl;
  struct dev_spi_ctrl_context_s *q = device_spi_ctrl_context(ctrl);

  lock_spin_irq(&q->lock);

  if (!rq->base.enqueued)
    {
      err = 0;

#  ifdef CONFIG_DEVICE_SPI_BYTECODE_TIMER
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
#  ifdef CONFIG_DEVICE_SPI_TRANSACTION
      rq->base.bytecode = 1;
#  endif
      rq->wakeup = 0;
      rq->wakeup_able = DEVICE_SPI_WAKEUP_NONE;

      if (device_spi_ctrl_entry(q, &rq->base))
        __dev_rq_queue_pushback(&q->queue, &rq->base.base);
    }

 err:
  lock_release_irq(&q->lock);

  return err;
}

error_t dev_spi_bytecode_start(struct device_spi_ctrl_s *ctrl,
                               struct dev_spi_ctrl_bytecode_rq_s *rq,
                               const void *pc, uint16_t mask, ...)
{
  va_list ap;
  va_start(ap, mask);
  error_t err = dev_spi_bytecode_start_va(ctrl, rq, pc, mask, ap);
  va_end(ap);

  return err;
}

error_t dev_spi_bytecode_wakeup(struct device_spi_ctrl_s *ctrl,
                                struct dev_spi_ctrl_bytecode_rq_s *rq)
{
  struct dev_spi_ctrl_context_s *q = device_spi_ctrl_context(ctrl);
  error_t err = 0;

  lock_spin_irq(&q->lock);

  if (!rq->base.enqueued)
    {
      err = -EBUSY;
    }
  else if (q->current != &rq->base)
    {
      enum device_spi_wakeup_e w = rq->wakeup_able;
      rq->wakeup_able = DEVICE_SPI_WAKEUP_NONE;
 #  ifdef CONFIG_DEVICE_SPI_BYTECODE_TIMER
      rq->sleep_before = 0;
 #  endif
      switch (w)
        {
        case DEVICE_SPI_WAKEUP_YIELDC:
          bc_skip(&rq->vm);
          if (!device_spi_ctrl_entry(q, &rq->base))
            __dev_rq_queue_remove(&q->queue, &rq->base.base);
          break;

        case DEVICE_SPI_WAKEUP_SLEEP:
          if (device_spi_ctrl_entry(q, &rq->base))
            __dev_rq_queue_pushback(&q->queue, &rq->base.base);
          break;

        case DEVICE_SPI_WAKEUP_NONE:
          rq->wakeup = 1;
          break;

        default:
          UNREACHABLE();
        }
    }
  else
    {
      rq->wakeup = 1;
    }

  lock_release_irq(&q->lock);
  return err;
}
# endif

error_t dev_spi_context_init(struct device_s *dev, struct dev_spi_ctrl_context_s *q)
{
  __unused__ error_t err;

# ifdef CONFIG_DEVICE_SPI_BYTECODE_TIMER
  if (device_get_param_dev_accessor(dev, "timer", &q->timer.base, DRIVER_CLASS_TIMER))
    device_init_accessor(&q->timer.base);

  q->timeout = NULL;
# endif

  q->config = NULL;
  q->current = NULL;
  dev_rq_queue_init(&q->queue);
  lock_init_irq(&q->lock);
  memset(&q->transfer, 0, sizeof(q->transfer));
  return 0;
}

void dev_spi_context_cleanup(struct dev_spi_ctrl_context_s *q)
{
  lock_destroy_irq(&q->lock);
  dev_rq_queue_destroy(&q->queue);
# ifdef CONFIG_DEVICE_SPI_BYTECODE_TIMER
  device_put_accessor(&q->timer.base);
# endif
}

static error_t dev_drv_spi_init(struct device_s *dev,
                                struct dev_spi_ctrl_rq_s *rq,
                                struct device_spi_ctrl_s *ctrl)
{
  uintptr_t x;

  rq->cs_cfg.polarity = rq->config->cs_pol;

  if (!device_get_param_uint(dev, "spi-cs-id", &x))
    {
# ifdef CONFIG_DEVICE_SPI_CTRL_CS
      rq->cs_ctrl = 1;
      rq->cs_cfg.id = x;
# else
      return -ENOTSUP;
# endif
    }
  else if (!device_get_param_uint(dev, "gpio-cs-id", &x))
    {
# ifdef CONFIG_DEVICE_SPI_GPIO_CS
      rq->cs_gpio = 1;
      rq->cs_cfg.id = x;
# else
      return -ENOTSUP;
# endif
    }

  if (device_get_param_dev_accessor(dev, "spi", &ctrl->base, DRIVER_CLASS_SPI_CTRL))
    return -ENOENT;

  return 0;
}

static error_t dev_drv_spi_cs_init(struct device_s *dev,
                                     struct dev_spi_ctrl_rq_s *rq,
                                     struct device_gpio_s **gpio,
                                     struct device_spi_ctrl_s *ctrl)
{
# ifdef CONFIG_LIBC_ASSERT
  rq->cs_state = 0;
# endif

# if defined(CONFIG_DEVICE_SPI_BYTECODE_GPIO) || defined(CONFIG_DEVICE_SPI_GPIO_CS)
  if (0
#  if defined(CONFIG_DEVICE_SPI_BYTECODE_GPIO)
      || gpio != NULL
#  endif
#  ifdef CONFIG_DEVICE_SPI_GPIO_CS
      || rq->cs_gpio
#  endif
      )
    {
      if (device_get_param_dev_accessor(dev, "gpio", &rq->gpio.base, DRIVER_CLASS_GPIO))
        return -ENOENT;
#  ifdef CONFIG_DEVICE_SPI_GPIO_CS
      if (rq->cs_gpio)
        {
          dev_gpio_mode(&rq->gpio, rq->cs_cfg.id, DEV_PIN_PUSHPULL);
          dev_gpio_out(&rq->gpio, rq->cs_cfg.id, rq->cs_cfg.polarity ^ 1);
        }
#  endif
    }
# endif

# ifdef CONFIG_DEVICE_SPI_CTRL_CS
  if (rq->cs_ctrl)
    {
      error_t err = DEVICE_OP(ctrl, cscfg, &rq->cs_cfg);
      if (err)
        return err;
    }
# endif

  if (gpio != NULL)
# ifdef CONFIG_DEVICE_SPI_BYTECODE_GPIO
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
                                  const struct dev_spi_ctrl_config_s *cfg,
                                  struct device_spi_ctrl_s *ctrl,
                                  struct device_gpio_s **gpio,
                                  struct device_timer_s **timer)
{
  dev_spi_bytecode_init(rq);

  rq->base.config = (struct dev_spi_ctrl_config_s *)cfg;

  error_t err = dev_drv_spi_init(dev, &rq->base, ctrl);
  if (err)
    return err;

  if (timer != NULL)
    {
#  ifdef CONFIG_DEVICE_SPI_BYTECODE_TIMER
      struct dev_spi_ctrl_context_s *q = device_spi_ctrl_context(ctrl);
      err = -ENOENT;
      if (!device_check_accessor(&q->timer.base))
        goto err;
      *timer = &q->timer;
#  else
      err = -ENOTSUP;
      goto err;
#  endif
    }

  err = dev_drv_spi_cs_init(dev, &rq->base, gpio, ctrl);
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
#  if defined(CONFIG_DEVICE_SPI_BYTECODE_GPIO) || defined(CONFIG_DEVICE_SPI_GPIO_CS)
  device_put_accessor(&rq->base.gpio.base);
#  endif
  device_put_accessor(&ctrl->base);
}
# endif

# ifdef CONFIG_DEVICE_SPI_TRANSACTION

error_t dev_drv_spi_transaction_init(struct device_s *dev,
                                     struct dev_spi_ctrl_transaction_rq_s *rq,
                                     const struct dev_spi_ctrl_config_s *cfg,
                                     struct device_spi_ctrl_s *ctrl,
                                     struct device_gpio_s **gpio)
{
  dev_spi_transaction_init(rq);

  rq->base.config = (struct dev_spi_ctrl_config_s *)cfg;

  error_t err = dev_drv_spi_init(dev, &rq->base, ctrl);
  if (err)
    return err;

  err = dev_drv_spi_cs_init(dev, &rq->base, gpio, ctrl);
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
#  if defined(CONFIG_DEVICE_SPI_BYTECODE_GPIO) || defined(CONFIG_DEVICE_SPI_GPIO_CS)
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

# ifdef CONFIG_DEVICE_SPI_BYTECODE
error_t
dev_spi_wait_bytecode(struct device_spi_ctrl_s *ctrl,
                      struct dev_spi_ctrl_bytecode_rq_s *rq,
                      const void *pc, uint16_t mask, ...)
{
  struct dev_request_status_s st;

  dev_request_sched_init(&rq->base.base, &st);
  va_list ap;
  va_start(ap, mask);
  error_t err = dev_spi_bytecode_start_va(ctrl, rq, pc, mask, ap);
  va_end(ap);
  if (err)
    return err;
  dev_request_sched_wait(&st);
  return rq->error;
}
# endif

# ifdef CONFIG_DEVICE_SPI_TRANSACTION
error_t
dev_spi_wait_transaction(struct device_spi_ctrl_s *ctrl,
                         struct dev_spi_ctrl_transaction_rq_s *rq)
{
  struct dev_request_status_s st;

  dev_request_sched_init(&rq->base.base, &st);
  dev_spi_transaction_start(ctrl, rq);
  dev_request_sched_wait(&st);
  return rq->error;
}
# endif

#endif


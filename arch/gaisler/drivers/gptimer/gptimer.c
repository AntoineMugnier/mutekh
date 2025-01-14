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

    Copyright (c) 2012 Alexandre Becoulet <alexandre.becoulet@telecom-paristech.fr>
    Copyright (c) 2012 Institut Telecom / Telecom ParisTech

*/

#include <string.h>
#include <stdio.h>

#include <hexo/types.h>
#include <hexo/iospace.h>
#include <hexo/endian.h>

#include <device/device.h>
#include <device/resources.h>
#include <device/driver.h>
#include <device/class/timer.h>
#include <device/irq.h>

#include <mutek/mem_alloc.h>
#include <mutek/printk.h>
#include <mutek/kroutine.h>

#define TIMER_REG_SCALER         0x00
#define TIMER_REG_SC_RELOAD      0x04
#define TIMER_REG_CFG            0x08
#define  TIMER_CFG_NTIMERS       0x00000007
#define  TIMER_CFG_SIRQ          0x00000100
#define  TIMER_CFG_DFREEZE       0x00000200

#define TIMER_REG_COUNTER        0x00
#define TIMER_REG_RELOAD         0x04
#define TIMER_REG_CTRL           0x08
#define  TIMER_CTRL_ENABLED      0x00000001
#define  TIMER_CTRL_RESTART      0x00000002
#define  TIMER_CTRL_LOAD         0x00000004
#define  TIMER_CTRL_IE           0x00000008
#define  TIMER_CTRL_IP           0x00000010
#define  TIMER_CTRL_CHAIN        0x00000020
#define  TIMER_CTRL_DEBUGHLT     0x00000040

#define TIMER_REG_ADDR(a, r, n)  ((a) + 0x10 + (r) + (n) * 0x10)

/*
  This driver provides 2 timer sub-devices for each available hardware
  timer of the gptimer. Timers at index N*2 use the hardware
  decrementing counter as a scaler; the reported timer value increment
  when the counter reaches zero and generate an irq. Timers at index
  N*2+1 report the actual value of the hardware timer but are not able
  to handle requests.
*/

struct gptimer_state_s
{
#ifdef CONFIG_DEVICE_IRQ
  dev_request_pqueue_root_t queue;
  dev_timer_value_t value;
  dev_timer_cfgrev_t rev;
  uint32_t t0_reload;
#endif
  int_fast8_t start_count;
};

DRIVER_PV(struct gptimer_private_s
{
  uintptr_t addr;
  uintptr_t t_count;
  struct dev_irq_src_s *irq_eps;
  struct gptimer_state_s t[0];
});

#ifdef CONFIG_DEVICE_IRQ
static inline bool_t gptimer_irq_process(struct device_s *dev, uint_fast8_t number)
{
  struct gptimer_private_s *pv = dev->drv_pv;

  if ((endian_be32(TIMER_CTRL_IP) & cpu_mem_read_32(TIMER_REG_ADDR(pv->addr, TIMER_REG_CTRL, number))) == 0)
    return 0;

  cpu_mem_write_32(TIMER_REG_ADDR(pv->addr, TIMER_REG_CTRL, number),
                   endian_be32(TIMER_CTRL_IP | TIMER_CTRL_ENABLED |
                               TIMER_CTRL_RESTART | TIMER_CTRL_IE));

  struct gptimer_state_s *p = pv->t + number;

  p->value++;

  if (p->start_count <= 0)
    return 1;

  while (1)
    {
      struct dev_timer_rq_s *rq = dev_timer_rq_head(&p->queue);

      if (rq == NULL)
        {
          /* stop timer if not in use */
          p->start_count &= ~1;
          if (p->start_count == 0)
            cpu_mem_write_32(TIMER_REG_ADDR(pv->addr, TIMER_REG_CTRL, number), endian_be32(TIMER_CTRL_IE));
          break;
        }

      assert(p->start_count & 1);

      if (rq->deadline > p->value)
        break;

      rq->base.drvdata = 0;
      dev_timer_rq_remove(&p->queue, rq);

      lock_release(&dev->lock);
      dev_timer_rq_done(rq);
      lock_spin(&dev->lock);
    }

  return 1;
}

static DEV_IRQ_SRC_PROCESS(gptimer_irq_single)
{
  struct device_s *dev = ep->base.dev;
  struct gptimer_private_s *pv = dev->drv_pv;

  lock_spin(&dev->lock);

  bool_t done;

  do {
    uint_fast8_t i;
    done = 1;

    for (i = 0; i < pv->t_count; i++)
      {
        if (gptimer_irq_process(dev, i))
          done = 0;
      }
  } while (!done);

  lock_release(&dev->lock);
}

static DEV_IRQ_SRC_PROCESS(gptimer_irq_separate)
{
  struct device_s *dev = ep->base.dev;
  struct gptimer_private_s *pv = dev->drv_pv;
  uint_fast8_t number = ep - pv->irq_eps;

  lock_spin(&dev->lock);

  while (gptimer_irq_process(dev, number))
    ;

  lock_release(&dev->lock);
}

#endif

static DEV_TIMER_CANCEL(gptimer_cancel)
{
# ifdef CONFIG_DEVICE_IRQ
  struct device_s *dev = accessor->dev;
  struct gptimer_private_s *pv = dev->drv_pv;
  error_t err = 0;
  uint_fast8_t number = accessor->number / 2;
  uint_fast8_t mode = accessor->number % 2;

  if (mode)
    return -ENOTSUP;

  struct gptimer_state_s *p = pv->t + number;

  LOCK_SPIN_IRQ(&dev->lock);

  if (rq->base.drvdata == p)
    {
      assert(p->start_count & 1);

      dev_timer_rq_remove(&p->queue, rq);
      rq->base.drvdata = NULL;

      // stop timer if not in use
      if (dev_rq_pqueue_isempty(&p->queue))
        {
          p->start_count &= ~1;
          if (p->start_count == 0)
            cpu_mem_write_32(TIMER_REG_ADDR(pv->addr, TIMER_REG_CTRL, number), endian_be32(TIMER_CTRL_IE));
        }
    }
  else
    {
      err = -ETIMEDOUT;
    }

  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
# else
  return -ENOTSUP;
# endif
}

#ifdef CONFIG_DEVICE_IRQ
static void gptimer_t0_start(struct gptimer_private_s *pv,
                             uint_fast8_t number)
{
  struct gptimer_state_s *p = pv->t + number;
  uint32_t c = endian_be32(cpu_mem_read_32(TIMER_REG_ADDR(pv->addr, TIMER_REG_COUNTER, number)));
  if (c > p->t0_reload)
    cpu_mem_write_32(TIMER_REG_ADDR(pv->addr, TIMER_REG_COUNTER, number), endian_be32(p->t0_reload));
  cpu_mem_write_32(TIMER_REG_ADDR(pv->addr, TIMER_REG_RELOAD, number), endian_be32(p->t0_reload));
  cpu_mem_write_32(TIMER_REG_ADDR(pv->addr, TIMER_REG_CTRL, number),
                   endian_be32(TIMER_CTRL_ENABLED | TIMER_CTRL_RESTART | TIMER_CTRL_IE));
}
#endif

static error_t gptimer_start(struct gptimer_private_s *pv,
                             uint_fast8_t number, uint_fast8_t mode)
{
  struct gptimer_state_s *p = pv->t + number;

  if (!mode)
    {
#ifdef CONFIG_DEVICE_IRQ
      if (p->start_count < 0)
        return -EBUSY;
      else
        {
          if (p->start_count == 0)
            gptimer_t0_start(pv, number);
          p->start_count += 2;
        }
#else
      return -ENOTSUP;
#endif
    }
  else if (p->start_count > 0)
    return -EBUSY;
  else
    {
      if (p->start_count == 0)
        {
          p->value = 0;
          cpu_mem_write_32(TIMER_REG_ADDR(pv->addr, TIMER_REG_COUNTER, number), 0xffffffff);
          cpu_mem_write_32(TIMER_REG_ADDR(pv->addr, TIMER_REG_RELOAD, number), 0xffffffff);
          cpu_mem_write_32(TIMER_REG_ADDR(pv->addr, TIMER_REG_CTRL, number),
                           endian_be32(TIMER_CTRL_ENABLED | TIMER_CTRL_RESTART | TIMER_CTRL_IE));
        }
      p->start_count -= 2;
    }

  return 0;
}

static error_t gptimer_stop(struct gptimer_private_s *pv,
                            uint_fast8_t number, uint_fast8_t mode)
{
  struct gptimer_state_s *p = pv->t + number;

  if (!mode)
    {
#ifdef CONFIG_DEVICE_IRQ
      if (p->start_count < 0)
        return -EBUSY;
      else
        {
          if (p->start_count < 2)
            return -EINVAL;
          else
            {
              p->start_count -= 2;
              if (p->start_count == 0)
                cpu_mem_write_32(TIMER_REG_ADDR(pv->addr, TIMER_REG_CTRL, number),
                                 endian_be32(TIMER_CTRL_IE));
            }
        }
#else
      return -ENOTSUP;
#endif
    }
  else if (p->start_count > 0)
    return -EBUSY;
  else
    {
      if (p->start_count == 0)
        return -EINVAL;
      else
        {
          p->start_count += 2;
          if (p->start_count == 0)
            cpu_mem_write_32(TIMER_REG_ADDR(pv->addr, TIMER_REG_CTRL, number), 0);
        }
    }

  return 0;
}

static DEV_TIMER_REQUEST(gptimer_request)
{
# ifdef CONFIG_DEVICE_IRQ
  struct device_s *dev = accessor->dev;
  struct gptimer_private_s *pv = dev->drv_pv;
  error_t err = 0;
  uint_fast8_t number = accessor->number / 2;
  uint_fast8_t mode = accessor->number % 2;

  if (mode)
    return -ENOTSUP;

  struct gptimer_state_s *p = pv->t + number;

  LOCK_SPIN_IRQ(&dev->lock);

  if (p->start_count < 0)  /* hardware timer already used in mode 0 */
    err = -EBUSY;
  if (rq->rev && rq->rev != p->rev)
    err = -EAGAIN;
  else
    {
      uint64_t val = p->value;

      if (rq->delay)
        rq->deadline = val + rq->delay;

      if (rq->deadline <= val)
        err = -ETIMEDOUT;
      else
        {
          rq->base.drvdata = p;
          dev_timer_rq_insert(&p->queue, rq);

          /* start timer if needed */
          if (p->start_count == 0)
            gptimer_t0_start(pv, number);
          p->start_count |= 1;
        }
    }

  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
# else
  return -ENOTSUP;
# endif
}

static DEV_USE(gptimer_use)
{
  struct device_accessor_s *accessor = param;

  switch (op)
    {
    case DEV_USE_GET_ACCESSOR: {
      struct gptimer_private_s *pv = accessor->dev->drv_pv;
      if (accessor->number / 2 >= pv->t_count)
        return -ENOTSUP;
      return 0;
    }
    case DEV_USE_START: {
      struct device_s *dev = accessor->dev;
      struct gptimer_private_s *pv = dev->drv_pv;
      return gptimer_start(pv, accessor->number / 2, accessor->number % 2);
    }
    case DEV_USE_STOP: {
      struct device_s *dev = accessor->dev;
      struct gptimer_private_s *pv = dev->drv_pv;
      return gptimer_stop(pv, accessor->number / 2, accessor->number % 2);
    }
    case DEV_USE_LAST_NUMBER: {
      struct gptimer_private_s *pv = accessor->dev->drv_pv;
      accessor->number = pv->t_count * 2 - 1;
      return 0;
    }
    default:
      return dev_use_generic(param, op);
    }
}

static DEV_TIMER_GET_VALUE(gptimer_get_value)
{
  struct device_s *dev = accessor->dev;
  struct gptimer_private_s *pv = dev->drv_pv;
  uint_fast8_t number = accessor->number / 2;
  uint_fast8_t mode = accessor->number % 2;
  error_t err = 0;

  LOCK_SPIN_IRQ(&dev->lock);

  struct gptimer_state_s *p = pv->t + number;

  if (!p->start_count || ((p->start_count > 0) ^ !mode))
    {
      /* timer not started in this mode */
      err = -EBUSY;
    }
  else if (!mode)
    {
# ifdef CONFIG_DEVICE_IRQ
      if (rev && rev != p->rev)
        err = -EAGAIN;
      else
        *value = p->value;
#else
      err = -ENOTSUP;
#endif
    }
  else
    {
      uint64_t v = ~endian_be32(cpu_mem_read_32(TIMER_REG_ADDR(pv->addr, TIMER_REG_COUNTER, number)));
#ifdef CONFIG_DEVICE_IRQ
      if (rev && rev != p->rev)
        err = -EAGAIN;
      if (v < 0x80000000 && ((endian_be32(TIMER_CTRL_IP) &
             cpu_mem_read_32(TIMER_REG_ADDR(pv->addr, TIMER_REG_CTRL, number)))))
        v += 0x100000000ULL;
      v += p->value << 32;
      *value = v;
#else
      if (rev && rev != 1)
        err = -EAGAIN;
      *value = v;
#endif
    }

  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
}

static DEV_TIMER_CONFIG(gptimer_config)
{
  struct device_s *dev = accessor->dev;
  struct gptimer_private_s *pv = dev->drv_pv;
  uint_fast8_t number = accessor->number / 2;
  uint_fast8_t mode = accessor->number % 2;
  error_t err = 0;

  if (cfg && device_get_res_freq(accessor->dev, &cfg->freq, 0))
    {
      cfg->freq.num = CONFIG_GAISLER_BASE_FREQ;
      cfg->freq.denom = 1;
    }

  LOCK_SPIN_IRQ(&dev->lock);

  if (!mode)
    {
# ifdef CONFIG_DEVICE_IRQ
      struct gptimer_state_s *p = pv->t + number;

      if (res)
        {
          if (p->start_count)
            {
              err = -EBUSY;
            }
          else
            {
              if (res < 2048)    /* This would generate too many interrupts */
                {
                  err = -ERANGE;
                  res = 2048;
                }
              uint32_t r = res % CONFIG_DRIVER_GAISLER_GPTIMER_PRESCALE;
              if (r)
                {
                  err = -ERANGE;
                  res += CONFIG_DRIVER_GAISLER_GPTIMER_PRESCALE - r;
                }
              p->t0_reload = res / CONFIG_DRIVER_GAISLER_GPTIMER_PRESCALE - 1;
              p->rev += 2;
            }
        }
      else
        {
          res = p->t0_reload + 1;
          res *= CONFIG_DRIVER_GAISLER_GPTIMER_PRESCALE;
        }

      if (cfg)
        {
          cfg->max = 0xffffffffffffffffULL;
          cfg->rev = p->rev;
          cfg->res = res;
          cfg->cap = DEV_TIMER_CAP_REQUEST | DEV_TIMER_CAP_STOPPABLE | DEV_TIMER_CAP_KEEPVALUE;
        }
#else
      err = -ENOTSUP;
#endif
    }
  else
    {
      if (res && res != CONFIG_DRIVER_GAISLER_GPTIMER_PRESCALE)
        err = -ERANGE;

      if (cfg)
        {
          cfg->rev = 1;
          cfg->res = CONFIG_DRIVER_GAISLER_GPTIMER_PRESCALE;
          cfg->cap = DEV_TIMER_CAP_STOPPABLE | DEV_TIMER_CAP_HIGHRES;
#ifdef CONFIG_DEVICE_IRQ
          cfg->max = 0xffffffffffffffffULL;
#else
          cfg->cap |= DEV_TIMER_CAP_TICKLESS;
          cfg->max = 0xffffffff;
#endif
        }
    }

  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
}

/************************************************************************/


static DEV_INIT(gptimer_init)
{
  struct gptimer_private_s  *pv;
  uint_fast8_t i;
  uintptr_t addr;


  if (device_res_get_uint(dev, DEV_RES_MEM, 0, &addr, NULL))
    return -ENOTSUP;

  uint32_t cfg = endian_be32(cpu_mem_read_32(addr + TIMER_REG_CFG));
  uint_fast8_t t_count = cfg & TIMER_CFG_NTIMERS;
#ifdef CONFIG_DEVICE_IRQ
  uint_fast8_t irq_count = cfg & TIMER_CFG_SIRQ ? t_count : 1;
#endif

  pv = mem_alloc(sizeof (*pv) + t_count * (sizeof(struct gptimer_state_s)
#ifdef CONFIG_DEVICE_IRQ
                              + irq_count * sizeof(struct dev_irq_src_s)
#endif
                                           ), (mem_scope_sys));
  if (!pv)
    return -ENOMEM;

  memset(pv, 0, sizeof(*pv));
  pv->t_count = t_count;
  dev->drv_pv = pv;
  pv->addr = addr;

  cpu_mem_write_32(addr + TIMER_REG_SC_RELOAD, endian_be32(CONFIG_DRIVER_GAISLER_GPTIMER_PRESCALE - 1));

#ifdef CONFIG_DEVICE_IRQ
  pv->irq_eps = (void*)(pv->t + t_count);

  if (cfg & TIMER_CFG_SIRQ)
    device_irq_source_init(dev, pv->irq_eps, irq_count,
                           gptimer_irq_separate);
  else
    device_irq_source_init(dev, pv->irq_eps, irq_count,
                           gptimer_irq_single);

  if (device_irq_source_link(dev, pv->irq_eps, irq_count, -1))
    goto err_mem;
#endif

  for (i = 0; i < t_count; i++)
    {
      struct gptimer_state_s *p = pv->t + i;
      p->start_count = 0;
      p->rev = 1;
# ifdef CONFIG_DEVICE_IRQ
      p->t0_reload = 9999;
      p->value = 0;
      dev_rq_pqueue_init(&p->queue);
      cpu_mem_write_32(TIMER_REG_ADDR(addr, TIMER_REG_CTRL, i),
                       endian_be32(TIMER_CTRL_IP | TIMER_CTRL_IE));
# else
      cpu_mem_write_32(TIMER_REG_ADDR(addr, TIMER_REG_COUNTER, i), 0xffffffff);
      cpu_mem_write_32(TIMER_REG_ADDR(addr, TIMER_REG_RELOAD, i), 0xffffffff);
      cpu_mem_write_32(TIMER_REG_ADDR(addr, TIMER_REG_CTRL, i),
                       endian_be32(TIMER_CTRL_ENABLED | TIMER_CTRL_RESTART));
# endif
    }

  return 0;

# ifdef CONFIG_DEVICE_IRQ
 err_mem:
  mem_free(pv);
  return -1;
#endif
}

static DEV_CLEANUP(gptimer_cleanup)
{
  struct gptimer_private_s *pv = dev->drv_pv;

  uint_fast8_t i;

  for (i = 0; i < pv->t_count; i++)
    {
      struct gptimer_state_s *p = pv->t + i;
      if (p->start_count > 0)
        return -EBUSY;
    }

  for (i = 0; i < pv->t_count; i++)
    {
      cpu_mem_write_32(TIMER_REG_ADDR(pv->addr, TIMER_REG_CTRL, i), 0);
#ifdef CONFIG_DEVICE_IRQ
      struct gptimer_state_s *p = pv->t + i;
      dev_rq_pqueue_destroy(&p->queue);
#endif
    }

#ifdef CONFIG_DEVICE_IRQ
  device_irq_source_unlink(dev, pv->irq_eps, pv->t_count);
#endif

  mem_free(pv);

  return 0;
}

DRIVER_DECLARE(gptimer_drv, 0, "Gaisler GPTIMER", gptimer,
               DRIVER_TIMER_METHODS(gptimer));

DRIVER_REGISTER(gptimer_drv,
                DEV_ENUM_GAISLER_ENTRY(0x1, 0x011));


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

    Copyright Alexandre Becoulet <alexandre.becoulet@lip6.fr> (c) 2006

*/

#include <hexo/types.h>
#include <hexo/endian.h>
#include <hexo/iospace.h>
#include <mutek/mem_alloc.h>
#include <hexo/interrupt.h>

#include <device/device.h>
#include <device/resources.h>
#include <device/class/char.h>
#include <device/irq.h>

#include <gct_platform.h>
#include <gct/container_ring.h>

#ifdef CONFIG_DEVICE_IRQ
#define GCT_CONTAINER_ALGO_tty_fifo RING
GCT_CONTAINER_TYPES(tty_fifo, uint8_t, 256);
GCT_CONTAINER_FCNS(tty_fifo, static inline, tty_fifo,
                   init, destroy, pop_array, pushback, isfull, isempty);
#endif

struct tty_soclib_tty_s
{
  /* tty input request queue and char fifo */
  dev_request_queue_root_t	read_q;
#ifdef CONFIG_DEVICE_IRQ
  tty_fifo_root_t		read_fifo;
#endif
};

struct tty_soclib_context_s
{
  struct dev_irq_src_s          irq_ep;
  uintptr_t                     addr;
  uint_fast8_t                  count;
  struct tty_soclib_tty_s       ttys[0];
};

#define TTY_SOCLIB_REG_WRITE	0
#define TTY_SOCLIB_REG_STATUS	4
#define TTY_SOCLIB_REG_READ	8
#define TTY_SOCLIB_REG_SPAN	16

static
void tty_soclib_try_read(struct device_s *dev, struct tty_soclib_tty_s *tty)
{
  struct tty_soclib_context_s	*pv = dev->drv_pv;
  __unused__ uintptr_t addr = pv->addr + TTY_SOCLIB_REG_SPAN * (tty - pv->ttys);
  struct dev_char_rq_s *rq;

  while ((rq = dev_char_rq_s_cast(dev_request_queue_head(&tty->read_q)))) {

#ifdef CONFIG_DEVICE_IRQ
    if (tty_fifo_isempty(&tty->read_fifo))
      return;

    if (rq->type & _DEV_CHAR_POLL)
      goto done;

    size_t size = tty_fifo_pop_array(&tty->read_fifo, rq->data, rq->size);
#else
    /* use polling if no IRQ support available */
    size_t size = 0;
    while (size < rq->size)
      if ((endian_le32(cpu_mem_read_32(addr + TTY_SOCLIB_REG_STATUS)) & 1)
        rq->data[size++] = cpu_mem_read_8(addr + TTY_SOCLIB_REG_READ);
#endif

    if (!size)
      return;

    rq->size -= size;
    rq->data += size;

    if (rq->size == 0 || (rq->type & _DEV_CHAR_PARTIAL)) {
    done:
      rq->error = 0;
      dev_request_queue_pop(&tty->read_q);
      rq->base.drvdata = NULL;
      kroutine_exec(&rq->base.kr);
    }
  }
}

static DEV_CHAR_CANCEL(tty_soclib_cancel)
{
  struct device_s               *dev = accessor->dev;
  struct tty_soclib_context_s	*pv = dev->drv_pv;
  error_t err = -ENOTSUP;
  struct tty_soclib_tty_s *tty = pv->ttys + accessor->number;

  LOCK_SPIN_IRQ(&dev->lock);

  switch (rq->type)
    {
    case DEV_CHAR_READ_PARTIAL:
    case DEV_CHAR_READ:
      err = -EBUSY;
      if (rq->base.drvdata == tty)
        {
          dev_request_queue_remove(&tty->read_q, dev_char_rq_s_base(rq));
          rq->base.drvdata = NULL;
          err = 0;
        }
    default:
      break;
    }

  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
}

static DEV_CHAR_REQUEST(tty_soclib_request)
{
  struct device_s               *dev = accessor->dev;
  struct tty_soclib_context_s	*pv = dev->drv_pv;
  struct dev_char_rq_s          *done_rq = NULL;

  assert(rq->size);

  struct tty_soclib_tty_s *tty = pv->ttys + accessor->number;

  LOCK_SPIN_IRQ(&dev->lock);

  switch (rq->type)
  {
#ifdef CONFIG_DEVICE_IRQ
  case DEV_CHAR_READ_POLL:
    if (rq->size > 1)
      goto notsup;
#endif
  case DEV_CHAR_READ_PARTIAL:
  case DEV_CHAR_READ:
    dev_request_queue_pushback(&tty->read_q, dev_char_rq_s_base(rq));
    rq->base.drvdata = tty;
    tty_soclib_try_read(dev, tty);
    break;

  case DEV_CHAR_WRITE_PARTIAL_FLUSH:
  case DEV_CHAR_WRITE_PARTIAL:
  case DEV_CHAR_WRITE_FLUSH:
  case DEV_CHAR_WRITE: {
    uintptr_t addr = pv->addr + TTY_SOCLIB_REG_SPAN * accessor->number;
    size_t i;

    for (i = 0; i < rq->size; i++)
      cpu_mem_write_32(addr + TTY_SOCLIB_REG_WRITE, endian_le32(rq->data[i]));

    rq->size = 0;
  case DEV_CHAR_WRITE_POLL:
  done:
    rq->error = 0;
    done_rq = rq;
    break;
  default:
  notsup:
    rq->error = -ENOTSUP;
    done_rq = rq;
    break;
  }
  }

  LOCK_RELEASE_IRQ(&dev->lock);

  if (done_rq)
    kroutine_exec(&done_rq->base.kr);
}

#ifdef CONFIG_DEVICE_IRQ

static DEV_IRQ_SRC_PROCESS(tty_soclib_irq)
{
  struct device_s *dev = ep->base.dev;
  struct tty_soclib_context_s *pv = dev->drv_pv;
  uint_fast8_t i;
  uint8_t c;

  lock_spin(&dev->lock);

  for (i = 0; i < pv->count; i++)
    {
      struct tty_soclib_tty_s *tty = pv->ttys + i;
      uintptr_t addr = pv->addr + TTY_SOCLIB_REG_SPAN * i;

      while (endian_le32(cpu_mem_read_32(addr + TTY_SOCLIB_REG_STATUS)) & 1) {

        do {
          /* get character from tty */
          c = cpu_mem_read_8(addr + TTY_SOCLIB_REG_READ);

          /* add character to driver fifo, discard if fifo full */
          tty_fifo_pushback(&tty->read_fifo, c);
        } while ((endian_le32(cpu_mem_read_32(addr + TTY_SOCLIB_REG_STATUS)) & 1)
                 && ! tty_fifo_isfull(&tty->read_fifo));

        tty_soclib_try_read(dev, tty);
      }
    }

  lock_release(&dev->lock);
}

#endif

static DEV_INIT(tty_soclib_init);
static DEV_CLEANUP(tty_soclib_cleanup);

static DEV_USE(tty_soclib_use)
{
  struct device_accessor_s *accessor = param;

  switch (op)
    {
    case DEV_USE_GET_ACCESSOR: {
      struct tty_soclib_context_s *pv = accessor->dev->drv_pv;
      if (accessor->number >= pv->count)
        return -ENOTSUP;
    }
    case DEV_USE_PUT_ACCESSOR:
      return 0;
    case DEV_USE_LAST_NUMBER: {
      struct tty_soclib_context_s *pv = accessor->dev->drv_pv;
      accessor->number = pv->count - 1;
      return 0;
    }
    default:
      return -ENOTSUP;
    }
}

DRIVER_DECLARE(tty_soclib_drv, 0, "Soclib Tty", tty_soclib,
               DRIVER_CHAR_METHODS(tty_soclib));

DRIVER_REGISTER(tty_soclib_drv,
                DEV_ENUM_FDTNAME_ENTRY("soclib:multi_tty"));

static DEV_INIT(tty_soclib_init)
{
  struct tty_soclib_context_s	*pv;
  device_mem_map( dev , 1 << 0 );


  uintptr_t addr;
  if (device_res_get_uint(dev, DEV_RES_MEM, 0, &addr, NULL))
    return -EINVAL;

  uint_fast8_t i, count = endian_le32(cpu_mem_read_32(addr + TTY_SOCLIB_REG_STATUS)) >> 24;
  if (!count)
    count = 1;

  pv = mem_alloc(sizeof(*pv) + count * sizeof(struct tty_soclib_tty_s), (mem_scope_sys));
  if (!pv)
    return -ENOMEM;

  dev->drv_pv = pv;
  pv->addr = addr;
  pv->count = count;

#ifdef CONFIG_DEVICE_IRQ
  device_irq_source_init(dev, &pv->irq_ep, 1,
                         &tty_soclib_irq);

  if (device_irq_source_link(dev, &pv->irq_ep, 1, 1))
    goto err_mem;
#endif

  for (i = 0; i < count; i++)
    {
      struct tty_soclib_tty_s *tty = pv->ttys + i;
      dev_request_queue_init(&tty->read_q);
#ifdef CONFIG_DEVICE_IRQ
      tty_fifo_init(&tty->read_fifo);
#endif
    }


  return 0;

 err_mem:
  mem_free(pv);
  return -1;
}

static DEV_CLEANUP(tty_soclib_cleanup)
{
  struct tty_soclib_context_s	*pv = dev->drv_pv;
  uint_fast8_t i;

  for (i = 0; i < pv->count; i++)
    {
      struct tty_soclib_tty_s *tty = pv->ttys + i;
      if (!dev_request_queue_isempty(&tty->read_q))
        return -EBUSY;
    }

  for (i = 0; i < pv->count; i++)
    {
      struct tty_soclib_tty_s *tty = pv->ttys + i;
#ifdef CONFIG_DEVICE_IRQ
      tty_fifo_destroy(&tty->read_fifo);
#endif
      dev_request_queue_destroy(&tty->read_q);
    }

#ifdef CONFIG_DEVICE_IRQ
  device_irq_source_unlink(dev, &pv->irq_ep, 1);
#endif

  mem_free(pv);

  return 0;
}

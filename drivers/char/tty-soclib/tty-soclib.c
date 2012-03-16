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

#include "tty-soclib.h"

#include "tty-soclib-private.h"

#include <hexo/types.h>
#include <hexo/endian.h>
#include <hexo/iospace.h>
#include <mutek/mem_alloc.h>
#include <hexo/interrupt.h>

#include <device/device.h>
#include <device/char.h>

#define TTY_SOCLIB_REG_WRITE	0
#define TTY_SOCLIB_REG_STATUS	4
#define TTY_SOCLIB_REG_READ	8

void tty_soclib_try_read(struct device_s *dev)
{
  struct tty_soclib_context_s	*pv = dev->drv_pv;
  struct dev_char_rq_s		*rq;

  while ((rq = dev_char_queue_head(&pv->read_q)))
    {
      size_t size;

#ifdef CONFIG_HEXO_IRQ
      size = tty_fifo_pop_array(&pv->read_fifo, rq->data, rq->size);
#else
      /* use polling if no IRQ support available */
      size = 0;
      while (cpu_mem_read_8(pv->addr + TTY_SOCLIB_REG_STATUS) && size < rq->size)
	rq->data[size++] = cpu_mem_read_8(pv->addr + TTY_SOCLIB_REG_READ);
#endif

      if (!size)
	break;

      rq->size -= size;
      rq->error = 0;

      if (rq->callback(rq, size) || rq->size == 0)
	dev_char_queue_remove(&pv->read_q, rq);
      else
	rq->data += size;
    }
}

DEVCHAR_REQUEST(tty_soclib_request)
{
  struct device_s               *dev = cdev->dev;
  struct tty_soclib_context_s	*pv = dev->drv_pv;

  assert(rq->size);
  assert(cdev->number == 0);

  LOCK_SPIN_IRQ(&dev->lock);

  rq->cdev = cdev;

  switch (rq->type)
    {
    case DEV_CHAR_READ: {
      bool_t empty;

      empty = dev_char_queue_isempty(&pv->read_q);
      dev_char_queue_pushback(&pv->read_q, rq);
      if (empty)
	tty_soclib_try_read(dev);
      break;
    }

    case DEV_CHAR_WRITE: {
      size_t i;
      size_t size = rq->size;

      for (i = 0; i < rq->size; i++)
	cpu_mem_write_32(pv->addr + TTY_SOCLIB_REG_WRITE, endian_le32(rq->data[i]));

      rq->size = 0;
      rq->error = 0;
      rq->callback(rq, size);

      break;
    }

    }

  LOCK_RELEASE_IRQ(&dev->lock);
}

/* 
 * device close operation
 */

DEV_CLEANUP(tty_soclib_cleanup)
{
  struct tty_soclib_context_s	*pv = dev->drv_pv;

#ifdef CONFIG_HEXO_IRQ
  if ( dev->icudev )
    DEV_ICU_UNBIND(dev->icudev, dev, dev->irq, tty_soclib_irq);

  tty_fifo_destroy(&pv->read_fifo);
#endif

  dev_char_queue_destroy(&pv->read_q);

  mem_free(pv);
}

#ifdef CONFIG_HEXO_IRQ

/*
 * device irq
 */

DEV_IRQ(tty_soclib_irq)
{
  struct tty_soclib_context_s *pv = dev->drv_pv;
  uint8_t c;

  lock_spin(&dev->lock);

  while ( cpu_mem_read_8(pv->addr + TTY_SOCLIB_REG_STATUS) ) {
	  /* get character from tty */
	  c = cpu_mem_read_8(pv->addr + TTY_SOCLIB_REG_READ);

	  /* add character to driver fifo, discard if fifo full */
	  tty_fifo_pushback(&pv->read_fifo, c);
  }

  tty_soclib_try_read(dev);

  lock_release(&dev->lock);

  return 1;
}

#endif

/* 
 * device open operation
 */

static const struct devenum_ident_s	tty_soclib_ids[] =
{
	DEVENUM_FDTNAME_ENTRY("soclib:tty", 0, 0),
	{ 0 }
};

static const struct driver_char_s	tty_soclib_char_drv =
{
  .class_		= DEVICE_CLASS_CHAR,
  .f_request		= tty_soclib_request,
};

const struct driver_s	tty_soclib_drv =
{
  .desc                 = "SoCLib TTY driver",
  .id_table		= tty_soclib_ids,
  .f_init		= tty_soclib_init,
  .f_cleanup		= tty_soclib_cleanup,
#ifdef CONFIG_HEXO_IRQ
  .f_irq		= tty_soclib_irq,
#endif
  .classes              = { &tty_soclib_char_drv, 0 }
};

REGISTER_DRIVER(tty_soclib_drv);

DEV_INIT(tty_soclib_init)
{
  struct tty_soclib_context_s	*pv;
  device_mem_map( dev , 1 << 0 );

  /* alocate private driver data */
  pv = mem_alloc(sizeof(*pv), (mem_scope_sys));

  if (!pv)
    return -ENOMEM;

  dev_char_queue_init(&pv->read_q);

  if (device_res_get_uint(dev, DEV_RES_MEM, 0, &pv->addr))
    goto err_mem;

#ifdef CONFIG_HEXO_IRQ
  tty_fifo_init(&pv->read_fifo);

  if ( dev->icudev )
	  DEV_ICU_BIND(dev->icudev, dev, dev->irq, tty_soclib_irq);
#endif

  dev->drv = &tty_soclib_drv;
  dev->drv_pv = pv;
  return 0;

 err_mem:
  mem_free(pv);
  return -1;
}


/*
    This file is part of MutekH.

    MutekH is free software; you can redistribute it and/or modify it
    under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    MutekH is distributed in the hope that it will be useful, but
    WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with MutekH; if not, write to the Free Software Foundation,
    Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA

    Copyright Alexandre Becoulet <alexandre.becoulet@lip6.fr> (c) 2006

    Synchronous read and write functions for block device.

*/

#include <hexo/device.h>
#include <hexo/device/block.h>
#include <hexo/driver.h>

#ifdef CONFIG_HEXO_SCHED
# include <hexo/scheduler.h>
# include <hexo/lock.h>
#endif

struct dev_block_wait_rq_s
{
#ifdef CONFIG_HEXO_SCHED
  volatile struct sched_context_s *ctx;
#endif
  volatile bool_t	done;
  size_t		block_size;
  uint8_t		**data;	/* target data */
};

static DEVBLOCK_CALLBACK(dev_block_sync_read)
{
  struct dev_block_wait_rq_s *status = rq->pvdata;
  size_t	i;

  for (i = 0; i < count; i++)
    memcpy(status->data[i], rq->data[i], status->block_size);

  status->data += count;

  if (rq->error || rq->count == 0)
    {
#ifdef CONFIG_HEXO_SCHED
      LOCK_SPIN_IRQ(&dev->lock);

      if (status->ctx != NULL)
	sched_context_start(status->ctx);
#endif
      status->done = 1;
#ifdef CONFIG_HEXO_SCHED
      LOCK_RELEASE_IRQ(&dev->lock);
#endif
    }
}

error_t dev_block_wait_read(struct device_s *dev, struct dev_block_rq_s *rq)
{
  struct dev_block_wait_rq_s status;
  uint8_t **data;

  status.done = 0;
  status.ctx = NULL;
  status.data = data = rq->data;
  status.block_size = dev_block_getparams(dev)->blk_size;
  rq->pvdata = &status;
  rq->callback = dev_block_sync_read;
  rq->error = 0;

  dev_block_read(dev, rq);

#ifdef CONFIG_HEXO_SCHED

  /* ensure callback doesn't occur here */
  CPU_INTERRUPT_SAVESTATE_DISABLE;

  lock_spin(&dev->lock);

  if (!status.done)
    {
      status.ctx = sched_get_current();
      lock_release(&dev->lock);
      sched_context_stop();
    }
  else
    {
      lock_release(&dev->lock);
    }

  CPU_INTERRUPT_RESTORESTATE;

#else

  assert(cpu_interrupt_getstate());

  while (!status.done)
    ;

#endif

  rq->data = data;

  return rq->error;
}



static DEVBLOCK_CALLBACK(dev_block_syncl_read)
{
  struct dev_block_wait_rq_s *status = rq->pvdata;
  size_t	i;

  for (i = 0; i < count; i++)
    memcpy(status->data[i], rq->data[i], status->block_size);

  status->data += count;

  if (rq->error || rq->count == 0)
    status->done = 1;
}

error_t dev_block_lock_read(struct device_s *dev, struct dev_block_rq_s *rq)
{
  struct dev_block_wait_rq_s status;
  uint8_t **data;

  status.done = 0;
  status.ctx = NULL;
  status.data = data = rq->data;
  status.block_size = dev_block_getparams(dev)->blk_size;
  rq->pvdata = &status;
  rq->callback = dev_block_syncl_read;

  dev_block_read(dev, rq);

  assert(cpu_interrupt_getstate());

  while (!status.done)
    ;

  rq->data = data;

  return rq->error;
}



static DEVBLOCK_CALLBACK(dev_block_sync_write)
{
  struct dev_block_wait_rq_s *status = rq->pvdata;

#ifdef CONFIG_HEXO_SCHED
  LOCK_SPIN_IRQ(&dev->lock);

  if (status->ctx != NULL)
    sched_context_start(status->ctx);
#endif
  status->done = 1;
#ifdef CONFIG_HEXO_SCHED
  LOCK_RELEASE_IRQ(&dev->lock);
#endif
}

error_t dev_block_wait_write(struct device_s *dev, struct dev_block_rq_s *rq)
{
  struct dev_block_wait_rq_s status;

  status.done = 0;
  status.ctx = NULL;
  rq->pvdata = &status;
  rq->callback = dev_block_sync_write;
  rq->error = 0;

  dev_block_write(dev, rq);

#ifdef CONFIG_HEXO_SCHED

  /* ensure callback doesn't occur here */
  CPU_INTERRUPT_SAVESTATE_DISABLE;

  lock_spin(&dev->lock);

  if (!status.done)
    {
      status.ctx = sched_get_current();
      lock_release(&dev->lock);
      sched_context_stop();
    }
  else
    {
      lock_release(&dev->lock);
    }

  CPU_INTERRUPT_RESTORESTATE;

#else

  assert(cpu_interrupt_getstate());

  while (!status.done)
    ;

#endif

  return rq->error;
}



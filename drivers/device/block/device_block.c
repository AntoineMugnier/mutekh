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
#include <device/block.h>
#include <device/driver.h>

#ifdef CONFIG_HEXO_SCHED
# include <hexo/scheduler.h>
# include <hexo/lock.h>
#endif

struct dev_block_wait_rq_s
{
#ifdef CONFIG_HEXO_SCHED
  lock_t lock;
  struct sched_context_s *ctx;
#endif
  volatile bool_t done;
};

static DEVBLOCK_CALLBACK(dev_block_sync_read)
{
  struct dev_block_wait_rq_s *status = rq->pvdata;

  if (rq->error || rq->count == 0)
    {
#ifdef CONFIG_HEXO_SCHED
      lock_spin(&status->lock);
      if (status->ctx != NULL)
	sched_context_start(status->ctx);
#endif
      status->done = 1;
#ifdef CONFIG_HEXO_SCHED
      lock_release(&status->lock);
#endif
    }
}

error_t dev_block_wait_read(struct device_s *dev, struct dev_block_rq_s *rq)
{
  struct dev_block_wait_rq_s status;
  uint8_t **data;

#ifdef CONFIG_HEXO_SCHED
  lock_init(&status.lock);
  status.ctx = NULL;
#endif
  status.done = 0;
  rq->pvdata = &status;
  rq->callback = dev_block_sync_read;
  rq->error = 0;

  dev_block_read(dev, rq);

#ifdef CONFIG_HEXO_SCHED

  /* ensure callback doesn't occur here */

  CPU_INTERRUPT_SAVESTATE_DISABLE;
  lock_spin(&status.lock);

  if (!status.done)
    {
      status.ctx = sched_get_current();
      sched_context_stop_unlock(&status.lock);
    }
  else
    lock_release(&status.lock);

  CPU_INTERRUPT_RESTORESTATE;

  lock_destroy(&status.lock);

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

  if (rq->error || rq->count == 0)
    status->done = 1;
}

error_t dev_block_lock_read(struct device_s *dev, struct dev_block_rq_s *rq)
{
  struct dev_block_wait_rq_s status;
  uint8_t **data;

  status.done = 0;
  rq->pvdata = &status;
  rq->callback = dev_block_syncl_read;
  rq->error = 0;

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

  if (rq->error || rq->count == 0)
    {
#ifdef CONFIG_HEXO_SCHED
      lock_spin(&status->lock);
      if (status->ctx != NULL)
	sched_context_start(status->ctx);
#endif
      status->done = 1;
#ifdef CONFIG_HEXO_SCHED
      lock_release(&status->lock);
#endif
    }
}

error_t dev_block_wait_write(struct device_s *dev, struct dev_block_rq_s *rq)
{
  struct dev_block_wait_rq_s status;

#ifdef CONFIG_HEXO_SCHED
  lock_init(&status.lock);
  status.ctx = NULL;
#endif
  status.done = 0;
  rq->pvdata = &status;
  rq->callback = dev_block_sync_write;
  rq->error = 0;

  dev_block_write(dev, rq);

#ifdef CONFIG_HEXO_SCHED

  /* ensure callback doesn't occur here */
  CPU_INTERRUPT_SAVESTATE_DISABLE;
  lock_spin(&status.lock);

  if (!status.done)
    {
      status.ctx = sched_get_current();
      sched_context_stop_unlock(&status.lock);
    }
  else
    lock_release(&status.lock);

  CPU_INTERRUPT_RESTORESTATE;

  lock_destroy(&status.lock);

#else

  assert(cpu_interrupt_getstate());

  while (!status.done)
    ;

#endif

  return rq->error;
}



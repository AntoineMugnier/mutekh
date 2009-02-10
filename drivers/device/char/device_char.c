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
#include <device/char.h>
#include <device/driver.h>

#ifdef CONFIG_MUTEK_SCHEDULER
# include <mutek/scheduler.h>
# include <hexo/lock.h>
#endif


struct dev_char_wait_rq_s
{
#ifdef CONFIG_MUTEK_SCHEDULER
  lock_t lock;
  struct sched_context_s *ctx;
#endif
  volatile bool_t done;
};


static DEVCHAR_CALLBACK(dev_char_syncl_read)
{
  struct dev_char_wait_rq_s *status = rq->pvdata;
  status->done = 1;
  return 1;
}

static ssize_t dev_char_lock_request(struct device_s *dev, uint8_t *data,
				     size_t size, enum dev_char_rq_type_e type)
{
  struct dev_char_rq_s rq;
  struct dev_char_wait_rq_s status;

  status.done = 0;
  rq.type = type;
  rq.pvdata = &status;
  rq.callback = dev_char_syncl_read;
  rq.error = 0;
  rq.data = data;
  rq.size = size;

  dev_char_request(dev, &rq);

  assert(cpu_interrupt_getstate());

  while (!status.done)
    ;

  assert(rq.error >= 0);
  return rq.error ? -rq.error : size - rq.size;
}


#ifdef CONFIG_MUTEK_SCHEDULER
static DEVCHAR_CALLBACK(dev_char_sync_read)
{
  struct dev_char_wait_rq_s *status = rq->pvdata;

  lock_spin(&status->lock);
  if (status->ctx != NULL) {
	  CPU_INTERRUPT_SAVESTATE_DISABLE;
	  sched_context_start(status->ctx);
	  CPU_INTERRUPT_RESTORESTATE;
  }
  status->done = 1;
  lock_release(&status->lock);

  return 1;
}

static ssize_t dev_char_wait_request(struct device_s *dev, uint8_t *data,
				     size_t size, enum dev_char_rq_type_e type)
{
  struct dev_char_rq_s rq;
  struct dev_char_wait_rq_s status;

  lock_init(&status.lock);
  status.ctx = NULL;
  status.done = 0;
  rq.type = type;
  rq.pvdata = &status;
  rq.callback = dev_char_sync_read;
  rq.error = 0;
  rq.data = data;
  rq.size = size;

  dev_char_request(dev, &rq);

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

  assert(rq.error >= 0);
  return rq.error ? -rq.error : size - rq.size;

}
#endif

ssize_t dev_char_wait_read(struct device_s *dev, uint8_t *data, size_t size)
{
#ifdef CONFIG_MUTEK_SCHEDULER
  return dev_char_wait_request(dev, data, size, DEV_CHAR_READ);
#else
  return dev_char_lock_request(dev, data, size, DEV_CHAR_READ);
#endif
}

ssize_t dev_char_spin_read(struct device_s *dev, uint8_t *data, size_t size)
{
  return dev_char_lock_request(dev, data, size, DEV_CHAR_READ);
}

ssize_t dev_char_wait_write(struct device_s *dev, const uint8_t *data, size_t size)
{
#ifdef CONFIG_MUTEK_SCHEDULER
  return dev_char_wait_request(dev, (uint8_t*)data, size, DEV_CHAR_WRITE);
#else
  return dev_char_lock_request(dev, (uint8_t*)data, size, DEV_CHAR_WRITE);
#endif
}

ssize_t dev_char_spin_write(struct device_s *dev, const uint8_t *data, size_t size)
{
  return dev_char_lock_request(dev, (uint8_t*)data, size, DEV_CHAR_WRITE);
}


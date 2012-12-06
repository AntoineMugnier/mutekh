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

    Synchronous read and write functions for block device.

*/

#include <device/device.h>
#include <device/class/dma.h>
#include <device/driver.h>

#ifdef CONFIG_MUTEK_SCHEDULER
# include <mutek/scheduler.h>
# include <hexo/lock.h>
#endif


struct dev_dma_wait_rq_s
{
#ifdef CONFIG_MUTEK_SCHEDULER
  lock_t lock;
  struct sched_context_s *ctx;
#endif
  bool_t done;
};


static DEVDMA_CALLBACK(dev_dma_lock_request_cb)
{
  struct dev_dma_wait_rq_s *status = rq->pvdata;
  status->done = 1;
}

static error_t dev_dma_lock_request(const struct device_dma_s *ddev,
                                    uintptr_t src, uintptr_t dst,
                                    size_t size, uint_fast8_t flags,
#ifdef CONFIG_DEVICE_ADDRESS_SPACES
                                    address_space_id_t src_as,
                                    address_space_id_t dst_as,
#endif
                                    devdma_callback_t *callback)
{
  struct dev_dma_rq_s rq;
  struct dev_dma_wait_rq_s status;

  if (size == 0)
    return 0;

  status.done = 0;
  rq.pvdata = &status;
  rq.callback = callback;
  rq.error = 0;
  rq.flags = flags;
#ifdef CONFIG_DEVICE_ADDRESS_SPACES
  rq.src_as = src_as;
  rq.dst_as = dst_as;
#endif
  rq.src = src;
  rq.dst = dst;
  rq.size = size;

  error_t err = DEVICE_OP(ddev, request, &rq);
  if (err)
    return err;

#ifdef CONFIG_DEVICE_IRQ
  assert(cpu_is_interruptible());
#endif

  while (!status.done)
    order_compiler_mem();

  return rq.error;
}


#ifdef CONFIG_MUTEK_SCHEDULER
static DEVDMA_CALLBACK(dev_dma_wait_request_cb)
{
  struct dev_dma_wait_rq_s *status = rq->pvdata;

  lock_spin(&status->lock);
  if (status->ctx != NULL)
	  sched_context_start(status->ctx);
  status->done = 1;
  lock_release(&status->lock);
}

static error_t dev_dma_wait_request(const struct device_dma_s *ddev,
                                    uintptr_t src, uintptr_t dst,
                                    size_t size, uint_fast8_t flags,
# ifdef CONFIG_DEVICE_ADDRESS_SPACES
                                    address_space_id_t src_as,
                                    address_space_id_t dst_as,
# endif
                                    devdma_callback_t *callback)
{
  struct dev_dma_rq_s rq;
  struct dev_dma_wait_rq_s status;

  if (size == 0)
    return 0;

  lock_init(&status.lock);
  status.ctx = NULL;
  status.done = 0;
  rq.pvdata = &status;
  rq.callback = callback;
  rq.error = 0;
  rq.flags = flags;
# ifdef CONFIG_DEVICE_ADDRESS_SPACES
  rq.src_as = src_as;
  rq.dst_as = dst_as;
# endif
  rq.src = src;
  rq.dst = dst;
  rq.size = size;

  error_t err = DEVICE_OP(ddev, request, &rq);
  if (err)
    return err;

  /* ensure callback doesn't occur here */

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

  return rq.error;
}
#endif


error_t dev_dma_wait_copy(const struct device_dma_s *ddev,
                          uintptr_t src, uintptr_t dst,
                          size_t size, uint_fast8_t flags)
{
#ifdef CONFIG_MUTEK_SCHEDULER
  return dev_dma_wait_request(ddev, src, dst, size, flags,
# ifdef CONFIG_DEVICE_ADDRESS_SPACES
                              0, 0,
# endif
                              dev_dma_wait_request_cb);
#else
  return dev_dma_lock_request(ddev, src, dst, size, flags,
# ifdef CONFIG_DEVICE_ADDRESS_SPACES
                              0, 0,
# endif
                              dev_dma_lock_request_cb);
#endif
}

error_t dev_dma_spin_copy(const struct device_dma_s *ddev,
                          uintptr_t src, uintptr_t dst,
                          size_t size, uint_fast8_t flags)
{
  return dev_dma_lock_request(ddev, src, dst, size, flags,
# ifdef CONFIG_DEVICE_ADDRESS_SPACES
                              0, 0,
# endif
                              dev_dma_lock_request_cb);
}


#ifdef CONFIG_DEVICE_ADDRESS_SPACES
error_t dev_dma_wait_copy_as(const struct device_dma_s *ddev,
                             uintptr_t src, uintptr_t dst,
                             size_t size, uint_fast8_t flags,
                             address_space_id_t src_as,
                             address_space_id_t dst_as)
{
# ifdef CONFIG_MUTEK_SCHEDULER
  return dev_dma_wait_request(ddev, src, dst, size, flags,
                              src_as, dst_as, dev_dma_wait_request_cb);
# else
  return dev_dma_lock_request(ddev, src, dst, size, flags,
                              src_as, dst_as, dev_dma_lock_request_cb);
# endif
}

error_t dev_dma_spin_copy_as(const struct device_dma_s *ddev,
                             uintptr_t src, uintptr_t dst,
                             size_t size, uint_fast8_t flags,
                             address_space_id_t src_as,
                             address_space_id_t dst_as)
{
  return dev_dma_lock_request(ddev, src, dst, size, flags,
                              src_as, dst_as, dev_dma_lock_request_cb);
}

#endif


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

    Synchronous read and write functions for character device.
*/

#include <device/device.h>
#include <device/class/char.h>
#include <device/driver.h>

#ifdef CONFIG_MUTEK_SCHEDULER
# include <mutek/scheduler.h>
# include <hexo/lock.h>
#endif

struct dev_char_helper_s
{
  struct dev_char_rq_s rq;
#ifdef CONFIG_MUTEK_SCHEDULER
  lock_t lock;
  struct sched_context_s *ctx;
#endif
  bool_t done;
};

static KROUTINE_EXEC(dev_char_helper_spin_kr)
{
  struct dev_char_helper_s *helper = KROUTINE_CONTAINER(kr, *helper, rq.kr);

  helper->done = 1;
}

#if defined(CONFIG_MUTEK_SCHEDULER)
static KROUTINE_EXEC(dev_char_helper_wait_kr)
{
  struct dev_char_helper_s *helper = KROUTINE_CONTAINER(kr, *helper, rq.kr);

  lock_spin(&helper->lock);

  helper->done = 1;
  if (helper->ctx != NULL)
    sched_context_start(helper->ctx);

  lock_release(&helper->lock);
}
#endif

static error_t dev_char_spin_request(
  const struct device_char_s *cdev,
  struct dev_char_helper_s *helper)
{
  kroutine_init(&helper->rq.kr, dev_char_helper_spin_kr, KROUTINE_IMMEDIATE);

  helper->done = 0;

  DEVICE_OP(cdev, request, &helper->rq);

  while (!helper->done)
    order_smp_read();

  return helper->rq.error;
}

static error_t dev_char_wait_request(
  const struct device_char_s *cdev,
  struct dev_char_helper_s *helper)
{
#if !defined(CONFIG_MUTEK_SCHEDULER)
  return dev_i2c_spin_transfer(i2cdev, helper);
#else
  kroutine_init(&helper->rq.kr, dev_char_helper_wait_kr, KROUTINE_IMMEDIATE);

  lock_init(&helper->lock);
  helper->ctx  = NULL;
  helper->done = 0;

  DEVICE_OP(cdev, request, &helper->rq);

  CPU_INTERRUPT_SAVESTATE_DISABLE;
  lock_spin(&helper->lock);

  if (!helper->done) {
    helper->ctx = sched_get_current();
    sched_stop_unlock(&helper->lock);
  } else {
    lock_release(&helper->lock);
  }

  CPU_INTERRUPT_RESTORESTATE;
  lock_destroy(&helper->lock);

  return helper->rq.error;
#endif
}

error_t dev_char_wait_read(const struct device_char_s *cdev, uint8_t *data, size_t size)
{
#if !defined(CONFIG_MUTEK_SCHEDULER)
  return dev_char_spin_read(cdev, data, size);
#else
  struct dev_char_helper_s req =
  {
    .rq = {
      .type = DEV_CHAR_READ,
      .data = data,
      .size = size,
    },
  };

  return dev_char_wait_request(cdev, &req);
#endif
}

error_t dev_char_spin_read(const struct device_char_s *cdev, uint8_t *data, size_t size)
{
  struct dev_char_helper_s req =
  {
    .rq = {
      .type = DEV_CHAR_READ,
      .data = data,
      .size = size,
    },
  };

  return dev_char_spin_request(cdev, &req);
}

error_t dev_char_wait_write(const struct device_char_s *cdev, const uint8_t *data, size_t size)
{
#if !defined(CONFIG_MUTEK_SCHEDULER)
  return dev_char_spin_write(cdev, data, size);
#else
  struct dev_char_helper_s req =
  {
    .rq = {
      .type = DEV_CHAR_WRITE,
      .data = (uint8_t *)data,
      .size = size,
    },
  };

  return dev_char_wait_request(cdev, &req);
#endif
}

error_t dev_char_spin_write(const struct device_char_s *cdev, const uint8_t *data, size_t size)
{
  struct dev_char_helper_s req =
  {
    .rq = {
      .type = DEV_CHAR_WRITE,
      .data = (uint8_t *)data,
      .size = size,
    },
  };

  return dev_char_spin_request(cdev, &req);
}

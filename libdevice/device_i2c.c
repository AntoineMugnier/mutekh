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

    Copyright Nicolas Pouillon <nipo@ssji.net> (c) 2009
    Copyright Julien Peeters <contact@julienpeeters.net> (c) 2014

    Synchronous read and write functions for i2c devices.

*/

#include <device/device.h>
#include <device/driver.h>
#include <device/class/i2c.h>

#if defined(CONFIG_MUTEK_SCHEDULER)
# include <mutek/scheduler.h>
# include <hexo/lock.h>
#endif

struct dev_i2c_ctrl_wait_rq_s
{
#if defined(CONFIG_MUTEK_SCHEDULER)
  lock_t                 lock;
  struct sched_context_s *ctx;
#endif
  bool_t                 done;
};

KROUTINE_EXEC(dev_i2c_ctrl_lock_transfer_end)
{
  struct dev_i2c_ctrl_transfer_s *tr     = KROUTINE_CONTAINER(kr, *tr, kr);
  struct dev_i2c_ctrl_wait_rq_s  *status = tr->pvdata;

  status->done = 1;
}

KROUTINE_EXEC(dev_i2c_ctrl_lock_transfer_whole_end)
{
  struct dev_i2c_ctrl_transfer_s *tr     = KROUTINE_CONTAINER(kr, *tr, kr);
  struct dev_i2c_ctrl_wait_rq_s  *status = tr->pvdata;

  if ( tr->count == 0 || tr->error )
    status->done = 1;
}

static ssize_t dev_i2c_lock_transfer(const struct device_i2c_ctrl_s *i2cdev,
                                     struct dev_i2c_ctrl_transfer_s *tr,
                                     kroutine_exec_t                *kr)
{
  struct dev_i2c_ctrl_wait_rq_s status;
  size_t                        size;

  /* setup the kroutine. */
  kroutine_init(&tr->kr, kr, KROUTINE_IMMEDIATE);
  tr->pvdata = &status;

  /* clear error. */
  tr->error = 0;

  /* setup the associated device. */
  tr->i2cdev = (struct device_i2c_ctrl_s *)i2cdev;

  status.done = 0;

  /* save requested byte count. */
  size = tr->count;

  /* launch the transfer and jump in the driver. */
  DEVICE_OP(i2cdev, transfer, tr);

#if defined(CONFIG_DEVICE_IRQ)
  assert(cpu_is_interruptible());
#endif

  while (!status.done)
    order_compiler_mem();

  assert (tr->error >= 0);
  return tr->error ? -tr->error : size - tr->count;
}

#if defined(CONFIG_MUTEK_SCHEDULER)

KROUTINE_EXEC(dev_i2c_ctrl_wait_transfer_end)
{
  struct dev_i2c_ctrl_transfer_s *tr     = KROUTINE_CONTAINER(kr, *tr, kr);
  struct dev_i2c_ctrl_wait_rq_s  *status = tr->pvdata;

  lock_spin(&status->lock);
  if (status->ctx != NULL)
    sched_context_start(status->ctx);
  status->done = 1;
  lock_release(&status->lock);
}

KROUTINE_EXEC(dev_i2c_ctrl_wait_transfer_whole_end)
{
  struct dev_i2c_ctrl_transfer_s *tr     = KROUTINE_CONTAINER(kr, *tr, kr);
  struct dev_i2c_ctrl_wait_rq_s  *status = tr->pvdata;

  if ( tr->count == 0 || tr->error )
    {
      lock_spin(&status->lock);
      if (status->ctx != NULL)
        sched_context_start(status->ctx);
      status->done = 1;
      lock_release(&status->lock);
    }
}

static ssize_t dev_i2c_wait_transfer(const struct device_i2c_ctrl_s *i2cdev,
                                     struct dev_i2c_ctrl_transfer_s *tr,
                                     kroutine_exec_t                *kr)
{
  struct dev_i2c_ctrl_wait_rq_s status;
  size_t                        size;

  /* setup the kroutine. */
  kroutine_init(&tr->kr, kr, KROUTINE_IMMEDIATE);
  tr->pvdata = &status;

  /* clear error. */
  tr->error  = 0;

  /* setup associated device. */
  tr->i2cdev = (struct device_i2c_ctrl_s *)i2cdev;

  lock_init(&status.lock);
  status.ctx = NULL;
  status.done = 0;

  /* save requested byte count. */
  size = tr->count;

  /* launch the transfer and jump in the driver. */
  DEVICE_OP(i2cdev, transfer, tr);

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

  assert (tr->error >= 0);
  return tr->error ? -tr->error : size - tr->count;
}

#endif

error_t dev_i2c_set_bit_rate(struct device_i2c_ctrl_s *i2cdev,
                             uint32_t                 bps)
{
  struct dev_i2c_ctrl_config_s cfg = { .bit_rate = bps };
  return DEVICE_OP(i2cdev, config, &cfg);
}

error_t dev_i2c_wait_scan(const struct device_i2c_ctrl_s    *i2cdev,
                          enum dev_i2c_ctrl_transfer_addr_e amode,
                          uint16_t                          saddr)
{
  return dev_i2c_wait_write(i2cdev, amode, saddr, 0x0, NULL, 0);
}

error_t dev_i2c_spin_scan(const struct device_i2c_ctrl_s    *i2cdev,
                          enum dev_i2c_ctrl_transfer_addr_e amode,
                          uint16_t                          saddr)
{
  return dev_i2c_spin_write(i2cdev, amode, saddr, 0x0, NULL, 0);
}

ssize_t dev_i2c_wait_read(const struct device_i2c_ctrl_s    *i2cdev,
                          enum dev_i2c_ctrl_transfer_addr_e amode,
                          uint16_t                          saddr,
                          uint8_t                           sraddr,
                          uint8_t                           *data,
                          size_t                            size)
{
  /* prepare the I2C transfer. */
  struct dev_i2c_ctrl_transfer_s tr =
  {
    .amode  = amode,
    .saddr  = saddr,
    .sraddr = sraddr,
    .dir    = DEV_I2C_TR_READ,
    .count  = size,
    .data   = data,
  };

#if defined(CONFIG_MUTEK_SCHEDULER)
  return dev_i2c_wait_transfer(
    i2cdev,                         /* device. */
    &tr,                            /* transfer. */
    &dev_i2c_ctrl_wait_transfer_end /* kroutine. */
  );
#else
  return dev_i2c_lock_transfer(
    i2cdev,                         /* device. */
    &tr,                            /* transfer. */
    &dev_i2c_ctrl_lock_transfer_end /* kroutine. */
  );
#endif
}

ssize_t dev_i2c_spin_read(const struct device_i2c_ctrl_s    *i2cdev,
                          enum dev_i2c_ctrl_transfer_addr_e amode,
                          uint16_t                          saddr,
                          uint8_t                           sraddr,
                          uint8_t                           *data,
                          size_t                            size)
{
  /* prepare the I2C transfer. */
  struct dev_i2c_ctrl_transfer_s tr =
  {
    .amode  = amode,
    .saddr  = saddr,
    .sraddr = sraddr,
    .dir    = DEV_I2C_TR_READ,
    .count  = size,
    .data   = data,
  };

  return dev_i2c_lock_transfer(
    i2cdev,                         /* device. */
    &tr,                            /* transfer. */
    &dev_i2c_ctrl_lock_transfer_end /* kroutine. */
  );
}

ssize_t dev_i2c_wait_write(const struct device_i2c_ctrl_s    *i2cdev,
                           enum dev_i2c_ctrl_transfer_addr_e amode,
                           uint16_t                          saddr,
                           uint8_t                           sraddr,
                           const uint8_t                     *data,
                           size_t                            size)
{
  /* prepare the I2C transfer. */
  struct dev_i2c_ctrl_transfer_s tr =
  {
    .amode  = amode,
    .saddr  = saddr,
    .sraddr = sraddr,
    .dir    = DEV_I2C_TR_WRITE,
    .count  = size,
    .data   = (uint8_t*)data,
  };

#if defined(CONFIG_MUTEK_SCHEDULER)
  return dev_i2c_wait_transfer(
    i2cdev,                                 /* device. */
    &tr,                                    /* transfer. */
    &dev_i2c_ctrl_wait_transfer_whole_end   /* kroutine. */
  );
#else
  return dev_i2c_lock_transfer(
    i2cdev,                                 /* device. */
    &tr,                                    /* transfer. */
    &dev_i2c_ctrl_lock_transfer_whole_end   /* kroutine. */
  );
#endif
}

ssize_t dev_i2c_spin_write(const struct device_i2c_ctrl_s    *i2cdev,
                           enum dev_i2c_ctrl_transfer_addr_e amode,
                           uint16_t                          saddr,
                           uint8_t                           sraddr,
                           const uint8_t                     *data,
                           size_t                            size)
{
  /* prepare the I2C transfer. */
  struct dev_i2c_ctrl_transfer_s tr =
  {
    .amode  = amode,
    .saddr  = saddr,
    .sraddr = sraddr,
    .dir    = DEV_I2C_TR_WRITE,
    .count  = size,
    .data   = (uint8_t*)data,
  };

  return dev_i2c_lock_transfer(
    i2cdev,                                 /* device. */
    &tr,                                    /* transfer. */
    &dev_i2c_ctrl_lock_transfer_whole_end   /* kroutine. */
  );
}


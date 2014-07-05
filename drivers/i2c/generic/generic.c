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

    Copyright Julien Peeters <contact@julienpeeters.net> (c) 2014

*/

#include <hexo/types.h>
#include <hexo/error.h>
#include <hexo/iospace.h>
#include <hexo/interrupt.h>

#include <mutek/mem_alloc.h>
#include <mutek///printk.h>

#include <device/device.h>
#include <device/driver.h>
#include <device/class/i2c.h>

#include <mutek/kroutine.h>

#include <pthread.h>

struct i2c_generic_private_s
{
  /** Master accessor. */
  struct device_i2c_ctrl_s      i2cdev;

  /** Slave address. */
  uint_fast16_t                 saddr;

  /** I2C request. */
  struct dev_i2c_ctrl_request_s rq;
};

static
KROUTINE_EXEC(i2c_generic_slave_request_end)
{
  struct dev_i2c_ctrl_request_s *rq    = KROUTINE_CONTAINER(kr, *rq, kr);
  struct dev_i2c_dev_request_s  *devrq = rq->pvdata;

  devrq->error = rq->error;
  devrq->size  = rq->error == 0 ? 0 : devrq->size;
  kroutine_exec(&devrq->kr, cpu_is_interruptible());
}

static
DEVI2C_DEV_REQUEST(i2c_generic_dev_request)
{
  struct device_s              *dev = sdev->dev;
  struct i2c_generic_private_s *pv  = dev->drv_pv;
  error_t                      err  = 0;

  LOCK_SPIN_IRQ(&dev->lock);

  /* if the request for this device is already enqueued, this means that
     another request is pending and this one cannot be serviced. */
  if (pv->rq.enqueued)
    {
      // FIXME: the enqueued is locked on scheduler lock, not on request.
      err = EBUSY;
      goto err_enqueued;
    }

  pv->rq.type         = DEV_I2C_REQ_INFO;
  pv->rq.u.info.saddr = pv->saddr;
  pv->rq.u.info.devrq = devrq;

  kroutine_init(&pv->rq.kr, i2c_generic_slave_request_end, KROUTINE_IMMEDIATE);
  pv->rq.pvdata = devrq;

  dev_i2c_request_sched(&pv->rq);

err_enqueued:
  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
}

static const struct driver_i2c_dev_s i2c_generic_i2c_dev_cls =
{
  .class_    = DRIVER_CLASS_I2C_DEV,
  .f_request = &i2c_generic_dev_request,
};

static DEV_INIT(i2c_generic_init);
static DEV_CLEANUP(i2c_generic_cleanup);

const struct driver_s i2c_generic_dev_drv =
{
  .desc      = "I2C generic device",
  .f_init    = &i2c_generic_init,
  .f_cleanup = &i2c_generic_cleanup,
  .classes   = {
    &i2c_generic_i2c_dev_cls,
    0
  }
};

REGISTER_DRIVER(i2c_generic_dev_drv);

static
DEV_INIT(i2c_generic_init)
{
  struct i2c_generic_private_s *pv;

  dev->status = DEVICE_DRIVER_INIT_FAILED;

  pv = mem_alloc(sizeof(*pv), (mem_scope_sys));
  if (!pv)
    return -ENOMEM;

  if (device_get_param_dev_accessor(
        dev, "i2c", &pv->i2cdev, DRIVER_CLASS_I2C_CTRL))
    goto err_mem;

  if (device_get_param_uint(dev, "i2c-addr", &pv->saddr))
    goto err_accessor;

  if (dev_i2c_request_init(dev, &pv->rq))
    goto err_accessor;

  dev->drv_pv = pv;
  dev->status = DEVICE_DRIVER_INIT_DONE;

  return 0;

err_accessor:
  device_put_accessor(&pv->i2cdev);

err_mem:
  mem_free(pv);
  return -ENOENT;
}

static
DEV_CLEANUP(i2c_generic_cleanup)
{
  struct i2c_generic_private_s *pv = dev->drv_pv;

  device_put_accessor(&pv->i2cdev);
  mem_free(pv);
}


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

    Copyright Nicolas Pouillon <nipo@ssji.net> (c) 2009, 2014
    Copyright Julien Peeters <contact@julienpeeters.net> (c) 2014

    Synchronous read and write functions for i2c devices.
*/

#include <device/device.h>
#include <device/driver.h>
#include <device/resources.h>
#include <device/class/i2c.h>

#include <hexo/lock.h>
#include <hexo/interrupt.h>

#if defined(CONFIG_MUTEK_SCHEDULER)
# include <mutek/scheduler.h>
#endif

#if defined(CONFIG_DEVICE_I2C_REQUEST)
GCT_CONTAINER_PROTOTYPES(dev_i2c_ctrl_queue, extern inline, dev_i2c_ctrl_queue,
                   init, destroy, pop, remove, push, push_back, isempty);
#endif

/* structure that is used for blocking calls. */
struct dev_i2c_helper_rq_s
{
  struct dev_i2c_request_s request;
  bool_t done;

#if defined(CONFIG_MUTEK_SCHEDULER)
  lock_t lock;
  struct sched_context_s *ctx;
#endif
};

#if defined(CONFIG_MUTEK_SCHEDULER)
static
KROUTINE_EXEC(dev_i2c_helper_wait_kr)
{
  struct dev_i2c_helper_rq_s *rq = KROUTINE_CONTAINER(kr, *rq, request.kr);

  lock_spin(&rq->lock);

  rq->done = 1;
  if (rq->ctx != NULL)
    sched_context_start(rq->ctx);

  lock_release(&rq->lock);
}
#endif

static
KROUTINE_EXEC(dev_i2c_helper_spin_kr)
{
  struct dev_i2c_helper_rq_s *rq = KROUTINE_CONTAINER(kr, *rq, request.kr);
  rq->done = 1;
}

static ssize_t dev_i2c_wait_request(
  const struct device_i2c_s *i2cdev,
  struct dev_i2c_helper_rq_s *req)
{
#if !defined(CONFIG_MUTEK_SCHEDULER)
  return dev_i2c_spin_request(i2cdev, req);
#else
  kroutine_init(&req->request.kr, dev_i2c_helper_wait_kr, KROUTINE_IMMEDIATE);

  lock_init(&req->lock);
  req->ctx  = NULL;
  req->done = 0;

  DEVICE_OP(i2cdev, request, &req->request);

  CPU_INTERRUPT_SAVESTATE_DISABLE;
  lock_spin(&req->lock);

  if (!req->done) {
    req->ctx = sched_get_current();
    sched_stop_unlock(&req->lock);
  } else {
    lock_release(&req->lock);
  }

  CPU_INTERRUPT_RESTORESTATE;
  lock_destroy(&req->lock);

  return req->request.error;
#endif
}

static ssize_t dev_i2c_spin_request(
  const struct device_i2c_s *i2cdev,
  struct dev_i2c_helper_rq_s *req)
{
  kroutine_init(&req->request.kr, dev_i2c_helper_spin_kr, KROUTINE_IMMEDIATE);

  req->done = 0;

  DEVICE_OP(i2cdev, request, &req->request);

  while (!req->done)
    order_smp_read();

  return req->request.error;
}

ssize_t dev_i2c_wait_write_read(
  const struct device_i2c_s *i2cdev,
  uint8_t saddr,
  const uint8_t *wdata,
  size_t wsize,
  uint8_t *rdata,
  size_t rsize)
{
#if !defined(CONFIG_MUTEK_SCHEDULER)
  return dev_i2c_spin_read_write(i2cdev, saddr, wdata, wsize, rdata, rsize);
#endif

  struct dev_i2c_helper_rq_s req =
  {
    .request = {
      .saddr = saddr,
      .wdata = wdata,
      .wdata_len = wsize,
      .rdata = rdata,
      .rdata_len = rsize,
    },
  };

  return dev_i2c_wait_request(i2cdev, &req);
}

ssize_t dev_i2c_spin_write_read(
  const struct device_i2c_s *i2cdev,
  uint8_t saddr,
  const uint8_t *wdata,
  size_t wsize,
  uint8_t *rdata,
  size_t rsize)
{
  struct dev_i2c_helper_rq_s req =
  {
    .request = {
      .saddr = saddr,
      .wdata = wdata,
      .wdata_len = wsize,
      .rdata = rdata,
      .rdata_len = rsize,
    },
  };

  return dev_i2c_spin_request(i2cdev, &req);
}

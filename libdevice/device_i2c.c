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

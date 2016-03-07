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

    Copyright Alexandre Becoulet <alexandre.becoulet@free.fr> (c) 2016

*/

#include <device/device.h>
#include <device/driver.h>

#include <mutek/startup.h>
#include <mutek/kroutine.h>

GCT_CONTAINER_KEY_FCNS(device_sleep, DESC, static inline, device_sleep_queue, sleep_order,
                       init, destroy, insert, pop);

static device_sleep_root_t device_sleep_cpuidle_q;
static struct kroutine_s device_sleep_cpuidle_kr;

static void device_sleep_process(device_sleep_root_t *queue)
{
  struct device_s *dev;

  while ((dev = device_sleep_queue_pop(queue)))
    {
      const struct driver_s *drv = dev->drv;
      drv->f_use(dev, DEV_USE_SLEEP);
    };
}

static KROUTINE_EXEC(device_sleep_cpuidle_process)
{
  device_sleep_process(&device_sleep_cpuidle_q);
}

void device_sleep_schedule(struct device_s *dev)
{
  if (!device_sleep_queue_isorphan(dev))
    return;

  switch (dev->sleep_policy)
    {
    case DEVICE_SLEEP_CPUIDLE:
      device_sleep_queue_insert(&device_sleep_cpuidle_q, dev);
      kroutine_exec(&device_sleep_cpuidle_kr);
      break;
    default:
      UNREACHABLE();
    }
}

void device_sleep_init(void)
{
  kroutine_init_idle(&device_sleep_cpuidle_kr, device_sleep_cpuidle_process);
  device_sleep_queue_init(&device_sleep_cpuidle_q);
}


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

    Copyright Institut Telecom / Telecom ParisTech (c) 2011
    Copyright Alexandre Becoulet <alexandre.becoulet@lip6.fr> (c) 2011
*/

#include <mutek/printk.h>
#include <mutek/startup.h>

#include <device/class/timer.h>
#include <device/device.h>
#include <device/driver.h>

struct device_timer_s    libnetwork_timer_dev = DEVICE_ACCESSOR_INIT;

void libnetwork_initsmp()
{
  if (!cpu_isbootstrap())
    return;

  if (device_get_accessor_by_path(&libnetwork_timer_dev, NULL, "network_timer timer", DRIVER_CLASS_TIMER))
    {
      printk("error: network: No `network_timer' or `timer' entry found in device tree.\n");
    }
  else
    {
      DEVICE_OP(&libnetwork_timer_dev, start_stop, 1);
      printk("network: using timer device `%p' for network timing\n", libnetwork_timer_dev.dev);
    }
}

void libnetwork_cleanupsmp()
{
  if (device_check_accessor(&libnetwork_timer_dev))
    {
      DEVICE_OP(&libnetwork_timer_dev, start_stop, 0);
      device_put_accessor(&libnetwork_timer_dev);
    }
}


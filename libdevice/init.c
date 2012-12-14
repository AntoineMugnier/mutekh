/*
   This file is part of MutekH.
   
   MutekH is free software; you can redistribute it and/or modify it
   under the terms of the GNU Lesser General Public License as published
   by the Free Software Foundation; version 2.1 of the License.
   
   MutekH is distributed in the hope that it will be useful, but WITHOUT
   ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
   FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public
   License for more details.
   
   You should have received a copy of the GNU Lesser General Public
   License along with MutekH; if not, write to the Free Software
   Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
   02110-1301 USA.

   Copyright Alexandre Becoulet <alexandre.becoulet@lip6.fr> (c) 2006

*/

#include <mutek/startup.h>
#include <mutek/printk.h>

#include <device/device.h>
#include <device/driver.h>

void libdevice_drivers_init()
{
  device_find_driver(NULL);
}


#include <device/class/cpu.h>

void libdevice_cpu_regs_initsmp()
{
  if (cpu_isbootstrap())
    printk("device: initialization of processor(s): ");

  mutekh_startup_smp_barrier();

  uint_fast8_t id = cpu_id();

  /* find processor device */
  struct device_s *dev = device_get_cpu(id, 0);

  if (!dev)
    {
      printk("\nerror: Can not find CPU %i in device tree.\n", id);
      abort();
    }

  struct device_cpu_s cpu_dev;

  if (device_get_accessor(&cpu_dev, dev, DRIVER_CLASS_CPU, 0))
    {
      printk("\nerror: Unable to use driver of CPU %i (device %p).\n", id, dev);
      abort();
    }

  DEVICE_OP(&cpu_dev, reg_init);

  device_put_accessor(&cpu_dev);

  printk("%u ", id);

  mutekh_startup_smp_barrier();

  if (cpu_isbootstrap())
    printk("\n");
}


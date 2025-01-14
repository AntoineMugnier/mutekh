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

#include <assert.h>
#include <stdlib.h>

#include <device/class/cpu.h>

void libdevice_cpu_regs_initsmp()
{
  if (cpu_isbootstrap())
    logk_debug("cpu reg init...");

  mutekh_startup_smp_barrier();

  cpu_id_t id = cpu_id();
  const struct cpu_tree_s *cpu = cpu_tree_lookup(id);
  assert(cpu != NULL && "processor id not found in the cpu tree.");

  struct device_cpu_s cpu_dev;

  /* get cpu API in processor device driver */
  if (device_get_accessor(&cpu_dev.base, cpu->cpu_dev, DRIVER_CLASS_CPU, 0))
    {
      logk_error("Unable to use driver to initialize CPU %i (device %p)", id, cpu->cpu_dev);
      abort();
    }

  /* perform initialization of processor registers */
  DEVICE_OP(&cpu_dev, reg_init);

  device_put_accessor(&cpu_dev.base);

  logk_debug(" - cpu %u done", id);

  mutekh_startup_smp_barrier();
}


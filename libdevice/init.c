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

void libdevice_drivers_init()
{
#ifdef CONFIG_DEVICE_TREE
  device_find_driver(NULL);
#else
  bool_t done;
  struct device_s *dev;

  do {
    done = 1;
    DEVICE_NODE_FOREACH(, node, {
        if (node->flags & DEVICE_FLAG_IGNORE)
          continue;
        if (!(dev = device_from_node(node)))
          continue;

        if (dev->status == DEVICE_DRIVER_INIT_PENDING &&
            device_init_driver(dev) != -EAGAIN)
          done = 0;
    });
  } while (!done);
#endif
}


#include <device/class/cpu.h>

void libdevice_cpu_regs_initsmp()
{
  if (cpu_isbootstrap())
    printk("device: initialization of processor(s): ");

  mutekh_startup_smp_barrier();

  cpu_id_t id = cpu_id();
  const struct cpu_tree_s *cpu = cpu_tree_lookup(id);
  assert(cpu != NULL && "processor id not found in the cpu tree.");

  struct device_cpu_s cpu_dev;

  /* get cpu API in processor device driver */
  if (device_get_accessor(&cpu_dev, cpu->cpu_dev, DRIVER_CLASS_CPU, 0))
    {
      printk("\nerror: Unable to use driver to initialize CPU %i (device %p).\n", id, cpu->cpu_dev);
      abort();
    }

  /* perform initialization of processor registers */
  DEVICE_OP(&cpu_dev, reg_init);

  device_put_accessor(&cpu_dev);

  printk(" %u ", id);

  mutekh_startup_smp_barrier();

  if (cpu_isbootstrap())
    printk("\n");
}


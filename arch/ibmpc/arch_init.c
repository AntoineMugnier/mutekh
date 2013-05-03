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

    Copyright Alexandre Becoulet <alexandre.becoulet@free.fr> (c) 2013

*/

#include <mutek/startup.h>
#include <hexo/endian.h>

static void *
mem_ibmpc_memsize_probe(void *start)
{
  volatile uint8_t	*x = ALIGN_ADDRESS_UP(start, 4096);
  size_t		step = 4096;

  while (1) {
    x += step;
    *x = 0x5a;
    *x = ~*x;

    if (*x == 0xa5)
      continue;

    x -= step;

    if (step == 1)
      break;

    step /= 2;
  }

  return (void*)x;
}

#include <mutek/mem_alloc.h>
#include <mutek/mem_region.h>
#include <mutek/memory_allocator.h>

void ibmpc_mem_init()
{
  default_region = memory_allocator_init(NULL, (void*)CONFIG_STARTUP_HEAP_ADDR,
                                         (void*)(CONFIG_STARTUP_HEAP_ADDR +
                                                 CONFIG_STARTUP_HEAP_SIZE));
}

/////////////////////////////////////////////////////////////////////

#include <device/device.h>
#include <device/driver.h>
#include <hexo/cpu.h>

void ibmpc_hw_enum_init()
{
  static struct device_s cpu_dev;

  device_init(&cpu_dev);
  cpu_dev.node.flags |= DEVICE_FLAG_CPU;
  device_res_add_id(&cpu_dev, 0, 0);
  device_attach(&cpu_dev, NULL);

  extern const struct driver_s x86_drv;

  device_bind_driver(&cpu_dev, &x86_drv);
  device_init_driver(&cpu_dev);
}

/////////////////////////////////////////////////////////////////////

#if defined(CONFIG_DEVICE_IRQ) && defined(CONFIG_ARCH_SMP)
bool_t arch_cpu_irq_affinity_test(struct device_s *cpu, struct dev_irq_ep_s *src)
{
    return 1;
}
#endif

/////////////////////////////////////////////////////////////////////

void ibmpc_start_cpus()
{
}


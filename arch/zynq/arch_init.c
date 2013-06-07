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
    
    Copyright Jeremie Brunel <jeremie.brunel@telecom-paristech.fr> (c) 2013
    Copyright Alexandre Becoulet <alexandre.becoulet@telecom-paristech.fr> (c) 2013

*/

#include <mutek/startup.h>

/////////////////////////////////////////////////////////////////////

#include <mutek/mem_alloc.h>
#include <mutek/memory_allocator.h>

void zynq_mem_init()
{
  default_region = memory_allocator_init(NULL, (void*)CONFIG_STARTUP_HEAP_ADDR,
                                         (void*)(CONFIG_STARTUP_HEAP_ADDR +
                                                 CONFIG_STARTUP_HEAP_SIZE));
}

/////////////////////////////////////////////////////////////////////

# include <device/driver.h>
# include <device/device.h>
# include <device/class/cpu.h>

void zynq_hw_enum_init()
{
  static struct device_s mpcore_dev;

  device_init(&mpcore_dev);
  device_res_add_mem(&mpcore_dev, 0xf8f00000, 0xf8f02000);
  device_attach(&mpcore_dev, NULL);

  extern const struct driver_s a9mpcore_drv;

  device_bind_driver(&mpcore_dev, &a9mpcore_drv);

#ifdef CONFIG_DRIVER_CHAR_CADENCE_UART
  extern const struct driver_s cadence_uart_drv;

  static struct device_s uart0_dev;
  device_init(&uart0_dev);
  uart0_dev.node.name = "uart0";
  device_res_add_mem(&uart0_dev, 0xe0000000, 0xe0001000);
  device_attach(&uart0_dev, NULL);

  device_bind_driver(&uart0_dev, &cadence_uart_drv);

  static struct device_s uart1_dev;
  device_init(&uart1_dev);
  uart1_dev.node.name = "uart1";
  device_res_add_mem(&uart1_dev, 0xe0001000, 0xe0002000);
  device_attach(&uart1_dev, NULL);

  device_bind_driver(&uart1_dev, &cadence_uart_drv);
#endif

}


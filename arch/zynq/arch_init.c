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
#include <device/resources.h>
# include <device/class/cpu.h>

DEV_DECLARE_STATIC(mpcore_dev, "mpcore0", 0, a9mpcore_drv,
                   DEV_STATIC_RES_MEM(0xf8f00000, 0xf8f02000)
                   );


#ifdef CONFIG_DRIVER_CHAR_CADENCE_UART

DEV_DECLARE_STATIC(uart0_dev, "uart0", 0, cadence_uart_drv,
                   DEV_STATIC_RES_MEM(0xe0000000, 0xe0001000),
                   DEV_STATIC_RES_IRQ(0, 59, 0, "/mpcore0/icu")
                   );


DEV_DECLARE_STATIC(uart1_dev, "uart1", 0, cadence_uart_drv,
                   DEV_STATIC_RES_MEM(0xe0001000, 0xe0002000),
                   DEV_STATIC_RES_IRQ(0, 82, 0, "/mpcore0/icu")
                   );

#endif


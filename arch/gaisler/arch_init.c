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

   Copyright (c) 2011 Alexandre Becoulet <alexandre.becoulet@telecom-paristech.fr>
   Copyright (c) 2011 Institut Telecom / Telecom ParisTech

*/

#include <mutek/startup.h>

#include <string.h>

#include <hexo/lock.h>

#ifndef CONFIG_CPU_SPARC_LEON3_CASA
lock_t __atomic_arch_lock;

void gaisler_arch_lock_init()
{
    lock_init(&__atomic_arch_lock);
}
#endif


/////////////////////////////////////////////////////////////////////

#ifdef CONFIG_GAISLER_AHB_ENUM

# include <device/driver.h>
# include <device/resources.h>
# include <device/device.h>

DEV_DECLARE_STATIC(ahbctrl_dev, "ahbctrl", 0, ahbctrl_drv,
                   DEV_STATIC_RES_MEM(CONFIG_GAISLER_AHB_ENUM_ADDR,
                                      CONFIG_GAISLER_AHB_ENUM_ADDR + 0xe00)
                   );

#endif


/////////////////////////////////////////////////////////////////////


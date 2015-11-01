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

    Copyright (c) 2009, Nicolas Pouillon <nipo@ssji.net>
*/

#include <hexo/types.h>

#include <device/class/enum.h>
#include <device/device.h>
#include <device/driver.h>

static
DEV_ENUM_MATCH_DRIVER(device_enum_root_match_driver)
{
  return 0;
}

static DEV_INIT(device_enum_root_init);

static DEV_CLEANUP(device_enum_root_cleanup)
{
  return 0;
}

#define device_enum_root_use dev_use_generic

DRIVER_DECLARE(device_enum_root_drv, DRIVER_FLAGS_EARLY_INIT,
               "MutekH root enumerator", device_enum_root,
               DRIVER_ENUM_METHODS(device_enum_root));

static DEV_INIT(device_enum_root_init)
{
  dev->drv = &device_enum_root_drv;
  dev->status = DEVICE_DRIVER_INIT_DONE;

  return 0;
}

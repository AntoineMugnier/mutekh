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

DEV_ENUM_MATCH_DRIVER(device_enum_root_match_driver)
{
	return 0;
}

static DEV_INIT(device_enum_root_init);

static DEV_CLEANUP(device_enum_root_cleanup)
{
}

static const struct driver_enum_s device_enum_root_enum_drv =
{
	.class_		= DRIVER_CLASS_ENUM,
	.f_match_driver	= &device_enum_root_match_driver,
};

const struct driver_s	device_enum_root_drv =
{
	.desc		= "MutekH root enumerator",
	.f_init		= &device_enum_root_init,
	.f_cleanup	= &device_enum_root_cleanup,
	.classes	= { &device_enum_root_enum_drv, 0 }
};

static DEV_INIT(device_enum_root_init)
{
	dev->drv = &device_enum_root_drv;
	dev->status = DEVICE_DRIVER_INIT_DONE;

	return 0;
}


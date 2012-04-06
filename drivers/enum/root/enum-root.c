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

#include <mutek/mem_alloc.h>
#include <hexo/lock.h>
#include <hexo/interrupt.h>

#include <string.h>

#include "enum-root.h"
//#include "enum-root-private.h"

static const struct driver_enum_s enum_root_enum_drv =
{
	.class_		= DEVICE_CLASS_ENUM,
	.f_match_driver	= &enum_root_match_driver,
};

const struct driver_s	enum_root_drv =
{
	.desc		= "MutekH root enumerator",
	.f_init		= &enum_root_init,
	.f_cleanup	= &enum_root_cleanup,
	.classes	= { &enum_root_enum_drv, 0 }
};

REGISTER_DRIVER(enum_root_drv);

DEVENUM_MATCH_DRIVER(enum_root_match_driver)
{
	return 0;
}

DEV_INIT(enum_root_init)
{
	dev->drv = &enum_root_drv;
	dev->status = DEVICE_DRIVER_INIT_DONE;

	return 0;
}

DEV_CLEANUP(enum_root_cleanup)
{
}


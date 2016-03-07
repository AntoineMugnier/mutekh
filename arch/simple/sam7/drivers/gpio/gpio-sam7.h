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

    Copyright (c) Nicolas Pouillon <nipo@ssji.net> 2009

*/

#ifndef DRIVER_GPIO_SAM7_H_
#define DRIVER_GPIO_SAM7_H_

#include <device/class/gpio.h>

struct gpio_sam7_param_s
{
	uint_fast8_t lun_count;
};

DEV_GPIO_SET_WAY(gpio_sam7_set_way);
DEV_GPIO_SET_VALUE(gpio_sam7_set_value);
DEV_GPIO_SET_PULLUP(gpio_sam7_set_pullup);
DEV_GPIO_ASSIGN_TO_PERIPHERAL(gpio_sam7_assign_to_peripheral);
DEV_GPIO_GET_VALUE(gpio_sam7_get_value);
DEV_GPIO_REGISTER_IRQ(gpio_sam7_register_irq);
DEV_INIT(gpio_sam7_init);
DEV_CLEANUP(gpio_sam7_cleanup);

#endif


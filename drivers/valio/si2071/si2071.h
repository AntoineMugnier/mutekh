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
    License along with this program.  If not, see
    <http://www.gnu.org/licenses/>.

    Copyright (c) 2020, Nicolas Pouillon <nipo@ssji.net>
*/

#ifndef _SI2071_H_
#define _SI2071_H_

#include <hexo/types.h>
#include <device/class/i2c.h>

#define SI2071_REG_HUM_HOLD    0xe5
#define SI2071_REG_HUM         0xf5
#define SI2071_REG_TEMP_HOLD   0xe3
#define SI2071_REG_TEMP        0xf3
#define SI2071_REG_TEMP_LAST   0xe0
#define SI2071_REG_RESET       0xfe
#define SI2071_RHT_USER_WRITE  0xe6
#define SI2071_RHT_USER_READ   0xe7
#define SI2071_HEATER_WRITE    0x51
#define SI2071_HEATER_READ     0x11
#define SI2071_ID1             0xfa0f
#define SI2071_ID2             0xfcc9
#define SI2071_FW_REV          0x84b8

#endif /* _SI2071_H_ */

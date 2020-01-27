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

    Copyright Vincent DEFILIPPI <vincentdefilippi@gmail.com> (c) 2016
    Copyright (c) 2017, Nicolas Pouillon <nipo@ssji.net>
*/

#ifndef _HTS221_H_
#define _HTS221_H_

#include <hexo/types.h>
#include <device/class/i2c.h>

#define HTS221_REG_WHO_AM_I        0x0F
#define HTS221_REG_AV_CONF         0x10
#define HTS221_REG_CTRL_REG1       0x20
#define HTS221_REG_CTRL_REG2       0x21
#define HTS221_REG_CTRL_REG3       0x22
#define HTS221_REG_STATUS_REG      0x27
#define HTS221_REG_HUMIDITY_OUT_L  0x28
#define HTS221_REG_HUMIDITY_OUT_H  0x29
#define HTS221_REG_TEMP_OUT_L      0x2A
#define HTS221_REG_TEMP_OUT_H      0x2B
#define HTS221_REG_CALIB           0x30
#define HTS221_CAL_H0_rH_x2        0x30
#define HTS221_CAL_H1_rH_x2        0x31
#define HTS221_CAL_T0_degC_x8      0x32
#define HTS221_CAL_T1_degC_x8      0x33
#define HTS221_CAL_T1_T0_msb       0x35
#define HTS221_CAL_H0_T0_OUT       0x36
#define HTS221_CAL_H1_T0_OUT       0x3a
#define HTS221_CAL_T0_OUT          0x3c
#define HTS221_CAL_T1_OUT          0x3e

#define HTS221_AUTO_INC_ADDR       0x80

#define HTS221_REG_CALIB_COUNT     16
#define HTS221_REG_CONV_COUNT      4
#define HTS221_READ_BUFFER_SIZE    HTS221_REG_CALIB_COUNT

#define HTS221_REG_STATUS_REG_TEMPERATURE_MASK    0x01
#define HTS221_REG_STATUS_REG_HUMIDITY_MASK       0x02

#endif /* _HTS221_H_ */

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

    Copyright (c) 2017, Nicolas Pouillon <nipo@ssji.net>
*/

#ifndef NTAG_STATE_H
#define NTAG_STATE_H

#define NC_PT_ON 0x40
#define NC_PT_TO_I2C 0x01
#define NC_FD_OFF_MASK 0x30
#define NC_FD_OFF_FIELD 0x00
#define NC_FD_OFF_PT 0x30
#define NC_FD_ON_MASK 0x0c
#define NC_FD_ON_FIELD 0x00
#define NC_FD_ON_PT 0x0c

#define NS_I2C_LOCKED 0x40
#define NS_NFC_LOCKED 0x20
#define NS_I2C_READY 0x10
#define NS_NFC_READY 0x08
#define NS_NFC_PRESENT 0x01

#endif

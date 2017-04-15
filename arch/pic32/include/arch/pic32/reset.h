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

    Copyright (c) 2017 Nicolas Pouillon <nipo@ssji.net>
*/

#ifndef PIC32_RESET_H_
#define PIC32_RESET_H_

#include <hexo/bit.h>

#define PIC32_RCON_ADDR    0xBF801240
#define PIC32_RSWRST_ADDR  0xBF801250
#define PIC32_RNMICON_ADDR 0xBF801260
#define PIC32_PWRCON_ADDR  0xBF801270

#define PIC32_RCON_POR      bit(0)
#define PIC32_RCON_BOR      bit(1)
#define PIC32_RCON_IDLE     bit(2)
#define PIC32_RCON_SLEEP    bit(3)
#define PIC32_RCON_WDTO     bit(4)
#define PIC32_RCON_DMTO     bit(5)
#define PIC32_RCON_SWR      bit(6)
#define PIC32_RCON_EXTR     bit(7)
#define PIC32_RCON_CMR      bit(9)
#define PIC32_RCON_BCFGFAIL bit(26)
#define PIC32_RCON_BCFGERR  bit(27)

#endif

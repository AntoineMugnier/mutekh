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

    Copyright (c) 2016, Nicolas Pouillon, <nipo@ssji.net>
*/

#ifndef HD44780_DEFS_H_
#define HD44780_DEFS_H_

#define CMD_CLEAR               0x01
#define CMD_HOME                0x02
#define CMD_ENTRY               0x04
#define CMD_ENTRY_INC            0x02
#define CMD_ENTRY_DEC            0x00
#define CMD_ENTRY_DISPLAY_SHIFT  0x01
#define CMD_ENABLE              0x08
#define CMD_ENABLE_DISPLAY       0x04
#define CMD_ENABLE_CURSOR        0x02
#define CMD_ENABLE_BLINK         0x01
#define CMD_SHIFT               0x10
#define CMD_SHIFT_DISPLAY        0x80
#define CMD_SHIFT_CURSOR         0x00
#define CMD_SHIFT_RIGHT          0x40
#define CMD_SHIFT_LEFT           0x00
#define CMD_FUNCTION            0x20
#define CMD_FUNCTION_8BIT        0x10
#define CMD_FUNCTION_4BIT        0x00
#define CMD_FUNCTION_2LINES      0x08
#define CMD_FUNCTION_1LINE       0x00
#define CMD_FUNCTION_FONT5x10    0x04
#define CMD_FUNCTION_FONT5x8     0x00
#define CMD_CGRAM_ADDR          0x40
#define CMD_DDRAM_ADDR          0x80

#endif

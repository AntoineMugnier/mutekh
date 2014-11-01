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

    Copyright Nicolas Pouillon <nipo@ssji.net> (c) 2014
*/

/**
   @file
   @module{Devices support library}
   @short Value IO interface for a calendar clock
*/

#ifndef LIBDEVICE_VALIO_HWCLOCK_H_
#define LIBDEVICE_VALIO_HWCLOCK_H_

enum valio_hwclock_att {
    /** A struct valio_hwclock_s */
    VALIO_HWCLOCK,
};

struct valio_hwclock_s
{
    /** 20xx */
    uint16_t year;
    /** 1-12 */
    uint8_t month;
    /** 1-31 */
    uint8_t date;
    /** 1-7 (day of week) */
    uint8_t day;
    /** 0-23 */
    uint8_t hour;
    /** 0-59 */
    uint8_t min, sec;
};

#endif

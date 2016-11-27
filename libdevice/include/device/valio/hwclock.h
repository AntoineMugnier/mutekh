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
   @module {Core::Devices support library::Valio device attributes}
   @short Value IO interface for a calendar clock
*/

#ifndef LIBDEVICE_VALIO_HWCLOCK_H_
#define LIBDEVICE_VALIO_HWCLOCK_H_

#include <device/class/valio.h>
#include <hexo/enum.h>

enum valio_hwclock_att {
    /** A struct valio_hwclock_s */
    VALIO_HWCLOCK_DATE = CONFIG_DEVICE_VALIO_HWCLOCK_ATTRIBUTE_FIRST,
};

ENUM_DESCRIPTOR(valio_hwclock_dow_e, strip:DEV_HWCLOCK_DOW_, cap);

/**
   Day of week
 */
enum valio_hwclock_dow_e {
  DEV_HWCLOCK_DOW_SUNDAY,
  DEV_HWCLOCK_DOW_MONDAY,
  DEV_HWCLOCK_DOW_TUESDAY,
  DEV_HWCLOCK_DOW_WEDNESDAY,
  DEV_HWCLOCK_DOW_THURSDAY,
  DEV_HWCLOCK_DOW_FRIDAY,
  DEV_HWCLOCK_DOW_SATURDAY,
};

struct valio_hwclock_s
{
  /** 20xx */
  uint16_t year;
  /** 1-12 */
  uint8_t month;
  /** 1-31 */
  uint8_t day;
  enum valio_hwclock_dow_e dow : 3;
  /** 0-23 */
  uint8_t hour;
  /** 0-59 */
  uint8_t min, sec;
};

/**
   @this converts a @tt HWClock time structure to a unix EPOCH-based
   timestamp.  Reference time structure is supposed in UTC.
 */
uint32_t valio_hwclock_to_epoch(const struct valio_hwclock_s *hc);

/**
   @this converts an EPOCH-based timestamp to a @tt HWClock time
   structure.  Returned structure is in UTC.
 */
void valio_hwclock_from_epoch(struct valio_hwclock_s *hc,
                              uint32_t ts);

#endif

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

#include <device/class/valio.h>
#include <device/valio/hwclock.h>
#include <hexo/enum.h>

static const uint8_t month_days[] = {31, 28, 31, 30,
                                     31, 30, 31, 31,
                                     30, 31, 30, 31};

uint32_t valio_hwclock_to_epoch(const struct valio_hwclock_s *hc)
{
  uint32_t years = hc->year - 1970;
  uint32_t days = hc->day - 1;
  days += years * 365 + years / 4;
  for (uint8_t month = 0; month < hc->month - 1; ++month)
    days += month_days[month];

  if (hc->year % 4 == 0 && hc->month > 2)
    days += 1;

  uint32_t hours = days * 24 + hc->hour;
  uint32_t minutes = hours * 60 + hc->min;

  return minutes * 60 + hc->sec;
}

void valio_hwclock_from_epoch(struct valio_hwclock_s *hc,
                              uint32_t ts)
{
  hc->sec = ts % 60;
  ts /= 60;
  hc->min = ts % 60;
  ts /= 60;
  hc->hour = ts % 24;
  ts /= 24;

  // Now we have days from epoch

  // Epoch was thursday
  hc->dow = (DEV_HWCLOCK_DOW_THURSDAY + ts) % 7;
  
  uint32_t whole_4_years = ts / (365 * 4 + 1);
  ts %= (365 * 4 + 1);

  hc->year = 1970 + whole_4_years * 4;

  bool_t leap = 0;
  if (ts > 365 + 365 + 366) {
    ts -= 365 + 365 + 366;
    hc->year += 3;
  } else if (ts > 365 + 365) {
    ts -= 365 + 365;
    hc->year += 2;
    leap = 1;
  } else if (ts > 365) {
    ts -= 365;
    hc->year += 1;
  }

  // ts is now in days within a year
  size_t month;
  for (month = 0; month < 12; ++month) {
    uint32_t month_dur = month_days[month] + (leap && month == 1);
    if (ts < month_dur)
      break;
    ts -= month_dur;
  }

  hc->month = month + 1;
  hc->day = ts + 1;
}

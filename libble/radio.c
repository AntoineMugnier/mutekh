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

    Copyright Nicolas Pouillon <nipo@ssji.net> (c) 2015
*/

#include <hexo/types.h>
#include <ble/protocol/radio.h>

uint16_t ble_channel_freq_mhz(uint8_t chan)
{
  switch (chan) {
  case 0 ... 10:
    return 2400 + chan * 2 + 4;
  case 11 ... 36:
    return 2400 + chan * 2 + 6;
  case 37:
    return 2402;
  case 38:
    return 2426;
  case 39:
    return 2480;
  }
  return 0;
}

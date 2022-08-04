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

#ifndef MAX44009_REGS_H_
#define MAX44009_REGS_H_

#define REG_INT_STATUS  0x00
#define  INT_LIMIT_CROSSED  0x01
#define REG_INT_ENABLE  0x01
#define REG_CONFIG      0x02
#define  CONFIG_CONTINUOUS  0x80
#define  CONFIG_MANUAL      0x40
#define  CONFIG_DIV8        0x08
#define  CONFIG_INTEG_TIME(n) (_CONFIG_INTEG_TIME_ ## n)
#define  _CONFIG_INTEG_TIME_800MS  0
#define  _CONFIG_INTEG_TIME_400MS  1
#define  _CONFIG_INTEG_TIME_200MS  2
#define  _CONFIG_INTEG_TIME_100MS  3
#define  _CONFIG_INTEG_TIME_50MS   4
#define  _CONFIG_INTEG_TIME_25MS   5
#define  _CONFIG_INTEG_TIME_12_5MS 6
#define  _CONFIG_INTEG_TIME_6_25MS 7

// From/to m^e to lux:
// mlux = (m_4 << e) * 720
// mlux = (m_8 << e) * 45
// m/e:
//   tmp = ((mlux >> 9) * 1475) >> 11
//   e = high_bit(tmp) - 4
//   m = tmp >> e

// EEEEMMMM
#define REG_LUX_H       0x03
// ----MMMM
#define REG_LUX_L       0x04

// EEEEMMMM
#define REG_THRESH_UP   0x05
// EEEEMMMM
#define REG_THRESH_DOWN 0x06

// In 100ms units
#define REG_THRESH_TIME 0x07

#endif

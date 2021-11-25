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

    Copyright (c) 2021, Nicolas Pouillon <nipo@ssji.net>
*/

#ifndef _KXTJ3_H_
#define _KXTJ3_H_

#include <hexo/types.h>
#include <device/class/i2c.h>

#define KXTJ3_XOUT_L             0x06
#define KXTJ3_XOUT_H             0x07
#define KXTJ3_YOUT_L             0x08
#define KXTJ3_YOUT_H             0x09
#define KXTJ3_ZOUT_L             0x0a
#define KXTJ3_ZOUT_H             0x0b

#define KXTJ3_DCST_RESP          0x0c
#define  KXTJ3_DCST_RESP_DEFAULT 0x55
#define  KXTJ3_DCST_RESP_TEST    0xaa

#define KXTJ3_WHO_AM_I           0x0f
#define  KXTJ3_WHO_AM_I_VALUE    0x35

#define KXTJ3_INT_SOURCE1        0x16
#define  KXTJ3_INT_SOURCE1_WUFS  0x02
#define  KXTJ3_INT_SOURCE1_DRDY  0x10

#define KXTJ3_INT_SOURCE2        0x17
#define  KXTJ3_INT_SOURCE2_XNWU  0x20
#define  KXTJ3_INT_SOURCE2_XPWU  0x10
#define  KXTJ3_INT_SOURCE2_YNWU  0x08
#define  KXTJ3_INT_SOURCE2_YPWU  0x04
#define  KXTJ3_INT_SOURCE2_ZNWU  0x03
#define  KXTJ3_INT_SOURCE2_ZPWU  0x01

#define KXTJ3_STATUS             0x18
#define  KXTJ3_STATUS_INT        0x10

#define KXTJ3_INT_REL            0x1a

#define KXTJ3_CTRL1              0x1b
#define  KXTJ3_CTRL1_WUFE        0x02
#define  KXTJ3_CTRL1_RES_MASK    0x1c
/*
  Gives resolution value for resolution = +/- 2^(N+1) g
 */
#define  KXTJ3_CTRL1_RES_LUT(N)  (((0x1420 >> (4 * (N))) & 0x7) << 2)

#define  KXTJ3_CTRL1_DRDYE       0x20
#define  KXTJ3_CTRL1_RES         0x40
#define  KXTJ3_CTRL1_PC1         0x80

#define KXTJ3_CTRL2              0x1d
#define  KXTJ3_CTRL2_OWU_MASK    0x07
/*
 Gives output data rate for wakeup function configuration value at nibble N
  for interval of 10ms * 2^N
  i.e. 100Hz * 2^-N
  0 <= N <= 7
 */
#define  KXTJ3_CTRL2_OWU_INTERVAL_LUT(N) ((0x7 & (N)) ^ 0x7)
#define  KXTJ3_CTRL2_DCST        0x10
#define  KXTJ3_CTRL2_SRST        0x80

#define KXTJ3_INT_CTRL1          0x1e
#define  KXTJ3_INT_CTRL1_IEN     0x20
#define  KXTJ3_INT_CTRL1_IEA     0x10
#define  KXTJ3_INT_CTRL1_IEL     0x08
#define  KXTJ3_INT_CTRL1_STPOL   0x02
#define  KXTJ3_INT_CTRL1_IEN     0x20

#define KXTJ3_INT_CTRL2          0x1f
#define  KXTJ3_INT_CTRL2_ZPWUE   0x01
#define  KXTJ3_INT_CTRL2_ZNWUE   0x02
#define  KXTJ3_INT_CTRL2_YPWUE   0x04
#define  KXTJ3_INT_CTRL2_YNWUE   0x08
#define  KXTJ3_INT_CTRL2_XPWUE   0x10
#define  KXTJ3_INT_CTRL2_XNWUE   0x20
#define  KXTJ3_INT_CTRL2_ULMODE  0x80

#define KXTJ3_DATA_CTRL          0x21
#define  KXTJ3_DATA_CTRL_OSA_MASK 0x0f
/*
 Gives LPF output data rate configuration value
  for interval of 625us * 2^N
  i.e. 1600Hz * 2^-N
  0 <= N <= 11
 */
#define  KXTJ3_DATA_CTRL_OSA_INTERVAL_LUT(N) ((0x89ab01234567ULL >> (4 * (N))) & 0xf)

#define KXTJ3_WAKEUP_COUNTER     0x29
#define KXTJ3_NA_COUNTER         0x2a

#define KXTJ3_SELF_TEST          0x3a
#define  KXTJ3_SELF_TEST_EN      0xca

#define KXTJ3_WAKEUP_THRESHOLD_H   0x6a
#define KXTJ3_WAKEUP_THRESHOLD_L   0x6b

#endif /* _KXTJ3_H_ */

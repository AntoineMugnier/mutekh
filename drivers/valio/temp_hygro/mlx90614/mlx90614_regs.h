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

#ifndef MLX90614_REGS_H_
#define MLX90614_REGS_H_

#define REG_TA            0x06
#define REG_TOBJ1         0x07
#define REG_TOBJ2         0x08
#define REG_TOMAX         0x20
#define REG_TOMIN         0x21
#define REG_PWMCTRL       0x22
#define REG_TA_RANGE      0x23
#define REG_KE            0x24
#define REG_CONFIG1       0x25
#define REG_SMBUS_ADDRESS 0x2e
#define REG_ID0           0x3c
#define REG_ID1           0x3d
#define REG_ID2           0x3e
#define REG_ID3           0x3f
#define REG_FLAGS         0xf0
#define CMD_SLEEP         0xff

#define PWMCTRL_EXTENDED       0x0001
#define PWMCTRL_ENABLE         0x0002
#define PWMCTRL_PUSHPULL       0x0004
#define PWMCTRL_THERMORELAY    0x0008
#define PWMCTRL_REPETITIONS(x) ((__MIN(x, 63) / 2) << 4)
#define PWMCTRL_CLOCK_DIV(x)   (((x) & 0x7f) << 9)

#define CONFIG1_IIR_0_5        0x0000
#define CONFIG1_IIR_0_571      0x0007
#define CONFIG1_IIR_0_666      0x0006
#define CONFIG1_IIR_0_8        0x0005
#define CONFIG1_IIR_1          0x0004
#define CONFIG1_AMB_PTC        0x0008
#define CONFIG1_PWM_TA_IR1     0x0000
#define CONFIG1_PWM_TA_IR2     0x0010
#define CONFIG1_PWM_IR1_IR1    0x0020
#define CONFIG1_PWM_IR2        0x0030
#define CONFIG1_IR_DUAL        0x0040
#define CONFIG1_KS             0x0080
#define CONFIG1_FIR_N_LOG2(x)  ((((x) - 3) & 0x7) << 8)
#define CONFIG1_AMP_GAIN_1     0x0000
#define CONFIG1_AMP_GAIN_3     0x0800
#define CONFIG1_AMP_GAIN_6     0x1000
#define CONFIG1_AMP_GAIN_12_5  0x1800
#define CONFIG1_AMP_GAIN_25    0x2000
#define CONFIG1_AMP_GAIN_50    0x2800
#define CONFIG1_AMP_GAIN_100   0x3000
#define CONFIG1_THERMOCOMP_NEG 0x8000

#define FLAGS_BUSY    0x8000
#define FLAGS_EE_DEAD 0x2000
#define FLAGS_INIT    0x1000

#endif

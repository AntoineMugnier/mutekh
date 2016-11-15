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

    Copyright Nicolas Pouillon <nipo@ssji.net> (c) 2016
*/

#ifndef ARCH_NRF_SAADC_H_
#define ARCH_NRF_SAADC_H_

#include "peripheral.h"
#include "ids.h"

enum nrf5x_saadc_task_e
{
  NRF_SAADC_START,
  NRF_SAADC_SAMPLE,
  NRF_SAADC_STOP,
  NRF_SAADC_CALIBRATEOFFSET,
};

enum nrf5x_saadc_event_e
{
  NRF_SAADC_STARTED,
  NRF_SAADC_END,
  NRF_SAADC_DONE,
  NRF_SAADC_RESULTDONE,
  NRF_SAADC_CALIBRATEDONE,
  NRF_SAADC_STOPPED,
  NRF_SAADC_CH0_LIMITH,
  NRF_SAADC_CH0_LIMITL,
#define NRF_SAADC_CH_LIMITH(x) (NRF_SAADC_CH0_LIMITH + (x) * 2)
#define NRF_SAADC_CH_LIMITL(x) (NRF_SAADC_CH0_LIMITL + (x) * 2)
};

enum nrf5x_saadc_register_s
{
  NRF_SAADC_STATUS = 0,
  NRF_SAADC_ENABLE = 64,
  NRF_SAADC_CH0_PSELP = 68,
  NRF_SAADC_CH0_PSELN = 69,
  NRF_SAADC_CH0_CONFIG = 70,
  NRF_SAADC_CH0_LIMIT = 71,
#define NRF_SAADC_CH_PSELP(x)  (NRF_SAADC_CH0_PSELP + (x) * 4)
#define NRF_SAADC_CH_PSELN(x)  (NRF_SAADC_CH0_PSELN + (x) * 4)
#define NRF_SAADC_CH_CONFIG(x) (NRF_SAADC_CH0_CONFIG + (x) * 4)
#define NRF_SAADC_CH_LIMIT(x)  (NRF_SAADC_CH0_LIMIT + (x) * 4)

  NRF_SAADC_RESOLUTION = 124,
  NRF_SAADC_OVERSAMPLE = 125,
  NRF_SAADC_SAMPLERATE = 126,
  NRF_SAADC_RESULT_PTR = 139,
  NRF_SAADC_RESULT_MAXCNT = 140,
  NRF_SAADC_RESULT_AMOUNT = 141,
};

#define NRF_SAADC_STATUS_MASK 1
#define NRF_SAADC_STATUS_READY 0
#define NRF_SAADC_STATUS_BUSY 1

#define NRF_SAADC_ENABLE_MASK 1
#define NRF_SAADC_ENABLE_DISABLED 0
#define NRF_SAADC_ENABLE_ENABLED 1

#define NRF_SAADC_CONFIG_RES_8BIT 0
#define NRF_SAADC_CONFIG_RES_9BIT 1
#define NRF_SAADC_CONFIG_RES_10BIT 2

#define NRF_SAADC_PSEL_NC 0
#define NRF_SAADC_PSEL_AIN(x) ((x) + 1)
#define NRF_SAADC_PSEL_VDD 9

#define NRF_SAADC_CONFIG_RESP_BYPASS     (0 << 0)
#define NRF_SAADC_CONFIG_RESP_PULLDOWN   (1 << 0)
#define NRF_SAADC_CONFIG_RESP_PULLUP     (2 << 0)
#define NRF_SAADC_CONFIG_RESP_VDD1_2     (3 << 0)

#define NRF_SAADC_CONFIG_RESN_BYPASS     (0 << 4)
#define NRF_SAADC_CONFIG_RESN_PULLDOWN   (1 << 4)
#define NRF_SAADC_CONFIG_RESN_PULLUP     (2 << 4)
#define NRF_SAADC_CONFIG_RESN_VDD1_2     (3 << 4)

#define NRF_SAADC_CONFIG_GAIN_1_6        (0 << 8)
#define NRF_SAADC_CONFIG_GAIN_1_5        (1 << 8)
#define NRF_SAADC_CONFIG_GAIN_1_4        (2 << 8)
#define NRF_SAADC_CONFIG_GAIN_1_3        (3 << 8)
#define NRF_SAADC_CONFIG_GAIN_1_2        (4 << 8)
#define NRF_SAADC_CONFIG_GAIN_1          (5 << 8)
#define NRF_SAADC_CONFIG_GAIN_2          (6 << 8)
#define NRF_SAADC_CONFIG_GAIN_4          (7 << 8)

#define NRF_SAADC_CONFIG_REFSEL_INTERNAL (0 << 12)
#define NRF_SAADC_CONFIG_REFSEL_VDD1_4   (1 << 12)

#define NRF_SAADC_CONFIG_TACQ_3US        (0 << 16)
#define NRF_SAADC_CONFIG_TACQ_5US        (1 << 16)
#define NRF_SAADC_CONFIG_TACQ_10US       (2 << 16)
#define NRF_SAADC_CONFIG_TACQ_15US       (3 << 16)
#define NRF_SAADC_CONFIG_TACQ_20US       (4 << 16)
#define NRF_SAADC_CONFIG_TACQ_40US       (5 << 16)

#define NRF_SAADC_CONFIG_MODE_SE         (0 << 20)
#define NRF_SAADC_CONFIG_MODE_DIFF       (1 << 20)

#define NRF_SAADC_CONFIG_BURST_DISABLED  (0 << 24)
#define NRF_SAADC_CONFIG_BURST_ENABLED   (1 << 24)

#define NRF_SAADC_RESOLUTION_8BIT        0
#define NRF_SAADC_RESOLUTION_10BIT       1
#define NRF_SAADC_RESOLUTION_12BIT       2
#define NRF_SAADC_RESOLUTION_14BIT       3

#define NRF_SAADC_OVERSAMPLE_BYPASS      0
#define NRF_SAADC_OVERSAMPLE_X(n)        (__builtin_ctz(n))

#define NRF_SAADC_SAMPLERATE_CC(x)       ((x) << 0)
#define NRF_SAADC_SAMPLERATE_MODE_TASK   (0 << 12)
#define NRF_SAADC_SAMPLERATE_MODE_TIMER  (1 << 12)

#endif

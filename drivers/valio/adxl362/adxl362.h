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
    License along with MutekH; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
    02110-1301 USA.

    Copyright Sebastien Cerdan <sebcerdan@gmail.com> (c) 2014
    Copyright Vincent DEFILIPPI <vincentdefilippi@gmail.com> (c) 2016
*/

#ifndef _ADXL362_H_
#define _ADXL362_H_

#include <hexo/types.h>
#include <device/class/spi.h>
#include <device/valio/motion_sensor.h>
#include "adxl362_regs.h"

/*----------------------------------------------------------------------------*/

#define ADXL362_CMD_WRITE_REG   0x0A
#define ADXL362_CMD_READ_REG    0x0B
#define ADXL362_CMD_READ_FIFO   0x0D

#define ADXL362_WRITE_REG(addr, val) ((val << 16) | ((addr) << 8) | (ADXL362_CMD_WRITE_REG))
#define ADXL362_READ_REG(addr)  (((addr) << 8) | (ADXL362_CMD_READ_REG))

/*----------------------------------------------------------------------------*/

#define ADXL362_SOFT_RESET_CODE       0x52

#define ADXL362_REG_DEVID_AD_CONST    0xAD
#define ADXL362_REG_DEVID_MST_CONST   0x1D
#define ADXL362_REG_PARTID_CONST      0xF2

/*----------------------------------------------------------------------------*/

#if CONFIG_DRIVER_ADXL362_RANGE_MG == 2000
# define ADXL362_DEFAULT_RANGE ADXL362_REG_FILTER_CTL_RANGE(2G)
#elif CONFIG_DRIVER_ADXL362_RANGE_MG == 4000
# define ADXL362_DEFAULT_RANGE ADXL362_REG_FILTER_CTL_RANGE(4G)
#elif CONFIG_DRIVER_ADXL362_RANGE_MG == 8000
# define ADXL362_DEFAULT_RANGE ADXL362_REG_FILTER_CTL_RANGE(8G)
#else
# error bad CONFIG_DRIVER_ADXL362_RANGE_MG
#endif

#define ADXL362_MG_PER_LSB (CONFIG_DRIVER_ADXL362_RANGE_MG / 128)

#if CONFIG_DRIVER_ADXL362_ACT_DATA_RATE_HZ == 6
# define ADXL362_DEFAULT_ODR    ADXL362_REG_FILTER_CTL_ODR(100_HZ)
# define ADXL362_DEFAULT_WAKEUP ADXL362_REG_POWER_CTL_WAKEUP_MASK
#elif CONFIG_DRIVER_ADXL362_ACT_DATA_RATE_HZ == 25
# define ADXL362_DEFAULT_ODR    ADXL362_REG_FILTER_CTL_ODR(25_HZ)
# define ADXL362_DEFAULT_WAKEUP 0
#elif CONFIG_DRIVER_ADXL362_ACT_DATA_RATE_HZ == 50
# define ADXL362_DEFAULT_ODR    ADXL362_REG_FILTER_CTL_ODR(50_HZ)
# define ADXL362_DEFAULT_WAKEUP 0
#elif CONFIG_DRIVER_ADXL362_ACT_DATA_RATE_HZ == 100
# define ADXL362_DEFAULT_ODR    ADXL362_REG_FILTER_CTL_ODR(100_HZ)
# define ADXL362_DEFAULT_WAKEUP 0
#elif CONFIG_DRIVER_ADXL362_ACT_DATA_RATE_HZ == 200
# define ADXL362_DEFAULT_ODR    ADXL362_REG_FILTER_CTL_ODR(200_HZ)
# define ADXL362_DEFAULT_WAKEUP 0
#elif CONFIG_DRIVER_ADXL362_ACT_DATA_RATE_HZ == 400
# define ADXL362_DEFAULT_ODR    ADXL362_REG_FILTER_CTL_ODR(400_HZ)
# define ADXL362_DEFAULT_WAKEUP 0
#else
# error bad CONFIG_DRIVER_ADXL362_ACT_DATA_RATE_HZ
#endif

#if CONFIG_DRIVER_ADXL362_INACT_DATA_RATE_HZ == 6
# define ADXL362_DEFAULT_AUTOSLEEP ADXL362_REG_POWER_CTL_AUTOSLEEP_MASK
#elif CONFIG_DRIVER_ADXL362_INACT_DATA_RATE_HZ == CONFIG_DRIVER_ADXL362_ACT_DATA_RATE_HZ
# define ADXL362_DEFAULT_AUTOSLEEP 0
#else
# error bad CONFIG_DRIVER_ADXL362_INACT_DATA_RATE_HZ
#endif

#define ADXL362_DEFAULT_FILTER_CTL \
  (ADXL362_DEFAULT_RANGE | ADXL362_REG_FILTER_CTL_HALF_BW_MASK | ADXL362_DEFAULT_ODR)

#define ADXL362_DEFAULT_POWER_CTL \
  (ADXL362_DEFAULT_AUTOSLEEP | ADXL362_REG_POWER_CTL_MEASURE(MEASUREMENT) | ADXL362_DEFAULT_WAKEUP)

#define ADXL362_DEFAULT_ACT_INACT_CTL \
  (ADXL362_REG_ACT_INACT_CTL_LINKLOOP(LINKED) | \
  ADXL362_REG_ACT_INACT_CTL_INACT_REF_MASK    | \
  ADXL362_REG_ACT_INACT_CTL_INACT_EN_MASK     | \
  ADXL362_REG_ACT_INACT_CTL_ACT_REF_MASK      | \
  ADXL362_REG_ACT_INACT_CTL_ACT_EN_MASK)

#if CONFIG_DRIVER_ADXL362_ACT_THRESHOLD_MG < CONFIG_DRIVER_ADXL362_RANGE_MG
# define ADXL362_DEFAULT_THRES_ACT \
  (((CONFIG_DRIVER_ADXL362_ACT_THRESHOLD_MG) << 11) / (CONFIG_DRIVER_ADXL362_RANGE_MG))
#else
# define ADXL362_DEFAULT_THRES_ACT (2048 - 1)
#endif

#define ADXL362_DEFAULT_THRESH_ACT_L (ADXL362_DEFAULT_THRES_ACT & 0xff)
#define ADXL362_DEFAULT_THRESH_ACT_H ((ADXL362_DEFAULT_THRES_ACT >> 8) & 0xff)

#if CONFIG_DRIVER_ADXL362_INACT_THRESHOLD_MG < CONFIG_DRIVER_ADXL362_RANGE_MG
# define ADXL362_DEFAULT_THRES_INACT \
  (((CONFIG_DRIVER_ADXL362_INACT_THRESHOLD_MG) << 11) / (CONFIG_DRIVER_ADXL362_RANGE_MG))
#else
# define ADXL362_DEFAULT_THRES_INACT (2048 - 1)
#endif

# define ADXL362_DEFAULT_THRESH_INACT_L (ADXL362_DEFAULT_THRES_INACT & 0xff)
# define ADXL362_DEFAULT_THRESH_INACT_H ((ADXL362_DEFAULT_THRES_INACT >> 8) & 0xff)

#if (CONFIG_DRIVER_ADXL362_INACT_DATA_RATE_HZ == 6 && CONFIG_DRIVER_ADXL362_ACT_TIME_MS != 0)
# error bad CONFIG_DRIVER_ADXL362_ACT_TIME_MS
#endif

#define ADXL362_TIME_ACT \
  (CONFIG_DRIVER_ADXL362_ACT_TIME_MS * CONFIG_DRIVER_ADXL362_INACT_DATA_RATE_HZ / 1000)

#if ADXL362_TIME_ACT < 0xff
# define ADXL362_DEFAULT_TIME_ACT ADXL362_TIME_ACT
#else
# error bad CONFIG_DRIVER_ADXL362_ACT_TIME_MS
#endif

#define ADXL362_TIME_INACT \
  (CONFIG_DRIVER_ADXL362_INACT_TIME_MS * CONFIG_DRIVER_ADXL362_ACT_DATA_RATE_HZ / 1000)

#if ADXL362_TIME_INACT < 0xffff
# define ADXL362_DEFAULT_TIME_INACT_L (ADXL362_TIME_INACT & 0xff)
# define ADXL362_DEFAULT_TIME_INACT_H ((ADXL362_TIME_INACT >> 8) & 0xff)
#else
# error bad CONFIG_DRIVER_ADXL362_INACT_TIME_MS
#endif

#define ADXL362_DEFAULT_INTMAP1 (ADXL362_REG_INTMAP1_INACT_MASK | ADXL362_REG_INTMAP1_ACT_MASK)
#define ADXL362_DEFAULT_INTMAP2 0x00

#define ADXL362_DEFAULT_FIFO_SAMPLES 0x80
#define ADXL362_DEFAULT_FIFO_CONTROL 0x00

/*----------------------------------------------------------------------------*/

enum adxl362_state_e
{
  ADXL362_STATE_INIT,
  ADXL362_STATE_STANDBY,
  ADXL362_STATE_READ_CONV,
  ADXL362_STATE_CONFIG,
  ADXL362_STATE_ERROR,
};

DRIVER_PV(struct adxl362_private_s
{
  struct device_spi_ctrl_s            spi;
  struct device_gpio_s                *gpio;
  struct device_timer_s               *timer;
  struct dev_irq_src_s                irq_ep;
  struct dev_spi_ctrl_bytecode_rq_s   spi_rq;

  dev_request_queue_root_t            queue;
  dev_request_queue_root_t            wait_event_queue;

  struct valio_ms_accel_config_s      new_config;
  struct valio_ms_accel_config_s      config;

  int16_t                             calibration[3];

  bool_t                              config_pending;
  bool_t                              read_pending;

  enum adxl362_state_e                state;
});

#endif /* _ADXL362_H_ */

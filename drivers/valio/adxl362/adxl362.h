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

/*----------------------------------------------------------------------------*/

#define ADXL362_SOFT_RESET_CODE       0x52

#define ADXL362_REG_DEVID_AD_CONST    0xAD
#define ADXL362_REG_DEVID_MST_CONST   0x1D
#define ADXL362_REG_PARTID_CONST      0xF2

/*----------------------------------------------------------------------------*/

#if CONFIG_DRIVER_ADXL362_RANGE_MG == 2000
# define ADXL362_DEFAULT_RANGE ADXL362_REG_FILTER_CTL_RANGE(2G)
# define ADXL362_MG_PER_LSB 1
#elif CONFIG_DRIVER_ADXL362_RANGE_MG == 4000
# define ADXL362_DEFAULT_RANGE ADXL362_REG_FILTER_CTL_RANGE(4G)
# define ADXL362_MG_PER_LSB 2
#elif CONFIG_DRIVER_ADXL362_RANGE_MG == 8000
# define ADXL362_DEFAULT_RANGE ADXL362_REG_FILTER_CTL_RANGE(8G)
# define ADXL362_MG_PER_LSB 3
#else
# error bad CONFIG_DRIVER_ADXL362_RANGE_MG
#endif

/*----------------------------------------------------------------------------*/

enum adxl362_state_e
{
  ADXL362_STATE_INIT,
  ADXL362_STATE_IDLE,
  ADXL362_STATE_RUNNING,
  ADXL362_STATE_ERROR,
};

struct adxl362_private_s
{
  struct device_spi_ctrl_s            spi;
  struct device_gpio_s                *gpio;
  struct device_timer_s               *timer;
  struct dev_irq_src_s                irq_ep;
  struct dev_spi_ctrl_bytecode_rq_s   spi_rq;

  dev_request_queue_root_t            queue;
  struct valio_ms_config_s            config;

  int16_t                             offset[3];

  bool_t                              config_pending;
  bool_t                              read_pending;
  bool_t                              was_active;

  enum adxl362_state_e                state;

  gpio_id_t pin_map[1];
};

DRIVER_PV(struct adxl362_private_s);

#endif /* _ADXL362_H_ */

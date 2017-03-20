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

    Copyright (c) Nicolas Pouillon <nipo@ssji.net>, 2016
*/

#ifndef MPU650X_H_
#define MPU650X_H_

#include <hexo/types.h>
#include <hexo/endian.h>
#include <hexo/iospace.h>
#include <hexo/interrupt.h>
#include <hexo/bit.h>

#include <mutek/mem_alloc.h>
#include <mutek/printk.h>
#include <mutek/bytecode.h>

#include <device/resources.h>
#include <device/device.h>
#include <device/driver.h>
#include <device/irq.h>

#include <device/class/icu.h>
#include <device/class/valio.h>
#include <device/valio/motion_sensor.h>
#ifdef CONFIG_DRIVER_MPU650X_POWERGATE
# include <device/clock.h>
#endif

#if defined(CONFIG_DRIVER_MPU650X_SPI)
# include <device/class/spi.h>
#elif defined(CONFIG_DRIVER_MPU650X_I2C)
# include <device/class/i2c.h>
#endif

enum mpu650x_state_e
{
  MPU650X_GATE_OFF,
  MPU650X_GATE_ON,
  MPU650X_POWER_OFF,
  MPU650X_POWER_ON,
  MPU650X_STREAMING,
  MPU650X_WOM,
};

#define STREAMING_FPS 16
#define GYRO_AUTO (STREAMING_FPS)
#define GYRO_AUTO_TIME (GYRO_AUTO * 4)
#define WOM_AUTO (STREAMING_FPS * 10)

struct mpu650x_context_s
{
#if defined(CONFIG_DRIVER_MPU650X_SPI)
  struct device_spi_ctrl_s bus;
  struct dev_spi_ctrl_bytecode_rq_s bus_rq;
#elif defined(CONFIG_DRIVER_MPU650X_I2C)
  struct device_i2c_ctrl_s bus;
  struct dev_i2c_ctrl_bytecode_rq_s bus_rq;
#endif
  struct dev_irq_src_s irq_ep;

  struct device_timer_s *timer;

#ifdef CONFIG_DRIVER_MPU650X_POWERGATE
  struct dev_clock_sink_ep_s power_source;
#endif

  int16_t value_last[6];
  int16_t offset[6];
  uint8_t stable_count;
  uint8_t stable_left;

  enum mpu650x_state_e state;
  enum mpu650x_state_e target_state;

  uint8_t irq_pending;
  uint8_t has_moved;
  uint8_t has_fresh_data;
  uint8_t wom_just_changed;
  uint8_t error_count;

  bool_t bus_started;

  dev_request_queue_root_t queue;
};

#endif

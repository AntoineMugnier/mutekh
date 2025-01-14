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
    Copyright Vincent Defilippi <vincentdefilippi@gmail.com> (c) 2016
*/

/**
   @file
   @module {Core::Devices support library::Valio device attributes}
   @short Value IO interface definition for motion sensors
*/

#ifndef LIBDEVICE_VALIO_MOTION_SENSOR_H_
#define LIBDEVICE_VALIO_MOTION_SENSOR_H_

#include <device/class/valio.h>

enum valio_ms_att_e {
    /** A struct valio_ms_state_s with current gravity vector, instant rotation
        and magnetic measurements.
        Read or Wait event based on a defined status. */
    VALIO_MS_STATE = CONFIG_DEVICE_VALIO_MOTION_SENSOR_ATTRIBUTE_FIRST,

    /** A struct valio_ms_data with all possible values.
        Set offset registers. */
    VALIO_MS_CALIB,

    /** A struct @ref valio_ms_config_s. Read or Write. */
    VALIO_MS_CONFIG,
};

/**
   Defined axis for a motion sensor unit. Not all devices are required
   to support all axis.
 */
enum valio_ms_axis_e {
    VALIO_MS_ACCEL_X,
    VALIO_MS_ACCEL_Y,
    VALIO_MS_ACCEL_Z,
    VALIO_MS_GYRO_X,
    VALIO_MS_GYRO_Y,
    VALIO_MS_GYRO_Z,
    VALIO_MS_COMPASS_X,
    VALIO_MS_COMPASS_Y,
    VALIO_MS_COMPASS_Z,
    VALIO_MS_AXIS_COUNT,
};

/**
   For an object lying still, in its natural orientation, nominal
   terrestrial gravity is -1g along Z axis.

   Acceleration values are in mG (1e-3 g).
   Rotation values are in 1/64 dps (2^-6 degrees per second).
   Magnetic sensing values are in µT (1e-6 Tesla)
 */
struct valio_ms_data_s
{
  int16_t axis[VALIO_MS_AXIS_COUNT];
};

struct valio_ms_state_s
{
  struct valio_ms_data_s data;

  /** Whether device is currently considered active (see config). */
  bool_t active;
};

struct valio_ms_config_s
{
  /** Main data streaming period, in ms. */
  uint16_t period;

  /** Accelerometer threshold, in mG, over/under which device decide
      it is active/inactive. */
  uint16_t threshold;

  /** Minimum time in @tt ms to stay with changes in acceleration less
      than @tt threshold in order to be declared inactive.  This is a
      best case timing.  Inactivity should not be declared before this
      time. */
  uint16_t sleep_time;

  /** Maximum time in @tt ms to run with changes in acceleration
      values over @tt threshold in order to be declared active.  This
      is a worst case timing.  Active mode could be declared before
      that. */
  uint16_t wakeup_time;
};

#endif

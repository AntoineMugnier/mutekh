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

    Copyright Julien Peeters <contact@julienpeeters.net> (c) 2014
*/

/**
   @file
   @module{Devices support library}
   @short Value IO interface for a accelerometer
*/

#ifndef LIBDEVICE_VALIO_MOTION_H_
#define LIBDEVICE_VALIO_MOTION_H_

/** The motion valio class is an interface for controlling and acquire data
    from motion tracking devices like an acceleromter, gyroscope or compass.

    One class instance interface a device that may integrate up to the three
*/

enum valio_motion_att_e
{
    /* Use with @tt DEVICE_VALIO_READ request type. @tt data is a pointer to a 
       @tt struct valio_motion_caps_s that defines the capability of the device,
       including what motion sensors are available.*/ 
    VALIO_MOTION_CAPS = CONFIG_DEVICE_VALIO_MOTION_ATTRIBUTE_FIRST,

    /* Use with @tt DEVICE_VALIO_WRITE request type. @tt data is NULL. This 
       calibrates device. Calibration can be implemented either in device when 
       hardware support or in driver. Return -ENOTSUP if calibration is not 
       supported.*/

    VALIO_MOTION_ACCEL_CALIBRATE,

    /* Use with @tt DEVICE_VALIO_WRITE request type. @tt data is a pointer to a 
       @tt struct valio_motion_accel_s that defines the configuration of an
       accelerometer. */
    VALIO_MOTION_ACCEL,

    /* Use with @tt DEVICE_VALIO_WRITE request type. @tt data is a pointer to a 
       @tt struct valio_motion_gyro_s that defines the configuration of a
       gyroscope. */
    VALIO_MOTION_GYRO,

    /* Use with @tt DEVICE_VALIO_WRITE request type. @tt data is a pointer to a 
       @tt struct valio_motion_comp_s that defines the configuration of a
       compass.(i.e. magnetometer). */
    VALIO_MOTION_COMP,

    /* Use with @tt DEVICE_VALIO_READ request type. @tt data is a pointer to a 
       @tt struct valio_motion_data_s that defines the value of each of the X, Y
        Z axis.  */
    VALIO_MOTION_DATA,

    /* Use with @tt DEVICE_VALIO_WAIT_UPDATE request type. @tt data is a pointer to a 
       @tt struct valio_motion_evt_s that defines the value of each of the X, Y
        Z axis. */
    VALIO_MOTION_EVENT,
};

/** @This is used for selecting one or more axis.

    Note: axis that cannot be distinguished must be all selected at the same
    time.
*/
enum valio_motion_axis_e
{
    VALIO_MOTION_ACC_X = (1 << 0),
    VALIO_MOTION_ACC_Y = (1 << 1),
    VALIO_MOTION_ACC_Z = (1 << 2),

    VALIO_MOTION_ACC_XYZ = VALIO_MOTION_ACC_X | VALIO_MOTION_ACC_Y |
        VALIO_MOTION_ACC_Z,

    VALIO_MOTION_GYR_X = (1 << 3),
    VALIO_MOTION_GYR_Y = (1 << 4),
    VALIO_MOTION_GYR_Z = (1 << 5),

    VALIO_MOTION_GYR_XYZ = VALIO_MOTION_GYR_X | VALIO_MOTION_GYR_Y |
        VALIO_MOTION_GYR_Z,

    VALIO_MOTION_CMP_X = (1 << 6),
    VALIO_MOTION_CMP_Y = (1 << 7),
    VALIO_MOTION_CMP_Z = (1 << 8),

    VALIO_MOTION_CMP_XYZ = VALIO_MOTION_CMP_X | VALIO_MOTION_CMP_Y |
        VALIO_MOTION_CMP_Z,
};

enum valio_motion_event_e
{
    /* Activity event */
    VALIO_MOTION_ACC_ACT    = (1 << 0),
    /* Inactivity event */
    VALIO_MOTION_ACC_INACT  = (1 << 1),
    /* Tap event */
    VALIO_MOTION_ACC_TAP    = (1 << 2),
    /* Double tap event */
    VALIO_MOTION_ACC_DBLTAP = (1 << 2),
    /* Free fall event */
    VALIO_MOTION_ACC_FF     = (1 << 3),
    /* Activity event */
    VALIO_MOTION_GYR_ACT    = (1 << 4),
    /* Activity event */
    VALIO_MOTION_CMP_ACT    = (1 << 5),
};

struct valio_motion_thresh_s
{
    /* Axis selection mask. */
    uint16_t axis:9;

    /* Minimum duration in us for the event to be emitted. When 0, any event will be 
       emitted */
    uint32_t duration;

    /* Threshold values in mg. If device use the same threshold on the three
       axis, @tt x, @tt y and @tt z must be eguals */ 
    int16_t  x;
    int16_t  y;
    int16_t  z;
};

/* Capabilities */
enum valio_motion_caps_e
{
    VALIO_MOTION_CAP_ACC_ACT    = (1 << 0),
    VALIO_MOTION_CAP_ACC_INACT  = (1 << 1),
    VALIO_MOTION_CAP_ACC_TAP    = (1 << 2),
    VALIO_MOTION_CAP_ACC_DBLTAP = (1 << 3),
    VALIO_MOTION_CAP_ACC_FF     = (1 << 4),

    /* Gyroscope caps */
    VALIO_MOTION_CAP_GYR_ACT    = (1 << 5),

    /* Compass caps */
    VALIO_MOTION_CAP_CMP_ACT    = (1 << 6),
};

struct valio_motion_caps_s
{
    uint16_t mask;
};

enum valio_motion_accel_opt_e
{
    VALIO_MOTION_ACC_OPT_ACT    = (1 << 0),
    VALIO_MOTION_ACC_OPT_INACT  = (1 << 1),
    VALIO_MOTION_ACC_OPT_TAP    = (1 << 2),
    VALIO_MOTION_ACC_OPT_DBLTAP = (1 << 3),
    VALIO_MOTION_ACC_OPT_FF     = (1 << 4),
};

struct valio_motion_accel_s
{
    /* A valio_motion_accel_opt_e selection mask. */
    uint8_t mask:4;

    struct valio_motion_thresh_s act;
    struct valio_motion_thresh_s inact;

    struct {
        /* Single tap */
        uint16_t axis;
        uint16_t duration; /* in us */

        /* Double tap */
        uint16_t latency; /* in us */
        uint16_t window;  /* in us */
    } tap;

    struct valio_motion_thresh_s ffall;
};

enum valio_motion_gyro_opt_e
{
    VALIO_MOTION_GYR_OPT_MOVE = (1 << 0),
};

struct valio_motion_gyro_s
{
    /* Option selection mask. If the mask value is zero, then the gyroscope
       is disabled. */
    uint8_t                      mask:1;
    struct valio_motion_thresh_s move;
};

enum valio_motion_comp_opt_e
{
    VALIO_MOTION_CMP_OPT_GAIN = (1 << 0),
};

struct valio_motion_comp_s
{
    /* Option selection mask. If the mask value is zero, then the compass
       is disabled. */
    uint8_t  mask:1;
    uint16_t gain;
};

struct valio_motion_axis_data_s
{
    /* This mask is set according to motion detection on each axis. If device
       is not able to differenciate move on each axes, driver must set all axis bits
       to 1 and return each value of axis */
    uint16_t axis:9;

    /** The value of each axis. */
    int16_t x;
    int16_t y;
    int16_t z;
};

struct valio_motion_data_s
{
    /* Accelerometer. */
    struct valio_motion_axis_data_s accel;
    /* Gyroscope */
    struct valio_motion_axis_data_s gyro;
    /* Compass (magnetometer). */
    struct valio_motion_axis_data_s comp;
};

struct valio_motion_evt_s
{
    /* @tt evts and @tt revts are valio_motion_event_e masks. @tt evts specifies which
       event are monitored. The request is terminated when at least one of these event
       occurs. @tt revts returns a mask specifying which event has occured */

    uint16_t                   evts:9;
    uint16_t                   revts:9;

    struct valio_motion_data_s data;
};

#endif


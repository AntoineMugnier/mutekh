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
    /* A struct valio_motion_caps_s that defines the capability of the device,
       including what motion sensors are available. */
    VALIO_MOTION_CAPS,

    /** A struct valio_motion_accel_s that defines the configuration of an
        accelerometer. */
    VALIO_MOTION_ACCEL,

    /** A struct valio_motion_axis_data_s that defines the calibration offset
        of each X, Y and Z axis of the accelerometer. */
    VALIO_MOTION_ACCEL_OFST,

    /** A struct valio_motion_gyro_s that defines the configuration of a
        gyroscope. */
    VALIO_MOTION_GYRO,

    /** A struct valio_motion_comp_s that defines the configuration of a
        compass (i.e. magnetometer). */
    VALIO_MOTION_COMP,

    /** A struct valio_motion_data_s that defines the value of each of the X, Y
        Z axis. */
    VALIO_MOTION_DATA,
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
    VALIO_MOTION_ACC_MOVE,
    VALIO_MOTION_ACC_TAP,
    VALIO_MOTION_ACC_FF,
    VALIO_MOTION_GYR_MOVE,
    VALIO_MOTION_CMP_MOVE,
};

struct valio_motion_thresh_s
{
    /* Axis selection mask. */
    uint16_t axis:9;

    /* Minimum duration for the event to be emitted. */
    uint16_t duration;

    /* Threshold values */
    int16_t  x;
    int16_t  y;
    int16_t  z;
};

/* Capabilities */
enum valio_motion_caps_e
{
    VALIO_MOTION_CAP_ACC        = (1 << 0),
    VALIO_MOTION_CAP_ACC_MOVE   = (1 << 1),
    VALIO_MOTION_CAP_ACC_TAP    = (1 << 2),
    VALIO_MOTION_CAP_ACC_DBLTAP = (1 << 3),
    VALIO_MOTION_CAP_ACC_FF     = (1 << 4),

    /* Gyroscope caps */
    VALIO_MOTION_CAP_GYR        = (1 << 5),
    VALIO_MOTION_CAP_GYR_MOVE   = (1 << 6),

    /* Compass caps */
    VALIO_MOTION_CAP_CMP        = (1 << 7),
    VALIO_MOTION_CAP_CMP_MOVE   = (1 << 8),
};

struct valio_motion_caps_s
{
    uint16_t mask;
};

enum valio_motion_accel_opt_e
{
    VALIO_MOTION_ACC_OPT_MOVE   = 0x1,
    VALIO_MOTION_ACC_OPT_TAP    = 0x2,
    VALIO_MOTION_ACC_OPT_DBLTAP = 0x4,
    VALIO_MOTION_ACC_OPT_FF     = 0x8,
};

struct valio_motion_accel_s
{
    /* Option selection mask. If the mask value is zero, then the accelerometer
       is disabled. */
    uint8_t mask:4;

    struct valio_motion_thresh_s move;

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
    /* Option selection mask. If the mask value is zero, then the accelerometer
       is disabled. */
    uint8_t  mask:1;
    uint16_t gain;
};

struct valio_motion_axis_data_s
{
    /* This mask selects the valid axis data in the structure. */
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
    uint16_t                   evts:9;
    struct valio_motion_data_s data;
};

#endif


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

    Copyright Sebastien Cerdan <sebcerdan@gmail.com> (c) 2018

    API for Bitbanging.
*/

/**
   @file
   @module {Core::Devices support library}
   @short IO bitbang driver API
   @index {IO bitbang} {Device classes}
   @csee DRIVER_CLASS_BITBANG

   @section {Purpose}

   This class allows generation and sampling of a digital signal on an
   IO line. The waveform is defined by an array containing duration
   of high and low levels of the signal.

   The hardware may support start of operation on a trigger. The input
   line used as a trigger may be either the signal input line or an
   alternate input line.

   Drivers which implement this class must check for a @ref
   DEV_RES_IOMUX resource named @tt {io}. When the trigger feature is
   implemented, an other such resource named @tt {trig} must be present.

   @end section
*/

#ifndef __DEVICE_BITBANG_H__
#define __DEVICE_BITBANG_H__

#include <mutek/kroutine.h>
#include <hexo/types.h>
#include <hexo/error.h>

#include <device/driver.h>
#include <device/class/timer.h>
#include <device/request.h>

#include <hexo/enum.h>

struct device_bitbang_s;

typedef uint16_t dev_bitbang_symbol_delay_t;

/***************************************** requests */

/** @internal */
#define _DEV_BITBANG_READ 0
/** @internal */
#define _DEV_BITBANG_WRITE 1

/** @internal start immediately */
#define _DEV_BITBANG_TRIG_IMMEDIATE 0
/** @internal start on rising edge */
#define _DEV_BITBANG_TRIG_RISING 4
/** @internal start on falling edge */
#define _DEV_BITBANG_TRIG_FALLING 2
/** @internal start on any edge */
#define _DEV_BITBANG_TRIG_ANYEDGE 6

/** @internal trig on an alternate pin instead of using the pin used
    for signal acquisition. */
#define _DEV_BITBANG_TRIG_ALTERNATE 8

/** @This specifies the various bigbang operations. */
enum dev_bitbang_rq_rtype_e
{
  /** The bitbang pin is configured as output and a signal is
      generated immediately. The pin is configured back as input when
      finished. */
  DEV_BITBANG_WR = _DEV_BITBANG_WRITE | _DEV_BITBANG_TRIG_IMMEDIATE,
  /** The alternate trig pin is monitored for rising edge, then the
      bitbang pin is configured as output and a signal is
      generated. The pin is configured back as input when
      finished. */
  DEV_BITBANG_WR_ALT_RISING = _DEV_BITBANG_WRITE | _DEV_BITBANG_TRIG_RISING | _DEV_BITBANG_TRIG_ALTERNATE,
  /** The alternate trig pin is monitored for falling edge, then the
      bitbang pin is configured as output and a signal is
      generated. The pin is configured back as input when
      finished. */
  DEV_BITBANG_WR_ALT_FALLING = _DEV_BITBANG_WRITE | _DEV_BITBANG_TRIG_FALLING | _DEV_BITBANG_TRIG_ALTERNATE,
  /** The alternate trig pin is monitored for rising or falling edge,
      then the bitbang pin is configured as output and a signal is
      generated. The pin is configured back as input when
      finished. */
  DEV_BITBANG_WR_ALT_ANYEDGE = _DEV_BITBANG_WRITE | _DEV_BITBANG_TRIG_ANYEDGE | _DEV_BITBANG_TRIG_ALTERNATE,
  /** A signal is acquired immediately. The waveform of the signal is
      stored in the array of symbols. */
  DEV_BITBANG_RD = _DEV_BITBANG_READ | _DEV_BITBANG_TRIG_IMMEDIATE,
  /** A signal is acquired when a rising edge is detected on the
      signal pin. The waveform of the signal is stored in the array of
      symbols. */
  DEV_BITBANG_RD_RISING = _DEV_BITBANG_READ | _DEV_BITBANG_TRIG_RISING,
  /** A signal is acquired when a falling edge is detected on the
      signal pin. The waveform of the signal is stored in the array of
      symbols. */
  DEV_BITBANG_RD_FALLING = _DEV_BITBANG_READ | _DEV_BITBANG_TRIG_FALLING,
  /** A signal is acquired when a rising or falling edge is detected
      on the signal pin. The waveform of the signal is stored in the
      array of symbols. */
  DEV_BITBANG_RD_ANYEDGE = _DEV_BITBANG_READ | _DEV_BITBANG_TRIG_ANYEDGE,
  /** A signal is acquired when a rising edge is detected on the
      alternate trig pin. The waveform of the signal is stored in the
      array of symbols. */
  DEV_BITBANG_RD_ALT_RISING = _DEV_BITBANG_READ | _DEV_BITBANG_TRIG_RISING | _DEV_BITBANG_TRIG_ALTERNATE,
  /** A signal is acquired when a falling edge is detected on the
      alternate trig pin. The waveform of the signal is stored in the
      array of symbols. */
  DEV_BITBANG_RD_ALT_FALLING = _DEV_BITBANG_READ | _DEV_BITBANG_TRIG_FALLING | _DEV_BITBANG_TRIG_ALTERNATE,
  /** A signal is acquired when a rising or falling edge is detected
      on the alternate trig pin. The waveform of the signal is stored
      in the array of symbols. */
  DEV_BITBANG_RD_ALT_ANYEDGE = _DEV_BITBANG_READ | _DEV_BITBANG_TRIG_ANYEDGE | _DEV_BITBANG_TRIG_ALTERNATE,
};

/** @This specifies the integer type used to store symbols for a
    bitbang request. */
enum dev_bitbang_sym_width_e
{
  /** Symbols are stored as @tt uint8_t */
  DEV_BITBANG_8BITS,
  /** Symbols are stored as @tt uint16_t */
  DEV_BITBANG_16BITS,
  /** Symbols are stored as @tt uint32_t */
  DEV_BITBANG_32BITS,
};

struct dev_bitbang_rq_s
{
  struct dev_request_s               base;

  /** @This specifies the operation to perform */
  enum dev_bitbang_rq_rtype_e        type;

  /** This specifies the type of integer used in the @tt symbols array. */
  enum dev_bitbang_sym_width_e       sym_width;

  /** Request completion error, set by the driver */
  error_t                            err;

  /** Number of symbols in the array. This must initialy be set to the
     size of the symbols array. When the request type is a read, this
     field is updated by the driver with the actual number of symbols
     stored. */
  uint16_t                           count;

  /** Array of symbols. The waveform of the signal is defined by the
      content of this array.

      Each value in this array specifies the duration of a
      level. Values at even indices in the array specifies the
      duration of the logic low levels of the signal whereas values at
      odd indices specifies the duration of the logic high levels. The
      value of the first entry may be 0 indicating a null duration. */
  void                              *symbols;

  /** A request waiting for a trigger is ended when no edge is detected for
      the specified duration. No timeout is used when this field is 0. */
  uint32_t                          trig_timeout;

  /** A running read request is ended when no change is detected for
      the specified duration. No timeout is used when this field is 0. */
  uint32_t                          read_timeout;

  /** This defines the base time unit used for symbol duration. */
  struct dev_freq_s                  unit;
};

DEV_REQUEST_INHERIT(bitbang); DEV_REQUEST_QUEUE_OPS(bitbang);

/** @see dev_bitbang_request_t */
#define DEV_BITBANG_REQUEST(n)	void  (n) (const struct device_bitbang_s *accessor,\
                                           struct dev_bitbang_rq_s * rq)

typedef DEV_BITBANG_REQUEST(dev_bitbang_request_t);

/** @see dev_bitbang_cancel_t */
#define DEV_BITBANG_CANCEL(n)  error_t (n) (const struct device_bitbang_s *accessor,\
                                            struct dev_bitbang_rq_s * rq)

typedef DEV_BITBANG_CANCEL(dev_bitbang_cancel_t);

/******************************************/

DRIVER_CLASS_TYPES(DRIVER_CLASS_BITBANG, bitbang,
                   dev_bitbang_request_t *f_request;
                   dev_bitbang_cancel_t *f_cancel;
                  );

/** @see driver_bitbang_s */
#define DRIVER_BITBANG_METHODS(prefix)                               \
  ((const struct driver_class_s*)&(const struct driver_bitbang_s){   \
    .class_ = DRIVER_CLASS_BITBANG,                                  \
    .f_request = prefix ## _request,                                 \
    .f_cancel = prefix ## _cancel                                    \
  })

#endif

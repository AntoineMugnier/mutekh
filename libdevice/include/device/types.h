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

    Copyright Alexandre Becoulet <alexandre.becoulet@lip6.fr> (c) 2013-2014

*/

#ifndef __DEVICE_TYPES_H__
#define __DEVICE_TYPES_H__

/**
 * @file
 * @module{Devices support library}
 * @short Device related types
 */

#include <hexo/types.h>

ENUM_DESCRIPTOR(dev_pin_driving_e, strip:DEV_PIN_, upper);

/** @This specifies IO signal driving modes */
enum dev_pin_driving_e
{
  DEV_PIN_DISABLED              = 0,
  DEV_PIN_ENABLED_              = 1,

  DEV_PIN_RESISTOR_UP_          = 2,
  DEV_PIN_RESISTOR_DOWN_        = 4,
  DEV_PIN_DRIVE_UP_             = 8,
  DEV_PIN_DRIVE_DOWN_           = 16,

  /** Output (symbol: @tt{>}).
      @tt {DEV_PIN_ENABLED_ | DEV_PIN_DRIVE_UP_ | DEV_PIN_DRIVE_DOWN_} */
  DEV_PIN_PUSHPULL              = 25,

  /** Input (symbol: @tt{<}).
   @tt {DEV_PIN_ENABLED_} */
  DEV_PIN_INPUT                 = 1,
  /** Input with pull resistor, pull direction depends on output value
      (symbol: @tt{=}).
      @tt {DEV_PIN_ENABLED_ | DEV_PIN_RESISTOR_UP_ | DEV_PIN_RESISTOR_DOWN_} */
  DEV_PIN_INPUT_PULL            = 7,
  /** Input with pullupup resistor (symbol: @tt{+}.).
      @tt {DEV_PIN_ENABLED_ | DEV_PIN_RESISTOR_UP_} */
  DEV_PIN_INPUT_PULLUP          = 3,
  /** Input with pullupdown resistor (symbol: @tt{-}).
      @tt {DEV_PIN_ENABLED_ | DEV_PIN_RESISTOR_DOWN_} */
  DEV_PIN_INPUT_PULLDOWN        = 5,

  /** Open drain output (symbol: @tt{_}).
      @tt {DEV_PIN_ENABLED_ | DEV_PIN_DRIVE_DOWN_} */
  DEV_PIN_OPENDRAIN             = 17,
  /** Open source output (symbol: @tt{^}).
      @tt {DEV_PIN_ENABLED_ | DEV_PIN_DRIVE_UP_} */
  DEV_PIN_OPENSOURCE            = 9,
  /** Open drain output with pullup resistor (symbol: @tt{,}).
      @tt {DEV_PIN_ENABLED_ | DEV_PIN_DRIVE_DOWN_ | DEV_PIN_RESISTOR_UP_} */
  DEV_PIN_OPENDRAIN_PULLUP      = 19,
  /** Open source output with pulldown resistor (symbol: @tt{`}).
      @tt {DEV_PIN_ENABLED_ | DEV_PIN_DRIVE_UP_ | DEV_PIN_RESISTOR_DOWN_} */
  DEV_PIN_OPENSOURCE_PULLDOWN   = 13,
};

/** Clock frequency in Hz = num / denom. The invalid frequency is
    encoded by setting denom to 0.
    @see #DEV_FREQ_INVALID @see #DEV_FREQ_IS_VALID
*/
struct dev_freq_s
{
  uint64_t num:CONFIG_DEVICE_CLOCK_OSCN_WIDTH;
  uint64_t denom:64-CONFIG_DEVICE_CLOCK_OSCN_WIDTH;
};

/** @This encodes the invalid frequency value @see dev_freq_s. */
#define DEV_FREQ_INVALID ((struct dev_freq_s){ .denom = 0 })

/** @This tests if the frequency value is valid @see dev_freq_s. */
#define DEV_FREQ_IS_VALID(f) ((f).denom != 0)

/** Clock frequency accuracy in ppb.  The value in ppb is
    @em {(8 | m) * 2^(e-4)}.

    The invalid accuracy value is encoded by setting e to 0.
    @see #DEV_FREQ_ACC
    @see #DEV_FREQ_ACC_INVALID @see #DEV_FREQ_ACC_IS_VALID
*/
struct dev_freq_accuracy_s
{
  uint8_t m:3;
  uint8_t e:5;
};

/** @This encodes a frequency accuracy value @see dev_freq_accuracy_s */
#define DEV_FREQ_ACC(m_, e_) ((struct dev_freq_accuracy_s){ .e = e_, .m = m_ })

/** @This encodes the invalid frequency accuracy value @see dev_freq_accuracy_s */
#define DEV_FREQ_ACC_INVALID ((struct dev_freq_accuracy_s){ .e = 0 })

/** @This tests if the frequency accuracy value is valid @see dev_freq_accuracy_s */
#define DEV_FREQ_ACC_IS_VALID(a) ((a).e != 0)

ALWAYS_INLINE uint32_t dev_freq_acc_ppb(const struct dev_freq_accuracy_s *acc)
{
  uint32_t r = (acc->m | 8);
  if (acc->e >= 4)
    r <<= acc->e - 4;
  else
    r >>= 4 - acc->e;
  return r;
}

/** Clock ratio */
struct dev_freq_ratio_s
{
  uint32_t num:CONFIG_DEVICE_CLOCK_FRAC_WIDTH;
  uint32_t denom:CONFIG_DEVICE_CLOCK_FRAC_WIDTH;
};

#endif


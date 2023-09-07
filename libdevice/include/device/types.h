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
 * @module {Core::Devices support library}
 * @short Device related types
 */

#include <hexo/types.h>
#include <hexo/enum.h>

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

    Clock frequency accuracy in ppb.  The value in ppb is
    @em {(8 | m) * 2^(e-4)}.

    The invalid accuracy value is encoded by setting e to 0.

    @see #DEV_FREQ_ACC
    @see #DEV_FREQ_INVALID @see #DEV_FREQ_IS_VALID
    @see #DEV_FREQ_ACC_IS_VALID

    @code
e \ m         0         1         2         3         4         5         6         7
    0   <Inval>  0.56 ppb  0.62 ppb  0.69 ppb  0.75 ppb  0.81 ppb  0.88 ppb  0.94 ppb
    1  1.00 ppb  1.12 ppb  1.25 ppb  1.38 ppb  1.50 ppb  1.62 ppb  1.75 ppb  1.88 ppb
    2  2.00 ppb  2.25 ppb  2.50 ppb  2.75 ppb  3.00 ppb  3.25 ppb  3.50 ppb  3.75 ppb
    3  4.00 ppb  4.50 ppb  5.00 ppb  5.50 ppb  6.00 ppb  6.50 ppb  7.00 ppb  7.50 ppb
    4  8.00 ppb  9.00 ppb  10.0 ppb  11.0 ppb  12.0 ppb  13.0 ppb  14.0 ppb  15.0 ppb
    5  16.0 ppb  18.0 ppb  20.0 ppb  22.0 ppb  24.0 ppb  26.0 ppb  28.0 ppb  30.0 ppb
    6  32.0 ppb  36.0 ppb  40.0 ppb  44.0 ppb  48.0 ppb  52.0 ppb  56.0 ppb  60.0 ppb
    7  64.0 ppb  72.0 ppb  80.0 ppb  88.0 ppb  96.0 ppb   104 ppb   112 ppb   120 ppb
    8   128 ppb   144 ppb   160 ppb   176 ppb   192 ppb   208 ppb   224 ppb   240 ppb
    9   256 ppb   288 ppb   320 ppb   352 ppb   384 ppb   416 ppb   448 ppb   480 ppb
   10   512 ppb   576 ppb   640 ppb   704 ppb   768 ppb   832 ppb   896 ppb   960 ppb
   11  1.02 ppm  1.15 ppm  1.28 ppm  1.41 ppm  1.54 ppm  1.66 ppm  1.79 ppm  1.92 ppm
   12  2.05 ppm  2.30 ppm  2.56 ppm  2.82 ppm  3.07 ppm  3.33 ppm  3.58 ppm  3.84 ppm
   13  4.10 ppm  4.61 ppm  5.12 ppm  5.63 ppm  6.14 ppm  6.66 ppm  7.17 ppm  7.68 ppm
   14  8.19 ppm  9.22 ppm  10.2 ppm  11.3 ppm  12.3 ppm  13.3 ppm  14.3 ppm  15.4 ppm
   15  16.4 ppm  18.4 ppm  20.5 ppm  22.5 ppm  24.6 ppm  26.6 ppm  28.7 ppm  30.7 ppm
   16  32.8 ppm  36.9 ppm  41.0 ppm  45.1 ppm  49.2 ppm  53.2 ppm  57.3 ppm  61.4 ppm
   17  65.5 ppm  73.7 ppm  81.9 ppm  90.1 ppm  98.3 ppm   106 ppm   114 ppm   122 ppm
   18   131 ppm   147 ppm   163 ppm   180 ppm   196 ppm   212 ppm   229 ppm   245 ppm
   19   262 ppm   294 ppm   327 ppm   360 ppm   393 ppm   425 ppm   458 ppm   491 ppm
   20   524 ppm   589 ppm   655 ppm   720 ppm   786 ppm   851 ppm   917 ppm   983 ppm
   21  1048 ppm  1179 ppm  1310 ppm  1441 ppm  1572 ppm  1703 ppm  1835 ppm  1966 ppm
   22  2097 ppm  2359 ppm  2621 ppm  2883 ppm  3145 ppm  3407 ppm  3670 ppm  3932 ppm
   23  4194 ppm  4718 ppm  5242 ppm  5767 ppm  6291 ppm  6815 ppm  7340 ppm  7864 ppm
   24  8388 ppm  9437 ppm    1.05 %    1.15 %    1.26 %    1.36 %    1.47 %    1.57 %
   25    1.68 %    1.89 %    2.10 %    2.31 %    2.52 %    2.73 %    2.94 %    3.15 %
   26    3.36 %    3.77 %    4.19 %    4.61 %    5.03 %    5.45 %    5.87 %    6.29 %
   27    6.71 %    7.55 %    8.39 %    9.23 %    10.1 %    10.9 %    11.7 %    12.6 %
   28    13.4 %    15.1 %    16.8 %    18.5 %    20.1 %    21.8 %    23.5 %    25.2 %
   29    26.8 %    30.2 %    33.6 %    36.9 %    40.3 %    43.6 %    47.0 %    50.3 %
   30    53.7 %    60.4 %    67.1 %    73.8 %    80.5 %    87.2 %    94.0 %     100 %
   31     107 %     120 %     134 %     147 %     161 %     174 %     187 %     201 %
    @end code
*/
struct dev_freq_s
{
  uint64_t BITFIELD(num,CONFIG_DEVICE_CLOCK_OSCN_WIDTH);
  uint64_t BITFIELD(denom,CONFIG_DEVICE_CLOCK_OSCD_WIDTH);
  uint64_t BITFIELD(acc_m,3);
  uint64_t BITFIELD(acc_e,5);
};

/** @This encodes the invalid frequency value @see dev_freq_s. */
#define DEV_FREQ_INVALID ((struct dev_freq_s){ .denom = 0, .acc_e = 0 })

/** @This tests if the frequency value is valid @see dev_freq_s. */
#define DEV_FREQ_IS_VALID(f) ((f).denom != 0)

/** @This encodes a frequency accuracy value @see dev_freq_s */
#define DEV_FREQ_ACC(m_, e_) ((struct dev_freq_s){ .acc_e = e_, .acc_m = m_ })

/** @This encodes a frequency and its accuracy @see dev_freq_s */
#define DEV_FREQ(freq_num_, freq_denom_, acc_m_, acc_e_)                \
  ((struct dev_freq_s){                                                 \
    .num = freq_num_,                                                   \
    .denom = freq_denom_,                                               \
    .acc_e = acc_e_,                                                    \
    .acc_m = acc_m_                                                     \
  })

/** @This tests if the frequency accuracy value is valid @see dev_freq_s */
#define DEV_FREQ_ACC_IS_VALID(a) ((a).acc_e != 0)

ALWAYS_INLINE uint32_t dev_acc_ppb(uint_fast8_t m, uint_fast8_t e)
{
  uint32_t r = (m | 8);
  if (e >= 4)
    r <<= e - 4;
  else
    r >>= 4 - e;
  return r;
}

ALWAYS_INLINE uint32_t dev_freq_acc_ppb(const struct dev_freq_s *freq)
{
  return dev_acc_ppb(freq->acc_m, freq->acc_e);
}

ALWAYS_INLINE void dev_freq_acc_set(struct dev_freq_s *freq, uint8_t acc_m, uint8_t acc_e)
{
  freq->acc_m = acc_m;
  freq->acc_e = acc_e;
}

/** Clock ratio */
struct dev_freq_ratio_s
{
  uint32_t BITFIELD(num,CONFIG_DEVICE_CLOCK_FRAC_WIDTH);
  uint32_t BITFIELD(denom,CONFIG_DEVICE_CLOCK_FRAC_WIDTH);
};

#endif


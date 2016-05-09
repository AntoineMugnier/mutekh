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

    Copyright (c) Nicolas Pouillon <nipo@ssji.net> 2015
*/

/**
   @file
   @module {Core::Devices support library::Valio device attributes}
   @short Value IO interface for a ADCs
*/

#ifndef LIBDEVICE_VALIO_ADC_H_
#define LIBDEVICE_VALIO_ADC_H_

#include <device/class/valio.h>

enum valio_adc_att_e
{
  /**
     Value for this attribute is an array is a @ref valio_adc_group_s.
     Selected inputs are masked through @tt mask field.
   */
  VALIO_ADC_VALUE = CONFIG_DEVICE_VALIO_ADC_ATTRIBUTE_FIRST,

  /**
     Value for this attribute is an array of thresholds to set for
     each masked input.  Thresholds are used for WAIT_UPDATE requests,
     if supported.  Write-only.
   */
  VALIO_ADC_THRESHOLD,
};

/**
  Caller must provide a @tt value array large enough to contain all
  masked values.  Values are stored contiguously from @tt value[0],
  even if @tt mask is sparse.
 */
struct valio_adc_group_s
{
  uint16_t mask;
  int16_t value[0];
};

#endif

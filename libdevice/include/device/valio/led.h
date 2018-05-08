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

    Copyright Sebastien Cerdan <sebcerdan@gmail.com> (c) 2014
*/

/**
   @file
   @module {Core::Devices support library::Valio device attributes}
   @short Value IO interface for a single led
*/

#ifndef LIBDEVICE_VALIO_LED_H_
#define LIBDEVICE_VALIO_LED_H_

#include <device/class/valio.h>

/* This interface is used to drive mono, bi or tricolor leds. 
   The @tt lum defines the luminosity in range [0-255] for each
   emitter.

   For a RGB led:

     RED   : @tt lum[0]
     GREEN : @tt lum[1]
     BLUE  : @tt lum[2]
   
   For others leds, luminosity of each emitter fill up
   @tt lum starting from @tt lum[0].
   For example, luminosity of a mono led is defined by
   @tt luminosity[0].
*/

#define VALIO_LED_LUMINOSITY_STEP 256

#define VALIO_LED CONFIG_DEVICE_VALIO_RGB_LED_ATTRIBUTE_FIRST

/* structure for @tt DEVICE_VALIO_WRITE and @tt DEVICE_VALIO_READ
   request type */ 

struct valio_led_luminosity_s
{
  uint8_t lum[3];
};


#endif

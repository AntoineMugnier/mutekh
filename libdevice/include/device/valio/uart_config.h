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

    Copyright Nicolas Pouillon <nipo@ssji.net> (c) 2018
    Copyright Julien Peeters <contact@julienpeeters.net> (c) 2014
*/

/**
   @file
   @module {Core::Devices support library::Valio device attributes}
   @short UART configuration driver API
   @index {UART configuration} {Device classes}
   @csee DRIVER_CLASS_CHAR

   UART valio subclass only defines configuration parameters for UART
   devices.  Data path of UART devices goes through the Character
   device class.

   Parameters addresed by this class are:
   @list
   @item Baud rate,
   @item Character bit size,
   @item Character framing (Start, stop bits, parity),
   @item Line control (flow control)
   @end list

   Depending on character device, device may be configuring the
   hardware (like for actual serial port), or receiving external
   instructions (like for CDC-ACM devices).  Using valio allows the
   application to wait for changes on parameters for the latter.
 */

#ifndef __DEVICE_VALIO_UART_H__
#define __DEVICE_VALIO_UART_H__

#include <hexo/types.h>
#include <hexo/error.h>

#include <device/device.h>
#include <device/resources.h>

#include <device/resource/uart.h>

#include <device/class/valio.h>

enum valio_uart_config_att {
    /** A struct valio_uart_config_s */
    VALIO_UART_CONFIG = CONFIG_DEVICE_VALIO_UART_CONFIG_ATTRIBUTE_FIRST,
};

#endif

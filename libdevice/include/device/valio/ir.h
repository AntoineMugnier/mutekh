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

    Copyright (c) Nicolas Pouillon <nipo@ssji.net> 2016
*/

/**
   @file
   @module {Core::Devices support library::Valio device attributes}
   @short Value IO interface for an Infra-red Transceiver
*/

#ifndef LIBDEVICE_VALIO_IR_H_
#define LIBDEVICE_VALIO_IR_H_

#include <device/class/valio.h>

enum valio_ir_e
{
  VALIO_IR_COMMAND = CONFIG_DEVICE_VALIO_IR_ATTRIBUTE_FIRST,
};

enum valio_ir_encoding_e
{
  /**
     Bit count is fixed to 12 bits: 1 toggle, 5 address, 6 command
   */
  VALIO_IR_RC5,
  /**
     Bit count is in (12, 20, 28, 36): 3 mode, 1 toggle, n * 8 command
   */
  VALIO_IR_RC6,
};

struct valio_ir_command_s
{
  enum valio_ir_encoding_e type;
  uint64_t value;
  size_t bit_count;
};

#endif

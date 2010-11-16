/*
    This file is part of MutekH.

    MutekH is free software; you can redistribute it and/or modify it
    under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    MutekH is distributed in the hope that it will be useful, but
    WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with MutekH; if not, write to the Free Software Foundation,
    Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA

    Copyright (c) Eric Guthmuller, 2010

*/

#ifndef __ICU_CM3_PRIVATE_H_
#define __ICU_CM3_PRIVATE_H_

#include <device/icu.h>
#include <device/device.h>

struct icu_cm3_handler_s
{
  dev_irq_t		*hndl;
  void			*data;
};

#define ICU_CM3_MAX_VECTOR	240

struct icu_cm3_private_s
{
  struct icu_cm3_handler_s	*table;
  uint_fast16_t intlinesnum;
  uint_fast8_t virq_refcount;
};

#endif


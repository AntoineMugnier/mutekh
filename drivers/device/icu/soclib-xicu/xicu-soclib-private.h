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

    Copyright (c) UPMC, Lip6
         Alexandre Becoulet <alexandre.becoulet@lip6.fr>, 2006-2009
         Nicolas Pouillon <nipo@ssji.net>, 2009

*/

#ifndef __XICU_soclib_PRIVATE_H_
#define __XICU_soclib_PRIVATE_H_

#include <device/icu.h>
#include <device/device.h>

#define XICU_WTI_REG 0
#define XICU_PTI_PER 1
#define XICU_PTI_VAL 2
#define XICU_PTI_ACK 3
#define XICU_MSK_PTI 4
#define XICU_MSK_PTI_ENABLE 5
#define XICU_MSK_PTI_DISABLE 6
#define XICU_PTI_ACTIVE 6
#define XICU_MSK_HWI 8
#define XICU_MSK_HWI_ENABLE 9
#define XICU_MSK_HWI_DISABLE 10
#define XICU_HWI_ACTIVE 10
#define XICU_MSK_WTI 12
#define XICU_MSK_WTI_ENABLE 13
#define XICU_MSK_WTI_DISABLE 14
#define XICU_WTI_ACTIVE 14
#define XICU_PRIO 15

#define XICU_REG_ADDR(base, op, id) (uintptr_t)((uint32_t*)(base) + ((op)<<5) + (id))

#define XICU_PRIO_WTI(val) (((val) >> 24) & 0x1f)
#define XICU_PRIO_HWI(val) (((val) >> 16) & 0x1f)
#define XICU_PRIO_PTI(val) (((val) >> 8) & 0x1f)
#define XICU_PRIO_HAS_WTI(val) ((val) & 0x4)
#define XICU_PRIO_HAS_HWI(val) ((val) & 0x2)
#define XICU_PRIO_HAS_PTI(val) ((val) & 0x1)

#define XICU_MAX_HWI 32
#define XICU_MAX_PTI 32

struct xicu_soclib_handler_s
{
  dev_irq_t		*hndl;
  void			*data;
};

struct xicu_soclib_private_s
{
	struct device_s timer;
	struct xicu_soclib_handler_s hwi_handlers[XICU_MAX_HWI];
	struct xicu_soclib_handler_s ipi_handlers[XICU_MAX_HWI];
	size_t output_line_no;
};



struct xicu_soclib_timer_handler_s
{
  devtimer_callback_t		*hndl;
  void			*data;
};

bool_t
xicu_timer_soclib_irq(struct device_s *timer_dev, int_fast8_t no);

struct xicu_soclib_timer_private_s
{
	struct xicu_soclib_timer_handler_s pti_handlers[XICU_MAX_PTI];
	size_t output_line_no;
};

#endif


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

    Copyright Alexandre Becoulet <alexandre.becoulet@lip6.fr> (c) 2006

*/


#ifndef DRIVER_XICU_soclib_H_
#define DRIVER_XICU_soclib_H_

#include <device/timer.h>
#include <device/icu.h>
#include <device/device.h>

/* icu device functions */

DEV_INIT(xicu_soclib_init);
DEVICU_ENABLE(xicu_soclib_enable);
DEVICU_SETHNDL(xicu_soclib_sethndl);
DEVICU_DELHNDL(xicu_soclib_delhndl);
DEVICU_SENDIPI(xicu_soclib_sendipi);
DEVICU_SETUPIPI(xicu_soclib_setupipi);
DEV_CLEANUP(xicu_soclib_cleanup);

DEVTIMER_SETCALLBACK(xicu_timer_soclib_setcallback);
DEVTIMER_SETPERIOD(xicu_timer_soclib_setperiod);
DEVTIMER_SETVALUE(xicu_timer_soclib_setvalue);
DEVTIMER_GETVALUE(xicu_timer_soclib_getvalue);
//DEV_IRQ(xicu_timer_soclib_irq);
DEV_CLEANUP(xicu_timer_soclib_cleanup);
DEV_INIT(xicu_timer_soclib_init);

#define XICU_IRQ_IPI 0x20

struct soclib_xicu_param_s
{
	size_t output_line_no;
};

#endif


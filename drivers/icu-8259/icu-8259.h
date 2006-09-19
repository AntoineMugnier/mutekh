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


#ifndef DRIVER_ICU_8259_H_
#define DRIVER_ICU_8259_H_

#include <hexo/device/icu.h>
#include <hexo/device.h>

/* devices addresses slots */

#define ICU_ADDR_MASTER		0
#define ICU_ADDR_SLAVE		1

/* icu device functions */

DEV_INIT(icu_8259_init);
DEVICU_ENABLE(icu_8259_enable);
DEVICU_SETHNDL(icu_8259_sethndl);
DEVICU_DELHNDL(icu_8259_delhndl);
DEV_CLEANUP(icu_8259_cleanup);

#endif


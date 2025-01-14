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

    Copyright Alexandre Becoulet <alexandre.becoulet@lip6.fr> (c) 2006

*/


#ifndef DRIVER_ICU_8259_H_
#define DRIVER_ICU_8259_H_

#include <device/class/icu.h>
#include <device/device.h>

/* icu device functions */

DEV_INIT(icu_8259_init);
DEV_ICU_ENABLE(icu_8259_enable);
DEV_ICU_SETHNDL(icu_8259_sethndl);
DEV_ICU_DELHNDL(icu_8259_delhndl);
DEV_CLEANUP(icu_8259_cleanup);

#define ICU_8259_MAX_LINES 16

#endif


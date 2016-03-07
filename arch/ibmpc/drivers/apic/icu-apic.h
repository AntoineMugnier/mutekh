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


#ifndef DRIVER_ICU_apic_H_
#define DRIVER_ICU_apic_H_

#include <device/class/icu.h>
#include <device/device.h>

#include <hexo/local.h>

extern CPU_LOCAL struct device_s apic_dev;

/* icu device functions */

DEV_INIT(icu_apic_init);
DEV_ICU_ENABLE(icu_apic_enable);
DEV_ICU_SETHNDL(icu_apic_sethndl);
DEV_ICU_DELHNDL(icu_apic_delhndl);
DEV_CLEANUP(icu_apic_cleanup);

#endif


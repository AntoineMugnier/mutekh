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

    Copyright Nicolas Pouillon <nipo@ssji.net> (c) 2014
*/

#ifndef ARCH_NRF_POWER_H_
#define ARCH_NRF_POWER_H_

#include "peripheral.h"
#include "ids.h"

enum nrf5x_power_task {
    NRF_POWER_CONSTLAT = 30,
    NRF_POWER_LOWPWR = 31,
};

enum nrf5x_power_event {
    NRF_POWER_POFWARN = 2,
#if 52000 <= CONFIG_NRF5X_MODEL && CONFIG_NRF5X_MODEL <= 52999
    NRF_POWER_SLEEPENTER = 5,
    NRF_POWER_SLEEPEXIT = 6,
#endif
#if CONFIG_NRF5X_MODEL == 52840
    NRF_POWER_USBDETECTED = 7,
    NRF_POWER_USBREMOVED = 8,
    NRF_POWER_USBPWRRDY = 9,
#endif
};

enum nrf5x_power_register {
    NRF_POWER_RESETREAS = 0,
    NRF_POWER_RAMSTATUS = 10,
#if CONFIG_NRF5X_MODEL == 52840
    NRF_POWER_USBREGSTATUS = 14,
    NRF_POWER_MAINREGSTATUS = 144,
    NRF_POWER_DCDCEN0 = 96,
#endif
    NRF_POWER_SYSTEMOFF = 64,
    NRF_POWER_ENABLE = 65,
    NRF_POWER_POFCON = 66,
    NRF_POWER_GPREGRET = 67,
#if 52000 <= CONFIG_NRF5X_MODEL && CONFIG_NRF5X_MODEL <= 52999
    NRF_POWER_GPREGRET2 = 68,
#endif
#if CONFIG_NRF5X_MODEL <= 51999
    NRF_POWER_RAMON = 73,
    NRF_POWER_RAMONB = 85,
#endif
    NRF_POWER_RESET = 81,
    NRF_POWER_DCDCEN = 94,
    NRF_POWER_DCDCFORCE = 386,
#if 52000 <= CONFIG_NRF5X_MODEL && CONFIG_NRF5X_MODEL <= 52999
    NRF_POWER_RAM0_POWER = 320,
    NRF_POWER_RAM0_POWERSET = 321,
    NRF_POWER_RAM0_POWERCLR = 322,
# define NRF_POWER_RAM_POWER(x) (320 + (x) * 4)
# define NRF_POWER_RAM_POWERSET(x) (321 + (x) * 4)
# define NRF_POWER_RAM_POWERCLR(x) (322 + (x) * 4)

    // See PAN-108
    NRF_POWER_RAM_CURRENT = 697,
#endif
};

#define NRF_POWER_RESETREAS_RESETPIN 0x1
#define NRF_POWER_RESETREAS_RESETPIN_BIT 0
#define NRF_POWER_RESETREAS_DOG      0x2
#define NRF_POWER_RESETREAS_DOG_BIT 1
#define NRF_POWER_RESETREAS_SREQ     0x4
#define NRF_POWER_RESETREAS_SREQ_BIT 2
#define NRF_POWER_RESETREAS_LOCKUP   0x8
#define NRF_POWER_RESETREAS_LOCKUP_BIT 3
#define NRF_POWER_RESETREAS_OFF      0x10000
#define NRF_POWER_RESETREAS_OFF_BIT 16
#define NRF_POWER_RESETREAS_LPCOMP   0x20000
#define NRF_POWER_RESETREAS_LPCOMP_BIT 17
#define NRF_POWER_RESETREAS_DIF      0x40000
#define NRF_POWER_RESETREAS_DIF_BIT 18
#define NRF_POWER_RESETREAS_NFC      0x80000
#define NRF_POWER_RESETREAS_NFC_BIT 19
#define NRF_POWER_RESETREAS_VBUS      0x100000
#define NRF_POWER_RESETREAS_VBUS_BIT 20

#define NRF_POWER_SYSTEMOFF_ENTER 0x1

#define NRF_POWER_POFCON_ENABLE 1
#if CONFIG_NRF5X_MODEL <= 51999
# define NRF_POWER_POFCON_V21 0
# define NRF_POWER_POFCON_V23 2
# define NRF_POWER_POFCON_V25 4
# define NRF_POWER_POFCON_V27 6
#endif

#define NRF_POWER_DCDCEN_ENABLE 0x1

#define NRF_POWER_USBREGSTATUS_VBUSDETECT 0x1
#define NRF_POWER_USBREGSTATUS_OUTPUTRDY 0x2
#define NRF_POWER_MAINREGSTATUS_NORMAL 0x0
#define NRF_POWER_MAINREGSTATUS_HIGH 0x1

#endif

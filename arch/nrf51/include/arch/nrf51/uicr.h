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

#ifndef ARCH_NRF51_UICR_H_
#define ARCH_NRF51_UICR_H_

#define NRF51_UICR_BASE           0x10001000

#define NRF51_UICR_CLENR0         0x10001000
#define NRF51_UICR_RBPCONF        0x10001004
#define NRF51_UICR_XTALFREQ       0x10001008
#define NRF51_UICR_FWID           0x10001010
#define NRF51_UICR_BOOTLOADERADDR 0x10001014
#define NRF51_UICR_NRFFW(x)       (0x10001014 + 4 * (x))
#define NRF51_UICR_NRFHW(x)       (0x10001050 + 4 * (x))
#define NRF51_UICR_CUSTOMER(x)    (0x10001080 + 4 * (x))

#endif

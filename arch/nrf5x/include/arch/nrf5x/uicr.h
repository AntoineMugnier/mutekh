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

#ifndef ARCH_NRF_UICR_H_
#define ARCH_NRF_UICR_H_

#define NRF_UICR_BASE           0x10001000

#if 52000 <= CONFIG_NRF5X_MODEL && CONFIG_NRF5X_MODEL <= 52999
# define NRF_UICR_CLENR0         0x10001000
# define NRF_UICR_RBPCONF        0x10001004
# define NRF_UICR_XTALFREQ       0x10001008
# define NRF_UICR_FWID           0x10001010
#endif

#define NRF_UICR_BOOTLOADERADDR 0x10001014
#define NRF_UICR_NRFFW(x)       (0x10001014 + 4 * (x))
#define NRF_UICR_NRFHW(x)       (0x10001050 + 4 * (x))
#define NRF_UICR_CUSTOMER(x)    (0x10001080 + 4 * (x))

#if 52000 <= CONFIG_NRF5X_MODEL && CONFIG_NRF5X_MODEL <= 52999
# define NRF_UICR_PSELRESET(x) (0x10001200 + 4 * (x))
# define NRF_UICR_APPROTECT 0x10001208
# define NRF_UICR_NFCPINS 0x1000120C
# define NRF_UICR_NFCPINS_PROTECT_MASK 0x1
# define NRF_UICR_NFCPINS_PROTECT_NFC 0x1
# define NRF_UICR_NFCPINS_PROTECT_GPIO 0
#endif

#endif

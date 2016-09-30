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

#ifndef ARCH_NRF_FICR_H_
#define ARCH_NRF_FICR_H_

#define NRF_FICR_CODEPAGESIZE     0x10000010 // Bytes
#define NRF_FICR_CODESIZE         0x10000014 // Count of pages
#if defined(CONFIG_ARCH_NRF51)
# define NRF_FICR_CLENR0           0x10000028
# define NRF_FICR_PPFC             0x1000002C
# define NRF_FICR_NUMRAMBLOCK      0x10000034
# define NRF_FICR_SIZERAMBLOCKS(x) (0x10000038 + 4 * (x))
#endif
#define NRF_FICR_CONFIGID         0x1000005C
#define NRF_FICR_DEVICEID(x)      (0x10000060 + 4 * (x))
#define NRF_FICR_ER(x)            (0x10000080 + 4 * (x))
#define NRF_FICR_IR(x)            (0x10000090 + 4 * (x))
#define NRF_FICR_DEVICEADDRTYPE   0x100000A0
#define NRF_FICR_DEVICEADDR(x)    (0x100000A4 + 4 * (x))
#define NRF_FICR_OVERRIDEEN       0x100000AC

#define NRF_FICR_OVERRIDE_NRF_1MBIT       0
#define NRF_FICR_OVERRIDE_BLE_1MBIT       3

#define NRF_FICR_OVERRIDE(y, x)     (0x100000B0 + 4 * ((NRF_FICR_OVERRIDE_ ## y * 5) + x))

/* This actually exists in latest nRF51s as well */
#define NRF_FICR_INFO_PART       0x10000100
#define NRF_FICR_INFO_VARIANT    0x10000104
#define NRF_FICR_INFO_PACKAGE    0x10000108
#define NRF_FICR_INFO_RAM        0x1000010c
#define NRF_FICR_INFO_FLASH      0x10000110

# define NRF_FICR_RAM_BANKS  0x10000258

#if defined(CONFIG_ARCH_NRF52)
# define NRF_FICR_TEMP_A(x)  (0x10000404 + 4 * (x))
# define NRF_FICR_TEMP_B(x)  (0x1000041c + 4 * (x))
# define NRF_FICR_TEMP_T(x)  (0x10000434 + 4 * (x))

# define NRF_FICR_NFC_TAGHEADER0  0x10000450
# define NRF_FICR_NFC_TAGHEADER1  0x10000454
# define NRF_FICR_NFC_TAGHEADER2  0x10000458
# define NRF_FICR_NFC_TAGHEADER3  0x1000045c
#endif

#endif

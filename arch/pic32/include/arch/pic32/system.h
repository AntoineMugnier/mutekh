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

    Copyright (c) 2017 Nicolas Pouillon <nipo@ssji.net>
*/

#ifndef PIC32_SYSTEM_H_
#define PIC32_SYSTEM_H_

#include <hexo/decls.h>
#include <hexo/iospace.h>
#include <arch/pic32/devaddr.h>

ALWAYS_INLINE
void pic32_system_unlock(void)
{
  cpu_mem_write_32(PIC32_SYSKEY_ADDR, 0);
  cpu_mem_write_32(PIC32_SYSKEY_ADDR, 0xAA996655);
  cpu_mem_write_32(PIC32_SYSKEY_ADDR, 0x556699AA);
}

ALWAYS_INLINE
void pic32_system_lock(void)
{
  cpu_mem_write_32(PIC32_SYSKEY_ADDR, 0);
}

#endif

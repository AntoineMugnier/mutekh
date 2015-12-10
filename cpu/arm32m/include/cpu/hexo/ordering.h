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

    Copyright (c) 2015 Nicolas Pouillon <nipo@ssji.net>
*/

/**
   @file CPU memory ordering operations
*/

#if !defined(_HEXO_ORDERING_H_) || defined(CPU_ORDERING_H_)
#error This file can not be included directly
#else
#define CPU_ORDERING_H_

#if CONFIG_CPU_ARM32M_ARCH_VERSION >= 7

#define CPU_ORDER_MEM "dsb"

#define CPU_ORDER_WRITE "dsb"

#define CPU_ORDER_IO_MEM "dsb"

#define CPU_ORDER_IO_WRITE "dsb"

#endif

#endif

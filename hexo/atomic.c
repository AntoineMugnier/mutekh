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

    Copyright Alexandre Becoulet <alexandre.becoulet@free.fr> (c) 2017

*/

#include <hexo/atomic.h>

#if defined(__ATOMIC_CPUBASED)
__ATOMIC_CPU_PROTO(extern inline, atomic)

# ifndef HAS_CPU_ATOMIC_FAST8_ALIAS
__ATOMIC_CPU_PROTO(extern inline, atomic_fast8)
# endif

# ifndef HAS_CPU_ATOMIC_FAST16_ALIAS
__ATOMIC_CPU_PROTO(extern inline, atomic_fast16)
# endif
#endif

#if defined(__ATOMIC_LOCKBASED)
__ATOMIC_PROTO(extern inline, atomic);
__ATOMIC_PROTO(extern inline, atomic_fast8);
__ATOMIC_PROTO(extern inline, atomic_fast16);
#endif


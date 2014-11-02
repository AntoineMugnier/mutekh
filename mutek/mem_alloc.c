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

    Copyright Dimitri Refauvelet <dimitri.refauvelet@lip6.fr> (c) 2009
*/

#include <mutek/mem_alloc.h>

extern inline void *
mem_alloc_align(size_t size, size_t align, enum mem_scope_e scope);

extern inline void *
mem_alloc_cpu(size_t size, enum mem_scope_e scope, cpu_id_t cpu_id);


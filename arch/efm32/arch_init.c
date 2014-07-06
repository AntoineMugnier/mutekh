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

    Copyright Alexandre Becoulet <alexandre.becoulet@free.fr> (c) 2013

*/

#include <mutek/startup.h>

#include <string.h>

#include <mutek/mem_alloc.h>
#include <mutek/memory_allocator.h>

void efm32_mem_init()
{
    default_region = memory_allocator_init(NULL, (void*)CONFIG_STARTUP_HEAP_ADDR,
                                           (void*)(CONFIG_STARTUP_HEAP_ADDR +
                                                   CONFIG_STARTUP_HEAP_SIZE));
}


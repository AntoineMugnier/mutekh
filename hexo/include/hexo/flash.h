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

/**
  @file
  @module {Core::Hardware abstraction layer}
  @short Write access to memory mapped flash

  This low level API provides a way to erase and write pages in
  platform specific memory mapped flash memory.

  This simple API is not asynchronous and is not designed to access
  any flash devices. This might not be available on platforms other
  than microcontrollers.

  The @xref{Memory device} class is the right way to access any flash
  device in a portable way.
*/

#ifndef HEXO_FLASH_H_
#define HEXO_FLASH_H_

#include "types.h"
#include "error.h"

C_HEADER_BEGIN

/** @This erases a page in the platform memory mapped flash memory.
    The page size is platform specific.

    The function returns 0 on success. */
reg_t flash_page_erase(uintptr_t addr);

/** @This writes a page in the platform memory mapped flash memory.

    The function returns 0 on success. It should not fail when
    requested to flip bits from 0 to 1. */
reg_t flash_page_write(uintptr_t addr,
                       const uint8_t *data,
                       size_t size);
C_HEADER_END

#endif

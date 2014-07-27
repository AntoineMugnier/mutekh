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

    Copyright Alexandre Becoulet <alexandre.becoulet@free.fr> (c) 2014
*/

#ifndef _EFM32_FLASH_H_
#define _EFM32_FLASH_H_

#include <hexo/types.h>

/** @This erases a page of flash. The @tt msc_addr parameter must
    point to the Memory System Controller registers.

    The size of the page is device dependent. The return value
    contains the error bits of the MSC status register.

    @see #CONFIG_EFM32_FLASH_PAGE_SIZE
*/
uint32_t efm32_flash_erase(uintptr_t msc_addr, uintptr_t flash_addr);

/** @This writes data to a page of flash. The @tt msc_addr parameter
    must point to the Memory System Controller registers. The write
    operation can not span across multiple pages.

    The return value indicates if the new data in flash is different
    from the passed data buffer.

    @see #CONFIG_EFM32_FLASH_PAGE_SIZE
*/
uint32_t efm32_flash_write(uintptr_t msc_addr, uintptr_t flash_addr,
                           const uint32_t *data, uint32_t words_count);

#endif


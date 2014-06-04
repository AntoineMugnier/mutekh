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

    Copyright Julien Peeters <contact@julienpeeters.net> (c) 2014

*/

#ifndef _STM32F4xx_MEMORY_MAP_H_
#define _STM32F4xx_MEMORY_MAP_H_

#if defined(CONFIG_BOARD_STM32_F401RE)
# include <arch/stm32f401re_memory_map.h>
#else
# error Unknown STM32 board definition
#endif

#define STM32F4xx_DEV_MEM_START(dev, id) \
  STM32F4xx_##dev##id##_ADDR             \
/**/

#define STM32F4xx_DEV_MEM_END(dev, id)                                 \
  ( STM32F4xx_DEV_MEM_START(dev, id) + STM32F4xx_##dev##id##_SIZE - 1) \
/**/

#endif


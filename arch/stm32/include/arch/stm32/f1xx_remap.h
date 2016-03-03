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

    Copyright (c) 2015 Julien Peeters <contact@julienpeeters.net>

*/

#ifndef _STM32F1XX_REMAP_H_
#define _STM32F1XX_REMAP_H_

#include <arch/stm32/f1xx_afio.h>

/* configuration value for the I/O remapping.

   MSb (31) -----(16)|-------------------------------- LSb (0)
   11111111111111111 | VALUE(2) | MASK(2) | OFFSET(5) | REG(3)
 */
#define STM32_AFIO_REMAP(_reg, _offset, _mask, _value) \
  (((_reg)    & 0x7)          |                        \
  (((_offset) & 0x1f) <<  3)  |                        \
  (((_mask)   & 0x3)  <<  8)  |                        \
  (((_value)  & 0x3)  << 10)  |                        \
  (0xffff << 16))

#define STM32_AFIO_REMAP_REG_ADDR(_config) \
  ((uintptr_t)(_config & 0x7) * 4 + STM32_AFIO_MAPR_ADDR + STM32_AFIO_ADDR)

#define STM32_AFIO_REMAP_OFFSET(_config) ((uint32_t)((_config) >>  3) & 0x1f)
#define STM32_AFIO_REMAP_MASK(_config)   ((uint32_t)((_config) >>  8) & 0x03)
#define STM32_AFIO_REMAP_VALUE(_config)  ((uint32_t)((_config) >> 10) & 0x03)

#define STM32_AFIO_REMAP_MAKE_MASK(_config) \
  (STM32_AFIO_REMAP_MASK(_config) << STM32_AFIO_REMAP_OFFSET(_config))

#define STM32_AFIO_REMAP_MAKE_VALUE(_config) \
  (STM32_AFIO_REMAP_VALUE(_config) << STM32_AFIO_REMAP_OFFSET(_config))

#define STM32_AFIO_REMAP_MAKE(_old, _config) \
  ((_old & ~STM32_AFIO_REMAP_MAKE_MASK(_config)) | STM32_AFIO_REMAP_MAKE_VALUE(_config))


/* Remap definitions for STM32 F1 boards. */

#define STM32_AFIO_REMAP_SPI1(_value)     STM32_AFIO_REMAP(0, 0, 0x1, _value)

#define STM32_AFIO_REMAP_I2C1(_value)     STM32_AFIO_REMAP(0, 1, 0x1, _value)

#define STM32_AFIO_REMAP_USART1(_value)   STM32_AFIO_REMAP(0, 2, 0x1, _value)
#define STM32_AFIO_REMAP_USART2(_value)   STM32_AFIO_REMAP(0, 3, 0x1, _value)
#define STM32_AFIO_REMAP_USART3(_value)   STM32_AFIO_REMAP(0, 4, 0x3, _value)

#define STM32_AFIO_REMAP_TIM1(_value)     STM32_AFIO_REMAP(0, 6, 0x3, _value)
#define STM32_AFIO_REMAP_TIM2(_value)     STM32_AFIO_REMAP(0, 8, 0x3, _value)
#define STM32_AFIO_REMAP_TIM3(_value)     STM32_AFIO_REMAP(0, 10, 0x3, _value)
#define STM32_AFIO_REMAP_TIM4(_value)     STM32_AFIO_REMAP(0, 12, 0x1, _value)
#define STM32_AFIO_REMAP_TIM5CH4I(_value) STM32_AFIO_REMAP(0, 16, 0x1, _value)
#define STM32_AFIO_REMAP_TIM9(_value)     STM32_AFIO_REMAP(5, 5, 0x1, _value)
#define STM32_AFIO_REMAP_TIM10(_value)    STM32_AFIO_REMAP(5, 6, 0x1, _value)
#define STM32_AFIO_REMAP_TIM11(_value)    STM32_AFIO_REMAP(5, 7, 0x1, _value)
#define STM32_AFIO_REMAP_TIM13(_value)    STM32_AFIO_REMAP(5, 8, 0x1, _value)
#define STM32_AFIO_REMAP_TIM14(_value)    STM32_AFIO_REMAP(5, 9, 0x1, _value)

#define STM32_AFIO_REMAP_CAN1(_value)     STM32_AFIO_REMAP(0, 13, 0x3, _value)

#define STM32_AFIO_REMAP_PD01(_value)     STM32_AFIO_REMAP(0, 15, 0x1, _value)

#endif


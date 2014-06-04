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

#ifndef _STM32F4xx_HELPERS_H
#define _STM32F4xx_HELPERS_H

#define __STM32F4xx_DEV_TYPED_PTR(base, type) \
  ( (type *) (base) )                         \
/**/

#define __STM32F4xx_REG_ADDR(base, offset) \
  ( (base) + (offset) )                    \
/**/

#define __STM32F4xx_REG_TYPED_PTR(base, offset, type) \
  ( (type *) __STM32F4xx_REG_ADDR(base, offset) )     \
/**/

#define __STM32F4xx_REG_TYPED(base, offset, type)   \
  ( * STM32F4xx_REG_TYPED_PTR(base, offset, type) ) \
/**/

#define __STM32F4xx_REG_32(base, offset)               \
  STM32F4xx_REG_TYPED(base, offset, uint32_t volatile) \
/**/

#define __STM32F4xx_REG_16(base, offset)               \
  STM32F4xx_REG_TYPED(base, offset, uint16_t volatile) \
/**/

#define __STM32F4xx_REG_8(base, offset)               \
  STM32F4xx_REG_TYPED(base, offset, uint8_t volatile) \
/**/

#define __STM32F4xx_REG_PASTE(a, b)     a ## b
#define __STM32F4xx_REG_PASTE_2(a, b)   __STM32F4xx_REG_PASTE(a, b)

#define __STM32F4xx_REG_PASTE_3(a, b, c)                    \
  __STM32F4xx_REG_PASTE_2(a, __STM32F4xx_REG_PASTE_2(b, c)) \
/**/

#define __STM32F4xx_REG_PASTE_4(a, b, c, d) \
  __STM32F4xx_REG_PASTE_2(                  \
    a,                                      \
    __STM32F4xx_REG_PASTE_3(b, c, d)        \
  )                                         \
/**/

#define __STM32F4xx_REG_PASTE_5(a, b, c, d, e) \
  __STM32F4xx_REG_PASTE_2(                     \
    a,                                         \
    __STM32F4xx_REG_PASTE_4(b, c, d, e)        \
  )                                            \
/**/

#define STM32F4xx_DEV_ADDR(dev, id)                      \
  __STM32F4xx_REG_PASTE_3(STM32F4xx, _##dev##id, _ADDR ) \
/**/

#define STM32F4xx_REG_ADDR_DEV(dev, base, reg)                \
  __STM32F4xx_REG_ADDR(                                       \
    (base),                                                   \
    __STM32F4xx_REG_PASTE_4(STM32F4xx, _##dev, _##reg, _ADDR) \
  )                                                           \
/**/

#define STM32F4xx_REG_ADDR(dev, id, reg)                        \
  STM32F4xx_REG_ADDR_DEV(dev, STM32F4xx_DEV_ADDR(dev, id), reg) \
/**/

#define STM32F4xx_REG_IDX_ADDR_DEV(dev, base, reg, ridx)            \
  __STM32F4xx_REG_ADDR(                                             \
    (base),                                                         \
    __STM32F4xx_REG_PASTE_4(STM32F4xx, _##dev, _##reg, _ADDR)(ridx) \
  )                                                                 \
/**/

#define STM32F4xx_REG_IDX_ADDR(dev, id, reg, ridx)                        \
  STM32F4xx_REG_IDX_ADDR_DEV(dev, STM32F4xx_DEV_ADDR(dev, id), reg, ridx) \
/**/

#define STM32F4xx_REG_MASK(dev, reg)                        \
  __STM32F4xx_REG_PASTE_4(STM32F4xx, _##dev, _##reg, _MASK) \
/**/

#define STM32F4xx_REG_VALUE_DEV(dev, base, reg)                          \
  endian_le32(cpu_mem_read_32(STM32F4xx_REG_ADDR_DEV(dev, (base), reg))) \
/**/

#define STM32F4xx_REG_IDX_VALUE_DEV(dev, base, reg, ridx)               \
  endian_le32(                                                          \
    cpu_mem_read_32(STM32F4xx_REG_IDX_ADDR_DEV(dev, (base), reg, ridx)) \
  )                                                                     \
/**/

#define STM32F4xx_REG_VALUE(dev, id, reg)                        \
  STM32F4xx_REG_VALUE_DEV(dev, STM32F4xx_DEV_ADDR(dev, id), reg) \
/**/

#define STM32F4xx_REG_IDX_VALUE(dev, id, reg, ridx)                        \
  STM32F4xx_REG_IDX_VALUE_DEV(dev, STM32F4xx_DEV_ADDR(dev, id), reg, ridx) \
/**/

#define STM32F4xx_REG_UPDATE_DEV(dev, base, reg, val) \
  cpu_mem_write_32(                                   \
    STM32F4xx_REG_ADDR_DEV(dev, (base), reg),         \
    endian_le32(val)                                  \
  )                                                   \
/**/

#define STM32F4xx_REG_IDX_UPDATE_DEV(dev, base, reg, ridx, val) \
  cpu_mem_write_32(                                             \
    STM32F4xx_REG_IDX_ADDR_DEV(dev, (base), reg, ridx),         \
    endian_le32(val)                                            \
  )                                                             \
/**/

#define STM32F4xx_REG_UPDATE(dev, id, reg, val)                        \
  STM32F4xx_REG_UPDATE_DEV(dev, STM32F4xx_DEV_ADDR(dev, id), reg, val) \
/**/

#define STM32F4xx_REG_IDX_UPDATE(dev, id, reg, ridx, val) \
  STM32F4xx_REG_IDX_UPDATE_DEV(                           \
    dev,                                                  \
    STM32F4xx_DEV_ADDR(dev, id),                          \
    reg,                                                  \
    ridx,                                                 \
    val                                                   \
  )                                                       \
/**/

#define STM32F4xx_REG_FIELD_MAKE(dev, reg, field, val)              \
  __STM32F4xx_REG_PASTE_4(STM32F4xx, _##dev, _##reg, _##field)(val) \
/**/

#define STM32F4xx_REG_FIELD_VALUE_DEV(dev, base, reg, field)          \
  __STM32F4xx_REG_PASTE_5(STM32F4xx, _##dev, _##reg, _##field, _GET)( \
    STM32F4xx_REG_VALUE_DEV(dev, (base), reg)                         \
  )                                                                   \
/**/

#define STM32F4xx_REG_IDX_FIELD_VALUE_DEV(dev, base, reg, ridx, field) \
  __STM32F4xx_REG_PASTE_5(STM32F4xx, _##dev, _##reg, _##field, _GET)(  \
    STM32F4xx_REG_IDX_VALUE_DEV(dev, (base), reg, ridx)                \
  )                                                                    \
/**/

#define STM32F4xx_REG_FIELD_VALUE(dev, id, reg, field)                        \
  STM32F4xx_REG_FIELD_VALUE_DEV(dev, STM32F4xx_DEV_ADDR(dev, id), reg, field) \
/**/

#define STM32F4xx_REG_IDX_FIELD_VALUE(dev, id, reg, ridx, field) \
  STM32F4xx_REG_IDX_FIELD_VALUE_DEV(                             \
    dev,                                                         \
    STM32F4xx_DEV_ADDR(dev, id),                                 \
    reg,                                                         \
    ridx,                                                        \
    field                                                        \
  )                                                              \
/**/

#define STM32F4xx_REG_FIELD_UPDATE_DEV(dev, base, reg, field, val)          \
  do                                                                        \
  {                                                                         \
    uint32_t register _reg = STM32F4xx_REG_VALUE_DEV(dev, (base), reg);     \
    STM32F4xx_REG_FIELD_UPDATE_VAR(dev, reg, field, val, _reg);             \
    STM32F4xx_REG_UPDATE_DEV(dev, (base), reg, _reg);                       \
  } while (0)                                                               \
/**/

#define STM32F4xx_REG_IDX_FIELD_UPDATE_DEV(dev, base, reg, ridx, field, val) \
  do                                                                         \
  {                                                                          \
    uint32_t register _reg =                                                 \
      STM32F4xx_REG_IDX_VALUE_DEV(dev, (base), reg, ridx);                   \
    STM32F4xx_REG_FIELD_UPDATE_VAR(dev, reg, field, val, _reg);              \
    STM32F4xx_REG_IDX_UPDATE_DEV(dev, (base), reg, ridx, _reg);              \
  } while (0)                                                                \
/**/

#define STM32F4xx_REG_FIELD_UPDATE(dev, id, reg, field, val) \
  STM32F4xx_REG_FIELD_UPDATE_DEV(                            \
    dev,                                                     \
    STM32F4xx_DEV_ADDR(dev, id),                             \
    reg,                                                     \
    field,                                                   \
    val                                                      \
  )                                                          \
/**/

#define STM32F4xx_REG_IDX_FIELD_UPDATE(dev, id, reg, ridx, field, val) \
  STM32F4xx_REG_IDX_FIELD_UPDATE_DEV(                                  \
    dev,                                                               \
    STM32F4xx_DEV_ADDR(dev, id),                                       \
    reg,                                                               \
    ridx,                                                              \
    field,                                                             \
    val                                                                \
  )                                                                    \
/**/

#define STM32F4xx_REG_FIELD_UPDATE_VAR(dev, reg, field, val, var)     \
  __STM32F4xx_REG_PASTE_5(STM32F4xx, _##dev, _##reg, _##field, _SET)( \
    (var),                                                            \
    val                                                               \
  )                                                                   \
/**/

#define STM32F4xx_REG_FIELD_SET_DEV(dev, base, reg, field)              \
  do                                                                    \
  {                                                                     \
    uint32_t register _reg = STM32F4xx_REG_VALUE_DEV(dev, (base), reg); \
    STM32F4xx_REG_FIELD_SET_VAR(dev, reg, field, _reg);                 \
    STM32F4xx_REG_UPDATE_DEV(dev, (base), reg, _reg);                   \
  } while (0)                                                           \
/**/

#define STM32F4xx_REG_IDX_FIELD_SET_DEV(dev, base, reg, ridx, field) \
  do                                                                 \
  {                                                                  \
    uint32_t register _reg =                                         \
      STM32F4xx_REG_IDX_VALUE_DEV(dev, (base), reg, ridx);           \
    STM32F4xx_REG_FIELD_SET_VAR(dev, reg, field, _reg);              \
    STM32F4xx_REG_IDX_UPDATE_DEV(dev, (base), reg, ridx, _reg);      \
  } while (0)                                                        \
/**/

#define STM32F4xx_REG_FIELD_SET(dev, id, reg, field)                        \
  STM32F4xx_REG_FIELD_SET_DEV(dev, STM32F4xx_DEV_ADDR(dev, id), reg, field) \
/**/

#define STM32F4xx_REG_IDX_FIELD_SET(dev, id, reg, ridx, field) \
  STM32F4xx_REG_IDX_FIELD_SET_DEV(                             \
    dev,                                                       \
    STM32F4xx_DEV_ADDR(dev, id),                               \
    reg,                                                       \
    ridx,                                                      \
    field                                                      \
  )                                                            \
/**/

#define STM32F4xx_REG_FIELD_SET_VAR(dev, reg, field, var)             \
  __STM32F4xx_REG_PASTE_5(STM32F4xx, _##dev, _##reg, _##field, _SET)( \
    (var),                                                            \
    1                                                                 \
  )                                                                   \
/**/

#define STM32F4xx_REG_FIELD_CLR_DEV(dev, base, reg, field)              \
  do                                                                    \
  {                                                                     \
    uint32_t register _reg = STM32F4xx_REG_VALUE_DEV(dev, (base), reg); \
    STM32F4xx_REG_FIELD_CLR_VAR(dev, reg, field, _reg);                 \
    STM32F4xx_REG_UPDATE_DEV(dev, (base), reg, _reg);                   \
  } while (0)                                                           \
/**/

#define STM32F4xx_REG_IDX_FIELD_CLR_DEV(dev, base, reg, ridx, field) \
  do                                                                 \
  {                                                                  \
    uint32_t register _reg =                                         \
      STM32F4xx_REG_IDX_VALUE_DEV(dev, (base), reg, ridx);           \
    STM32F4xx_REG_FIELD_CLR_VAR(dev, reg, field, _reg);              \
    STM32F4xx_REG_IDX_UPDATE_DEV(dev, (base), reg, ridx, _reg);      \
  } while (0)                                                        \
/**/

#define STM32F4xx_REG_FIELD_CLR(dev, id, reg, field)                        \
  STM32F4xx_REG_FIELD_CLR_DEV(dev, STM32F4xx_DEV_ADDR(dev, id), reg, field) \
/**/

#define STM32F4xx_REG_IDX_FIELD_CLR(dev, id, reg, ridx, field) \
  STM32F4xx_REG_IDX_FIELD_CLR_DEV(                             \
    dev,                                                       \
    STM32F4xx_DEV_ADDR(dev, id),                               \
    reg,                                                       \
    ridx,                                                      \
    field                                                      \
  )                                                            \
/**/

#define STM32F4xx_REG_FIELD_CLR_VAR(dev, reg, field, var)             \
  __STM32F4xx_REG_PASTE_5(STM32F4xx, _##dev, _##reg, _##field, _SET)( \
    (var),                                                            \
    0                                                                 \
  )                                                                   \
/**/

#define STM32F4xx_REG_FIELD_IDX_MAKE(dev, reg, field, fidx, val)          \
  __STM32F4xx_REG_PASTE_4(STM32F4xx, _##dev, _##reg, _##field)(fidx, val) \
/**/

#define STM32F4xx_REG_FIELD_IDX_VALUE_DEV(dev, base, reg, field, fidx) \
  __STM32F4xx_REG_PASTE_5(STM32F4xx, _##dev, _##reg, _##field, _GET)(  \
    fidx,                                                              \
    STM32F4xx_REG_VALUE_DEV(dev, (base), reg)                          \
  )                                                                    \
/**/

#define STM32F4xx_REG_IDX_FIELD_IDX_VALUE_DEV(                        \
    dev, base, reg, ridx, field, fidx)                                \
  __STM32F4xx_REG_PASTE_5(STM32F4xx, _##dev, _##reg, _##field, _GET)( \
    fidx,                                                             \
    STM32F4xx_REG_VALUE_DEV(dev, (base), reg, ridx)                   \
  )                                                                   \
/**/

#define STM32F4xx_REG_FIELD_IDX_VALUE(dev, id, reg, field, fidx) \
  STM32F4xx_REG_FIELD_IDX_VALUE_DEV(                             \
    dev,                                                         \
    STM32F4xx_DEV_ADDR(dev, id),                                 \
    reg,                                                         \
    field,                                                       \
    fidx                                                         \
  )                                                              \
/**/

#define STM32F4xx_REG_IDX_FIELD_IDX_VALUE(dev, id, reg, ridx, field, fidx) \
  STM32F4xx_REG_IDX_FIELD_IDX_VALUE_DEV(                                   \
    dev,                                                                   \
    STM32F4xx_DEV_ADDR(dev, id),                                           \
    reg,                                                                   \
    ridx,                                                                  \
    field,                                                                 \
    fidx                                                                   \
  )                                                                        \
/**/

#define STM32F4xx_REG_FIELD_IDX_UPDATE_DEV(dev, base, reg, field, fidx, val) \
  do                                                                         \
  {                                                                          \
    uint32_t register _reg = STM32F4xx_REG_VALUE_DEV(dev, (base), reg);      \
    STM32F4xx_REG_FIELD_IDX_UPDATE_VAR(dev, reg, field, fidx, val, _reg);    \
    STM32F4xx_REG_UPDATE_DEV(dev, (base), reg, _reg);                        \
  } while (0)                                                                \
/**/

#define STM32F4xx_REG_IDX_FIELD_IDX_UPDATE_DEV(                           \
    dev, base, reg, ridx, field, fidx, val)                               \
  do                                                                      \
  {                                                                       \
    uint32_t register _reg =                                              \
      STM32F4xx_REG_IDX_VALUE_DEV(dev, (base), reg, ridx);                \
    STM32F4xx_REG_FIELD_IDX_UPDATE_VAR(dev, reg, field, fidx, val, _reg); \
    STM32F4xx_REG_IDX_UPDATE_DEV(dev, (base), reg, ridx, _reg);           \
  } while (0)                                                             \
/**/

#define STM32F4xx_REG_FIELD_IDX_UPDATE(dev, id, reg, field, fidx, val) \
  STM32F4xx_REG_FIELD_IDX_UPDATE_DEV(                                  \
    dev,                                                               \
    STM32F4xx_DEV_ADDR(dev, id),                                       \
    reg,                                                               \
    field,                                                             \
    fidx,                                                              \
    val                                                                \
  ) \
/**/

#define STM32F4xx_REG_IDX_FIELD_IDX_UPDATE( \
    dev, id, reg, ridx, field, fidx, val)   \
  STM32F4xx_REG_IDX_FIELD_IDX_UPDATE_DEV(   \
    dev,                                    \
    STM32F4xx_DEV_ADDR(dev, id),            \
    reg,                                    \
    ridx,                                   \
    field,                                  \
    fidx,                                   \
    val                                     \
  )                                         \
/**/

#define STM32F4xx_REG_FIELD_IDX_UPDATE_VAR(dev, reg, field, fidx, val, var) \
  __STM32F4xx_REG_PASTE_5(STM32F4xx, _##dev, _##reg, _##field, _SET)(       \
    fidx,                                                                   \
    var,                                                                    \
    val                                                                     \
  )                                                                         \
/**/

#define STM32F4xx_REG_FIELD_IDX_SET_DEV(dev, base, reg, field, fidx)  \
  do                                                                  \
  {                                                                   \
    uint32_t register _reg = STM32F4xx_REG_VALUE_DEV(dev, (base), reg);        \
    STM32F4xx_REG_FIELD_IDX_SET_VAR(dev, reg, field, fidx, _reg);     \
    STM32F4xx_REG_UPDATE_DEV(dev, (base), reg, _reg);                 \
  } while (0)                                                         \
/**/

#define STM32F4xx_REG_IDX_FIELD_IDX_SET_DEV(                      \
    dev, base, reg, ridx, field, fidx)                            \
  do                                                              \
  {                                                               \
    uint32_t register _reg =                                      \
      STM32F4xx_REG_IDX_VALUE_DEV(dev, (base), reg, ridx);        \
    STM32F4xx_REG_FIELD_IDX_SET_VAR(dev, reg, field, fidx, _reg); \
    STM32F4xx_REG_IDX_UPDATE_DEV(dev, (base), reg, ridx, _reg);   \
  } while (0)                                                     \
/**/

#define STM32F4xx_REG_FIELD_IDX_SET(dev, id, reg, field, fidx) \
  STM32F4xx_REG_FIELD_IDX_SET_DEV(                             \
    dev,                                                       \
    STM32F4xx_DEV_ADDR(dev, id),                               \
    reg,                                                       \
    field,                                                     \
    fidx                                                       \
  )                                                            \
/**/

#define STM32F4xx_REG_IDX_FIELD_IDX_SET(dev, id, reg, ridx, field, fidx) \
  STM32F4xx_REG_IDX_FIELD_IDX_SET_DEV(                                   \
    dev,                                                                 \
    STM32F4xx_DEV_ADDR(dev, id),                                         \
    reg,                                                                 \
    ridx,                                                                \
    field,                                                               \
    fidx                                                                 \
  )                                                                      \
/**/

#define STM32F4xx_REG_FIELD_IDX_SET_VAR(dev, reg, field, fidx, var)   \
  __STM32F4xx_REG_PASTE_5(STM32F4xx, _##dev, _##reg, _##field, _SET)( \
    fidx,                                                             \
    (var),                                                            \
    1                                                                 \
  )                                                                   \
/**/

#define STM32F4xx_REG_FIELD_IDX_CLR_DEV(dev, base, reg, field, fidx)    \
  do                                                                    \
  {                                                                     \
    uint32_t register _reg = STM32F4xx_REG_VALUE_DEV(dev, (base), reg); \
    STM32F4xx_REG_FIELD_IDX_SET_VAR(dev, reg, field, fidx, _reg);       \
    STM32F4xx_REG_UPDATE_DEV(dev, (base), reg, _reg);                   \
  } while (0)                                                           \
/**/

#define STM32F4xx_REG_IDX_FIELD_IDX_CLR_DEV(                      \
    dev, base, reg, ridx, field, fidx)                            \
  do                                                              \
  {                                                               \
    uint32_t register _reg =                                      \
      STM32F4xx_REG_IDX_VALUE_DEV(dev, (base), reg, ridx);        \
    STM32F4xx_REG_FIELD_IDX_SET_VAR(dev, reg, field, fidx, _reg); \
    STM32F4xx_REG_IDX_UPDATE_DEV(dev, (base), reg, ridx, _reg);   \
  } while (0)                                                     \
/**/

#define STM32F4xx_REG_FIELD_IDX_CLR(dev, id, reg, field, fidx) \
  STM32F4xx_REG_FIELD_IDX_CLR_DEV(                             \
    dev,                                                       \
    STM32F4xx_DEV_ADDR(dev, id),                               \
    reg,                                                       \
    field,                                                     \
    fidx                                                       \
  )                                                            \
/**/

#define STM32F4xx_REG_IDX_FIELD_IDX_CLR(dev, id, reg, ridx, field, fidx) \
  STM32F4xx_REG_IDX_FIELD_IDX_CLR_DEV(                                   \
    dev,                                                                 \
    STM32F4xx_DEV_ADDR(dev, id),                                         \
    reg,                                                                 \
    ridx,                                                                \
    field,                                                               \
    fidx                                                                 \
  )                                                                      \
/**/

#define STM32F4xx_REG_FIELD_IDX_CLR_VAR(dev, reg, field, fidx, var)   \
  __STM32F4xx_REG_PASTE_5(STM32F4xx, _##dev, _##reg, _##field, _SET)( \
    fidx,                                                             \
    (var),                                                            \
    0                                                                 \
  )                                                                   \
/**/

#endif


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

    Copyright Nicolas Pouillon <nipo@ssji.net> (c) 2014
*/

#ifndef ARCH_NRF_PERIPHERAL_H_
#define ARCH_NRF_PERIPHERAL_H_

/**
   @file
   @short nRF51/nRF52 peripheral access
   @module {Platforms::nRF5X platform}

   Inside a device register map, there are tasks, events, shorts,
   interrupt handling, and then configuration registers.

   This regular pattern explains why nRF5x architecture declaration
   adds an overlay on top of @ref cpu_mem_read_32 and @ref
   cpu_mem_write_32 to access hardware registers.  nRF5x code use the
   following accessors:
   @list
     @item @ref nrf_task_trigger,
     @item @ref nrf_event_check,
     @item @ref nrf_event_clear,
     @item @ref nrf_event_wait,
     @item @ref nrf_event_wait_clear,
     @item @ref nrf_it_is_enabled,
     @item @ref nrf_it_set_mask,
     @item @ref nrf_it_enable,
     @item @ref nrf_it_enable_mask,
     @item @ref nrf_it_disable,
     @item @ref nrf_it_disable_mask,
     @item @ref nrf_short_set,
     @item @ref nrf_short_enable,
     @item @ref nrf_short_enable_mask,
     @item @ref nrf_short_disable,
     @item @ref nrf_short_disable_mask,
     @item @ref nrf_reg_get,
     @item @ref nrf_reg_set.
   @end list

   All these accessors take device base address and task/event/register
   number as arguments.  These accessors make the code obvious about
   what the driver intends to do.
 */

#include <hexo/iospace.h>
#include <hexo/bit.h>

/** @internal @multiple */
#define NRF_TASK     0x0
#define NRF_EVENT    0x100
#define NRF_SHORT    0x200
#define NRF_INTEN     0x300
#define NRF_INTENSET  0x304
#define NRF_INTENCLR  0x308
#define NRF_EVTEN    0x340
#define NRF_EVTENSET 0x344
#define NRF_EVTENCLR 0x348
#define NRF_REGISTER 0x400

/**
   @this triggers task @tt task for peripheral at base address @tt
   base.
 */
ALWAYS_INLINE
void nrf_task_trigger(uintptr_t base, uint8_t task)
{
  uintptr_t addr = base + NRF_TASK + (task << 2);
  cpu_mem_write_32(addr, 1);
}

/**
   @this checks whether interrupt @tt it for peripheral at base
   address @tt base is enabled.
 */
ALWAYS_INLINE
bool_t nrf_it_is_enabled(uintptr_t base, uint8_t it)
{
  uintptr_t addr = base + NRF_INTENSET;
  return bit_get(cpu_mem_read_32(addr), it);
}

/**
   @this updates the interrupt mask of device at base address @tt base
   with mask @tt mask.
 */
ALWAYS_INLINE
void nrf_it_set_mask(uintptr_t base, uint32_t mask)
{
  uintptr_t addr = base + NRF_INTEN;
  cpu_mem_write_32(addr, mask);
}

/**
   @this adds interrupts in mask @tt mask to enabled interrupt list for
   device at base address @tt addr.
 */
ALWAYS_INLINE
void nrf_it_enable_mask(uintptr_t base, uint32_t mask)
{
  if (__builtin_constant_p(mask) && mask == 0)
    return;

  uintptr_t addr = base + NRF_INTENSET;
  cpu_mem_write_32(addr, mask);
}

/**
   @this adds interrupt @tt it in interrupt list for device at base
   address @tt addr.
 */
ALWAYS_INLINE
void nrf_it_enable(uintptr_t base, uint8_t it)
{
  nrf_it_enable_mask(base, bit(it));
}

/**
   @this removes interrupts in mask @tt mask from enabled interrupt
   list for device at base address @tt addr.
 */
ALWAYS_INLINE
void nrf_it_disable_mask(uintptr_t base, uint32_t mask)
{
  if (__builtin_constant_p(mask) && mask == 0)
    return;

  if (__builtin_constant_p(mask) && mask == (uint32_t)-1) {
    nrf_it_set_mask(base, 0);
  } else {
    uintptr_t addr = base + NRF_INTENCLR;
    cpu_mem_write_32(addr, mask);
  }
}

/**
   @this removes interrupt @tt it from enabled interrupt list for
   device at base address @tt addr.
 */
ALWAYS_INLINE
void nrf_it_disable(uintptr_t base, uint8_t it)
{
  nrf_it_disable_mask(base, bit(it));
}

/**
   @this checks whether event @tt evt is enabled for peripheral at
   base address @tt base.
 */
ALWAYS_INLINE
bool_t nrf_evt_is_enabled(uintptr_t base, uint8_t evt)
{
  uintptr_t addr = base + NRF_EVTENSET;
  return bit_get(cpu_mem_read_32(addr), evt);
}

/**
   @this adds events in mask @tt mask to enabled event list for device
   at base address @tt addr.
 */
ALWAYS_INLINE
void nrf_evt_enable_mask(uintptr_t base, uint32_t mask)
{
  uintptr_t addr = base + NRF_EVTENSET;
  cpu_mem_write_32(addr, mask);
}

/**
   @this adds event @tt it in enabled event list for device at base
   address @tt addr.
 */
ALWAYS_INLINE
void nrf_evt_enable(uintptr_t base, uint8_t evt)
{
    nrf_evt_enable_mask(base, bit(evt));
}

/**
   @this removes events in mask @tt mask from enabled event list for
   device at base address @tt addr.
 */
ALWAYS_INLINE
void nrf_evt_disable_mask(uintptr_t base, uint32_t mask)
{
    uintptr_t addr = base + NRF_EVTENCLR;
    cpu_mem_write_32(addr, mask);
}

/**
   @this removes event @tt it from enabled event list for device at
   base address @tt addr.
 */
ALWAYS_INLINE
void nrf_evt_disable(uintptr_t base, uint8_t evt)
{
    nrf_evt_disable_mask(base, bit(evt));
}

/**
   @this retrieves value register @tt id in device at base address @tt
   addr.
 */
ALWAYS_INLINE
uint32_t nrf_reg_get(uintptr_t base, uint16_t id)
{
    uintptr_t addr = base + NRF_REGISTER + ((uintptr_t)id << 2);
    return cpu_mem_read_32(addr);
}

/**
   @this sets register @tt id to value @tt data in device at base
   address @tt addr.
 */
ALWAYS_INLINE
void nrf_reg_set(uintptr_t base, uint16_t id, uint32_t data)
{
    uintptr_t addr = base + NRF_REGISTER + ((uintptr_t)id << 2);
    cpu_mem_write_32(addr, data);
}

/**
   @this retrieves current short enable mask for device at base
   address @tt addr.
 */
ALWAYS_INLINE
uint32_t nrf_short_get(uintptr_t base)
{
    uintptr_t addr = base + NRF_SHORT;
    return cpu_mem_read_32(addr);
}

/**
   @this sets current short enable mask to @tt shorts for device at
   base address @tt addr.
 */
ALWAYS_INLINE
void nrf_short_set(uintptr_t base, uint32_t shorts)
{
    uintptr_t addr = base + NRF_SHORT;
    cpu_mem_write_32(addr, shorts);
}

/**
   @this adds shorts in mask @tt mask to current short enable mask for
   device at base address @tt addr.
 */
ALWAYS_INLINE
void nrf_short_enable_mask(uintptr_t base, uint32_t mask)
{
    uintptr_t addr = base + NRF_SHORT;
    cpu_mem_write_32(addr, mask | cpu_mem_read_32(addr));
}

/**
   @this adds short @tt id in current short enable mask for device at
   base address @tt addr.
 */
ALWAYS_INLINE
void nrf_short_enable(uintptr_t base, uint8_t id)
{
  nrf_short_enable_mask(base, bit(id));
}

/**
   @this checks whether short @tt id is enabled for device at base
   address @tt addr.
 */
ALWAYS_INLINE
bool_t nrf_short_is_enabled(uintptr_t base, uint8_t id)
{
    uintptr_t addr = base + NRF_SHORT;
    return bit_get(cpu_mem_read_32(addr), id);
}

/**
   @this removes shorts in mask @tt mask from current short enable
   mask for device at base address @tt addr.
 */
ALWAYS_INLINE
void nrf_short_disable_mask(uintptr_t base, uint32_t mask)
{
  if (__builtin_constant_p(mask) && mask == 0)
    return;

  if (__builtin_constant_p(mask) && mask == (uint32_t)-1) {
    nrf_short_set(base, 0);
  } else {
    uintptr_t addr = base + NRF_SHORT;
    cpu_mem_write_32(addr, ~mask & cpu_mem_read_32(addr));
  }
}

/**
   @this removes short @tt id from current short enable mask for
   device at base address @tt addr.
 */
ALWAYS_INLINE
void nrf_short_disable(uintptr_t base, uint8_t id)
{
  nrf_short_disable_mask(base, bit(id));
}

/**
   @this checks whether event @tt event is currently pending for
   device at base address @tt addr.
 */
ALWAYS_INLINE
uint32_t nrf_event_check(uintptr_t base, uint8_t event)
{
    uintptr_t addr = base + NRF_EVENT + (event << 2);
    return cpu_mem_read_32(addr);
}

/**
   @this acknowledges event @tt event for device at base address @tt
   addr.
 */
ALWAYS_INLINE
void nrf_event_clear(uintptr_t base, uint8_t event)
{
    uintptr_t addr = base + NRF_EVENT + (event << 2);
    cpu_mem_write_32(addr, 0);
}

/**
   @this busy-waits for event @tt event from device at base address
   @tt addr.
 */
ALWAYS_INLINE
void nrf_event_wait(uintptr_t base, uint8_t event)
{
#if 1
  nrf_it_disable(base, event);
  while (!nrf_event_check(base, event))
    ;
#else
  CPU_INTERRUPT_SAVESTATE_DISABLE;

  nrf_it_enable(base, event);
  while (!nrf_event_check(base, event))
    __asm__ __volatile__("wfi");
  nrf_it_disable(base, event);

  CPU_INTERRUPT_RESTORESTATE;
#endif
}

/**
   @this busy-waits for event @tt event from device at base address
   @tt addr and acknowledges it.
 */
ALWAYS_INLINE
void nrf_event_wait_clear(uintptr_t base, uint8_t event)
{
  nrf_event_wait(base, event);
  nrf_event_clear(base, event);
}

#endif

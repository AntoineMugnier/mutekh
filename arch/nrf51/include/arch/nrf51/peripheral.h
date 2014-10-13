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

#include <hexo/iospace.h>

#define NRF_TASK     0x0
#define NRF_EVENT    0x100
#define NRF_SHORT    0x200
#define NRF_ITEN     0x300
#define NRF_ITENSET  0x304
#define NRF_ITENCLR  0x308
#define NRF_EVTEN    0x340
#define NRF_EVTENSET 0x344
#define NRF_EVTENCLR 0x348
#define NRF_REGISTER 0x400

ALWAYS_INLINE
void nrf_task_trigger(uintptr_t base, uint8_t task)
{
    uintptr_t addr = base | NRF_TASK | (task << 2);
    cpu_mem_write_32(addr, 1);
}

ALWAYS_INLINE
bool_t nrf_it_is_enabled(uintptr_t base, uint8_t it)
{
    uintptr_t addr = base | NRF_ITENSET;
    return 1 & (cpu_mem_read_32(addr) >> it);
}

ALWAYS_INLINE
void nrf_it_set_mask(uintptr_t base, uint32_t mask)
{
    uintptr_t addr = base | NRF_ITEN;
    cpu_mem_write_32(addr, mask);
}

ALWAYS_INLINE
void nrf_it_enable_mask(uintptr_t base, uint32_t mask)
{
    uintptr_t addr = base | NRF_ITENSET;
    cpu_mem_write_32(addr, mask);
}

ALWAYS_INLINE
void nrf_it_enable(uintptr_t base, uint8_t it)
{
    nrf_it_enable_mask(base, 1 << it);
}

ALWAYS_INLINE
void nrf_it_disable_mask(uintptr_t base, uint32_t mask)
{
    uintptr_t addr = base | NRF_ITENCLR;
    cpu_mem_write_32(addr, mask);
}

ALWAYS_INLINE
void nrf_it_disable(uintptr_t base, uint8_t it)
{
    nrf_it_disable_mask(base, 1 << it);
}

ALWAYS_INLINE
bool_t nrf_evt_is_enabled(uintptr_t base, uint8_t evt)
{
    uintptr_t addr = base | NRF_EVTENSET;
    return 1 & (cpu_mem_read_32(addr) >> evt);
}

ALWAYS_INLINE
void nrf_evt_enable_mask(uintptr_t base, uint32_t mask)
{
    uintptr_t addr = base | NRF_EVTENSET;
    cpu_mem_write_32(addr, mask);
}

ALWAYS_INLINE
void nrf_evt_enable(uintptr_t base, uint8_t evt)
{
    nrf_evt_enable_mask(base, 1 << evt);
}

ALWAYS_INLINE
void nrf_evt_disable_mask(uintptr_t base, uint32_t mask)
{
    uintptr_t addr = base | NRF_EVTENCLR;
    cpu_mem_write_32(addr, mask);
}

ALWAYS_INLINE
void nrf_evt_disable(uintptr_t base, uint8_t evt)
{
    nrf_evt_disable_mask(base, 1 << evt);
}

ALWAYS_INLINE
uint32_t nrf_reg_get(uintptr_t base, uint16_t id)
{
    uintptr_t addr = base | NRF_REGISTER | ((uintptr_t)id << 2);
    return cpu_mem_read_32(addr);
}

ALWAYS_INLINE
void nrf_reg_set(uintptr_t base, uint16_t id, uint32_t data)
{
    uintptr_t addr = base | NRF_REGISTER | ((uintptr_t)id << 2);
    cpu_mem_write_32(addr, data);
}

ALWAYS_INLINE
uint32_t nrf_short_get(uintptr_t base)
{
    uintptr_t addr = base | NRF_SHORT;
    return cpu_mem_read_32(addr);
}

ALWAYS_INLINE
void nrf_short_set(uintptr_t base, uint32_t shorts)
{
    uintptr_t addr = base | NRF_SHORT;
    cpu_mem_write_32(addr, shorts);
}

ALWAYS_INLINE
void nrf_short_enable_mask(uintptr_t base, uint32_t mask)
{
    uintptr_t addr = base | NRF_SHORT;
    cpu_mem_write_32(addr, mask | cpu_mem_read_32(addr));
}

ALWAYS_INLINE
void nrf_short_enable(uintptr_t base, uint8_t id)
{
  nrf_short_enable_mask(base, 1 << id);
}

ALWAYS_INLINE
bool_t nrf_short_is_enabled(uintptr_t base, uint8_t id)
{
    uintptr_t addr = base | NRF_SHORT;
    return (cpu_mem_read_32(addr) >> id) & 1;
}

ALWAYS_INLINE
void nrf_short_disable_mask(uintptr_t base, uint32_t mask)
{
    uintptr_t addr = base | NRF_SHORT;
    cpu_mem_write_32(addr, ~mask & cpu_mem_read_32(addr));
}

ALWAYS_INLINE
void nrf_short_disable(uintptr_t base, uint8_t id)
{
  nrf_short_disable_mask(base, 1 << id);
}

ALWAYS_INLINE
uint32_t nrf_event_check(uintptr_t base, uint8_t task)
{
    uintptr_t addr = base | NRF_EVENT | (task << 2);
    return cpu_mem_read_32(addr);
}

ALWAYS_INLINE
void nrf_event_clear(uintptr_t base, uint8_t task)
{
    uintptr_t addr = base | NRF_EVENT | (task << 2);
    cpu_mem_write_32(addr, 0);
}

ALWAYS_INLINE
void nrf_event_wait(uintptr_t base, uint8_t evt)
{
#if 1
  nrf_it_disable(base, evt);
  while (!nrf_event_check(base, evt))
    ;
#else
  CPU_INTERRUPT_SAVESTATE_DISABLE;

  nrf_it_enable(base, evt);
  while (!nrf_event_check(base, evt))
    __asm__ __volatile__("wfi");
  nrf_it_disable(base, evt);

  CPU_INTERRUPT_RESTORESTATE;
#endif
}

ALWAYS_INLINE
void nrf_event_wait_clear(uintptr_t base, uint8_t evt)
{
  nrf_event_wait(base, evt);
  nrf_event_clear(base, evt);
}

#endif

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

#include <mutek/startup.h>

#include <string.h>

#include <mutek/mem_alloc.h>
#include <mutek/memory_allocator.h>

#include <arch/nrf51/clock.h>

void nrf51_mem_init()
{
    default_region = memory_allocator_init(
        NULL,
        (void*)CONFIG_STARTUP_HEAP_ADDR,
        (void*)(CONFIG_STARTUP_HEAP_ADDR + CONFIG_STARTUP_HEAP_SIZE));
}

void nrf51_clock_init()
{
    uintptr_t clock = nrf_peripheral_addr(NRF51_CLOCK);

#if defined(CONFIG_NRF51_LFCLK_XOSC)
    nrf_reg_set(clock, NRF51_CLOCK_LFCLKSRC, NRF51_CLOCK_LFCLKSTAT_SRC_XTAL);
// Cannot run synth until HF is started
#elif defined(CONFIG_NRF51_LFCLK_RC) || defined(CONFIG_NRF51_LFCLK_SYNTH)
    nrf_reg_set(clock, NRF51_CLOCK_LFCLKSRC, NRF51_CLOCK_LFCLKSTAT_SRC_RC);
//#elif defined(CONFIG_NRF51_LFCLK_SYNTH)
//    nrf_reg_set(clock, NRF51_CLOCK_LFCLKSRC, NRF51_CLOCK_LFCLKSTAT_SRC_SYNTH);
#else
# error "No LF clock declared"
#endif

    nrf_task_trigger(
        clock,
        NRF51_CLOCK_LFCLKSTART);

    while (!nrf_event_check(clock, NRF51_CLOCK_LFCLKSTARTED))
        ;

    nrf_task_trigger(clock, NRF51_CLOCK_HFCLKSTART);

    while (!nrf_event_check(clock, NRF51_CLOCK_HFCLKSTARTED))
        ;
}

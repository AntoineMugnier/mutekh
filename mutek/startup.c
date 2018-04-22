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

#include <hexo/cpu.h>
#include <hexo/types.h>
#include <hexo/lock.h>

#include <mutek/startup.h>
#include <mutek/printk.h>

#ifdef CONFIG_DEVICE
# include <device/device.h>
# include <device/class/cpu.h>
#endif

#include <stdlib.h>
#include <string.h>

////////////////////////////////////////////////////// copy .data

#ifdef CONFIG_LOAD_ROM
void section_data_init(void)
{
  extern __ldscript_symbol_t __data_start;
  extern __ldscript_symbol_t __data_load_start;
  extern __ldscript_symbol_t __data_load_end;

  memcpy((uint8_t*)&__data_start,
         (uint8_t*)&__data_load_start,
         (uint8_t*)&__data_load_end-(uint8_t*)&__data_load_start);
}
#endif

///////////////////////////////////////////////////// zero .bss

#if defined(CONFIG_LOAD_ROM) || defined(CONFIG_LOAD_BOOTLOAD)

void section_bss_init(void)
{
  extern __ldscript_symbol_t __bss_start;
  extern __ldscript_symbol_t __bss_end;

  memset((uint8_t*)&__bss_start, 0,
         (uint8_t*)&__bss_end-(uint8_t*)&__bss_start);
}

#endif

////////////////////////////////////////////////////// copy .excep

#ifdef CONFIG_LOAD_EXCEPTIONS_COPY

void section_excep_init()
{
  extern __ldscript_symbol_t CPU_NAME_DECL(exception_vector);
  extern __ldscript_symbol_t __exception_load_start;
  extern __ldscript_symbol_t __exception_load_end;

  memcpy((uint8_t*)&CPU_NAME_DECL(exception_vector),
         (uint8_t*)&__exception_load_start,
         (uint8_t*)&__exception_load_end - (uint8_t*)&__exception_load_start);
}

#endif

////////////////////////////////////////////////////// copy .excep

#ifdef CONFIG_LOAD_SMP_RESET_COPY

void section_smpreset_init()
{
  extern __ldscript_symbol_t CPU_NAME_DECL(smp_reset_vector);
  extern __ldscript_symbol_t __smp_reset_load_start;
  extern __ldscript_symbol_t __smp_reset_load_end;

  memcpy((uint8_t*)&CPU_NAME_DECL(smp_reset_vector),
         (uint8_t*)&__smp_reset_load_start,
         (uint8_t*)&__smp_reset_load_end - (uint8_t*)&__smp_reset_load_start);
}

#endif

/////////////////////////////////// smp startup barrier implementation

#ifdef CONFIG_ARCH_SMP

static uint_fast8_t startup_barrier_count;
static uint_fast8_t startup_barrier_init;
lock_t startup_barrier_lock;

void mutekh_startup_smp_barrier()
{
  if (startup_barrier_init == 2)  // only a single cpu ?
    return;

  lock_spin(&startup_barrier_lock);

  while (startup_barrier_count & 1) // wait for previous unlock end
    {
      // wait some time
      lock_release(&startup_barrier_lock);
      uint_fast8_t i = 255;
      while (i--)
        asm volatile ("nop");
      lock_spin(&startup_barrier_lock);
    }

  if (startup_barrier_count == 2)  // last cpu to enter ?
    {
      startup_barrier_count = 3;   // unlock others
    }
  else
    {
      startup_barrier_count -= 2;

      while (!(startup_barrier_count & 1)) // wait for unlock from last cpu
        {
          // wait some time
          lock_release(&startup_barrier_lock);
          uint_fast8_t i = 255;
          while (i--)
            asm volatile ("nop");
          lock_spin(&startup_barrier_lock);
        }

      startup_barrier_count += 2;

      if (startup_barrier_count > startup_barrier_init)  // last to leave ?
        {
          startup_barrier_count--;    // remove unlock flag
          assert(startup_barrier_count == startup_barrier_init);
        }
    }

  lock_release(&startup_barrier_lock);
}

void mutek_startup_barrier_init()
{
  /* init the smp startup barrier */
  startup_barrier_count = startup_barrier_init =
    device_get_cpu_count() * 2;
  assert(startup_barrier_count > 0);
}

#endif

////////////////////////////////////// actual startup main functions

#include <hexo/context.h>

void mutekh_startup(void *arg)
{
  /* call all bootstrap init functions */
  INIT_BOOTSTRAP_INIT();

#if defined(CONFIG_DEVICE_CPU)
  const struct cpu_tree_s *cpu = cpu_tree_lookup(CONFIG_ARCH_BOOTSTRAP_CPU_ID);
  assert(cpu != NULL && "processor id not found in the cpu tree.");

  /* use processor stack instead of startup stack from now */
  cpu_context_set(cpu->stack, CONFIG_HEXO_CPU_STACK_SIZE, &mutekh_startup_smp);
#else
  mutekh_startup_smp();
#endif
}

void mutekh_startup_smp()
{
  /* call smp init functions */
  INIT_SMP_INIT();

  abort();
}

#ifndef CONFIG_DEVICE
void mutekh_startup_nodev()
{
  INIT_DEVREADY_INIT();
}
#endif

/////////////////////////////////////// user application start

#ifdef CONFIG_APP_START
void mutek_app_initsmp()
{
  mutekh_startup_smp_barrier();

# ifndef CONFIG_APP_START_SMP
  if (cpu_isbootstrap())
# endif
    app_start();
}
#endif


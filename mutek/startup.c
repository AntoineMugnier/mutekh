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

#include <hexo/types.h>
#include <hexo/lock.h>

#include <mutek/startup.h>
#include <mutek/printk.h>

#include <device/device.h>

#include <stdlib.h>

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


void mutekh_startup(void *arg)
{
#ifdef CONFIG_ARCH_SMP
  if (!cpu_isbootstrap())
    return mutekh_startup_smp();
#endif

  /* call all bootstrap init functions */
  INIT_MUTEKH_STARTUP_INIT();

  abort();
}

void mutekh_startup_smp()
{
  /* call smp init functions */
  INIT_SMP_INIT();

  abort();
}

/////////////////////////////////////// user application start

void mutek_app_initsmp()
{
  mutekh_startup_smp_barrier();

  if (cpu_isbootstrap())
    {
      printk("MutekH is alive.\n");
      app_start();
    }
  else
    {
#ifdef CONFIG_MUTEK_SMP_APP_START
      app_start();
#endif
    }
}


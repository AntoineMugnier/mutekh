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

#include <mutek/startup.h>
#include <hexo/types.h>

#include <device/device.h>
#include <device/driver.h>

__compiler_sint_t __bootstrap_pid = 0;

void emu_cpus_enum_init()
{
  extern const struct driver_s emu_cpu_drv;
  size_t i;

  /* add bootstrap processor to device tree */
  __bootstrap_pid = emu_do_syscall(EMU_SYSCALL_GETPID, 0);
  struct device_s *d = device_alloc(1);
  device_res_add_id(d, __bootstrap_pid, 0);
  d->node.flags |= DEVICE_FLAG_CPU;
  device_attach(d, NULL);
  device_bind_driver(d, &emu_cpu_drv);

#ifdef CONFIG_ARCH_SMP
  /* add other processors to device tree */
  for (i = 1; i < CONFIG_CPU_MAXCOUNT; i++)
  {
    __compiler_sint_t pid;

    /* fork */
    pid = emu_do_syscall(EMU_SYSCALL_FORK, 0);
    if (pid < 0)
      {
        printk("error: unable to create more UNIX process to emulate processors");
        break;
      }

    if (pid)
      {
        struct device_s *d = device_alloc(1);
        device_res_add_id(d, pid, 0);
        d->node.flags |= DEVICE_FLAG_CPU;
        device_attach(d, NULL);
        device_bind_driver(d, &emu_cpu_drv);
      }
    else
      {
        /* non-bootstrap processors stop themselves */
        pid = emu_do_syscall(EMU_SYSCALL_GETPID, 0);
        emu_do_syscall(EMU_SYSCALL_KILL, 2, pid, EMU_SIG_STOP);

        return mutek_start_smp();
      }
  }
#endif
}

void emu_device_enum_init()
{
#if defined(CONFIG_DRIVER_CHAR_EMUTTY)
  extern const struct driver_s emu_tty_drv;
  static struct device_s tty_dev;
  device_init(&tty_dev);
  device_set_name(&tty_dev, "tty");
  device_attach(&tty_dev, NULL);
  device_bind_driver(&tty_dev, &emu_tty_drv);
#endif

#ifdef CONFIG_DRIVER_ICU_EMU
  static struct device_s icu_dev;
#endif

#ifdef CONFIG_DEVICE_TIMER
  static device_s timer_dev;
#endif

#ifdef CONFIG_DEVICE_BLOCK
  static device_s block_dev;
#endif
}


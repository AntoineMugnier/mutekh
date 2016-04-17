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

    Copyright (c) 2012 Alexandre Becoulet <alexandre.becoulet@telecom-paristech.fr>

*/

#include <hexo/types.h>
#include <hexo/local.h>
#include <mutek/startup.h>
#include <arch/hexo/emu_syscalls.h>

/////////////////////////////////////////////////////////////////////

#include <string.h>
#include <hexo/endian.h>

#ifdef CONFIG_ARCH_SMP
void emu_remap_shared_init(void)
{
  extern __ldscript_symbol_t __data_start /*, __data_end*/;
  extern __ldscript_symbol_t /*__bss_start, */__bss_end;

    uint8_t *data_start = ALIGN_ADDRESS_LOW(&__data_start, CONFIG_ARCH_EMU_PAGESIZE);
    uint8_t *bss_end = ALIGN_ADDRESS_UP(&__bss_end, CONFIG_ARCH_EMU_PAGESIZE);
    size_t size = bss_end - data_start;

    uint8_t copy[size];
    memcpy(copy, data_start, size);

    if ((void*)emu_do_syscall(EMU_SYSCALL_MMAP, 6, data_start, size,
            EMU_PROT_READ | EMU_PROT_WRITE,
            EMU_MAP_FIXED | EMU_MAP_SHARED | EMU_MAP_ANONYMOUS, -1, 0) == EMU_MAP_FAILED)
        emu_do_syscall(EMU_SYSCALL_EXIT, 1, 42);  
    
    memcpy(data_start, copy, size);
}
#endif

/////////////////////////////////////////////////////////////////////

#include <mutek/mem_alloc.h>
#include <mutek/mem_region.h>
#include <mutek/memory_allocator.h>

void emu_mem_init(void)
{
  void	*mem_start;
  void	*mem_end;


  mem_start = (void*)emu_do_syscall(EMU_SYSCALL_MMAP, 6, NULL, 
				    CONFIG_ARCH_EMU_MEMORY,
				    EMU_PROT_READ | EMU_PROT_WRITE | EMU_PROT_EXEC,
				    EMU_MAP_SHARED | EMU_MAP_ANONYMOUS, -1, 0);

  if (mem_start == EMU_MAP_FAILED)
    emu_do_syscall(EMU_SYSCALL_EXIT, 1);

  mem_end = (uint8_t*)mem_start + CONFIG_ARCH_EMU_MEMORY;

  mem_end = ALIGN_ADDRESS_LOW(mem_end, CONFIG_MUTEK_MEMALLOC_ALIGN);
  mem_start = ALIGN_ADDRESS_UP(mem_start, CONFIG_MUTEK_MEMALLOC_ALIGN);

  default_region = memory_allocator_init(NULL, mem_start, mem_end);
}

/////////////////////////////////////////////////////////////////////

#include <device/device.h>
#include <device/resources.h>
#include <device/driver.h>
#include <mutek/printk.h>

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
  device_set_name(d, "cpu0");
  device_attach(d, NULL);
  device_bind_driver(d, &emu_cpu_drv);

#ifdef CONFIG_ARCH_SMP
  /* add other processors to device tree */
  for (i = 1; i < CONFIG_ARCH_EMU_CPUS; i++)
  {
    __compiler_sint_t pid;

    /* fork */
    pid = emu_do_syscall(EMU_SYSCALL_FORK, 0);
    if (pid < 0)
      {
        printk("error: unable to create more UNIX process to emulate processors\n");
        break;
      }

    if (pid)
      {
        struct device_s *d = device_alloc(1);
        device_res_add_id(d, pid, 0);
        d->node.flags |= DEVICE_FLAG_CPU;

        char name[8];
        sprintf(name, "cpu%u", i);
        device_set_name(d, name);

        device_attach(d, NULL);
        device_bind_driver(d, &emu_cpu_drv);

        /* wait for the child to stop */
        //        emu_do_syscall(EMU_SYSCALL_WAITPID, 3, pid, 0, EMU_WAITPID_WUNTRACED);
        emu_do_syscall(EMU_SYSCALL_WAIT4, 4, pid, 0, EMU_WAITPID_WUNTRACED, 0);
      }
    else
      {
        /* non-bootstrap processors stop themselves */
        pid = emu_do_syscall(EMU_SYSCALL_GETPID, 0);
        emu_do_syscall(EMU_SYSCALL_KILL, 2, pid, EMU_SIG_STOP);

        return mutekh_startup_smp();
      }
  }
#endif
}

/////////////////////////////////////////////////////////////////////

DEV_DECLARE_STATIC(tty_dev, "tty", 0, emu_tty_drv);


/////////////////////////////////////////////////////////////////////

#include <device/device.h>

#ifdef CONFIG_ARCH_SMP
static DEVICE_TREE_WALKER(emu_start_cpus_cont)
{
  uint32_t *mask = priv;

  if (dev->node.flags & DEVICE_FLAG_CPU &&
      dev->status == DEVICE_INIT_DONE)
    {
      uintptr_t maj, min;
      if (!device_res_get_uint(dev, DEV_RES_ID, 0, &maj, &min) &&
          maj != __bootstrap_pid)
        {
          emu_do_syscall(EMU_SYSCALL_KILL, 2, maj, EMU_SIG_CONT);
        }
    }

  return 0;
}

void emu_start_cpus()
{
  /* send SIG_CONT to other processors */
  device_tree_walk(NULL, &emu_start_cpus_cont, NULL);
}
#endif

/////////////////////////////////////////////////////////////////////

#include <mutek/printk.h>

#ifdef CONFIG_EMU_PRINTK
static PRINTF_OUTPUT_FUNC(early_console_fd1)
{
  emu_do_syscall(EMU_SYSCALL_WRITE, 3, 1, str, len);  
}

void emu_early_console_init()
{
  printk_set_output(early_console_fd1, NULL);
}
#endif

/////////////////////////////////////////////////////////////////////

//    cpu_pids[0] = emu_do_syscall(EMU_SYSCALL_GETPID, 0);

#ifdef CONFIG_HEXO_IRQ
//    emu_interrupts_init();
#endif

//    arch_hw_init();

# ifdef CONFIG_HEXO_IPI
//    dev_icu_setup_ipi_ep(&icu_dev, CPU_LOCAL_ADDR(ipi_endpoint), cpu_id());
# endif

//    emu_do_syscall(EMU_SYSCALL_EXIT, 1, 1);  


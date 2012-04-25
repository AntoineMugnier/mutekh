/*
   This file is part of MutekH.
   
   MutekH is free software; you can redistribute it and/or modify it
   under the terms of the GNU Lesser General Public License as published
   by the Free Software Foundation; version 2.1 of the License.
   
   MutekH is distributed in the hope that it will be useful, but WITHOUT
   ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
   FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public
   License for more details.
   
   You should have received a copy of the GNU Lesser General Public
   License along with MutekH; if not, write to the Free Software
   Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
   02110-1301 USA.

   Copyright (c) 2011 Alexandre Becoulet <alexandre.becoulet@telecom-paristech.fr>
   Copyright (c) 2011 Institut Telecom / Telecom ParisTech

*/

# include <device/driver.h>
# include <device/device.h>
# include <device/class/char.h>
# include <device/class/cpu.h>

#include <mutek/printk.h>
#include <mutek/scheduler.h>
#include <hexo/context.h>
#include <hexo/init.h>
#include <mutek/memory_allocator.h>

#ifdef CONFIG_DEVICE_IRQ
struct device_s *gaisler_icu = NULL;
#endif

#if defined(CONFIG_GAISLER_EARLY_CONSOLE)
void gaisler_early_console(uintptr_t addr);
#endif

#if defined (CONFIG_MUTEK_SCHEDULER)
extern struct sched_context_s main_ctx;
#else
struct context_s main_ctx;
#endif

extern __ldscript_symbol_t __system_uncached_heap_start, __system_uncached_heap_end;

static void cpu_reg_init()
{
    /* find processor device */
    struct device_s *dev = device_get_cpu(cpu_id(), 0);

    if (!dev)
        return;

    struct device_cpu_s cpu_dev;

    if (device_get_accessor(&cpu_dev, dev, DRIVER_CLASS_CPU, 0))
        return;

    DEVICE_OP(&cpu_dev, reg_init);

    device_put_accessor(&cpu_dev);
}

void arch_init(uintptr_t init_sp)
{
    extern __ldscript_symbol_t __bss_start;
    extern __ldscript_symbol_t __bss_end;

    memset(
        (uint8_t*)&__bss_start,
        0,
        (uint8_t*)&__bss_end-(uint8_t*)&__bss_start);

#if defined(CONFIG_GAISLER_EARLY_CONSOLE)
    gaisler_early_console(CONFIG_GAISLER_EARLY_CONSOLE_ADDR);
#endif

    default_region = memory_allocator_init(NULL, 
                                           &__system_uncached_heap_start, 
                                           (void*)((uintptr_t)&__system_uncached_heap_end -
                                                   (1 << CONFIG_HEXO_RESET_STACK_SIZE) * CONFIG_CPU_MAXCOUNT));

    hexo_global_init();

#if 1
    extern const struct driver_s ahbctrl_drv;
    static struct device_s ahbctrl_dev;

    device_init(&ahbctrl_dev);
    device_attach(&ahbctrl_dev, NULL);
    device_res_add_mem(&ahbctrl_dev, 0xfffff000, 0xffffffe0);
    device_bind_driver(&ahbctrl_dev, &ahbctrl_drv);
    device_init_driver(&ahbctrl_dev);

    device_find_driver(NULL);
#endif

    cpu_reg_init();

#if defined(CONFIG_MUTEK_SCHEDULER)
    sched_global_init();
    sched_cpu_init();

    /* FIXME initial stack space will never be freed ! */
    context_bootstrap(&main_ctx.context, 0, init_sp);
    sched_context_init(&main_ctx);
#else
    context_bootstrap(&main_ctx, 0, init_sp);
#endif

    mutek_start();
}

void boot_from_reset_vector(uintptr_t init_sp)
{
  arch_init(init_sp);
}


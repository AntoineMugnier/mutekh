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

/**
   @file
   @module {Core::Devices support library}
   @short Processor driver API
   @index {Processor} {Device classes}
   @csee DRIVER_CLASS_CPU
 */

#ifndef __DEVICE_CPU_H__
#define __DEVICE_CPU_H__

#include <hexo/types.h>
#include <hexo/error.h>
#include <hexo/local.h>

#include <device/driver.h>

/** @internal */
extern CPU_LOCAL struct device_s *cpu_device;

/** @This returns a pointer to local cpu device */
ALWAYS_INLINE struct device_s * cpu_local_device(void)
{
  return CPU_LOCAL_GET(cpu_device);
}

struct device_s;
struct driver_s;
struct device_cpu_s;

/** @see dev_cpu_reg_init_t */
#define DEV_CPU_REG_INIT(n)	void (n) (struct device_cpu_s *accessor)

/** @This executes processor registers initialization which can not be
    performed from an other processor on driver init. */
typedef DEV_CPU_REG_INIT(dev_cpu_reg_init_t);

/** dev_cpu_get_node_t */
#define DEV_CPU_GET_NODE(n)	struct cpu_tree_s * (n) (struct device_cpu_s *accessor)

/** @This returns pointer to the cpu tree node. @see cpu_tree_s. */
config_depend(CONFIG_ARCH_SMP)
typedef DEV_CPU_GET_NODE(dev_cpu_get_node_t);

/** ICU device class methodes */

DRIVER_CLASS_TYPES(DRIVER_CLASS_CPU, cpu,
                   dev_cpu_reg_init_t *f_reg_init;
#ifdef CONFIG_ARCH_SMP
                   dev_cpu_get_node_t *f_get_node;
#endif
                   );

#ifdef CONFIG_ARCH_SMP
/** @see driver_cpu_s */
# define DRIVER_CPU_METHODS(prefix)                               \
  ((const struct driver_class_s*)&(const struct driver_cpu_s){    \
    .class_ = DRIVER_CLASS_CPU,                                   \
    .f_reg_init = prefix ## _reg_init,                            \
    .f_get_node = prefix ## _get_node,                            \
  })
#else
/** @see driver_cpu_s */
# define DRIVER_CPU_METHODS(prefix)                               \
  ((const struct driver_class_s*)&(const struct driver_cpu_s){    \
    .class_ = DRIVER_CLASS_CPU,                                   \
    .f_reg_init = prefix ## _reg_init,                            \
  })
#endif

#endif


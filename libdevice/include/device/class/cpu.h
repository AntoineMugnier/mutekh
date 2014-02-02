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
 * @file
 * @module{Devices support library}
 * @short Processor driver API
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
static inline struct device_s * cpu_local_device()
{
  return CPU_LOCAL_GET(cpu_device);
}

struct device_s;
struct driver_s;
struct device_cpu_s;

#define DEVCPU_REG_INIT(n)	void (n) (struct device_cpu_s *cdev)

/** @This executes processor registers initialization which can not be
    performed from an other processor on driver init. */
typedef DEVCPU_REG_INIT(devcpu_reg_init_t);

#define DEVCPU_GET_NODE(n)	struct cpu_tree_s * (n) (struct device_cpu_s *cdev)

/** @This returns pointer to the cpu tree node. @see cpu_tree_s. */
typedef DEVCPU_GET_NODE(devcpu_get_node_t);

/** ICU device class methodes */

DRIVER_CLASS_TYPES(cpu, 
                   devcpu_reg_init_t *f_reg_init;
#ifdef CONFIG_ARCH_SMP
                   devcpu_get_node_t *f_get_node;
#endif
                   );

#endif


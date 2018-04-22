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

/** @internal @This specifies the value which should be xored with the
    cpu id for lookup in the cpu tree. */
#define CPU_TREE_XOR_VALUE 0x55555555

/** @internal This defines the cpu tree node structure. This node is
    designed to be part of a binary tree which is used from both C and
    assembly code to lookup the processor from its numerical id. It's
    first used on startup to setup the processor stack.

    This node must be inserted in the cpu tree when the processor
    device driver initialization takes place.
*/
struct cpu_tree_s
{
  uintptr_t         stack;        //< address of the cpu stack. @see #CPU_TREE_STACK
  struct device_s   *cpu_dev;     //< pointer to the cpu device. @see #CPU_TREE_CPU_DEV
#ifdef CONFIG_ARCH_SMP
  struct cpu_tree_s *childs[2];   //< left and right childs of the binary tree. @see #CPU_TREE_CHILDS
  uintptr_t         cpu_id;       //< physical cpu id value. @see #CPU_TREE_CHILDS, @see #CPU_TREE_XOR_VALUE
  void              *cls;         //< cpu local storage pointer. @see #CPU_TREE_CLS
#endif
};

/** @internal @This inserts a node in the cpu tree, the @ref
    cpu_tree_s::cpu_id, @ref cpu_tree_s::stack and @ref
    cpu_tree_s::cpu_dev fields must be initialized . */
config_depend(CONFIG_DEVICE_CPU)
error_t cpu_tree_insert(struct cpu_tree_s *node);

/** @internal @This removes a node from the cpu tree. */
config_depend(CONFIG_DEVICE_CPU)
void cpu_tree_remove(struct cpu_tree_s *node);

/** @internal @This lookup a cpu tree node from the cpu numerical id. */
config_depend(CONFIG_DEVICE_CPU)
const struct cpu_tree_s * cpu_tree_lookup(uintptr_t id);

/** @internal @This function initializes the various fields of the cpu
    node. It allocates the stack and allocates and initializes the cpu
    local storage. This helper function is designed to be called from
    the cpu driver initialization code. */
config_depend(CONFIG_DEVICE_CPU)
error_t cpu_tree_node_init(struct cpu_tree_s *node, cpu_id_t id, struct device_s *dev);

/** @internal @This function free the resources allocated by the @ref
    cpu_tree_node_init function. */
config_depend(CONFIG_DEVICE_CPU)
void cpu_tree_node_cleanup(struct cpu_tree_s *node);


#endif


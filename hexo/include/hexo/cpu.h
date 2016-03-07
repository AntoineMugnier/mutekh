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
 * @module{Hardware abstraction layer}
 * @short Startup and misc cpu related functions
 */

#ifndef __CPU_H_
#define __CPU_H_

#include <hexo/types.h>
#include <hexo/error.h>

#include <hexo/decls.h>

C_HEADER_BEGIN

#include <cpu/hexo/cpu.h>

#define __CPU_NAME_DECL(t, x) t##_##x
#define _CPU_NAME_DECL(t, x) __CPU_NAME_DECL(t, x)
/** @this can be used to declare and refer to a variable
    or function prefixed by cpu type name. */
#define CPU_NAME_DECL(x) _CPU_NAME_DECL(CONFIG_CPU_NAME, x)

/** @internal @This specifies the offset of the @ref cpu_tree_s::stack field. */
#define CPU_TREE_STACK     (INT_PTR_SIZE/8 * 0)
/** @internal @This specifies the offset of the @ref cpu_tree_s::cpu_dev field. */
#define CPU_TREE_CPU_DEV   (INT_PTR_SIZE/8 * 1)
#ifdef CONFIG_ARCH_SMP
/** @internal @This specifies the offset of the @ref cpu_tree_s::childs field. */
# define CPU_TREE_CHILDS(n) (INT_PTR_SIZE/8 * (2 + n))
/** @internal @This specifies the offset of the @ref cpu_tree_s::cpu_id field. */
# define CPU_TREE_CPU_ID    (INT_PTR_SIZE/8 * 4)
/** @internal @This specifies the offset of the @ref cpu_tree_s::cls field. */
# define CPU_TREE_CLS       (INT_PTR_SIZE/8 * 5)
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
error_t cpu_tree_insert(struct cpu_tree_s *node);

/** @internal @This removes a node from the cpu tree. */
void cpu_tree_remove(struct cpu_tree_s *node);

/** @internal @This lookup a cpu tree node from the cpu numerical id. */
const struct cpu_tree_s * cpu_tree_lookup(uintptr_t id);

/** @internal @This function initializes the various fields of the cpu
    node. It allocates the stack and allocates and initializes the cpu
    local storage. This helper function is designed to be called from
    the cpu driver initialization code. */
error_t cpu_tree_node_init(struct cpu_tree_s *node, cpu_id_t id, struct device_s *dev);

/** @internal @This function free the resources allocated by the @ref
    cpu_tree_node_init function. */
void cpu_tree_node_cleanup(struct cpu_tree_s *node);

/** return CPU architecture type name */
ALWAYS_INLINE const char *cpu_type_name(void);

/** return true if bootstap processor */
ALWAYS_INLINE bool_t cpu_isbootstrap(void);

/** cpu trap instruction */
ALWAYS_INLINE void cpu_trap(void);

/** get cpu cache line size, return 0 if no dcache */
ALWAYS_INLINE size_t cpu_dcache_line_size(void);

/** invalidate the cpu data cache line containing this address */
ALWAYS_INLINE void cpu_dcache_invld(void *ptr);

# if defined(CONFIG_CPU_CACHE)

/** invalidate all the cpu data cache lines within given range.
    size is in bytes. */
void cpu_dcache_invld_buf(void *ptr, size_t size);

/** invalidate all the cpu instruction cache lines within given range.
    size is in bytes. */
void cpu_icache_invld_buf(void *ptr, size_t size);

/** invalidate all the cpu data cache lines within given range.
    size is in bytes. */
void cpu_dcache_flush_buf(void *ptr, size_t size);

# else

ALWAYS_INLINE void
cpu_dcache_invld_buf(void *ptr, size_t size)
{
}

ALWAYS_INLINE void
cpu_icache_invld_buf(void *ptr, size_t size)
{
}

ALWAYS_INLINE void
cpu_dcache_flush_buf(void *ptr, size_t size)
{
}

# endif

# define _TO_STR(x) #x
# define TO_STR(x) _TO_STR(x)

/** @this returns the cpu type name */
ALWAYS_INLINE const char *
cpu_type_name(void)
{
  return TO_STR(CONFIG_CPU_NAME);
}

# undef _TO_STR
# undef TO_STR

C_HEADER_END

#endif

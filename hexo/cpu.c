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

#include <hexo/cpu.h>
#include <hexo/bit.h>

#include <mutek/mem_alloc.h>

#include <string.h>

#if defined(CONFIG_CPU_CACHE)

void cpu_dcache_invld_buf(void *ptr, size_t size)
{
  size_t ls = cpu_dcache_line_size();

  if (ls != 0)
  {
    uint8_t *ptr_;

    for (ptr_ = address_align_down(ptr, ls);
        ptr_ < (uint8_t*)ptr + size;
        ptr_ += ls)
      cpu_dcache_invld(ptr_);
  }
}

void cpu_dcache_flush_buf(void *ptr, size_t size)
{
  size_t ls = cpu_dcache_line_size();

  if (ls != 0)
  {
    uint8_t *ptr_;

    for (ptr_ = address_align_down(ptr, ls);
        ptr_ < (uint8_t*)ptr + size;
        ptr_ += ls)
      cpu_dcache_flush(ptr_);
  }
}

#endif

#ifdef CONFIG_DEVICE_CPU

/* This is the root node of the cpus tree. */
struct cpu_tree_s *cpu_tree = NULL;

error_t cpu_tree_insert(struct cpu_tree_s *node)
{
  struct cpu_tree_s **r = &cpu_tree;

#ifdef CONFIG_ARCH_SMP
  uintptr_t xid = node->cpu_id ^ CPU_TREE_XOR_VALUE;

  while (*r != NULL)
    {
      uintptr_t nxid = (*r)->cpu_id ^ CPU_TREE_XOR_VALUE;
      if (nxid == xid)
        return -EEXISTS;

      r = (*r)->childs + (nxid < xid);
    }

  node->childs[0] = node->childs[1] = NULL;
#endif
  *r = node;

  return 0;
}

void cpu_tree_remove(struct cpu_tree_s *node)
{
#warning FIXME missing code
}

const struct cpu_tree_s * cpu_tree_lookup(uintptr_t id)
{
  struct cpu_tree_s *r = cpu_tree;

#ifdef CONFIG_ARCH_SMP
  id ^= CPU_TREE_XOR_VALUE;

  while (r != NULL)
    {
      uintptr_t nxid = r->cpu_id ^ CPU_TREE_XOR_VALUE;
      if (nxid == id)
        break;

      r = r->childs[nxid < id];
    }
#endif

  return r;
}


error_t cpu_tree_node_init(struct cpu_tree_s *node, cpu_id_t id, struct device_s *dev)
{
#ifdef CONFIG_ARCH_SMP
  node->cpu_id = id;

  extern __ldscript_symbol_t __cpu_data_start, __cpu_data_end;

  node->cls = mem_alloc_cpu((char*)&__cpu_data_end - (char*)&__cpu_data_start,
                            mem_scope_cpu, id);
  if (!node->cls)
    return -ENOMEM;

  memcpy(node->cls, (char*)&__cpu_data_start, (char*)&__cpu_data_end - (char*)&__cpu_data_start);
#endif

  void *stack = mem_alloc_cpu(CONFIG_HEXO_CPU_STACK_SIZE, mem_scope_cpu, id);
  if (!stack)
    {
#ifdef CONFIG_ARCH_SMP
      mem_free(node->cls);
#endif
      return -ENOMEM;
    }

  node->stack = (uintptr_t)stack;
  node->cpu_dev = dev;

  return 0;
}

void cpu_tree_node_cleanup(struct cpu_tree_s *node)
{
  mem_free((void*)node->stack);
#ifdef CONFIG_ARCH_SMP
  mem_free(node->cls);
#endif
}

#endif


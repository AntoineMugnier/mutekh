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
    License along with this program.  If not, see
    <http://www.gnu.org/licenses/>.

    Copyright Nicolas Pouillon <nipo@ssji.net> (c) 2014
*/

#ifndef MUTEK_THREAD_H_
#define MUTEK_THREAD_H_

#include <mutek/mem_alloc.h>
#include <hexo/types.h>
#include <hexo/context.h>

struct cpu_tree_s;

/** Thread attributes for the @ref thread_create function. */
struct thread_attr_s
{
  /** Byte size of the thread stack */
  size_t stack_size;
  /** Memory allocation scope for the thread and associated stack */
  enum mem_scope_e scope;
  /** Processor affinity or @tt NULL */
  struct cpu_tree_s *cpu;
};

/** Default initializer for @ref thread_attr_s */
#define THREAD_ATTR_INITIALIZER                         \
  {                                                     \
    .stack_size = CONFIG_HEXO_CPU_STACK_SIZE,           \
    .scope = mem_scope_sys,                             \
    .cpu = NULL,                                        \
  }

/**
   @this creates a standalone scheduler context. This allocates the
   thread object and its stack then push the thread in the scheduler
   running queue.

   The @tt entry point function is called with the @tt arg argument
   when the new thread is first scheduled.

   Context data and stack are freed when the entry point function
   returns. There is no special function to call before entry point
   exit.

   @param entry Entry point
   @param arg Entry point argument
   @param attr Thread attributes, may be @tt NULL.

   @returns 0 when done, -ENOMEM if allocation failed.
 */
config_depend(CONFIG_MUTEK_THREAD)
error_t
thread_create(context_entry_t *entry, void *arg,
              const struct thread_attr_s *attr);

#endif


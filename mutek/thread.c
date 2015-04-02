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

#include <mutek/thread.h>
#include <mutek/scheduler.h>

struct thread_s
{
  struct context_s context;
  struct sched_context_s sched_context;
  context_entry_t *entry;
  void *arg;
};

static void thread_cleanup(void *param)
{
  struct thread_s *th = param;

  context_destroy(&th->context);

  mem_free(th);

  sched_context_exit();
}

static CONTEXT_ENTRY(thread_entry)
{
  struct thread_s *th = param;

  cpu_interrupt_enable();
  th->entry(th->arg);
  cpu_interrupt_disable();

  cpu_context_stack_use(sched_tmp_context(),
                        thread_cleanup, th);
}

error_t thread_create(context_entry_t *entry, void *arg,
                      const struct thread_attr_s *attr)
{
  struct thread_s *th;
  size_t s = ALIGN_VALUE_UP(sizeof(*th), CONFIG_HEXO_STACK_ALIGN);
  enum mem_scope_e scope;
  size_t stack_size;

  if (attr)
    {
      stack_size = ALIGN_VALUE_UP(attr->stack_size, CONFIG_HEXO_STACK_ALIGN);
      scope = attr->scope;
    }
  else
    {
      stack_size = CONFIG_HEXO_CPU_STACK_SIZE;
      scope = mem_scope_default;
    }

  th = mem_alloc_align(s + stack_size, CONFIG_HEXO_STACK_ALIGN, scope);
  if (!th)
    return -ENOMEM;

  th->entry = entry;
  th->arg = arg;

  uint8_t *stack = (uint8_t*)th + s;
  context_init(&th->context, stack,
               stack + stack_size,
               thread_entry, th);

  sched_context_init(&th->sched_context, &th->context);
  CPU_INTERRUPT_SAVESTATE_DISABLE;
  sched_context_start(&th->sched_context);
  CPU_INTERRUPT_RESTORESTATE;

  return 0;
}


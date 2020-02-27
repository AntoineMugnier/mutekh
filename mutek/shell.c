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

    Copyright Julien Peeters <contact@julienpeeters.net> (c) 2014
    Copyright Alexandre Becoulet <alexandre.becoulet@free.fr> (c) 2014

*/

#include <mutek/shell.h>
#include <mutek/startup.h>
#include <mutek/mem_alloc.h>
#include <mutek/printk.h>

#include <device/class/char.h>

#include <termui/term.h>
#include <termui/getline.h>
#include <termui/mutekh.h>
#include <termui/console.h>

static void shell_context_init(struct mutek_shell_context_s *sctx)
{
  lock_init(&sctx->lock);
#ifdef CONFIG_MUTEK_SHELL_BUFFER
  shell_buffer_pool_init(&sctx->bufs);
  sctx->next_id = 0;
#endif
}

static error_t shell_context_flush(struct mutek_shell_context_s *sctx)
{
#ifdef CONFIG_MUTEK_SHELL_BUFFER
  bool_t r;
  LOCK_SPIN_IRQ(&sctx->lock);
  r = shell_buffer_collect_all(sctx);
  LOCK_RELEASE_IRQ(&sctx->lock);

  if (r)
    return -EBUSY;

  shell_buffer_pool_destroy(&sctx->bufs);
#endif
  lock_destroy(&sctx->lock);

  return 0;
}

/* console commands descriptors array */

#ifdef CONFIG_LIBTERMUI_CON_HELP
MUTEK_SHELL_ROOT_ENTRY(help, TERMUI_CON_BUILTIN_HELP(-1) );
#endif
MUTEK_SHELL_ROOT_ENTRY(list, TERMUI_CON_BUILTIN_LIST(-1) );
MUTEK_SHELL_ROOT_ENTRY(quit, TERMUI_CON_BUILTIN_QUIT(-1) );

extern __ldscript_symbol_t shell_cmd_table;

error_t mutek_shell_start(const struct device_char_s *c, const char *term,
                       const struct termui_con_entry_s * const *root,
                       const char *prompt)
{
  error_t res = -ENOMEM;
  struct termui_term_s *tm;
  struct termui_console_s *con;
  struct mutek_shell_context_s *sctx;

  logk("Shell thread start");

  if (root == NULL)
    root = (void*)&shell_cmd_table;

  tm = mem_alloc(sizeof(*tm) + sizeof(*con) + sizeof(*sctx), mem_scope_sys);
  if (tm == NULL)
    goto err;

  con = (void*)(tm + 1);
  sctx = (void*)(con + 1);

  shell_context_init(sctx);

  termui_dev_io_init(tm, c, term);

  if (termui_con_init(con, tm, root, CONFIG_MUTEK_SHELL_BUF_SIZE,
                      CONFIG_MUTEK_SHELL_LINE_SIZE))
    goto err_free;

  termui_con_set_private(con, sctx);
  termui_con_set_prompt(con, prompt);

  termui_term_printf(tm, "You may type `list' and `help'.\n\n");

  /* process user commands */
  do {
    res = termui_con_process(con);
    if (res == -EINVAL)
      termui_term_printf(tm, "Failed\n");
  } while (res != -ECANCELED || shell_context_flush(sctx));

  termui_term_printf(tm, "Terminated\n");

  /* free allocated resources */
  termui_con_cleanup(con);
  termui_termio_cleanup(tm);

 err_free:
  mem_free(tm);
 err:
  logk("Shell thread exit (%i)", res);

  return res;
}

#ifdef CONFIG_MUTEK_SHELL_THREAD

#include <mutek/thread.h>
#include <mutek/console.h>

static CONTEXT_ENTRY(shell_thread)
{
  while (mutek_shell_start(param, "xterm", NULL, CONFIG_MUTEK_SHELL_PROMPT)
         == -ECANCELED)
    ;
}

void mutek_shell_thread_init()
{
  static const struct thread_attr_s a = {
    .stack_size = CONFIG_MUTEK_SHELL_STACK_SIZE,
    .scope = mem_scope_sys,
  };

  if (device_check_accessor(&console_dev.base))
    thread_create(shell_thread, &console_dev, &a);
}

#endif


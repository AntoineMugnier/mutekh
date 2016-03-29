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

#include <device/class/char.h>

#include <termui/term.h>
#include <termui/getline.h>
#include <termui/mutekh.h>
#include <termui/console.h>

/* console commands descriptors array */

#ifdef CONFIG_LIBTERMUI_CON_HELP
MUTEK_SHELL_ROOT_ENTRY(help, TERMUI_CON_BUILTIN_HELP(-1) );
#endif
MUTEK_SHELL_ROOT_ENTRY(list, TERMUI_CON_BUILTIN_LIST(-1) );
MUTEK_SHELL_ROOT_ENTRY(quit, TERMUI_CON_BUILTIN_QUIT(-1) );

extern __ldscript_symbol_t shell_cmd_table;

void mutek_shell_start(struct device_char_s *c, const char *term,
                       const struct termui_con_entry_s * const *root,
                       const char *prompt, void *con_pv)
{
  struct termui_term_s *tm;
  struct termui_console_s *con;

  if (root == NULL)
    root = (void*)&shell_cmd_table;

  tm = mem_alloc(sizeof(*tm) + sizeof(*con), mem_scope_sys);
  if (tm == NULL)
    return;
  con = (void*)(tm + 1);

  termui_dev_io_init(tm, c, term);
  termui_con_init(con, tm, root);

  termui_con_set_private(con, con_pv);
  termui_con_set_prompt(con, prompt);

  termui_term_printf(tm, "You may type `list' and `help'.\n\n");

  /* process user commands */
  error_t res;
  do {
    res = termui_con_process(con);
    if (res == -EINVAL)
      termui_term_printf(tm, "Failed\n");
  } while (res != -EIO && res != -ECANCELED);

  termui_term_printf(tm, "Terminated\n");

  /* free allocated resources */
  termui_con_cleanup(con);
  termui_termio_cleanup(tm);

  mem_free(tm);
}

#ifdef CONFIG_MUTEK_SHELL_THREAD

#include <mutek/thread.h>
#include <mutek/console.h>

static CONTEXT_ENTRY(shell_thread)
{
  while (1)
    mutek_shell_start(param, "xterm", NULL, CONFIG_MUTEK_SHELL_PROMPT, NULL);
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


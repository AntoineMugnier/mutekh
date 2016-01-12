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

/**
 * @file
 * @module{Mutek}
 * @short Interactive shell
 */

#include <hexo/decls.h>

#ifndef MUTEK_SHELL_H_
#define MUTEK_SHELL_H_

#ifdef CONFIG_MUTEK_SHELL
# include <termui/console.h>
# include <termui/console_opt.h>
#endif

struct device_char_s;
struct termui_con_entry_s;

/** @This starts an interactive shell. This function does not returns
    until the shell is exited by the user. */
config_depend(CONFIG_MUTEK_SHELL)
void mutek_shell_start(struct device_char_s *c, const char *term,
                       const struct termui_con_entry_s * const *root,
                       const char *prompt, void *con_pv);

/** @internal */
struct mutek_shell_root_groups_s
{
  const struct termui_con_entry_s *e;
};

#ifdef CONFIG_MUTEK_SHELL
# define MUTEK_SHELL_ROOT_GROUP(group, name)                            \
  __attribute__ ((aligned (sizeof(void*))))                             \
  __attribute__((section(".shell." #group)))                            \
  const struct mutek_shell_root_groups_s group##_ptr = { TERMUI_CON_GROUP_ENTRY(group, name) };
# define MUTEK_SHELL_ROOT_ENTRY(decl, ...)                              \
  __attribute__ ((aligned (sizeof(void*))))                             \
  __attribute__((section(".shell." #decl)))                             \
  const struct mutek_shell_root_groups_s decl##_ptr = { __VA_ARGS__ };
#else
# define MUTEK_SHELL_ROOT_GROUP(group, name)
# define MUTEK_SHELL_ROOT_ENTRY(decl, ...)
#endif

#endif


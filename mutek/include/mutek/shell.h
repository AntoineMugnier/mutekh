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

#ifndef MUTEK_SHELL_H_
#define MUTEK_SHELL_H_

#include <termui/console.h>

struct device_char_s;

/** @This starts an interactive shell. This function does not returns
    until the shell is exited by the user. */
void mutek_shell_start(struct device_char_s *c, const char *term);

#define MUTEK_SHELL_GROUP_REGISTER(group)             \
  __attribute__((section(".shell")))                  \
  extern TERMUI_CON_GROUP_DECL(group);

#endif


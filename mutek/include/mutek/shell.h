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
 * @module {Core::Kernel services}
 * @short Interactive shell
 */

#ifndef MUTEK_SHELL_H_
#define MUTEK_SHELL_H_

#include <hexo/decls.h>
#include <hexo/lock.h>

#ifdef CONFIG_MUTEK_SHELL
# include <termui/console.h>
# include <termui/console_opt.h>
#endif

struct device_char_s;
struct termui_con_entry_s;
struct termui_console_s;

#ifdef CONFIG_MUTEK_SHELL_BUFFER

#include <gct_platform.h>
#include <gct_lock_hexo_lock.h>
#include <gct/container_clist.h>

#define GCT_CONTAINER_ALGO_shell_buffer CLIST
struct mutek_shell_context_s;

/** @internal */
struct mutek_shell_buffer_s
{
  GCT_CONTAINER_ENTRY    (shell_buffer, list_entry);
  char name[16];
  struct mutek_shell_context_s *sctx;
  const void  *type;
  uintptr_t  size:24;
  uintptr_t  use:1;
  uintptr_t  nocopy:1;
  uint8_t    data[0];
};

GCT_CONTAINER_TYPES      (shell_buffer, struct mutek_shell_buffer_s *, list_entry);
GCT_CONTAINER_KEY_TYPES  (shell_buffer, PTR, STRING, name);
GCT_CONTAINER_KEY_FCNS   (shell_buffer, ASC, static inline, shell_buffer_pool, name,
			  init, destroy, push, remove, lookup, head);

#endif

/** @internal */
struct mutek_shell_context_s
{
  lock_t lock;
#ifdef CONFIG_MUTEK_SHELL_BUFFER
  shell_buffer_root_t bufs;
  uint32_t next_id;
#endif
};

/** @This allocates a new buffer. The reference to the buffer must be
    released by calling the @ref shell_buffer_drop function.

    The type argument can be used to indentify the type of data
    contained in the buffer. It allows trusting the content of an
    existing buffer for casting to a C structure. Editing the buffer
    at byte level from the shell will change the type to @tt NULL. The
    @tt nocopy flag can also be used to drop the type when the buffer
    is duplicated.

    When the @ref #CONFIG_MUTEK_SHELL_BUFFER token is undefined,
    this is equivalent to calling @ref mem_alloc.
*/
config_depend(CONFIG_MUTEK_SHELL)
void * shell_buffer_new(const struct termui_console_s *con,
                        size_t size, const char *prefix,
                        const void *type, bool_t nocopy);

/** @This lookup a buffer with a matching type and a no reference.
    If no such buffer exist a new buffer is allocated. The
    reference to the buffer must be released by calling the @ref
    shell_buffer_drop function.

    The function return @tt NULL if the allocation failed.

    When the @ref #CONFIG_MUTEK_SHELL_BUFFER token is undefined,
    this is equivalent to calling @ref mem_alloc.
*/
config_depend(CONFIG_MUTEK_SHELL)
void * shell_buffer_reuse(const struct termui_console_s *con,
                          size_t size, const char *prefix,
                          const void *type, bool_t nocopy);

/** @This drops the reference to a buffer. The buffer is released when
    the @ref shell_buffer_collect function is called provided that
    no one else has taken the reference.

    When the @ref #CONFIG_MUTEK_SHELL_BUFFER token is undefined,
    this is equivalent to calling @ref mem_free.
*/
config_depend(CONFIG_MUTEK_SHELL)
void shell_buffer_drop(void * data);

/** @This can be used to advertise the availability of new data in a
    buffer. When the @ref #CONFIG_MUTEK_SHELL_BUFFER token is defined,
    this show the name of the buffer. In the other case, it hexdumps
    the buffer content.
*/
config_depend(CONFIG_MUTEK_SHELL)
void shell_buffer_advertise(struct termui_console_s *con,
                            void *data, size_t size);

/** @This takes a reference to a buffer. If the @tt name parameter is
    not @tt NULL, a buffer with the specified name must exist for the
    function to succeed. If the @tt name parameter is @tt NULL, the
    last created buffer is returned.

    The current size of the buffer is stored in @tt size if not @tt
    NULL. The reference to the buffer must be released by calling the
    @ref shell_buffer_drop function.

    The function return @tt NULL on failure.
    @see #TERMUI_CON_OPT_SHELL_BUFFER_GET_ENTRY
*/
config_depend(CONFIG_MUTEK_SHELL_BUFFER)
void * shell_buffer_get(const struct termui_console_s *con,
                        const char *name, size_t *size, const void *type);

/** @This returns the name of a referenced buffer */
config_depend(CONFIG_MUTEK_SHELL_BUFFER)
const char * shell_buffer_name(void * data);

/** @This destroy a buffer which is not in use. All buffers are
    released when the @tt name pointer is @tt NULL. */
config_depend(CONFIG_MUTEK_SHELL_BUFFER)
void shell_buffer_collect(const struct termui_console_s *con,
                          const char *name);

/** @internal This is for internal purpose, shell context must be
    locked. */
config_depend(CONFIG_MUTEK_SHELL_BUFFER)
bool_t shell_buffer_collect_all(struct mutek_shell_context_s *sctx);

/** @This starts an interactive shell. This function does not returns
    until the shell is exited by the user. */
config_depend(CONFIG_MUTEK_SHELL)
error_t mutek_shell_start(const struct device_char_s *c, const char *term,
                       const struct termui_con_entry_s * const *root,
                       const char *prompt);

/** This is used to store a pointer to a shell buffer or other
    storage in a console options parsing context.
    @see #TERMUI_CON_OPT_SHELL_BUFFER_GET_ENTRY */
struct shell_opt_buffer_s
{
#ifdef CONFIG_MUTEK_SHELL
  /** pointer to storage */
  void *addr;
  /** size storage */
  uintptr_t size:31;
  /** this is set when the @ref shell_buffer_drop function must be
      called on cleanup. */
  uintptr_t buffered:1;
#endif
};

/** @internal @see #TERMUI_CON_OPT_SHELL_BUFFER_GET_ENTRY */
struct shell_opt_buffer_desc_s
{
#ifdef CONFIG_MUTEK_SHELL
  struct termui_con_opts_s opt;
  const void *type;
  uint16_t offset;
#endif
};

/** @This calls @ref shell_buffer_new only if the @ref
    #TERMUI_CON_OPT_SHELL_BUFFER_GET_ENTRY option has not parsed a
    buffer. In either cases the buffer should be released by the
    options cleanup handler. */
config_depend(CONFIG_MUTEK_SHELL_BUFFER)
void * shell_opt_buffer_new_if_null(struct shell_opt_buffer_s *b,
                                    const struct termui_console_s *con,
                                    size_t size, const char *prefix,
                                    const void *type, bool_t nocopy);

#ifdef CONFIG_MUTEK_SHELL
/** @internal */
TERMUI_CON_PARSE_OPT_PROTOTYPE(shell_opt_buffer_raw_parse);
/** @internal */
config_depend(CONFIG_MUTEK_SHELL_BUFFER)
TERMUI_CON_PARSE_OPT_PROTOTYPE(shell_opt_buffer_get_parse);
/** @internal */
TERMUI_CON_ARGS_COLLECT_PROTOTYPE(shell_opt_buffer_comp);
/** @internal */
config_depend(CONFIG_MUTEK_SHELL_BUFFER)
TERMUI_CON_ARGS_COLLECT_PROTOTYPE(shell_opt_buffer_name_comp);
#endif

/** @This can be used to declare a libtermui console option which
    accepts an exisiting shell buffer or raw data. */
#define TERMUI_CON_OPT_SHELL_BUFFER_RAW_ENTRY(sname_, lname_, id_, type_, field_, typeptr_, ...) \
  TERMUI_CON_OPT_CUSTOM_ENTRY(shell_opt_buffer_desc_s, sname_, lname_, id_, \
    TERMUI_CON_OPT_PARSE(shell_opt_buffer_raw_parse, 1)                  \
    TERMUI_CON_OPT_COMPLETE(shell_opt_buffer_comp, NULL)                 \
    .type = typeptr_,                                                    \
    .offset = offsetof(type_, field_),                                   \
    __VA_ARGS__                                                          \
  )

/** @This can be used to declare a libtermui console option which
    specifies a shell buffer or raw data. */
#define TERMUI_CON_OPT_SHELL_BUFFER_GET_ENTRY(sname_, lname_, id_, type_, field_, typeptr_, ...) \
  TERMUI_CON_OPT_CUSTOM_ENTRY(shell_opt_buffer_desc_s, sname_, lname_, id_, \
    TERMUI_CON_OPT_PARSE(shell_opt_buffer_get_parse, 1)                  \
    TERMUI_CON_OPT_COMPLETE(shell_opt_buffer_name_comp, NULL)            \
    .type = typeptr_,                                                    \
    .offset = offsetof(type_, field_),                                   \
    __VA_ARGS__                                                          \
  )

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


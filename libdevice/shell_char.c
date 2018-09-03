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

    Copyright Jeremie Brunel <jeremie@brunel-ejm.org> (c) 2015

*/

#include <device/class/char.h>
#include <device/driver.h>
#include <device/resources.h>
#include <device/shell.h>

#include <mutek/console.h>
#include <mutek/shell.h>
#include <mutek/mem_alloc.h>
#include <hexo/enum.h>

enum char_opts_e
{
  CHAR_OPT_DEV = 0x01,
  CHAR_OPT_SIZE = 0x02,
  CHAR_OPT_DATA = 0x04,
  CHAR_OPT_HEX = 0x08,
  CHAR_OPT_PARTIAL = 0x10,
  CHAR_OPT_FRAME = 0x20,
  CHAR_OPT_POLL = 0x40,
  CHAR_OPT_FLUSH = 0x80,
  CHAR_OPT_NONBLOCK = 0x100
};

struct termui_optctx_dev_char_opts
{
  struct device_char_s accessor;
  size_t size;
  struct termui_con_string_s data;
};

static TERMUI_CON_ARGS_CLEANUP_PROTOTYPE(char_opts_cleanup)
{
  struct termui_optctx_dev_char_opts *c = ctx;

  if (device_check_accessor(&c->accessor.base))
    device_put_accessor(&c->accessor.base);
}

static enum dev_char_rq_type_e
char_rq_type(enum dev_char_rq_type_e type, enum char_opts_e used)
{
  if (used & CHAR_OPT_PARTIAL)
    type |= _DEV_CHAR_PARTIAL;
  else if (used & CHAR_OPT_FRAME)
    type |= _DEV_CHAR_FRAME;
  else if (used & CHAR_OPT_POLL)
    type |= _DEV_CHAR_POLL;
  else if (used & CHAR_OPT_NONBLOCK)
    type |= _DEV_CHAR_NONBLOCK;
  else
    type |= _DEV_CHAR_ALL;

  if (used & CHAR_OPT_FLUSH)
    type |= _DEV_CHAR_FLUSH;

  return type;
}

static TERMUI_CON_COMMAND_PROTOTYPE(shell_char_read)
{
  struct termui_optctx_dev_char_opts *c = ctx;

  uint8_t *data = mem_alloc(c->size, mem_scope_sys);

  if (data == NULL)
    {
      termui_con_printf(con, "invalid size\n");
      return -EINVAL;
    }

  struct dev_char_rq_s rq;
  rq.type = char_rq_type(0, used);
  rq.size = c->size;
  rq.data = data;

  dev_char_wait_rq(&c->accessor, &rq);

  if (rq.error)
    {
      termui_con_printf(con, "error %zi reading from char device\n", rq.error);
      mem_free(data);
      return -EINVAL;
    }

  size_t s_read = c->size - rq.size;
  if (used & CHAR_OPT_HEX)
    termui_con_printf(con, "read %zu bytes: %P\n", s_read, data, s_read);
  else
    termui_con_printf(con, "read %zu bytes: %S\n", s_read, data, s_read);

  mem_free(data);
  return 0;
}

static TERMUI_CON_COMMAND_PROTOTYPE(shell_char_write)
{
  struct termui_optctx_dev_char_opts *c = ctx;

  struct dev_char_rq_s rq;
  rq.type = char_rq_type(_DEV_CHAR_WRITE, used);
  rq.size = c->data.len;
  rq.data = (uint8_t*)c->data.str;

  dev_char_wait_rq(&c->accessor, &rq);

  if (rq.error)
    {
      termui_con_printf(con, "error %zi writing to char device\n", rq.error);
      return -EINVAL;
    }

  if (used & CHAR_OPT_PARTIAL)
    termui_con_printf(con, "write %zu bytes\n", c->data.len - rq.size);

  return 0;
}

#if defined(CONFIG_MUTEK_THREAD) && defined(CONFIG_MUTEK_SEMAPHORE)
# include <mutek/thread.h>
# include <mutek/semaphore.h>

struct thread_params_s
{
  struct semaphore_s sem;
  struct termui_optctx_dev_char_opts *c;
};

static CONTEXT_ENTRY(shell_thread)
{
  struct thread_params_s *p = param;
  struct device_char_s accessor;

  bool_t ok = !device_copy_accessor(&accessor.base, &p->c->accessor.base);
  semaphore_give(&p->sem, 1);
  if (ok)
    {
      mutek_shell_start(&accessor, "xterm", NULL, CONFIG_MUTEK_SHELL_PROMPT);
      device_put_accessor(&accessor.base);
    }
}

static TERMUI_CON_COMMAND_PROTOTYPE(shell_char_shell)
{
  struct thread_params_s p;
  p.c = ctx;
  semaphore_init(&p.sem, 0);
  thread_create(shell_thread, &p, NULL);
  semaphore_take(&p.sem, 1);
  semaphore_destroy(&p.sem);
  return 0;
}
#endif

static TERMUI_CON_OPT_DECL(dev_char_opts) =
{
  TERMUI_CON_OPT_DEV_ACCESSOR_ENTRY("-d", "--char-dev", CHAR_OPT_DEV,
                                    struct termui_optctx_dev_char_opts, accessor, DRIVER_CLASS_CHAR,
                                    TERMUI_CON_OPT_CONSTRAINTS(CHAR_OPT_DEV, 0)
                                    )

  TERMUI_CON_OPT_INTEGER_RANGE_ENTRY("-s", "--size", CHAR_OPT_SIZE, struct termui_optctx_dev_char_opts, size, 1, 1, 1024,
                               TERMUI_CON_OPT_CONSTRAINTS(CHAR_OPT_SIZE, 0)
                               )

  TERMUI_CON_OPT_STRING_ENTRY("-d", "--data", CHAR_OPT_DATA, struct termui_optctx_dev_char_opts, data, 1,
                              TERMUI_CON_OPT_CONSTRAINTS(CHAR_OPT_DATA, 0)
                              )

  TERMUI_CON_OPT_ENTRY("-h", "--hex", CHAR_OPT_HEX,
		       TERMUI_CON_OPT_CONSTRAINTS(CHAR_OPT_HEX, 0)
		       )

  TERMUI_CON_OPT_ENTRY("-p", "--partial", CHAR_OPT_PARTIAL,
		       TERMUI_CON_OPT_CONSTRAINTS(CHAR_OPT_FRAME | CHAR_OPT_PARTIAL | CHAR_OPT_POLL | CHAR_OPT_NONBLOCK, 0)
		       )

  TERMUI_CON_OPT_ENTRY("-f", "--frame", CHAR_OPT_FRAME,
		       TERMUI_CON_OPT_CONSTRAINTS(CHAR_OPT_FRAME | CHAR_OPT_PARTIAL | CHAR_OPT_POLL | CHAR_OPT_NONBLOCK | CHAR_OPT_FLUSH, 0)
		       )

  TERMUI_CON_OPT_ENTRY("-F", "--flush", CHAR_OPT_FLUSH,
		       TERMUI_CON_OPT_CONSTRAINTS(CHAR_OPT_FLUSH | CHAR_OPT_FRAME | CHAR_OPT_POLL | CHAR_OPT_NONBLOCK, 0)
		       )

  TERMUI_CON_OPT_ENTRY("-P", "--poll", CHAR_OPT_POLL,
		       TERMUI_CON_OPT_CONSTRAINTS(CHAR_OPT_FRAME | CHAR_OPT_PARTIAL | CHAR_OPT_POLL | CHAR_OPT_NONBLOCK | CHAR_OPT_FLUSH, 0)
		       )

  TERMUI_CON_OPT_ENTRY("-n", "--nonblock", CHAR_OPT_NONBLOCK,
		       TERMUI_CON_OPT_CONSTRAINTS(CHAR_OPT_FRAME | CHAR_OPT_PARTIAL | CHAR_OPT_POLL | CHAR_OPT_NONBLOCK, 0)
		       )

  TERMUI_CON_LIST_END
};

TERMUI_CON_GROUP_DECL(dev_shell_char_group) =
{
  TERMUI_CON_ENTRY(shell_char_read, "read",
    TERMUI_CON_OPTS_CTX(dev_char_opts,
                        CHAR_OPT_DEV | CHAR_OPT_SIZE,
                        CHAR_OPT_HEX | CHAR_OPT_PARTIAL | CHAR_OPT_FRAME | CHAR_OPT_POLL | CHAR_OPT_NONBLOCK,
                        char_opts_cleanup)
  )

  TERMUI_CON_ENTRY(shell_char_write, "write",
    TERMUI_CON_OPTS_CTX(dev_char_opts,
                        CHAR_OPT_DEV | CHAR_OPT_DATA,
                        CHAR_OPT_PARTIAL | CHAR_OPT_FRAME | CHAR_OPT_POLL | CHAR_OPT_NONBLOCK | CHAR_OPT_FLUSH,
                        char_opts_cleanup)
  )

#if defined(CONFIG_MUTEK_THREAD) && defined(CONFIG_MUTEK_SEMAPHORE)
  TERMUI_CON_ENTRY(shell_char_shell, "shell",
    TERMUI_CON_OPTS_CTX(dev_char_opts,
                        CHAR_OPT_DEV, 0,
                        char_opts_cleanup)
  )
#endif

  TERMUI_CON_LIST_END
};

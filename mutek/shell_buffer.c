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

    Copyright Alexandre Becoulet <alexandre.becoulet@free.fr> (c) 2017

*/

#include <mutek/shell.h>
#include <mutek/mem_alloc.h>
#include <termui/mutekh.h>
#include <termui/console.h>

#include <stdio.h>

void * shell_buffer_new(const struct termui_console_s *con,
                        size_t size, const char *prefix,
                        const void *type)
{
#ifdef CONFIG_MUTEK_SHELL_BUFFER
  struct mutek_shell_context_s *sctx = termui_con_get_private(con);
  struct mutek_shell_buffer_s *b = NULL;

  if (size >= 1 << 24)
    return NULL;

  LOCK_SPIN_IRQ(&sctx->lock);

  b = mem_alloc(sizeof(*b) + size, mem_scope_sys);

  if (b)
    {
      snprintf(b->name, sizeof(b->name), "%04u_%s", sctx->next_id++, prefix);
      b->name[sizeof(b->name) - 1] = 0;
      b->sctx = sctx;
      b->type = type;
      b->size = size;
      b->use = 1;
      shell_buffer_pool_push(&sctx->bufs, b);
    }

  LOCK_RELEASE_IRQ(&sctx->lock);

  return b ? b->data : NULL;
#else
  return mem_alloc(size, mem_scope_sys);
#endif
}

void * shell_buffer_reuse(const struct termui_console_s *con,
                          size_t size, const char *prefix,
                          const void *type)
{
#ifdef CONFIG_MUTEK_SHELL_BUFFER
  struct mutek_shell_context_s *sctx = termui_con_get_private(con);
  struct mutek_shell_buffer_s *b = NULL;

  LOCK_SPIN_IRQ(&sctx->lock);

  GCT_FOREACH(shell_buffer, &sctx->bufs, i, {
      if (type == i->type && !i->use &&
          size + sizeof(*b) <= mem_getsize(i))
        {
          b = i;
          b->use = 1;
          b->size = size;
          GCT_FOREACH_BREAK;
        }
  });

  LOCK_RELEASE_IRQ(&sctx->lock);

  if (b)
    return b->data;

  return shell_buffer_new(con, size, prefix, type);
#else
  return mem_alloc(size, mem_scope_sys);
#endif
}

void shell_buffer_drop(void * data)
{
#ifdef CONFIG_MUTEK_SHELL_BUFFER
  struct mutek_shell_buffer_s *b = data;
  b--;
  struct mutek_shell_context_s *sctx = b->sctx;

  LOCK_SPIN_IRQ(&sctx->lock);
  ensure(b->use--);
  LOCK_RELEASE_IRQ(&sctx->lock);
#else
  mem_free(data);
#endif
}

void shell_buffer_advertise(struct termui_console_s *con,
                            void *data, size_t size)
{
#ifdef CONFIG_MUTEK_SHELL_BUFFER
  termui_con_printf(con, "%u bytes of data are available in buffer `%s'.\n",
                    size, shell_buffer_name(data));
#else
  termui_con_printf(con, "%u bytes of data are available: `%P'.\n",
                    size, data, size);
#endif
}

#ifdef CONFIG_MUTEK_SHELL_BUFFER

const char * shell_buffer_name(void * data)
{
  struct mutek_shell_buffer_s *b = data;
  b--;
  ensure(b->use);
  return b->name;
}

void * shell_buffer_get(const struct termui_console_s *con,
                        const char *name, size_t *size, const void *type)
{
  struct mutek_shell_context_s *sctx = termui_con_get_private(con);
  struct mutek_shell_buffer_s *b;

  LOCK_SPIN_IRQ(&sctx->lock);
  b = shell_buffer_pool_lookup(&sctx->bufs, name);

  if (b->use)
    b = NULL;

  LOCK_RELEASE_IRQ(&sctx->lock);

  if (!b || (type && b->type != type))
    return NULL;

  b->use = 1;
  if (size)
    *size = b->size;

  return b->data;
}

bool_t shell_buffer_collect_all(struct mutek_shell_context_s *sctx)
{
  bool_t r = 0;

  GCT_FOREACH(shell_buffer, &sctx->bufs, i, {
      if (!i->use)
        {
          mem_free(i);
          GCT_FOREACH_DROP;
        }
      r = 1;
  });

  return r;
}

void shell_buffer_collect(const struct termui_console_s *con,
                          const char *name)
{
  struct mutek_shell_context_s *sctx = termui_con_get_private(con);
  struct mutek_shell_buffer_s *b;

  LOCK_SPIN_IRQ(&sctx->lock);

  if (name)
    {
      b = shell_buffer_pool_lookup(&sctx->bufs, name);
      if (!b->use)
        {
          shell_buffer_pool_remove(&sctx->bufs, b);
          mem_free(b);
        }
    }
  else
    {
      shell_buffer_collect_all(sctx);
    }

  LOCK_RELEASE_IRQ(&sctx->lock);
}

#endif

TERMUI_CON_PARSE_OPT_PROTOTYPE(shell_opt_buffer_get_parse)
{
  struct shell_opt_buffer_desc_s *optd = (void*)opt;

  struct shell_opt_buffer_s *ob = (void*)((uint8_t*)ctx + optd->offset);

  if (!strncmp(argv[0], "raw:", 4))
    {
      ob->addr = argv[0] + 4;
      ob->size = argl[0] - 4;
      ob->buffered = 0;
    }
  else
    {
#ifdef CONFIG_MUTEK_SHELL_BUFFER
      size_t s;
      void *a = shell_buffer_get(con, argv[0], &s, optd->type);
      if (!a)
        return -ECANCELED;

      ob->size = s;
      ob->addr = a;
      ob->buffered = 1;
#else
      return -ECANCELED;
#endif
    }

  return 0;
}

#ifdef CONFIG_LIBTERMUI_CON_COMPLETION
TERMUI_CON_ARGS_COLLECT_PROTOTYPE(shell_opt_buffer_comp)
{
# ifdef CONFIG_MUTEK_SHELL_BUFFER
  struct mutek_shell_context_s *sctx = termui_con_get_private(con);
  struct shell_opt_buffer_desc_s *optd = (void*)entry;
  bool_t match = 0;

  GCT_FOREACH(shell_buffer, &sctx->bufs, i, {
      if (optd->type && optd->type != i->type)
        GCT_FOREACH_CONTINUE;
      if (termui_con_comp_match(cctx, i->name, NULL, 0) <= 0)
        GCT_FOREACH_CONTINUE;
      if (!termui_con_comp_append(cctx, i->name))
        return NULL;
      match = 1;
  });

  if (!match)
# endif
    {
      termui_con_comp_append(cctx, "raw:");
      cctx->suffix = 0;
    }

  return NULL;
}

#endif

# ifdef CONFIG_MUTEK_SHELL_BUFFER

struct termui_optctx_shell_buffer_opts
{
  struct termui_con_string_s bufname;
  struct termui_con_string_s data;
  size_t size;
  uintptr_t offset;
};

enum
{
  BUFFER_OPT_BUFNAME = 1,
  BUFFER_OPT_DATA = 2,
  BUFFER_OPT_SIZE = 4,
  BUFFER_OPT_OFFSET = 8,
  BUFFER_OPT_DELETE = 16,
  BUFFER_OPT_INSERT = 32,
};

static TERMUI_CON_OPT_DECL(shell_buffer_opts) =
{
  TERMUI_CON_OPT_STRING_ENTRY("-n", "--name", BUFFER_OPT_BUFNAME,
                              struct termui_optctx_shell_buffer_opts, bufname, 1,
                              TERMUI_CON_OPT_COMPLETE(shell_opt_buffer_comp, NULL)
                              TERMUI_CON_OPT_CONSTRAINTS(BUFFER_OPT_BUFNAME, 0))

  TERMUI_CON_OPT_STRING_ENTRY("-d", "--data", BUFFER_OPT_DATA,
                              struct termui_optctx_shell_buffer_opts, data, 1,
                              TERMUI_CON_OPT_CONSTRAINTS(BUFFER_OPT_DATA | BUFFER_OPT_SIZE, 0))

  TERMUI_CON_OPT_INTEGER_ENTRY("-s", "--size", BUFFER_OPT_SIZE,
                               struct termui_optctx_shell_buffer_opts, size, 1,
                               TERMUI_CON_OPT_CONSTRAINTS(BUFFER_OPT_SIZE | BUFFER_OPT_DATA, 0))

  TERMUI_CON_OPT_INTEGER_ENTRY("-o", "--offset", BUFFER_OPT_OFFSET,
                               struct termui_optctx_shell_buffer_opts, offset, 1,
                               TERMUI_CON_OPT_CONSTRAINTS(BUFFER_OPT_OFFSET, 0))

  TERMUI_CON_OPT_ENTRY("-i", "--insert", BUFFER_OPT_INSERT,
                       TERMUI_CON_OPT_CONSTRAINTS(BUFFER_OPT_DELETE | BUFFER_OPT_INSERT, 0))

  TERMUI_CON_OPT_ENTRY("-d", "--delete", BUFFER_OPT_DELETE,
                       TERMUI_CON_OPT_CONSTRAINTS(BUFFER_OPT_DELETE | BUFFER_OPT_INSERT | BUFFER_OPT_DATA, 0))

  TERMUI_CON_LIST_END
};

static TERMUI_CON_COMMAND_PROTOTYPE(shell_buffer_cmd_list)
{
  struct mutek_shell_context_s *sctx = termui_con_get_private(con);

  termui_con_printf(con, " name               used  size       type    address\n");
  GCT_FOREACH(shell_buffer, &sctx->bufs, i, {
      termui_con_printf(con, "%-20s %3u %5u %p %p\n",
                        i->name, i->use,
                        i->size, i->type, i->data);
  });

  return 0;
}

static TERMUI_CON_COMMAND_PROTOTYPE(shell_buffer_cmd_hexdump)
{
  struct termui_optctx_shell_buffer_opts *c = ctx;

  size_t bsize;
  uint8_t *data = shell_buffer_get(con, c->bufname.str, &bsize, NULL);
  if (!data)
    return -EINVAL;

  size_t size = __MIN(bsize, 512);
  if (used & BUFFER_OPT_SIZE)
    size = c->size;

  if (c->offset > bsize)
    size = 0;
  else if (c->offset + size > bsize)
    size = bsize - c->offset;

  uintptr_t i = c->offset;

  if (size != bsize)
    termui_con_printf(con, "Dumping %u bytes of %u.\n", size, bsize);

  while (size)
    {
      size_t s = __MIN(16, size);
      termui_con_printf(con, "%p: %P\n", i, data + i, s);
      size -= s;
      i += s;
    }

  shell_buffer_drop(data);
  return 0;
}

static TERMUI_CON_COMMAND_PROTOTYPE(shell_buffer_cmd_collect)
{
  struct termui_optctx_shell_buffer_opts *c = ctx;
  shell_buffer_collect(con, c->bufname.str);
  return 0;
}

static TERMUI_CON_COMMAND_PROTOTYPE(shell_buffer_cmd_new)
{
  struct termui_optctx_shell_buffer_opts *c = ctx;

  size_t size = used & BUFFER_OPT_SIZE ? c->size : c->data.len;
  const char *prefix = used & BUFFER_OPT_BUFNAME ? c->bufname.str : "noname";
  void *r = shell_buffer_new(con, size, prefix, NULL);

  if (!r)
    return -EINVAL;

  if (used & BUFFER_OPT_DATA)
    memcpy(r, c->data.str, size);

  shell_buffer_drop(r);
  return 0;
}

static TERMUI_CON_COMMAND_PROTOTYPE(shell_buffer_cmd_edit)
{
  struct mutek_shell_context_s *sctx = termui_con_get_private(con);
  struct termui_optctx_shell_buffer_opts *c = ctx;
  struct mutek_shell_buffer_s *b;
  error_t err = -EINVAL;

  LOCK_SPIN_IRQ(&sctx->lock);
  b = shell_buffer_pool_lookup(&sctx->bufs, c->bufname.str);

  if (b && !b->use)
    {
      size_t s = used & BUFFER_OPT_SIZE ? c->size : c->data.len;
      uintptr_t o = c->offset;

      if (used & BUFFER_OPT_INSERT)
        {
          if (mem_getsize(b) < b->size + s)
            goto err;
          memmove(b->data + o + s,
                  b->data + o,
                  b->size - o);
          b->size += s;
        }
      else if (used & BUFFER_OPT_DELETE)
        {
          if (o + s > b->size)
            goto err;
          b->size -= s;
          memmove(b->data + o,
                  b->data + o + s,
                  b->size - o);
        }
      else
        {
          if (mem_getsize(b) < o + s)
            goto err;
          if ((used & BUFFER_OPT_SIZE) || o + s > b->size)
            b->size = o + s;
        }

      if (used & BUFFER_OPT_DATA)
        memcpy(b->data + o, c->data.str, s);
    }

  err = 0;
 err:;
  LOCK_RELEASE_IRQ(&sctx->lock);

  return err;
}

static TERMUI_CON_GROUP_DECL(shell_buffer_subgroup) =
{
  TERMUI_CON_ENTRY(shell_buffer_cmd_list, "list",
  )

  TERMUI_CON_ENTRY(shell_buffer_cmd_hexdump, "hexdump",
    TERMUI_CON_OPTS_CTX(shell_buffer_opts,
                        BUFFER_OPT_BUFNAME, BUFFER_OPT_OFFSET | BUFFER_OPT_SIZE, NULL)
  )

  TERMUI_CON_ENTRY(shell_buffer_cmd_collect, "collect",
    TERMUI_CON_OPTS_CTX(shell_buffer_opts,
                        0, BUFFER_OPT_BUFNAME, NULL)
  )

  TERMUI_CON_ENTRY(shell_buffer_cmd_new, "new",
    TERMUI_CON_OPTS_CTX(shell_buffer_opts,
                        BUFFER_OPT_SIZE | BUFFER_OPT_DATA, BUFFER_OPT_BUFNAME, NULL)
  )

  TERMUI_CON_ENTRY(shell_buffer_cmd_edit, "edit",
    TERMUI_CON_OPTS_CTX(shell_buffer_opts,
                        BUFFER_OPT_SIZE | BUFFER_OPT_DATA | BUFFER_OPT_BUFNAME,
                        BUFFER_OPT_INSERT | BUFFER_OPT_DELETE | BUFFER_OPT_OFFSET, NULL)
  )

  TERMUI_CON_LIST_END
};

MUTEK_SHELL_ROOT_GROUP(shell_buffer_subgroup, "buffer")

#endif

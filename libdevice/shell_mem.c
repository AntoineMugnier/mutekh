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

    Copyright Alexandre Becoulet <alexandre.becoulet@free.fr> (c) 2014

*/

#include <device/shell.h>
#include <device/device.h>
#include <device/driver.h>

#include <mutek/mem_alloc.h>
#include <device/class/mem.h>

enum mem_opts_e
{
  MEM_OPT_DEV    = 0x01,
  MEM_OPT_BAND   = 0x02,
  MEM_OPT_ADDR   = 0x04,
  MEM_OPT_DATA   = 0x08,
  MEM_OPT_SIZE   = 0x10,
  MEM_OPT_PAGE   = 0x20,
  MEM_OPT_ALL    = 0x40
};

struct termui_optctx_dev_mem_opts
{
  struct device_mem_s mem;
  uint_fast8_t band;
  uintptr_t addr;
  struct shell_opt_buffer_s data;
  uintptr_t size;
  uintptr_t page;
};

static TERMUI_CON_ARGS_CLEANUP_PROTOTYPE(mem_opts_cleanup)
{
  struct termui_optctx_dev_mem_opts *c = ctx;

  if (c->data.buffered)
    shell_buffer_drop(c->data.addr);

  if (device_check_accessor(&c->mem.base))
      device_put_accessor(&c->mem.base);
}

static TERMUI_CON_COMMAND_PROTOTYPE(dev_shell_mem_info)
{
  struct termui_optctx_dev_mem_opts *c = ctx;
  uint_fast8_t i;

  for (i = 0; ; i++)
    {
      struct dev_mem_info_s info;
      if (DEVICE_OP(&c->mem, info, &info, i))
        return 0;
      if (i > 0)
        termui_con_printf(con, "Band %u:\n", i);
      termui_con_printf(con,
                        "  Type        : ");
      termui_con_print_enum(con, dev_mem_type_e, info.type);
      termui_con_printf(con, "\n"
                        "  Flags       : ");
      termui_con_print_enum(con, dev_mem_flags_e, info.flags);
      termui_con_printf(con, "\n"
                        "  Size        : %llu\n",
                        info.size);
      if (info.flags & (DEV_MEM_MAPPED_READ | DEV_MEM_MAPPED_WRITE))
        termui_con_printf(con, "  Mapped at   : %p\n", info.map_base);
      if (info.flags & (DEV_MEM_PAGE_READ | DEV_MEM_PAGE_WRITE))
        termui_con_printf(con, "  Page size   : %u\n", 1 << info.page_log2);
      if (info.read_cycles_m)
        termui_con_printf(con, "  Read cycles : %u\n", info.read_cycles_m << info.read_cycles_p);
      if (info.flags & (DEV_MEM_ERASE_ONE | DEV_MEM_ERASE_ZERO))
        {
          termui_con_printf(con, "  Erase size  : %u\n", 1 << info.erase_log2);
          if (info.erase_cycles_m)
            termui_con_printf(con, "  Erase cycles: %u\n", info.erase_cycles_m << info.erase_cycles_p);
          if ((info.flags & (DEV_MEM_PARTIAL_READ | DEV_MEM_PARTIAL_WRITE)) && info.partial_write)
            termui_con_printf(con, "  Partial Writes : %u\n", info.partial_write);
        }
      if (info.flags & (DEV_MEM_PARTIAL_READ | DEV_MEM_PARTIAL_WRITE))
        termui_con_printf(con, "  Partial access: %u byte(s)\n", 1 << info.partial_log2);
    }
}

static error_t dev_shell_set_rq(struct dev_mem_rq_s *rq, size_t *sc_cnt,
                                const struct termui_optctx_dev_mem_opts *c,
                                uint_fast16_t used, size_t size)
{
  if (used & MEM_OPT_PAGE)
    {
      size_t psize = c->page;
      if (!is_pow2(psize) || psize > size || (size & (psize - 1)))
        return -EINVAL;
      rq->type |= _DEV_MEM_PAGE;
      rq->page.page_log2 = bit_msb_index(psize);
      size_t c = size >> rq->page.page_log2;
      *sc_cnt = c;
      rq->page.sc_count = c;
    }
  else if (used & MEM_OPT_ALL)
    {
      rq->type |= _DEV_MEM_ALL;
    }
  else
    {
      rq->type |= _DEV_MEM_PARTIAL;
      rq->partial.addr = c->addr;
      rq->partial.size = size;
    }

  return 0;
}

static void dev_shell_set_sc(struct dev_mem_rq_s *rq, uint8_t *data,
                             struct dev_mem_page_sc_s sc[], size_t sc_cnt,
                             const struct termui_optctx_dev_mem_opts *c)
{
  if (sc_cnt)
    {
      uintptr_t a = 0;
      uint_fast8_t i;
      for (i = 0; i < sc_cnt; i++)
        {
          sc[i].addr = c->addr + a;
          sc[i].data = data + a;
          a += c->page;
        }
      rq->page.sc = sc;
    }
  else
    {
      rq->partial.data = data;
    }
}

static TERMUI_CON_COMMAND_PROTOTYPE(dev_shell_mem_read)
{
  struct termui_optctx_dev_mem_opts *c = ctx;
  struct dev_mem_rq_s rq;
  size_t size = c->size;

  void *buf = shell_buffer_new(con, size, "mem", NULL, 0);
  void *data = buf;
  if (!data)
    return -EINVAL;

  rq.type = _DEV_MEM_READ;
  rq.band_mask = 1 << c->band;

  size_t sc_cnt = 0;
  error_t err = -EINVAL;

  if (!dev_shell_set_rq(&rq, &sc_cnt, c, used, size))
    {
      struct dev_mem_page_sc_s sc[sc_cnt];
      dev_shell_set_sc(&rq, data, sc, sc_cnt, c);

      if ((err = dev_mem_wait_rq(&c->mem, &rq)))
        {
          termui_con_printf(con, "error %i\n", err);
          err = -EINVAL;
        }
      else
        {
          shell_buffer_advertise(con, buf, size);
          err = 0;
        }
    }

  shell_buffer_drop(buf);
  return err;
}

static TERMUI_CON_COMMAND_PROTOTYPE(dev_shell_mem_write)
{
  struct termui_optctx_dev_mem_opts *c = ctx;
  struct dev_mem_rq_s rq;
  size_t size = c->data.size;
  uint8_t *data = c->data.addr;

  if (used & MEM_OPT_SIZE)
    {
      if (size < c->size)
        return -EINVAL;
      size = c->size;
    }

  rq.type = _DEV_MEM_WRITE;
  rq.band_mask = 1 << c->band;

  size_t sc_cnt = 0;
  if (dev_shell_set_rq(&rq, &sc_cnt, c, used, size))
    return -EINVAL;
  struct dev_mem_page_sc_s sc[sc_cnt];
  dev_shell_set_sc(&rq, data, sc, sc_cnt, c);

  error_t err = dev_mem_wait_rq(&c->mem, &rq);

  if (err)
    termui_con_printf(con, "error %i\n", err);

  return err ? -EINVAL : 0;
}

static TERMUI_CON_COMMAND_PROTOTYPE(dev_shell_mem_erase)
{
  struct termui_optctx_dev_mem_opts *c = ctx;
  struct dev_mem_rq_s rq;
  size_t size = c->size;

  if (!(used & MEM_OPT_SIZE))
    size = c->page;

  rq.type = _DEV_MEM_ERASE;
  rq.band_mask = 1 << c->band;

  size_t sc_cnt = 0;
  if (dev_shell_set_rq(&rq, &sc_cnt, c, used, size))
    return -EINVAL;
  struct dev_mem_page_sc_s sc[sc_cnt];
  dev_shell_set_sc(&rq, NULL, sc, sc_cnt, c);

  error_t err = dev_mem_wait_rq(&c->mem, &rq);

  if (err)
    termui_con_printf(con, "error %i\n", err);

  return err ? -EINVAL : 0;
}

static TERMUI_CON_OPT_DECL(dev_mem_opts) =
{
  TERMUI_CON_OPT_DEV_ACCESSOR_ENTRY("-d", "--mem-dev", MEM_OPT_DEV,
                                    struct termui_optctx_dev_mem_opts, mem, DRIVER_CLASS_MEM,
                                    TERMUI_CON_OPT_CONSTRAINTS(MEM_OPT_DEV, 0)
                                    )

  TERMUI_CON_OPT_INTEGER_RANGE_ENTRY("-B", "--band", MEM_OPT_BAND, struct termui_optctx_dev_mem_opts,
                                     band, 1, 0, 255,
                                     TERMUI_CON_OPT_CONSTRAINTS(MEM_OPT_BAND, 0))

  TERMUI_CON_OPT_INTEGER_ENTRY("-a", "--addr", MEM_OPT_ADDR, struct termui_optctx_dev_mem_opts, addr, 1,
                              TERMUI_CON_OPT_CONSTRAINTS(MEM_OPT_ADDR, 0))

  TERMUI_CON_OPT_SHELL_BUFFER_RAW_ENTRY("-D", "--data", MEM_OPT_DATA, struct termui_optctx_dev_mem_opts, data, NULL,
                              TERMUI_CON_OPT_CONSTRAINTS(MEM_OPT_DATA, 0))

  TERMUI_CON_OPT_INTEGER_ENTRY("-s", "--size", MEM_OPT_SIZE, struct termui_optctx_dev_mem_opts, size, 1,
                               TERMUI_CON_OPT_CONSTRAINTS(MEM_OPT_SIZE, 0))

  TERMUI_CON_OPT_INTEGER_ENTRY("-p", "--page-size", MEM_OPT_PAGE, struct termui_optctx_dev_mem_opts, page, 1,
                               TERMUI_CON_OPT_CONSTRAINTS(MEM_OPT_PAGE | MEM_OPT_ALL, MEM_OPT_ADDR))

  TERMUI_CON_OPT_ENTRY("-A", "--all", MEM_OPT_ALL,
		       TERMUI_CON_OPT_CONSTRAINTS(MEM_OPT_ALL | MEM_OPT_PAGE, 0)
		       )

  TERMUI_CON_LIST_END
};

TERMUI_CON_GROUP_DECL(dev_shell_mem_group) =
{
  TERMUI_CON_ENTRY(dev_shell_mem_info, "info",
		   TERMUI_CON_OPTS_CTX(dev_mem_opts, MEM_OPT_DEV, 0, mem_opts_cleanup)
                   )

  TERMUI_CON_ENTRY(dev_shell_mem_write, "write",
		   TERMUI_CON_OPTS_CTX(dev_mem_opts, MEM_OPT_DEV | MEM_OPT_DATA | MEM_OPT_ADDR,
                                       MEM_OPT_BAND | MEM_OPT_PAGE | MEM_OPT_SIZE, mem_opts_cleanup)
                   )

  TERMUI_CON_ENTRY(dev_shell_mem_read, "read",
		   TERMUI_CON_OPTS_CTX(dev_mem_opts, MEM_OPT_DEV | MEM_OPT_SIZE | MEM_OPT_ADDR,
                                       MEM_OPT_BAND | MEM_OPT_PAGE, mem_opts_cleanup)
                   )

  TERMUI_CON_ENTRY(dev_shell_mem_erase, "erase",
		   TERMUI_CON_OPTS_CTX(dev_mem_opts, MEM_OPT_DEV | MEM_OPT_PAGE | MEM_OPT_ALL,
                                       MEM_OPT_BAND | MEM_OPT_SIZE | MEM_OPT_ADDR, mem_opts_cleanup)
                   )

  TERMUI_CON_LIST_END
};


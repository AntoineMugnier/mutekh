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

#include <device/class/mem.h>

enum mem_opts_e
{
  MEM_OPT_DEV    = 0x01,
  MEM_OPT_BAND   = 0x02,
  MEM_OPT_RQ     = 0x04,
  MEM_OPT_ADDR   = 0x08,
  MEM_OPT_DATA   = 0x10,
  MEM_OPT_BUFFER = 0x20,
  MEM_OPT_SIZE   = 0x40,
  MEM_OPT_SCLOG2 = 0x80
};

struct termui_optctx_dev_mem_opts
{
  struct device_mem_s mem;
  uint_fast8_t band;
  enum dev_mem_rq_type_e rqtype;
  uintptr_t addr;
  uint8_t sc_log2;
  union {
    struct {
      uintptr_t buffer;
      uintptr_t size;
    };
    struct termui_con_string_s data;
  };
};

static TERMUI_CON_ARGS_CLEANUP_PROTOTYPE(mem_opts_cleanup)
{
  struct termui_optctx_dev_mem_opts *c = ctx;

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
      if (info.page_log2)
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

static TERMUI_CON_COMMAND_PROTOTYPE(dev_shell_mem_request)
{
  struct termui_optctx_dev_mem_opts *c = ctx;

  struct dev_mem_info_s info;
  if (DEVICE_OP(&c->mem, info, &info, c->band))
    return -EINVAL;

  struct dev_mem_rq_s rq;

  rq.type = c->rqtype;
  rq.band_mask = 1 << c->band;
  rq.sc_log2 = c->sc_log2;
  rq.addr = c->addr;

  size_t sc = c->size >> c->sc_log2;
  uint8_t *d[sc];

  if (c->rqtype & (DEV_MEM_OP_PAGE_ERASE | DEV_MEM_OP_PAGE_READ | DEV_MEM_OP_PAGE_WRITE))
    {
      if (used & MEM_OPT_DATA)
        return -EINVAL;
      size_t i;
      for (i = 0; i < sc; i++)
        d[i] = (uint8_t*)c->buffer + (i << (info.page_log2 + c->sc_log2));
      rq.sc_data = d;
      rq.size = c->size;
    }
  else
    {
      if (used & MEM_OPT_BUFFER)
        {
          rq.data = (uint8_t*)c->data.str;
          rq.size = c->data.len;
        }
      else
        {
          if (c->rqtype & DEV_MEM_OP_PARTIAL_READ)
            return -EINVAL;
          rq.data = (uint8_t*)c->buffer;
          rq.size = c->size;
        }
    }

  termui_con_printf(con, "%i\n", dev_mem_wait_op(&c->mem, &rq));

  return 0;
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

  TERMUI_CON_OPT_ENUM_ENTRY("-r", "--request", MEM_OPT_RQ, struct termui_optctx_dev_mem_opts,
                            rqtype, dev_mem_rq_type_e,
                            TERMUI_CON_OPT_CONSTRAINTS(MEM_OPT_RQ, 0))

  TERMUI_CON_OPT_INTEGER_ENTRY("-a", "--addr", MEM_OPT_ADDR, struct termui_optctx_dev_mem_opts, addr, 1,
                              TERMUI_CON_OPT_CONSTRAINTS(MEM_OPT_ADDR, 0))

  TERMUI_CON_OPT_STRING_ENTRY("-d", "--data", MEM_OPT_DATA, struct termui_optctx_dev_mem_opts, data, 1,
                              TERMUI_CON_OPT_CONSTRAINTS(MEM_OPT_DATA | MEM_OPT_BUFFER | MEM_OPT_SIZE, 0))

  TERMUI_CON_OPT_INTEGER_ENTRY("-b", "--buffer", MEM_OPT_BUFFER, struct termui_optctx_dev_mem_opts, buffer, 1,
                               TERMUI_CON_OPT_CONSTRAINTS(MEM_OPT_DATA | MEM_OPT_BUFFER, MEM_OPT_SIZE))

  TERMUI_CON_OPT_INTEGER_ENTRY("-s", "--size", MEM_OPT_SIZE, struct termui_optctx_dev_mem_opts, size, 1,
                               TERMUI_CON_OPT_CONSTRAINTS(MEM_OPT_SIZE, 0))

  TERMUI_CON_OPT_INTEGER_ENTRY("-S", "--sc-log2", MEM_OPT_SCLOG2, struct termui_optctx_dev_mem_opts, sc_log2, 1,
                               TERMUI_CON_OPT_CONSTRAINTS(MEM_OPT_SCLOG2, 0))

  TERMUI_CON_LIST_END
};

TERMUI_CON_GROUP_DECL(dev_shell_mem_group) =
{
  TERMUI_CON_ENTRY(dev_shell_mem_info, "info",
		   TERMUI_CON_OPTS_CTX(dev_mem_opts, MEM_OPT_DEV , MEM_OPT_DEV, mem_opts_cleanup)
                   )

  TERMUI_CON_ENTRY(dev_shell_mem_request, "request",
		   TERMUI_CON_OPTS_CTX(dev_mem_opts, MEM_OPT_DEV | MEM_OPT_RQ |
                                       MEM_OPT_ADDR | MEM_OPT_DATA | MEM_OPT_BUFFER,
                                       MEM_OPT_BAND | MEM_OPT_SIZE | MEM_OPT_SCLOG2, mem_opts_cleanup)
                   )

  TERMUI_CON_LIST_END
};


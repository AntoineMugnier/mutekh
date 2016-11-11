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

    Copyright Nicolas Pouillon <nipo@ssji.net> (c) 2016
*/

#include <device/class/mem.h>
#include <device/driver.h>
#include <device/resources.h>
#include <device/shell.h>

#include <mutek/mem_alloc.h>
#include <mutek/console.h>
#include <mutek/shell.h>
#include <hexo/enum.h>

enum smi_opts_e
{
  SMI_OPT_DEV       = 0x01,
  SMI_OPT_REG       = 0x02,
  SMI_OPT_VALUE     = 0x04,
};

struct termui_optctx_smi_opts_s
{
  struct device_mem_s accessor;

  struct { 
    uint16_t reg;
    uint16_t value;
  } req;
};

static TERMUI_CON_ARGS_CLEANUP_PROTOTYPE(smi_opts_cleanup)
{
  struct termui_optctx_smi_opts_s *c = ctx;

  if (device_check_accessor(&c->accessor.base))
    device_put_accessor(&c->accessor.base);
}

static TERMUI_CON_COMMAND_PROTOTYPE(shell_smi_set)
{
  struct termui_optctx_smi_opts_s *c = ctx;
  struct dev_mem_rq_s rq;

  rq.type = DEV_MEM_OP_PARTIAL_WRITE;
  rq.band_mask = 1;
  rq.sc_log2 = 0;
  rq.size = 2;
  rq.data = (void*)&c->req.value;
  rq.addr = c->req.reg * 2;

#if defined(CONFIG_MUTEK_CONTEXT_SCHED)
  error_t err = dev_mem_wait_op(&c->accessor, &rq);
#else
  error_t err = dev_mem_spin_op(&c->accessor, &rq);
#endif

  if (err)
    termui_con_printf(con, "Request failed with error: %d\n", err);

  return err;
}

static TERMUI_CON_COMMAND_PROTOTYPE(shell_smi_get)
{
  struct termui_optctx_smi_opts_s *c = ctx;
  struct dev_mem_rq_s rq;
  uint16_t value;

  rq.type = DEV_MEM_OP_PARTIAL_READ;
  rq.band_mask = 1;
  rq.sc_log2 = 0;
  rq.size = 2;
  rq.data = (void*)&value;
  rq.addr = c->req.reg * 2;

#if defined(CONFIG_MUTEK_CONTEXT_SCHED)
  error_t err = dev_mem_wait_op(&c->accessor, &rq);
#else
  error_t err = dev_mem_spin_op(&c->accessor, &rq);
#endif

  if (err)
    termui_con_printf(con, "Request failed with error: %d\n", err);
  else
    termui_con_printf(con, "Value %d : %04x\n",
                      c->req.reg, value);

  return err;
}

static TERMUI_CON_OPT_DECL(smi_opts_s) =
{
  TERMUI_CON_OPT_DEV_ACCESSOR_ENTRY("-d", "--dev", SMI_OPT_DEV,
                                    struct termui_optctx_smi_opts_s, accessor, DRIVER_CLASS_MEM,
                                    TERMUI_CON_OPT_CONSTRAINTS(SMI_OPT_DEV, 0)
                                    )

  TERMUI_CON_OPT_INTEGER_RANGE_ENTRY("-r", "--reg", SMI_OPT_REG, struct termui_optctx_smi_opts_s, req.reg, 1, 0, 32,
                                     TERMUI_CON_OPT_CONSTRAINTS(SMI_OPT_REG, SMI_OPT_REG | SMI_OPT_DEV))

  TERMUI_CON_OPT_INTEGER_RANGE_ENTRY("-v", "--value", SMI_OPT_VALUE, struct termui_optctx_smi_opts_s, req.value, 1, 0, 0xffff,
                                     TERMUI_CON_OPT_CONSTRAINTS(SMI_OPT_VALUE, SMI_OPT_VALUE | SMI_OPT_DEV | SMI_OPT_DEV))

  TERMUI_CON_LIST_END
};

TERMUI_CON_GROUP_DECL(dev_shell_smi_group) =
{
  TERMUI_CON_ENTRY(shell_smi_get, "get",
    TERMUI_CON_OPTS_CTX(smi_opts_s,
                        SMI_OPT_DEV | SMI_OPT_REG,
                        SMI_OPT_DEV | SMI_OPT_REG,
                        smi_opts_cleanup)
  )

  TERMUI_CON_ENTRY(shell_smi_set, "set",
    TERMUI_CON_OPTS_CTX(smi_opts_s,
                        SMI_OPT_DEV | SMI_OPT_REG | SMI_OPT_VALUE,
                        SMI_OPT_DEV | SMI_OPT_REG | SMI_OPT_VALUE,
                        smi_opts_cleanup)
  )

  TERMUI_CON_LIST_END
};

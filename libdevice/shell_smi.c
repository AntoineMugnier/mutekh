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

    Copyright Nicolas Pouillon <nipo@ssji.net> (c) 2019
*/

#include <device/class/smi.h>
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
  SMI_OPT_CLAUSE    = 0x02,
  SMI_OPT_PRTAD     = 0x04,
  SMI_OPT_DEVAD     = 0x08,
  SMI_OPT_ADDRESS   = 0x10,
  SMI_OPT_VALUE     = 0x20,
};

struct termui_optctx_smi_opts_s
{
  struct device_smi_s accessor;

  enum dev_smi_clause_e clause;
  enum dev_smi_op_e op;
  uint8_t prtad;
  uint8_t devad;
  uint16_t address;
  uint16_t value;

  struct dev_smi_transfer_rq_s transfer;
};

static TERMUI_CON_ARGS_CLEANUP_PROTOTYPE(smi_opts_cleanup)
{
  struct termui_optctx_smi_opts_s *c = ctx;

  if (device_check_accessor(&c->accessor.base))
    device_put_accessor(&c->accessor.base);
}

static TERMUI_CON_COMMAND_PROTOTYPE(shell_smi_write)
{
  struct termui_optctx_smi_opts_s *c = ctx;
  error_t err;

  if (!(used & SMI_OPT_DEVAD))
    c->devad = 0;

  c->transfer.data.clause = c->clause;
  c->transfer.data.op = c->op;
  c->transfer.data.prtad = c->prtad;
  c->transfer.data.devad = c->devad;
  c->transfer.data.address = c->address;
  c->transfer.data.value = c->value;
  
  err = dev_smi_wait_rq(&c->accessor, &c->transfer.base);

  if (err)
    termui_con_printf(con, "Request failed with error: %d\n", err);

  return err;
}

static TERMUI_CON_COMMAND_PROTOTYPE(shell_smi_read)
{
  struct termui_optctx_smi_opts_s *c = ctx;
  error_t err;

  if (!(used & SMI_OPT_DEVAD))
    c->devad = 0;

  c->transfer.data.clause = c->clause;
  c->transfer.data.op = c->op;
  c->transfer.data.prtad = c->prtad;
  c->transfer.data.devad = c->devad;
  c->transfer.data.address = c->address;
  c->transfer.data.value = c->value;

  err = dev_smi_wait_rq(&c->accessor, &c->transfer.base);

  if (err)
    termui_con_printf(con, "Request failed with error: %d\n", err);
  else
    termui_con_printf(con, "Value : %04x\n", c->transfer.data.value);

  return err;
}

static TERMUI_CON_OPT_DECL(smi_opts_s) =
{
  TERMUI_CON_OPT_DEV_ACCESSOR_ENTRY("-d", "--dev", SMI_OPT_DEV,
                                    struct termui_optctx_smi_opts_s, accessor, DRIVER_CLASS_SMI,
                                    TERMUI_CON_OPT_CONSTRAINTS(SMI_OPT_DEV, 0)
                                    )

  TERMUI_CON_OPT_ENUM_ENTRY("-c", "--clause", SMI_OPT_CLAUSE,
                            struct termui_optctx_smi_opts_s, clause, dev_smi_clause_e,
                            TERMUI_CON_OPT_CONSTRAINTS(SMI_OPT_CLAUSE, SMI_OPT_CLAUSE | SMI_OPT_DEV))

  TERMUI_CON_OPT_INTEGER_RANGE_ENTRY("-p", "--prtad", SMI_OPT_PRTAD, struct termui_optctx_smi_opts_s, prtad, 1, 0, 31,
                                     TERMUI_CON_OPT_CONSTRAINTS(SMI_OPT_PRTAD, SMI_OPT_CLAUSE | SMI_OPT_PRTAD | SMI_OPT_DEV))

  TERMUI_CON_OPT_INTEGER_RANGE_ENTRY("-d", "--devad", SMI_OPT_DEVAD, struct termui_optctx_smi_opts_s, devad, 1, 0, 31,
                                     TERMUI_CON_OPT_CONSTRAINTS(SMI_OPT_DEVAD, SMI_OPT_CLAUSE | SMI_OPT_DEVAD | SMI_OPT_PRTAD | SMI_OPT_DEV))

  TERMUI_CON_OPT_INTEGER_RANGE_ENTRY("-a", "--address", SMI_OPT_ADDRESS, struct termui_optctx_smi_opts_s, address, 1, 0, 0xffff,
                                     TERMUI_CON_OPT_CONSTRAINTS(SMI_OPT_ADDRESS, SMI_OPT_CLAUSE | SMI_OPT_ADDRESS | SMI_OPT_DEVAD | SMI_OPT_PRTAD | SMI_OPT_DEV))

  TERMUI_CON_OPT_INTEGER_RANGE_ENTRY("-v", "--value", SMI_OPT_VALUE, struct termui_optctx_smi_opts_s, value, 1, 0, 0xffff,
                                     TERMUI_CON_OPT_CONSTRAINTS(SMI_OPT_VALUE, SMI_OPT_CLAUSE | SMI_OPT_VALUE | SMI_OPT_DEVAD | SMI_OPT_PRTAD | SMI_OPT_DEV))

  TERMUI_CON_LIST_END
};

TERMUI_CON_GROUP_DECL(dev_shell_smi_group) =
{
  TERMUI_CON_ENTRY(shell_smi_read, "read",
    TERMUI_CON_OPTS_CTX(smi_opts_s,
                        SMI_OPT_CLAUSE | SMI_OPT_DEV | SMI_OPT_PRTAD | SMI_OPT_ADDRESS,
                        SMI_OPT_CLAUSE | SMI_OPT_DEV | SMI_OPT_PRTAD | SMI_OPT_DEVAD | SMI_OPT_ADDRESS,
                        smi_opts_cleanup)
  )

  TERMUI_CON_ENTRY(shell_smi_write, "write",
    TERMUI_CON_OPTS_CTX(smi_opts_s,
                        SMI_OPT_CLAUSE | SMI_OPT_DEV | SMI_OPT_PRTAD | SMI_OPT_ADDRESS | SMI_OPT_VALUE,
                        SMI_OPT_CLAUSE | SMI_OPT_DEV | SMI_OPT_PRTAD | SMI_OPT_ADDRESS | SMI_OPT_DEVAD | SMI_OPT_VALUE,
                        smi_opts_cleanup)
  )

  TERMUI_CON_LIST_END
};

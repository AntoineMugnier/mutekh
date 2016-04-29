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

    Copyright (c) 2016 Alexandre Becoulet <alexandre.becoulet@free.fr>

*/

#include <device/class/enum.h>
#include <device/driver.h>
#include <device/resources.h>
#include <device/shell.h>

#include <mutek/console.h>
#include <mutek/shell.h>
#include <mutek/mem_alloc.h>
#include <hexo/enum.h>

enum enum_opts_e
{
  ENUM_OPT_DEV = 0x01,
  ENUM_OPT_CDEV = 0x02,
  ENUM_OPT_CLASS = 0x04,
};

struct termui_optctx_dev_enum_opts
{
  struct device_enum_s accessor;
  struct device_s *dev;
  uint_fast8_t num;
  enum driver_class_e cl;
};

static TERMUI_CON_ARGS_CLEANUP_PROTOTYPE(enum_opts_cleanup)
{
  struct termui_optctx_dev_enum_opts *c = ctx;

  if (device_check_accessor(&c->accessor.base))
    device_put_accessor(&c->accessor.base);
}

#ifdef CONFIG_MUTEK_CONTEXT_SCHED

static TERMUI_CON_COMMAND_PROTOTYPE(shell_enum_wait_init)
{
  struct termui_optctx_dev_enum_opts *c = ctx;

  struct device_accessor_s acc;
  error_t err = device_wait_accessor(&acc, c->dev, c->cl, c->num);
  if (err)
    return -EINVAL;
  device_put_accessor(&acc);
  return 0;
}

#endif

static TERMUI_CON_OPT_DECL(dev_enum_opts) =
{
  TERMUI_CON_OPT_DEV_ACCESSOR_ENTRY("-d", "--enum-dev", ENUM_OPT_DEV,
                                    struct termui_optctx_dev_enum_opts, accessor, DRIVER_CLASS_ENUM,
                                    TERMUI_CON_OPT_CONSTRAINTS(ENUM_OPT_DEV, 0)
                                    )

  TERMUI_CON_OPT_DEV_DEVICE_ENTRY("-D", "--dev", ENUM_OPT_CDEV,
                                  struct termui_optctx_dev_enum_opts, dev, num, NULL,
                                  TERMUI_CON_OPT_CONSTRAINTS(ENUM_OPT_CDEV, 0)
                                  )

  TERMUI_CON_OPT_ENUM_ENTRY("-c", "--class", ENUM_OPT_CLASS, struct termui_optctx_dev_enum_opts,
                            cl, driver_class_e,
                            TERMUI_CON_OPT_CONSTRAINTS(ENUM_OPT_CLASS, 0))

  TERMUI_CON_LIST_END
};

TERMUI_CON_GROUP_DECL(dev_shell_enum_group) =
{
#ifdef CONFIG_MUTEK_CONTEXT_SCHED
  TERMUI_CON_ENTRY(shell_enum_wait_init, "wait_init",
    TERMUI_CON_OPTS_CTX(dev_enum_opts,
                        ENUM_OPT_CDEV | ENUM_OPT_CLASS, 0,
                        enum_opts_cleanup)
  )
#endif

  TERMUI_CON_LIST_END
};


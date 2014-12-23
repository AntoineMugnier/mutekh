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

    Copyright Sebastien Cerdan <sebcerdan@gmail.com> (c) 2014

*/

#include <limits.h>
#include <stdlib.h>
#include <string.h>

#include <mutek/mem_alloc.h>

#include <device/request.h>
#include <device/shell.h>
#include <device/device.h>
#include <device/driver.h>

#include <device/class/spi.h>

#include <termui/console_opt.h>

enum spi_opt_e
{
  SPI_OPT_DEV       = 0x01,
  SPI_OPT_BIT_RATE  = 0x02,
  SPI_OPT_BIT_ORDER = 0x04,
  SPI_OPT_CLK_MODE  = 0x08,
  SPI_OPT_POL       = 0x10,
  SPI_OPT_CS_ID     = 0x20,
  SPI_OPT_CS_POLICY = 0x40,
  SPI_OPT_WR_DATA   = 0x80,
  SPI_OPT_SIZE      = 0x100,

};

struct termui_optctx_dev_spi_opts
{
  /* spi device. */
  struct device_spi_ctrl_s       spi;
  
  struct dev_spi_ctrl_transfer_s tr;

  union 
    {
      size_t count;
      union 
        {
          /* Chip select */
          struct 
            {
              enum dev_spi_cs_policy_e policy;
              uint8_t csid;
            }cs;
          /* config */
          struct 
            {
              uint32_t bit_rate;
              enum dev_spi_ckmode_e ck_mode;
              enum dev_spi_bit_order_e bit_order;
            }cfg;
        };
      
      enum dev_spi_polarity_e  pol;
    };
};

/****************************** config ************************/

static TERMUI_CON_ARGS_CLEANUP_PROTOTYPE(spi_opts_cleanup)
{
  struct termui_optctx_dev_spi_opts *data = ctx;

  if (device_check_accessor(&data->spi))
    device_put_accessor(&data->spi);
}

static TERMUI_CON_COMMAND_PROTOTYPE(dev_shell_spi_config)
{
  struct termui_optctx_dev_spi_opts *data = ctx;

  struct dev_spi_ctrl_config_s cfg = {0};

  if (used & SPI_OPT_CLK_MODE)
    cfg.ck_mode = data->cfg.ck_mode;

  if (used & SPI_OPT_BIT_ORDER)
    cfg.bit_order = data->cfg.bit_order;

  if (used & SPI_OPT_POL)
    {
      cfg.miso_pol = data->pol;
      cfg.mosi_pol = data->pol;
    }

  if (used & SPI_OPT_BIT_RATE)
    cfg.bit_rate   = data->cfg.bit_rate;

  cfg.word_width = 8;

  error_t err = DEVICE_OP(&data->spi, config, &cfg);

  if (err)
    termui_con_printf(con, "Failed to apply spi configuration : error %d\n", err);

  return err;
}

static TERMUI_CON_COMMAND_PROTOTYPE(dev_shell_spi_select)
{
  struct termui_optctx_dev_spi_opts *data = ctx;
 
  error_t err = DEVICE_OP(&data->spi, select, data->cs.policy, data->pol, data->cs.csid);

  if (err == -ENOTSUP)
    termui_con_printf(con, "Chip select policy not supported by controller\n");
  else if (err)
    termui_con_printf(con, "Failed to select spi slave : error %d\n", err);

  return err;
}

static TERMUI_CON_COMMAND_PROTOTYPE(dev_shell_spi_read)
{
  struct termui_optctx_dev_spi_opts *data = ctx;
  struct dev_spi_ctrl_transfer_s *tr = &data->tr;
 
  tr->accessor = &data->spi;
  tr->out_width = 1;
  tr->in_width = 1;
  tr->out = NULL;
  tr->count = 0;

  uint8_t *in = NULL;

  if (!data->count)
    return -EINVAL;

  tr->count = data->count;
  uint16_t size = tr->count;  

  in = mem_alloc(tr->count, (mem_scope_sys));

  if (in == NULL)
    {
      termui_con_printf(con, "error: cannot allocate buffer for read transfer.");
      return -ENOMEM;
    }

  tr->in = in;

  error_t err = dev_spi_wait_transfer(tr); 

  if (err)
    termui_con_printf(con, "Failed to transfert spi data with error: %d\n", err);

  termui_con_printf(con, "read (%u bytes): %P\n", size, in, size);
  mem_free(in);
  
  return err;
}
static TERMUI_CON_COMMAND_PROTOTYPE(dev_shell_spi_swap)
{
  struct termui_optctx_dev_spi_opts *data = ctx;
  struct dev_spi_ctrl_transfer_s *tr = &data->tr;
 
  tr->accessor = &data->spi;
  tr->out_width = 1;
  tr->in_width = 1;

  uint8_t *in = NULL;
  uint8_t *out = (uint8_t*)tr->out;

  if (!tr->count)
    return -EINVAL;

  in = mem_alloc(tr->count, (mem_scope_sys));

  if (in == NULL)
    {
      termui_con_printf(con, "Failed to allocate buffer for read transfer.");
      return -ENOMEM;
    }
  
  uint16_t size = tr->count;  

  tr->in = in;

  error_t err = dev_spi_wait_transfer(tr); 

  if (err)
    termui_con_printf(con, "Failed to transfert spi data with error: %d\n", err);

  termui_con_printf(con, "write (%u bytes): %P\n", size, out, size);
  termui_con_printf(con, "read (%u bytes): %P\n", size, in, size);
  mem_free(in);
  
  return err;
}

static TERMUI_CON_COMMAND_PROTOTYPE(dev_shell_spi_write)
{
  struct termui_optctx_dev_spi_opts *data = ctx;
  struct dev_spi_ctrl_transfer_s *tr = &data->tr;
 
  uint8_t *out = (uint8_t*)tr->out;

  tr->accessor = &data->spi;

  tr->out_width = 1;
  tr->in_width = 1;

  tr->in = NULL;

  if (!tr->count)
    return -EINVAL;

  uint16_t size = tr->count;  

  error_t err = dev_spi_wait_transfer(tr); 

  if (err)
    termui_con_printf(con, "Failed to transfert spi data with error: %d\n", err);

  termui_con_printf(con, "write (%u bytes): %P\n", size, out, size);

  return err;
}

static TERMUI_CON_PARSE_OPT_PROTOTYPE(dev_shell_spi_parse_write)
{
  struct termui_optctx_dev_spi_opts *data = ctx;

  struct dev_spi_ctrl_transfer_s *tr = &data->tr;

  tr->out = (uint8_t *)argv[0];
  tr->count = argl[0];

  return 0;
}


static TERMUI_CON_OPT_DECL(dev_spi_opts) =
{
  TERMUI_CON_OPT_DEV_ACCESSOR_ENTRY("-d", "--spi-dev", SPI_OPT_DEV,
    struct termui_optctx_dev_spi_opts, spi, DRIVER_CLASS_SPI_CTRL,
    TERMUI_CON_OPT_CONSTRAINTS(SPI_OPT_DEV, 0)
    TERMUI_CON_OPT_HELP("This option selects an spi device", NULL)
  )

  TERMUI_CON_OPT_INTEGER_ENTRY("-i", "--cs-id", SPI_OPT_CS_ID,
    struct termui_optctx_dev_spi_opts, cs.csid, 1,
    TERMUI_CON_OPT_CONSTRAINTS(SPI_OPT_CS_ID, SPI_OPT_DEV)) 

  TERMUI_CON_OPT_ENUM_ENTRY("-c", "--cs-policy", SPI_OPT_CS_POLICY,
    struct termui_optctx_dev_spi_opts, cs.policy, dev_spi_cs_policy_e,
    TERMUI_CON_OPT_CONSTRAINTS(SPI_OPT_CS_POLICY, SPI_OPT_DEV)) 

  TERMUI_CON_OPT_INTEGER_ENTRY("-r", "--bit-rate", SPI_OPT_BIT_RATE,
    struct termui_optctx_dev_spi_opts, cfg.bit_rate, 1,
    TERMUI_CON_OPT_CONSTRAINTS(SPI_OPT_BIT_RATE, SPI_OPT_DEV))

  TERMUI_CON_OPT_ENUM_ENTRY("-o", "--bit-order", SPI_OPT_BIT_ORDER,
    struct termui_optctx_dev_spi_opts, cfg.bit_order, dev_spi_bit_order_e,
    TERMUI_CON_OPT_CONSTRAINTS(SPI_OPT_BIT_ORDER, SPI_OPT_DEV)
    TERMUI_CON_OPT_HELP("This option defines the bit order of the spi data",
                        NULL)
  )

  TERMUI_CON_OPT_ENUM_ENTRY("-p", "--polarity", SPI_OPT_POL,
    struct termui_optctx_dev_spi_opts, pol, dev_spi_polarity_e,
    TERMUI_CON_OPT_CONSTRAINTS(SPI_OPT_POL, SPI_OPT_DEV)
    TERMUI_CON_OPT_HELP("This option defines the polarity of the chip select signal",
                        NULL)
  )

  TERMUI_CON_OPT_ENUM_ENTRY("-m", "--clock-mode", SPI_OPT_CLK_MODE,
    struct termui_optctx_dev_spi_opts, cfg.ck_mode, dev_spi_ckmode_e,
    TERMUI_CON_OPT_CONSTRAINTS(SPI_OPT_CLK_MODE, SPI_OPT_DEV)
    TERMUI_CON_OPT_HELP("This option defines the polarity of the clock mode",
                        NULL)
  )

  TERMUI_CON_OPT_ENTRY("-w", "--data", SPI_OPT_WR_DATA,
    TERMUI_CON_OPT_PARSE(dev_shell_spi_parse_write, 1)
    TERMUI_CON_OPT_CONSTRAINTS(SPI_OPT_WR_DATA, SPI_OPT_DEV) 
    TERMUI_CON_OPT_HELP("This option writes the given buffer to the slave",
                        NULL)
  )

  TERMUI_CON_OPT_INTEGER_ENTRY("-s", "--size", SPI_OPT_SIZE,
    struct termui_optctx_dev_spi_opts, count, 1,
    TERMUI_CON_OPT_CONSTRAINTS(SPI_OPT_SIZE, SPI_OPT_DEV) 
    TERMUI_CON_OPT_HELP("This option defines size of read transfer",
                        NULL)
  )
  TERMUI_CON_LIST_END
};

TERMUI_CON_GROUP_DECL(dev_shell_spi_ctrl_group) =
{
  TERMUI_CON_ENTRY(dev_shell_spi_config, "config",
    TERMUI_CON_OPTS_CTX(dev_spi_opts,
                        SPI_OPT_DEV,
                        SPI_OPT_BIT_RATE | SPI_OPT_BIT_ORDER |
                        SPI_OPT_POL | SPI_OPT_CLK_MODE,
                        spi_opts_cleanup)
  )

  TERMUI_CON_ENTRY(dev_shell_spi_select, "select",
    TERMUI_CON_OPTS_CTX(dev_spi_opts,
                        SPI_OPT_DEV,
                        SPI_OPT_POL | SPI_OPT_CS_POLICY | SPI_OPT_CS_ID,
                        spi_opts_cleanup)
  )

  TERMUI_CON_ENTRY(dev_shell_spi_write, "write",
    TERMUI_CON_OPTS_CTX(dev_spi_opts,
                        SPI_OPT_DEV | SPI_OPT_WR_DATA,
                        0, 
                        spi_opts_cleanup)
  )

  TERMUI_CON_ENTRY(dev_shell_spi_read, "read",
    TERMUI_CON_OPTS_CTX(dev_spi_opts,
                        SPI_OPT_DEV | SPI_OPT_SIZE,
                        0, 
                        spi_opts_cleanup)
  )

  TERMUI_CON_ENTRY(dev_shell_spi_swap, "swap",
    TERMUI_CON_OPTS_CTX(dev_spi_opts,
                        SPI_OPT_DEV | SPI_OPT_WR_DATA,
                        0,
                        spi_opts_cleanup)
  )
  TERMUI_CON_LIST_END
};


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

#include <device/class/nfc.h>
#include <device/driver.h>
#include <device/resources.h>
#include <device/shell.h>

#include <mutek/mem_alloc.h>
#include <mutek/console.h>
#include <mutek/shell.h>
#include <hexo/enum.h>

enum nfc_opts_e
{
  NFC_OPT_DEV       = 0x01,
  NFC_OPT_MODE      = 0x02,
  NFC_OPT_SIDE      = 0x04,
  NFC_OPT_RATE      = 0x08,
  NFC_OPT_OP        = 0x10,
  NFC_OPT_DATA      = 0x20,
  NFC_OPT_BIT_COUNT = 0x40,
  NFC_OPT_RAW       = 0x80,
};

struct dev_nfc_config_s shell_nfc_config;

struct termui_optctx_nfc_opts_s
{
  struct device_nfc_s accessor;

  union {
    struct { 
      enum dev_nfc_req_type_e type;
      size_t bit_count;
      struct termui_con_string_s data;
      bool_t raw;
    } req;
    
    struct {
      enum dev_nfc_mode_e mode;
      enum dev_nfc_side_e side;
      uint32_t baudrate;
    } config;
  };
};

static TERMUI_CON_ARGS_CLEANUP_PROTOTYPE(nfc_opts_cleanup)
{
  struct termui_optctx_nfc_opts_s *c = ctx;

  if (device_check_accessor(&c->accessor.base))
    device_put_accessor(&c->accessor.base);
}

static TERMUI_CON_COMMAND_PROTOTYPE(shell_nfc_configure)
{
  struct termui_optctx_nfc_opts_s *c = ctx;

  if (used & NFC_OPT_RATE)
    shell_nfc_config.baudrate = c->config.baudrate;
  if (used & NFC_OPT_MODE)
    shell_nfc_config.mode = c->config.mode;
  if (used & NFC_OPT_SIDE)
    shell_nfc_config.side = c->config.side;

  return 0;
}

static TERMUI_CON_COMMAND_PROTOTYPE(shell_nfc_request)
{
  struct termui_optctx_nfc_opts_s *c = ctx;
  struct dev_nfc_rq_s rq;
  static const size_t rx_data_buffer_size = 64;
  uint32_t div = 8;

  rq.config = &shell_nfc_config;
  rq.config_cache.dirty = 1;
  rq.type = c->req.type;
  rq.rx_data = mem_alloc(rx_data_buffer_size, mem_scope_sys);

  if (used & NFC_OPT_RAW) {
    if (!(used & NFC_OPT_BIT_COUNT))
      return -EINVAL;

    rq.auto_parity = 0;
    rq.tx_data = (const void *)c->req.data.str;
    rq.tx_data_bit_count = c->req.bit_count;
    rq.rx_data_bit_count = rx_data_buffer_size * 8 / 9;
    div = 9;
  } else {
    rq.auto_parity = 1;
    rq.tx_data = (const void *)c->req.data.str;

    if (used & NFC_OPT_BIT_COUNT)
      rq.tx_data_bit_count = c->req.bit_count;
    else
      rq.tx_data_bit_count = c->req.data.len * 8;

    rq.rx_data_bit_count = rx_data_buffer_size * 8;
    div = 8;
  }

#if defined(CONFIG_MUTEK_CONTEXT_SCHED)
  error_t err = dev_nfc_wait_request(&c->accessor, &rq);
#else
  error_t err = dev_nfc_spin_request(&c->accessor, &rq);
#endif

  if (err)
    termui_con_printf(con, "Request failed with error: %d\n", err);
  else
    termui_con_printf(con, "RX %d bits: %P\n",
                      rq.rx_data_bit_count, rq.rx_data,
                      (rq.rx_data_bit_count + 7) / div);

  mem_free(rq.rx_data);

  return err;
}

static TERMUI_CON_OPT_DECL(nfc_opts_s) =
{
  TERMUI_CON_OPT_DEV_ACCESSOR_ENTRY("-d", "--nfc-dev", NFC_OPT_DEV,
                                    struct termui_optctx_nfc_opts_s, accessor, DRIVER_CLASS_NFC,
                                    TERMUI_CON_OPT_CONSTRAINTS(NFC_OPT_DEV, 0)
                                    )

  TERMUI_CON_OPT_STRING_ENTRY("-D", "--data", NFC_OPT_DATA, struct termui_optctx_nfc_opts_s,
                              req.data, 1, TERMUI_CON_OPT_CONSTRAINTS(NFC_OPT_DATA, NFC_OPT_DEV))

  TERMUI_CON_OPT_INTEGER_RANGE_ENTRY("-s", "--size", NFC_OPT_BIT_COUNT, struct termui_optctx_nfc_opts_s, req.bit_count, 1, 1, 256,
                                     TERMUI_CON_OPT_CONSTRAINTS(NFC_OPT_BIT_COUNT, NFC_OPT_DATA | NFC_OPT_DEV))
  
  TERMUI_CON_OPT_ENUM_ENTRY("-o", "--op", NFC_OPT_OP,  struct termui_optctx_nfc_opts_s, req.type, 
                            dev_nfc_req_type_e,
                            TERMUI_CON_OPT_CONSTRAINTS(0, NFC_OPT_DEV)
                            )

  TERMUI_CON_OPT_ENUM_ENTRY("-S", "--side", NFC_OPT_SIDE,  struct termui_optctx_nfc_opts_s, config.side, 
                            dev_nfc_side_e,
                            TERMUI_CON_OPT_CONSTRAINTS(NFC_OPT_SIDE, 0)
                            )

  TERMUI_CON_OPT_ENUM_ENTRY("-m", "--mode", NFC_OPT_MODE,  struct termui_optctx_nfc_opts_s, config.mode, 
                            dev_nfc_mode_e,
                            TERMUI_CON_OPT_CONSTRAINTS(NFC_OPT_MODE, 0)
                            )

  TERMUI_CON_OPT_INTEGER_ENTRY("-b", "--baudrate", NFC_OPT_RATE, struct termui_optctx_nfc_opts_s, config.baudrate, 1,
                               TERMUI_CON_OPT_CONSTRAINTS(NFC_OPT_RATE, 0))

  TERMUI_CON_OPT_ENTRY("-r", "--raw", NFC_OPT_RAW,
                       TERMUI_CON_OPT_CONSTRAINTS(NFC_OPT_RAW, NFC_OPT_DATA))

  TERMUI_CON_LIST_END
};

TERMUI_CON_GROUP_DECL(dev_shell_nfc_group) =
{
  TERMUI_CON_ENTRY(shell_nfc_request, "transceive",
    TERMUI_CON_OPTS_CTX(nfc_opts_s,
                        NFC_OPT_DEV,
                        NFC_OPT_DATA | NFC_OPT_BIT_COUNT | NFC_OPT_OP | NFC_OPT_RAW,
                        nfc_opts_cleanup)
  )

  TERMUI_CON_ENTRY(shell_nfc_configure, "configure",
    TERMUI_CON_OPTS_CTX(nfc_opts_s,
                        0,
                        NFC_OPT_RATE | NFC_OPT_SIDE | NFC_OPT_MODE,
                        nfc_opts_cleanup)
  )

  TERMUI_CON_LIST_END
};


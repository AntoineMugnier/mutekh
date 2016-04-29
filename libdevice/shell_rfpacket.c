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

    Copyright Sebastien Cerdan <sebcerdan@gmail.com@free.fr> (c) 2014

*/

#include <device/class/rfpacket.h>
#include <device/driver.h>
#include <device/resources.h>
#include <device/shell.h>

#include <mutek/mem_alloc.h>
#include <mutek/console.h>
#include <mutek/shell.h>
#include <hexo/enum.h>

enum rfpacket_opts_e
{
  RFPACKET_OPT_DEV       = 0x01,
  RFPACKET_OPT_LIFETIME  = 0x02,
  RFPACKET_OPT_DATA      = 0x04,
  RFPACKET_OPT_PWR       = 0x08,
  RFPACKET_OPT_SIZE      = 0x10,
  RFPACKET_OPT_FREQ      = 0x20,
  RFPACKET_OPT_DEVIATION = 0x40,
  RFPACKET_OPT_BW        = 0x80,
  RFPACKET_OPT_DRATE     = 0x100,
  RFPACKET_OPT_RSSI      = 0x200,
  RFPACKET_OPT_MOD       = 0x400,
  RFPACKET_OPT_SYMB      = 0x800,
};

struct termui_optctx_dev_rfpacket_opts
{
  struct device_rfpacket_s accessor;

  union 
    {
      /* TX */
      struct 
        { 
          uint32_t lifetime;
          size_t size;
          struct termui_con_string_s data;
          int16_t pwr;
        };
 
      /* CFG */
      struct
        {
          /** Modulation */
          enum dev_rfpacket_modulation_e  mod;
          /** number of symbols */
          uint8_t symbols;
          /** RF frequency in Hz */
          uint32_t frequency;
          /** frequency deviation in Hz (FSK modulation) */
          uint32_t deviation;
          /** bandwidth in Hz (OOK modulation) */
          uint32_t bw;
          /** Datarate in bps */
          uint32_t drate;
          /** RSSI threshold in 0.125 dBm for RX */
          int16_t rssi_th;
        };
    };
};

static TERMUI_CON_ARGS_CLEANUP_PROTOTYPE(rfpacket_opts_cleanup)
{
  struct termui_optctx_dev_rfpacket_opts *c = ctx;

  if (device_check_accessor(&c->accessor.base))
    device_put_accessor(&c->accessor.base);
}

static TERMUI_CON_COMMAND_PROTOTYPE(shell_rfpacket_configure)
{
  struct termui_optctx_dev_rfpacket_opts *c = ctx;

  enum dev_rfpacket_cfg_msk_e mask = 0;

  if (used & RFPACKET_OPT_FREQ)
    mask |= DEV_RFPACKET_FREQUENCY;
  if (used & RFPACKET_OPT_RSSI)
    mask |= DEV_RFPACKET_RSSI_THRD;
  if (used & RFPACKET_OPT_DEVIATION)
    mask |= DEV_RFPACKET_DEVIATION;
  if (used & RFPACKET_OPT_BW)
    mask |= DEV_RFPACKET_BW;
  if (used & RFPACKET_OPT_DRATE)
    mask |= DEV_RFPACKET_DRATE;
  if (used & RFPACKET_OPT_MOD)
    mask |= DEV_RFPACKET_MOD;
  if (used & RFPACKET_OPT_SYMB)
    mask |= DEV_RFPACKET_SYMBOLS;

  struct dev_rfpacket_config_s cfg;

  cfg.frequency = c->frequency;
  cfg.deviation = c->deviation;
  cfg.bw = c->bw;
  cfg.drate = c->drate;
  cfg.rssi_th = c->rssi_th;
  cfg.mod = c->mod;
  cfg.symbols = c->symbols;

#if defined(CONFIG_MUTEK_CONTEXT_SCHED)
  error_t err = dev_rfpacket_wait_config(&c->accessor, &cfg, mask);
#else
  error_t err = dev_rfpacket_spin_config(&c->accessor, &cfg, mask);
#endif

  if (err)
    termui_con_printf(con, "Failed to apply radio configuration.\n");

  return 0;

}

static TERMUI_CON_COMMAND_PROTOTYPE(shell_rfpacket_request)
{
  struct termui_optctx_dev_rfpacket_opts *c = ctx;

  size_t txsize = 0;
  uint8_t *buf = NULL;

  if (used & RFPACKET_OPT_DATA)
    {
      if (!c->data.len)
        return -EINVAL;

      buf = (uint8_t*)c->data.str;
      txsize = c->data.len;
    }
  else if (used & RFPACKET_OPT_SIZE)
    {
      if (!c->size)
        return -EINVAL;

      buf = mem_alloc(c->size, (mem_scope_sys));
      if (!buf)
        return -EINVAL;

      for (uint16_t i = 0; i<c->size; i++)
        buf[i] = i + c->size - 1;

      txsize = c->size;
    }

  if (!(used & RFPACKET_OPT_LIFETIME))
    c->lifetime = 10000;

#if defined(CONFIG_MUTEK_CONTEXT_SCHED)
  error_t err = dev_rfpacket_wait_send_packet(&c->accessor, buf, txsize, c->pwr * 8, c->lifetime);
#else
  error_t err = dev_rfpacket_spin_send_packet(&c->accessor, buf, txsize, c->pwr * 8, c->lifetime);
#endif

  if (used & RFPACKET_OPT_SIZE)
    mem_free(buf);

  if (err)
    termui_con_printf(con, "TX failed with error: %d\n", err);

  return 0;
}

static TERMUI_CON_OPT_DECL(dev_rfpacket_opts) =
{
  TERMUI_CON_OPT_DEV_ACCESSOR_ENTRY("-d", "--rfpacket-dev", RFPACKET_OPT_DEV,
                                    struct termui_optctx_dev_rfpacket_opts, accessor, DRIVER_CLASS_RFPACKET,
                                    TERMUI_CON_OPT_CONSTRAINTS(RFPACKET_OPT_DEV, 0)
                                    )

  TERMUI_CON_OPT_STRING_ENTRY("-D", "--data", RFPACKET_OPT_DATA, struct termui_optctx_dev_rfpacket_opts,
                            data, 1, TERMUI_CON_OPT_CONSTRAINTS(RFPACKET_OPT_DATA | RFPACKET_OPT_SIZE, 0))

  TERMUI_CON_OPT_INTEGER_RANGE_ENTRY("-s", "--size", RFPACKET_OPT_SIZE, struct termui_optctx_dev_rfpacket_opts, size, 1, 1, 256,
                              TERMUI_CON_OPT_CONSTRAINTS(RFPACKET_OPT_DATA | RFPACKET_OPT_SIZE, 0))

  TERMUI_CON_OPT_INTEGER_ENTRY("-p", "--power", RFPACKET_OPT_PWR, struct termui_optctx_dev_rfpacket_opts, pwr, 1,
                              TERMUI_CON_OPT_CONSTRAINTS(RFPACKET_OPT_PWR, 0)
                              TERMUI_CON_OPT_HELP("This option defines the output power in Dbm",
                              NULL))

  TERMUI_CON_OPT_INTEGER_ENTRY("-l", "--lifetime", RFPACKET_OPT_LIFETIME, struct termui_optctx_dev_rfpacket_opts, lifetime, 1,
                              TERMUI_CON_OPT_CONSTRAINTS(RFPACKET_OPT_LIFETIME, RFPACKET_OPT_DEV))

  TERMUI_CON_OPT_INTEGER_ENTRY("-f", "--freq", RFPACKET_OPT_FREQ, struct termui_optctx_dev_rfpacket_opts, frequency, 1,
                              TERMUI_CON_OPT_CONSTRAINTS(RFPACKET_OPT_FREQ, RFPACKET_OPT_DEV))

  TERMUI_CON_OPT_INTEGER_ENTRY("-v", "--dev", RFPACKET_OPT_DEVIATION, struct termui_optctx_dev_rfpacket_opts, deviation, 1,
                              TERMUI_CON_OPT_CONSTRAINTS(RFPACKET_OPT_DEVIATION, RFPACKET_OPT_DEV))

  TERMUI_CON_OPT_INTEGER_ENTRY("-b", "--bw", RFPACKET_OPT_BW, struct termui_optctx_dev_rfpacket_opts, bw, 1,
                              TERMUI_CON_OPT_CONSTRAINTS(RFPACKET_OPT_BW, RFPACKET_OPT_DEV))

  TERMUI_CON_OPT_INTEGER_ENTRY("-t", "--drate", RFPACKET_OPT_DRATE, struct termui_optctx_dev_rfpacket_opts, drate, 1,
                              TERMUI_CON_OPT_CONSTRAINTS(RFPACKET_OPT_DRATE, RFPACKET_OPT_DEV))

  TERMUI_CON_OPT_INTEGER_ENTRY("-r", "--rssi", RFPACKET_OPT_RSSI, struct termui_optctx_dev_rfpacket_opts, rssi_th, 1,
                              TERMUI_CON_OPT_CONSTRAINTS(RFPACKET_OPT_RSSI, RFPACKET_OPT_DEV))

  TERMUI_CON_OPT_INTEGER_ENTRY("-y", "--symbols", RFPACKET_OPT_SYMB, struct termui_optctx_dev_rfpacket_opts, symbols, 1,
                              TERMUI_CON_OPT_CONSTRAINTS(RFPACKET_OPT_SYMB, RFPACKET_OPT_DEV))
  
  TERMUI_CON_OPT_ENUM_ENTRY("-m", "--modulation", RFPACKET_OPT_MOD,  struct termui_optctx_dev_rfpacket_opts, mod, 
     dev_rfpacket_modulation_e,
    TERMUI_CON_OPT_CONSTRAINTS(RFPACKET_OPT_MOD, RFPACKET_OPT_DEV | RFPACKET_OPT_SYMB)
    TERMUI_CON_OPT_HELP("This option defines the polarity of the pwm signal",
                        NULL)
  )
  TERMUI_CON_LIST_END
};

TERMUI_CON_GROUP_DECL(dev_shell_rfpacket_group) =
{
  TERMUI_CON_ENTRY(shell_rfpacket_request, "send",
    TERMUI_CON_OPTS_CTX(dev_rfpacket_opts,
                        RFPACKET_OPT_DEV,
                        RFPACKET_OPT_DATA | RFPACKET_OPT_SIZE | RFPACKET_OPT_PWR | RFPACKET_OPT_LIFETIME,
                        rfpacket_opts_cleanup)
  )

  TERMUI_CON_ENTRY(shell_rfpacket_configure, "configure",
    TERMUI_CON_OPTS_CTX(dev_rfpacket_opts,
                        RFPACKET_OPT_DEV,
                        RFPACKET_OPT_FREQ | RFPACKET_OPT_DEVIATION | RFPACKET_OPT_BW | RFPACKET_OPT_DRATE | RFPACKET_OPT_RSSI | RFPACKET_OPT_MOD | RFPACKET_OPT_SYMB,
                        rfpacket_opts_cleanup)
  )

  TERMUI_CON_LIST_END
};


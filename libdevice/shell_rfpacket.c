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
#include <device/shell_rfpacket.h>
#include <device/driver.h>
#include <device/resources.h>
#include <device/shell.h>

#include <mutek/printk.h>
#include <mutek/mem_alloc.h>
#include <mutek/console.h>
#include <mutek/shell.h>
#include <hexo/enum.h>

static TERMUI_CON_ARGS_CLEANUP_PROTOTYPE(rfpacket_opts_cleanup)
{
  struct termui_optctx_dev_rfpacket_opts *c = ctx;

  if (c->data.buffered)
    shell_buffer_drop(c->data.addr);

  if (c->cfg.buffered)
    shell_buffer_drop(c->cfg.addr);

  if (device_check_accessor(&c->accessor.base))
    device_put_accessor(&c->accessor.base);
}

static void shell_rfpacket_print_cfg(struct termui_console_s *con, 
                                     struct dev_rfpacket_rf_cfg_s *rf,
                                     struct dev_rfpacket_pk_cfg_s *pk)
{
  termui_con_printf(con, "   \n");
  termui_con_printf(con, "   Modulation:   %d\n", rf->mod);
  termui_con_printf(con, "   Drate:        %d\n", rf->drate);
  termui_con_printf(con, "   Frequency:    %d\n", rf->frequency);
  termui_con_printf(con, "   Chan spacing: %d\n", rf->chan_spacing);
  termui_con_printf(con, "   Bandwidth:    %d\n", rf->bw);

  switch (rf->mod)
    {
    case DEV_RFPACKET_FSK: 
    case DEV_RFPACKET_GFSK: {
      struct dev_rfpacket_rf_cfg_fsk_s *fsk = dev_rfpacket_rf_cfg_fsk_s_cast(rf);
      termui_con_printf(con, "   Deviation:    %d\n", fsk->deviation);
      termui_con_printf(con, "   Symbols:      %d\n", fsk->symbols);
      break;}
    case DEV_RFPACKET_ASK: {
      struct dev_rfpacket_rf_cfg_ask_s *ask = dev_rfpacket_rf_cfg_ask_s_cast(rf);
      termui_con_printf(con, "   Symbols:      %d\n", ask->symbols);
      break;}
    default:
      break;
    }

  termui_con_printf(con, "   \n");
  termui_con_printf(con, "   Format:         %d\n", pk->format);
  termui_con_printf(con, "   Encoding:       %d\n", pk->encoding);

  switch (pk->format)
    {
    case DEV_RFPACKET_FMT_SLPC:{
      struct dev_rfpacket_pk_cfg_basic_s *slpc = dev_rfpacket_pk_cfg_basic_s_cast(pk);
      termui_con_printf(con, "   CRC:            0x%x\n", slpc->crc);
      termui_con_printf(con, "   SW value:       0x%x\n", slpc->sw_value);
      termui_con_printf(con, "   SW len:         %d\n", slpc->sw_len);
      termui_con_printf(con, "   PB pattern:     0x%x\n", slpc->pb_pattern);
      termui_con_printf(con, "   PB pattern len: %d\n", slpc->pb_pattern_len);
      termui_con_printf(con, "   PB RX len:      %d\n", slpc->rx_pb_len);
      termui_con_printf(con, "   PB TX len:      %d\n", slpc->tx_pb_len);
      break;}
    default:
      break;
    }
}

struct shell_rfpacket_config_s{
  union
    {
      struct dev_rfpacket_rf_cfg_fsk_s fsk;
      struct dev_rfpacket_rf_cfg_ask_s ask;
      struct dev_rfpacket_rf_cfg_lora_s lora;
    }rf;
   struct dev_rfpacket_pk_cfg_basic_s pk;
};

TERMUI_CON_COMMAND_PROTOTYPE(shell_rfpacket_configure)
{
  struct termui_optctx_dev_rfpacket_opts *c = ctx;
  void * buffer;
  
  if (used & RFPACKET_OPT_CFG)
    buffer = c->cfg.addr;
  else
    buffer = shell_buffer_new(con, sizeof(struct shell_rfpacket_config_s), "cfg", shell_rfpacket_configure, 0);

  struct shell_rfpacket_config_s *cfg = (struct shell_rfpacket_config_s *)buffer;

  if (cfg == NULL) 
    return -EINVAL;

  struct dev_rfpacket_rf_cfg_s *rf = &cfg->rf.fsk.base;

  if (used & RFPACKET_OPT_RF_CFG_MSK)
    rf->cache.dirty = 1;
  if (used & RFPACKET_OPT_FREQ)
    rf->frequency = c->frequency;
  if (used & RFPACKET_OPT_MOD)
    rf->mod = c->mod;
  if (used & RFPACKET_OPT_BW)
    rf->bw = c->bw;
  if (used & RFPACKET_OPT_DRATE)
    rf->drate = c->drate;

  /* Unconfigurable parameters */
  rf->chan_spacing = 10000;
  rf->jam_rssi = (-90) << 3;

  switch (rf->mod)
    {
    case DEV_RFPACKET_FSK: 
    case DEV_RFPACKET_GFSK: {
      struct dev_rfpacket_rf_cfg_fsk_s *fsk = &cfg->rf.fsk;
      fsk->fairtx.lbt.rssi = (-95) << 3;
      fsk->fairtx.lbt.duration = 5000;
      if (used & RFPACKET_OPT_SYMB)
        fsk->symbols = c->symbols;
      if (used & RFPACKET_OPT_DEVIATION)
        fsk->deviation = c->deviation;
      break;}
    case DEV_RFPACKET_ASK: {
      struct dev_rfpacket_rf_cfg_ask_s *ask = &cfg->rf.ask;
      ask->fairtx.lbt.rssi = (-95) << 3;
      ask->fairtx.lbt.duration = 5000;
      if (used & RFPACKET_OPT_SYMB)
        ask->symbols = c->symbols;
      break;}
    default:
      if (!(used & RFPACKET_OPT_CFG))
        shell_buffer_drop(buffer);
      termui_con_printf(con, "You must specify a valid modulation\n");
      return -EINVAL;
    }

  struct dev_rfpacket_pk_cfg_basic_s *pk = &cfg->pk;

  if (used & RFPACKET_OPT_PKT_CFG_MSK)
    pk->base.cache.dirty = 1;

  pk->base.format = DEV_RFPACKET_FMT_SLPC;

  if (used & RFPACKET_OPT_ENCODING)
    pk->base.encoding = c->encoding;
  if (used & RFPACKET_OPT_CRC)
    pk->crc = c->crc;
  if (used & RFPACKET_OPT_SW_VAL)
    pk->sw_value = c->sw_value;
  if (used & RFPACKET_OPT_SW_LEN)
    pk->sw_len = c->sw_len;
  if (used & RFPACKET_OPT_RX_PB_LEN)
    pk->rx_pb_len = c->rx_pb_len;
  if (used & RFPACKET_OPT_TX_PB_LEN)
    pk->tx_pb_len = c->tx_pb_len;
  if (used & RFPACKET_OPT_PB_LEN)
    pk->pb_pattern_len = c->pb_pattern_len;
  if (used & RFPACKET_OPT_PB_VAL)
    pk->pb_pattern = c->pb_pattern;

  shell_rfpacket_print_cfg(con, rf, &pk->base);

  if (!(used & RFPACKET_OPT_CFG))
    shell_buffer_drop(buffer);

  return 0;
}

struct shell_rfpacket_rx_ctx_s
{
  const struct termui_console_s *con;
  struct dev_rfpacket_rq_s rq;
};

STRUCT_COMPOSE(shell_rfpacket_rx_ctx_s, rq);

struct shell_rfpacket_rx_pkt_s
{
  struct dev_rfpacket_rx_s      rx;
  uint8_t                       *data;
};

STRUCT_COMPOSE(shell_rfpacket_rx_pkt_s, rx);

static KROUTINE_EXEC(shell_rfpacket_rx_cb)
{
  struct dev_rfpacket_rx_s *rx = dev_rfpacket_rx_s_from_kr(kr);
  struct shell_rfpacket_rx_pkt_s *rxpkt = shell_rfpacket_rx_pkt_s_from_rx(rx);

  if (rx->size)
    printk("Rx packet size: %d\n", rx->size);

  /* Drop allocated RX buffer here */
  shell_buffer_drop(rxpkt->data);
  shell_buffer_drop(rxpkt);
}

static struct dev_rfpacket_rx_s *shell_rfpacket_rx_alloc(struct dev_rfpacket_rq_s *rq, size_t size)
{
  struct shell_rfpacket_rx_ctx_s *ctx = shell_rfpacket_rx_ctx_s_from_rq(rq);

  const struct termui_console_s *con = ctx->con;

  /* Allocation of a new buffer here which contains a dev_rfpacket_rx_s structure */

  struct shell_rfpacket_rx_pkt_s *rxpkt = shell_buffer_reuse(con, sizeof(struct shell_rfpacket_rx_pkt_s), "rxpkt", shell_rfpacket_rx_alloc, 0);

  rxpkt->data = shell_buffer_new(con, size, "rxdata", 0, 0);

  if (!rxpkt || !rxpkt->data)
    return NULL;

  struct dev_rfpacket_rx_s *rx = &rxpkt->rx;

  kroutine_init_deferred(&rx->kr, &shell_rfpacket_rx_cb);

  rx->size = size;
  rx->buf = rxpkt->data;

  return rx;
}

static TERMUI_CON_COMMAND_PROTOTYPE(shell_rfpacket_receive)
{
  struct termui_optctx_dev_rfpacket_opts *c = ctx;

  assert(used & RFPACKET_OPT_CFG);

  struct shell_rfpacket_config_s *cfg = (struct shell_rfpacket_config_s *)c->cfg.addr;

  assert(cfg != NULL);

  struct shell_rfpacket_rx_ctx_s rx_ctx;
  struct dev_rfpacket_rq_s *rq = &rx_ctx.rq;
  rx_ctx.con = (const struct termui_console_s *)con;

  rq->deadline = 0;
  rq->channel = 0;
  rq->type = DEV_RFPACKET_RQ_RX;
  rq->rx_alloc = &shell_rfpacket_rx_alloc;
  rq->err_group = 0;
  rq->rf_cfg = &cfg->rf.fsk.base;
  rq->pk_cfg = &cfg->pk.base;

  if (!(used & RFPACKET_OPT_LIFETIME && c->lifetime))
    c->lifetime = 2000;

  struct device_timer_s timer;

  if (device_get_accessor(&timer.base, c->accessor.dev, DRIVER_CLASS_TIMER, 0))
    return -EINVAL;

  if (dev_timer_init_sec(&timer, &rq->lifetime, 0, c->lifetime, 1000))
    return -EINVAL;

#if defined(CONFIG_MUTEK_CONTEXT_SCHED)
  error_t err = dev_rfpacket_wait_request(&c->accessor, rq);
#else
  error_t err = dev_rfpacket_spin_request(&c->accessor, &rq);
#endif

  if (err)
    termui_con_printf(con, "RX failed with error: %d\n", err);

  return 0;
}

static TERMUI_CON_COMMAND_PROTOTYPE(shell_rfpacket_send)
{
  struct termui_optctx_dev_rfpacket_opts *c = ctx;

  struct dev_rfpacket_rq_s rq;
  uint8_t *data = NULL;

  assert(used & RFPACKET_OPT_CFG);

  struct shell_rfpacket_config_s *cfg = (struct shell_rfpacket_config_s *)c->cfg.addr;

  assert(cfg != NULL);

  if (used & RFPACKET_OPT_DATA)
    {
      if (!c->data.size || c->data.size > 255)
        return -EINVAL;

      rq.tx_buf = (uint8_t*)c->data.addr;
      rq.tx_size = c->data.size;
    }
  else if (used & RFPACKET_OPT_SIZE)
    {
      data = shell_buffer_reuse((const struct termui_console_s *)con, c->size, "data", NULL, 0);

      if (data == NULL)
        return -EINVAL;

      for (uint16_t i = 0; i<c->size; i++)
        data[i] = i;

      rq.tx_buf = data;
      rq.tx_size = c->size;
    }

  if (rq.tx_buf == NULL)
    return -EINVAL;

  if (used & RFPACKET_OPT_PWR)
    rq.tx_pwr = (c->pwr << 3);
  else
    rq.tx_pwr = (13 << 3);

  rq.deadline = 0;
  rq.channel = 0;
  rq.type = DEV_RFPACKET_RQ_TX;
  rq.err_group = 0;
  rq.rf_cfg = &cfg->rf.fsk.base;
  rq.pk_cfg = &cfg->pk.base;

  if (!(used & RFPACKET_OPT_LIFETIME))
    c->lifetime = 10000;

#if defined(CONFIG_MUTEK_CONTEXT_SCHED)
  error_t err = dev_rfpacket_wait_request(&c->accessor, &rq);
#else
  error_t err = dev_rfpacket_spin_request(&c->accessor, &rq);
#endif

  if (used & RFPACKET_OPT_SIZE)
    shell_buffer_drop(data);
    
  if (err == 0)
    return 0;

  termui_con_printf(con, "TX error: %d\n", err);
  return -EINVAL;
}

static TERMUI_CON_OPT_DECL(dev_rfpacket_opts) =
{
  TERMUI_CON_OPT_DEV_ACCESSOR_ENTRY("-d", "--rfp-dev", RFPACKET_OPT_DEV,
                                    struct termui_optctx_dev_rfpacket_opts, accessor, DRIVER_CLASS_RFPACKET,
                                    TERMUI_CON_OPT_CONSTRAINTS(RFPACKET_OPT_DEV, 0)
                                    )

  TERMUI_CON_OPT_SHELL_BUFFER_RAW_ENTRY("-D", "--data", RFPACKET_OPT_DATA,
    struct termui_optctx_dev_rfpacket_opts, data, NULL,
    TERMUI_CON_OPT_CONSTRAINTS(RFPACKET_OPT_SIZE | RFPACKET_OPT_DATA, 0)
  )

  TERMUI_CON_OPT_SHELL_BUFFER_GET_ENTRY("-C", "--cfg", RFPACKET_OPT_CFG,
    struct termui_optctx_dev_rfpacket_opts, cfg, shell_rfpacket_configure,
                                        TERMUI_CON_OPT_CONSTRAINTS(RFPACKET_OPT_CFG, 0)
  )

  TERMUI_CON_OPT_INTEGER_RANGE_ENTRY("-s", "--size", RFPACKET_OPT_SIZE, struct termui_optctx_dev_rfpacket_opts, size, 1, 1, 255,
                              TERMUI_CON_OPT_CONSTRAINTS(RFPACKET_OPT_DATA | RFPACKET_OPT_SIZE, 0))

  TERMUI_CON_OPT_INTEGER_RANGE_ENTRY("-p", "--power", RFPACKET_OPT_PWR, struct termui_optctx_dev_rfpacket_opts, pwr, 1, -50, 20,
                              TERMUI_CON_OPT_CONSTRAINTS(RFPACKET_OPT_PWR, 0)
                              TERMUI_CON_OPT_HELP("This option defines the output power in Dbm",
                              NULL))

  TERMUI_CON_OPT_INTEGER_ENTRY("-l", "--lifetime-msec", RFPACKET_OPT_LIFETIME, struct termui_optctx_dev_rfpacket_opts, lifetime, 1,
                              TERMUI_CON_OPT_CONSTRAINTS(RFPACKET_OPT_LIFETIME, RFPACKET_OPT_DEV))

  TERMUI_CON_OPT_INTEGER_ENTRY("-f", "--freq", RFPACKET_OPT_FREQ, struct termui_optctx_dev_rfpacket_opts, frequency, 1,
                              TERMUI_CON_OPT_CONSTRAINTS(RFPACKET_OPT_FREQ, 0))

  TERMUI_CON_OPT_INTEGER_ENTRY("-v", "--dev", RFPACKET_OPT_DEVIATION, struct termui_optctx_dev_rfpacket_opts, deviation, 1,
                              TERMUI_CON_OPT_CONSTRAINTS(RFPACKET_OPT_DEVIATION, 0))

  TERMUI_CON_OPT_INTEGER_ENTRY("-b", "--bw", RFPACKET_OPT_BW, struct termui_optctx_dev_rfpacket_opts, bw, 1,
                              TERMUI_CON_OPT_CONSTRAINTS(RFPACKET_OPT_BW, 0))

  TERMUI_CON_OPT_INTEGER_ENTRY("-R", "--drate", RFPACKET_OPT_DRATE, struct termui_optctx_dev_rfpacket_opts, drate, 1,
                              TERMUI_CON_OPT_CONSTRAINTS(RFPACKET_OPT_DRATE, 0))

  TERMUI_CON_OPT_INTEGER_ENTRY("-y", "--symbols", RFPACKET_OPT_SYMB, struct termui_optctx_dev_rfpacket_opts, symbols, 1,
                              TERMUI_CON_OPT_CONSTRAINTS(RFPACKET_OPT_SYMB, 0))
  
  TERMUI_CON_OPT_ENUM_ENTRY("-m", "--modulation", RFPACKET_OPT_MOD,  struct termui_optctx_dev_rfpacket_opts, mod, 
     dev_rfpacket_modulation_e, TERMUI_CON_OPT_CONSTRAINTS(RFPACKET_OPT_MOD, RFPACKET_OPT_SYMB))

  TERMUI_CON_OPT_INTEGER_ENTRY("-c", "--crc", RFPACKET_OPT_CRC, struct termui_optctx_dev_rfpacket_opts, crc, 1,
                              TERMUI_CON_OPT_CONSTRAINTS(RFPACKET_OPT_CRC, 0))

  TERMUI_CON_OPT_INTEGER_ENTRY("-a", "--sw_value", RFPACKET_OPT_SW_VAL, struct termui_optctx_dev_rfpacket_opts, sw_value, 1,
                              TERMUI_CON_OPT_CONSTRAINTS(RFPACKET_OPT_SW_VAL, 0))

  TERMUI_CON_OPT_INTEGER_ENTRY("-n", "--sw_len", RFPACKET_OPT_SW_LEN, struct termui_optctx_dev_rfpacket_opts, sw_len, 1,
                              TERMUI_CON_OPT_CONSTRAINTS(RFPACKET_OPT_SW_LEN, 0))

  TERMUI_CON_OPT_INTEGER_ENTRY("-P", "--pb_pattern", RFPACKET_OPT_PB_VAL, struct termui_optctx_dev_rfpacket_opts, pb_pattern, 1,
                              TERMUI_CON_OPT_CONSTRAINTS(RFPACKET_OPT_PB_VAL, 0))

  TERMUI_CON_OPT_INTEGER_ENTRY("-g", "--pb_len", RFPACKET_OPT_PB_LEN, struct termui_optctx_dev_rfpacket_opts, pb_pattern_len, 1,
                              TERMUI_CON_OPT_CONSTRAINTS(RFPACKET_OPT_PB_LEN, 0))

  TERMUI_CON_OPT_INTEGER_ENTRY("-r", "--rx_pblen", RFPACKET_OPT_RX_PB_LEN, struct termui_optctx_dev_rfpacket_opts, rx_pb_len, 1,
                              TERMUI_CON_OPT_CONSTRAINTS(RFPACKET_OPT_RX_PB_LEN, 0))

  TERMUI_CON_OPT_INTEGER_ENTRY("-v", "--tx_pblen", RFPACKET_OPT_TX_PB_LEN, struct termui_optctx_dev_rfpacket_opts, tx_pb_len, 1,
                              TERMUI_CON_OPT_CONSTRAINTS(RFPACKET_OPT_TX_PB_LEN, 0))

  TERMUI_CON_OPT_ENUM_ENTRY("-e", "--encoding", RFPACKET_OPT_ENCODING,  struct termui_optctx_dev_rfpacket_opts, encoding, 
     dev_rfpacket_encoding_e, TERMUI_CON_OPT_CONSTRAINTS(RFPACKET_OPT_ENCODING, 0))

  TERMUI_CON_LIST_END
};

TERMUI_CON_GROUP_DECL(dev_shell_rfpacket_group) =
{
  TERMUI_CON_ENTRY(shell_rfpacket_send, "send",
    TERMUI_CON_OPTS_CTX(dev_rfpacket_opts,
                        RFPACKET_OPT_DEV | RFPACKET_OPT_DATA |
                        RFPACKET_OPT_SIZE | RFPACKET_OPT_CFG,
                        RFPACKET_OPT_PWR | RFPACKET_OPT_CFG,
                        rfpacket_opts_cleanup)
  )

  TERMUI_CON_ENTRY(shell_rfpacket_receive, "receive",
    TERMUI_CON_OPTS_CTX(dev_rfpacket_opts,
                        RFPACKET_OPT_DEV |
                        RFPACKET_OPT_CFG,
                        RFPACKET_OPT_LIFETIME | RFPACKET_OPT_CFG,
                        rfpacket_opts_cleanup)
  )


  TERMUI_CON_ENTRY(shell_rfpacket_configure, "config",
    TERMUI_CON_OPTS_CTX(dev_rfpacket_opts,
                        0,
                        RFPACKET_OPT_CFG | RFPACKET_OPT_FREQ | RFPACKET_OPT_DEVIATION | RFPACKET_OPT_BW | RFPACKET_OPT_DRATE | RFPACKET_OPT_MOD | RFPACKET_OPT_SYMB |
                        RFPACKET_OPT_SW_VAL | RFPACKET_OPT_SW_LEN | RFPACKET_OPT_PB_VAL | RFPACKET_OPT_PB_LEN | RFPACKET_OPT_RX_PB_LEN | RFPACKET_OPT_TX_PB_LEN |
                        RFPACKET_OPT_ENCODING | RFPACKET_OPT_CRC,
                        rfpacket_opts_cleanup)
  )

  TERMUI_CON_LIST_END
};


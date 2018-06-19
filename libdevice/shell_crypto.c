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

#include <device/class/crypto.h>
#include <device/driver.h>
#include <device/resources.h>
#include <device/shell.h>

#include <mutek/console.h>
#include <mutek/shell.h>
#include <hexo/enum.h>

enum crypto_opts_e
{
  CRYPTO_OPT_DEV = 0x01,
  CRYPTO_OPT_MODE = 0x02,
  CRYPTO_OPT_OP = 0x04,
  CRYPTO_OPT_AUTH = 0x08,
  CRYPTO_OPT_AUTH_LEN = 0x10,
  CRYPTO_OPT_IV = 0x20,
  CRYPTO_OPT_AD = 0x40,
  CRYPTO_OPT_DUMP_STATE = 0x80,
  CRYPTO_OPT_OUT_LEN = 0x100,
};

struct termui_optctx_dev_crypto_opts
{
  struct device_crypto_s accessor;
  enum dev_crypto_mode_e mode;
  enum dev_crypto_op_e op;
  union {
    size_t auth_len;
    size_t out_len;
    struct termui_con_string_s auth;
  };
  struct termui_con_string_s iv;
  struct termui_con_string_s ad;
};

static TERMUI_CON_ARGS_CLEANUP_PROTOTYPE(crypto_opts_cleanup)
{
  struct termui_optctx_dev_crypto_opts *c = ctx;

  if (device_check_accessor(&c->accessor.base))
    device_put_accessor(&c->accessor.base);
}

static TERMUI_CON_COMMAND_PROTOTYPE(shell_crypto_cipher)
{
  struct termui_optctx_dev_crypto_opts *c = ctx;

  switch (c->mode)
    {
    case DEV_CRYPTO_MODE_ECB:
    case DEV_CRYPTO_MODE_CBC:
    case DEV_CRYPTO_MODE_CFB:
    case DEV_CRYPTO_MODE_OFB:
    case DEV_CRYPTO_MODE_CTR:
    case DEV_CRYPTO_MODE_GCM:
    case DEV_CRYPTO_MODE_CCM:
    case DEV_CRYPTO_MODE_OCB3:
    case DEV_CRYPTO_MODE_STREAM:
      break;
    default:
      return -EINVAL;
    }

  struct dev_crypto_info_s info;
  if (DEVICE_OP(&c->accessor, info, &info))
    return -EINVAL;

  void *state = NULL;

  if ((c->op & (DEV_CRYPTO_INIT | DEV_CRYPTO_FINALIZE))
    != (DEV_CRYPTO_INIT | DEV_CRYPTO_FINALIZE))
    {
      state = alloca(info.state_size);
      memset(state, 0x55, info.state_size);
    }

  struct dev_crypto_context_s cctx = {
    .cache_ptr = NULL,
    .state_data = state,
    .key_data = (uint8_t*)argv[0],
    .key_len = argl[0],
    .mode = c->mode,
  };

  size_t len = argl[1];
  uint8_t out[len];

  struct dev_crypto_rq_s rq = {
    .ctx = &cctx,
    .op = c->op,
    .in = (uint8_t*)argv[1],
    .out = out,
    .len = len,
  };

  if (used & CRYPTO_OPT_AUTH_LEN)
    {
      if (c->op & DEV_CRYPTO_INVERSE)
        return -EINVAL;
      cctx.auth_len = c->auth_len;
      rq.auth = alloca(c->auth_len);
    }
  else if (used & CRYPTO_OPT_AUTH)
    {
      if (!(c->op & DEV_CRYPTO_INVERSE))
        return -EINVAL;
      cctx.auth_len = c->auth.len;
      rq.auth = (void*)c->auth.str;
    }

  if (used & CRYPTO_OPT_IV)
    {
      cctx.iv_len = c->iv.len;
      rq.iv_ctr = (void*)c->iv.str;
    }

  if (used & CRYPTO_OPT_AD)
    {
      rq.ad_len = c->ad.len;
      rq.ad = (void*)c->ad.str;
    }

  dev_crypto_wait_op(&c->accessor, &rq);

  if (rq.err)
    {
      termui_con_printf(con, "err: %i\n", rq.err);
      return -EINVAL;
    }

  termui_con_printf(con, "algo: %s\nout : %P\n", info.name, out, len);
  if (used & CRYPTO_OPT_AUTH_LEN)
    termui_con_printf(con, "auth: %P\n", rq.auth, cctx.auth_len);
  if (used & CRYPTO_OPT_IV)
    termui_con_printf(con, "iv  : %P\n", rq.iv_ctr, cctx.iv_len);
  if ((used & CRYPTO_OPT_DUMP_STATE) && state != NULL)
    termui_con_printf(con, "state : %P\n", state, info.state_size);

  return 0;
}

static TERMUI_CON_COMMAND_PROTOTYPE(shell_crypto_hash)
{
  struct termui_optctx_dev_crypto_opts *c = ctx;

  struct dev_crypto_info_s info;
  if (DEVICE_OP(&c->accessor, info, &info))
    return -EINVAL;

  void *state = NULL;

  if ((c->op & (DEV_CRYPTO_INIT | DEV_CRYPTO_FINALIZE))
    != (DEV_CRYPTO_INIT | DEV_CRYPTO_FINALIZE))
    {
      state = alloca(info.state_size);
      memset(state, 0x55, info.state_size);
    }

  struct dev_crypto_context_s cctx = {
    .cache_ptr = NULL,
    .state_data = state,
    .mode = DEV_CRYPTO_MODE_HASH,
  };

  uint8_t out[c->out_len];

  struct dev_crypto_rq_s rq = {
    .ctx = &cctx,
    .op = c->op,
    .ad = (uint8_t*)argv[0],
    .ad_len = argl[0],
    .out = out,
    .len = c->out_len,
  };

  dev_crypto_wait_op(&c->accessor, &rq);

  if (rq.err)
    {
      termui_con_printf(con, "err: %i\n", rq.err);
      return -EINVAL;
    }

  termui_con_printf(con, "algo: %s\nout : %P\n", info.name, out, c->out_len);

  return 0;
}

static TERMUI_CON_COMMAND_PROTOTYPE(shell_crypto_random)
{
  error_t err = 0;
  struct termui_optctx_dev_crypto_opts *c = ctx;

  struct dev_crypto_info_s info;
  if (DEVICE_OP(&c->accessor, info, &info))
    return -EINVAL;

  void *state = malloc(info.state_size);
  if (info.state_size && !state)
    return -EINVAL;

  memset(state, 0x55, info.state_size);

  struct dev_crypto_context_s cctx = {
    .cache_ptr = NULL,
    .state_data = state,
    .mode = DEV_CRYPTO_MODE_RANDOM,
  };

  struct dev_crypto_rq_s rq = {
    .ctx = &cctx,
    .op = DEV_CRYPTO_INIT
  };

  if (used & CRYPTO_OPT_IV)
    {
      rq.op |= DEV_CRYPTO_INVERSE;
      rq.ad = (uint8_t*)c->iv.str;
      rq.ad_len = c->iv.len;
    }

  rq.out = NULL;
  if (used & CRYPTO_OPT_OUT_LEN)
    {
      rq.op |= DEV_CRYPTO_FINALIZE;
      rq.out = shell_buffer_new(con, c->out_len, "random", NULL, 0);
      if (!rq.out)
        {
          err = -EINVAL;
          goto err_state;
        }
      rq.len = c->out_len;
    }

  dev_crypto_wait_op(&c->accessor, &rq);

  if (rq.err)
    {
      termui_con_printf(con, "err: %i\n", rq.err);
      err = -EINVAL;
      goto err_rqout;
    }

  if (used & CRYPTO_OPT_OUT_LEN)
    {
      termui_con_printf(con, "algo: %s\n", info.name);
      shell_buffer_advertise(con, rq.out, rq.len);
    }

 err_rqout:
  shell_buffer_drop(rq.out);
 err_state:
  free(state);

  return err;
}

static TERMUI_CON_OPT_DECL(dev_crypto_opts) =
{
  TERMUI_CON_OPT_DEV_ACCESSOR_ENTRY("-d", "--crypto-dev", CRYPTO_OPT_DEV,
                                    struct termui_optctx_dev_crypto_opts, accessor, DRIVER_CLASS_CRYPTO,
                                    TERMUI_CON_OPT_CONSTRAINTS(CRYPTO_OPT_DEV, 0)
                                    )

  TERMUI_CON_OPT_ENUM_ENTRY("-m", "--mode", CRYPTO_OPT_MODE, struct termui_optctx_dev_crypto_opts,
                            mode, dev_crypto_mode_e,
                            TERMUI_CON_OPT_CONSTRAINTS(CRYPTO_OPT_MODE, 0))

  TERMUI_CON_OPT_ENUM_ENTRY("-o", "--op", CRYPTO_OPT_OP, struct termui_optctx_dev_crypto_opts,
                            op, dev_crypto_op_e,
                            TERMUI_CON_OPT_CONSTRAINTS(CRYPTO_OPT_OP, 0))

  TERMUI_CON_OPT_STRING_ENTRY("-a", "--auth", CRYPTO_OPT_AUTH, struct termui_optctx_dev_crypto_opts, auth, 1,
                              TERMUI_CON_OPT_CONSTRAINTS(CRYPTO_OPT_AUTH | CRYPTO_OPT_AUTH_LEN, 0))

  TERMUI_CON_OPT_INTEGER_ENTRY("-L", "--auth-len", CRYPTO_OPT_AUTH_LEN, struct termui_optctx_dev_crypto_opts, auth_len, 1,
                               TERMUI_CON_OPT_CONSTRAINTS(CRYPTO_OPT_AUTH | CRYPTO_OPT_AUTH_LEN, 0))

  TERMUI_CON_OPT_STRING_ENTRY("-i", "--iv", CRYPTO_OPT_IV, struct termui_optctx_dev_crypto_opts, iv, 1,
                              TERMUI_CON_OPT_CONSTRAINTS(CRYPTO_OPT_IV, 0))

  TERMUI_CON_OPT_STRING_ENTRY("-A", "--ad", CRYPTO_OPT_AD, struct termui_optctx_dev_crypto_opts, ad, 1,
                              TERMUI_CON_OPT_CONSTRAINTS(CRYPTO_OPT_AD, 0))

  TERMUI_CON_OPT_INTEGER_ENTRY("-O", "--out-len", CRYPTO_OPT_OUT_LEN, struct termui_optctx_dev_crypto_opts, out_len, 1,
                               TERMUI_CON_OPT_CONSTRAINTS(CRYPTO_OPT_OUT_LEN, 0))

  TERMUI_CON_OPT_ENTRY("", "--dump-state", CRYPTO_OPT_DUMP_STATE)

  TERMUI_CON_LIST_END
};

TERMUI_CON_GROUP_DECL(dev_shell_crypto_group) =
{
  TERMUI_CON_ENTRY(shell_crypto_cipher, "cipher",
    TERMUI_CON_OPTS_CTX(dev_crypto_opts,
                        CRYPTO_OPT_DEV | CRYPTO_OPT_MODE | CRYPTO_OPT_OP,
                        CRYPTO_OPT_AUTH | CRYPTO_OPT_AUTH_LEN | CRYPTO_OPT_IV | CRYPTO_OPT_AD |
                        CRYPTO_OPT_DUMP_STATE,
                        crypto_opts_cleanup)
    TERMUI_CON_ARGS(2, 2)
  )

  TERMUI_CON_ENTRY(shell_crypto_hash, "hash",
    TERMUI_CON_OPTS_CTX(dev_crypto_opts,
                        CRYPTO_OPT_DEV | CRYPTO_OPT_OP | CRYPTO_OPT_OUT_LEN, CRYPTO_OPT_DUMP_STATE,
                        crypto_opts_cleanup)
    TERMUI_CON_ARGS(1, 1)
  )

  TERMUI_CON_ENTRY(shell_crypto_random, "random",
    TERMUI_CON_OPTS_CTX(dev_crypto_opts,
                        CRYPTO_OPT_DEV | CRYPTO_OPT_OUT_LEN, CRYPTO_OPT_IV,
                        crypto_opts_cleanup)
  )

  TERMUI_CON_LIST_END
};


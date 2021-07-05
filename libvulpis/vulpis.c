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

    Copyright Alexandre Becoulet <alexandre.becoulet@free.fr> (c) 2020

*/

#define LOGK_MODULE_ID "Vlps"

#include <vulpis/vulpis.h>

#include <mutek/printk.h>

#define VULPIS_TX_FREQUENCY 868130000
#define VULPIS_MACRO_CHANNEL_WIDTH 192000
#define VULPIS_CHANNEL_STEP_HZ 100
#define VULPIS_GUARD_BAND 21000

static const struct persist_descriptor_s vulpis_persist_cnt_desc =
{
 .uid = CONFIG_VULPIS_PERSIST_UID_CNT,
 .type = PERSIST_COUNTER,
 .size = 16,
};

const struct persist_descriptor_s vulpis_persist_id_desc =
{
 .uid = CONFIG_VULPIS_PERSIST_UID_ID,
 .type = PERSIST_BLOB,
 .size = 4 + 16,
};

static const struct dev_rfpacket_pk_cfg_basic_s vulpis_pkcfg = {
    .base = {
        .format = DEV_RFPACKET_FMT_SIGFOX,
    },
};

static const uint8_t vulpis_clen_to_lindex[14] = {
  0,				/* 0 */
  1,				/* 1 */
  2, 2, 2,			/* 2 to 4 */
  3, 3, 3, 3,			/* 5 to 8 */
  4, 4, 4, 4, 4			/* 9 to 12 */
};

static const uint8_t vulpis_lindex_to_flen[/* len index */ 5] =
  { 14, 15, 18, 22, 26 };

static const uint8_t vulpis_lindex_to_dlen[/* len index */ 5] =
  { 2, 3, 4, 8, 12 };

static const uint16_t vulpis_lindex_to_sync[/* len index */ 5][/* repeat */ 3] =
{
  { 0xa06b, 0xa6e0, 0xa034 },
  { 0xa08d, 0xa0d2, 0xa302 },
  { 0xa35f, 0xa598, 0xa5a3 },
  { 0xa611, 0xa6bf, 0xa72c },
  { 0xa94c, 0xa971, 0xa997 }
};

static uint16_t vulpis_pn11(uint16_t x, uint16_t min, uint16_t max)
{
  if ( !x )
    x = 2047;

  do {
    x = (((x >> 8) ^ (x >> 10)) & 1) | (x << 1);
  } while (x < min || x > max);

  return x & 2047;
}

static void vulpis_crc16(uint8_t crc_le[2], const uint8_t *data, size_t data_len)
{
  static const uint16_t table[256] = {
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
    0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef,
    0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6,
    0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de,
    0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485,
    0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d,
    0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4,
    0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc,
    0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823,
    0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b,
    0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12,
    0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a,
    0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41,
    0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49,
    0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70,
    0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78,
    0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f,
    0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e,
    0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256,
    0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d,
    0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
    0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c,
    0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634,
    0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab,
    0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3,
    0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a,
    0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92,
    0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9,
    0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1,
    0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8,
    0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0
  };

  uint16_t crc = 0;

  for (size_t i = 0; i < data_len; i++)
    crc = (crc << 8) ^ table[data[i] ^ (uint8_t)(crc >> 8)];
  crc ^= 0xffff;

  crc_le[0] = crc >> 8;
  crc_le[1] = crc;
}

static void vulpis_encode(uint8_t *out, const uint8_t *in, size_t len, uint16_t lut)
{
  uint16_t x = 0, y = 0;
  uint_fast8_t i = 0;

  /*
    TX 0: lut = 0x0000 (copy)
    TX 1: lut = 0x0a00
    TX 2: lut = 0x1100
   */

  /* straightforward implementation of conditional bit reversing ;) */
  while (i < len * 8)
    {
      if (!(i & 7))
	x |= *in++;

      y ^= (lut >> ((x >> 7) & 6)) & 0xc0;

      x <<= 1;
      y <<= 1;
      i++;

      if (!(i & 7))
	*out++ = (y ^ x) >> 8;
    }
}

static inline
void vulpis_set_state(struct vulpis_context_s *ctx, enum vulpis_state_e state)
{
  logk_trace("state = %u", state);
  ctx->state = state;
}

void vulpis_send(struct vulpis_context_s *ctx, const uint8_t *data, size_t bits)
{
  logk_trace("%s", __func__);

  if (ctx->state != VULPIS_ST_IDLE)
    {
      ctx->error = -EBUSY;
      kroutine_exec(&ctx->kr);
      return;
    }

  uint_fast8_t hmac_len, bb;
  size_t len;

  switch (bits)
    {
    case 1:
      bb = (data[0] & 1) | 2;
      hmac_len = 2;
      len = 0;
      break;

    case 8:
      bb = 0;
      hmac_len = 2;
      len = 1;
      break;

    default:
      if (bits > 12 * 8 || (bits & 7))
	{
	case 0:
	  ctx->error = -EINVAL;
          kroutine_exec(&ctx->kr);
          return;
	}
      len = bits / 8;

      bb = 3 - ((len - 1) & 3);
      hmac_len = bb + 2;
      break;
    }

  uint8_t *frame = ctx->frame;

  ctx->hmac_len = hmac_len;
  ctx->len = len;

  frame[0] = bb << 6;
  memcpy(frame + 6, data, len);

  ctx->pr_rq.descriptor = &vulpis_persist_cnt_desc;
  ctx->pr_rq.counter = 1;

  vulpis_set_state(ctx, VULPIS_ST_SEQINC);
  persist_write(ctx->pr_ctx, &ctx->pr_rq);
}

static KROUTINE_EXEC(vulpis_persist_done)
{
  struct persist_rq_s *pr_rq = persist_rq_s_from_kr(kr);
  struct vulpis_context_s *ctx = vulpis_context_s_from_pr_rq(pr_rq);

  logk_trace("%s %i", __func__, pr_rq->error);

  if (pr_rq->error)
    {
      ctx->error = pr_rq->error;
      kroutine_exec(&ctx->kr);
      return;
    }

  switch (ctx->state)
    {
    case VULPIS_ST_SEQINC: {
      uint_fast16_t seq = pr_rq->counter & 0xfff;

      logk_trace("counter: %u", seq);

      ctx->frame[0] |= (seq >> 8) & 15;
      ctx->frame[1] = seq;

      ctx->pr_rq.descriptor = &vulpis_persist_id_desc;

      vulpis_set_state(ctx, VULPIS_ST_ID);
      persist_read(ctx->pr_ctx, &ctx->pr_rq);
      break;
    }

    case VULPIS_ST_ID: {
      logk_trace("id_key: %P", pr_rq->data, 4 + 16);

      memcpy(ctx->frame + 2, pr_rq->data, 4);
      ctx->aes_ctx.key_data = pr_rq->data + 4;

      /* hmac */
      uint_fast8_t hmac_data_len = 6 + ctx->len;
      uint_fast8_t s = (1 + (hmac_data_len > 16)) << 4;
      uint_fast8_t i, j;

      for (i = j = 0; i < s; i++)
	{
	  ctx->hmac[i] = ctx->frame[j];
	  if (++j == hmac_data_len)
	    j = 0;
	}

      ctx->aes_rq.in = ctx->hmac;
      ctx->aes_rq.out = ctx->hmac;
      ctx->aes_rq.len = s;
      ctx->aes_rq.iv_ctr = (uint8_t *)"\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00";

      vulpis_set_state(ctx, VULPIS_ST_HMAC);
      DEVICE_OP(&ctx->aes_dev, request, &ctx->aes_rq);
      break;
    }

    default:
      UNREACHABLE();
    }
}

static void vulpis_send_once(struct vulpis_context_s *ctx)
{
  logk_trace("%s", __func__);

  size_t frame_len = 10 + ctx->len + ctx->hmac_len + 2;
  uint_fast8_t li = vulpis_clen_to_lindex[ctx->len];
  uint16_t fs;

  switch (ctx->state)
    {
    case VULPIS_ST_SEND1:
      fs = vulpis_lindex_to_sync[li][0];
      memcpy(ctx->enc_frame + 4, ctx->frame, frame_len - 4);
      break;
    case VULPIS_ST_SEND2:
      fs = vulpis_lindex_to_sync[li][1];
      vulpis_encode(ctx->enc_frame + 4, ctx->frame, frame_len - 4, 0xa00);
      break;
    case VULPIS_ST_SEND3:
      fs = vulpis_lindex_to_sync[li][2];
      vulpis_encode(ctx->enc_frame + 4, ctx->frame, frame_len - 4, 0x1100);
      break;
    default:
      UNREACHABLE();
    }

  ctx->enc_frame[2] = 0xa0 | (fs >> 8);
  ctx->enc_frame[3] = fs;

  ctx->pn_chan = vulpis_pn11(ctx->pn_chan, VULPIS_GUARD_BAND / VULPIS_CHANNEL_STEP_HZ,
			     VULPIS_MACRO_CHANNEL_WIDTH / VULPIS_CHANNEL_STEP_HZ
			     - VULPIS_GUARD_BAND / VULPIS_CHANNEL_STEP_HZ);

  uint32_t freq = VULPIS_TX_FREQUENCY + ctx->pn_chan * VULPIS_CHANNEL_STEP_HZ
                  - VULPIS_MACRO_CHANNEL_WIDTH / 2;
  ctx->rf_cfg.base.frequency = freq;
  ctx->rf_cfg.base.cache.dirty = 1;

  ctx->rf_rq.tx_buf = ctx->enc_frame;
  ctx->rf_rq.tx_size = frame_len;
  ctx->rf_rq.deadline = 0;
  ctx->rf_rq.lifetime = 0;

  logk_debug("send: %P, freq: %u", ctx->enc_frame, frame_len, freq);

#if 0   /* skip rf send */
  kroutine_exec(&ctx->rf_rq.base.kr);
#else
  DEVICE_OP(&ctx->rf_dev, request, &ctx->rf_rq, NULL);
#endif
}

static KROUTINE_EXEC(vulpis_hmac_done)
{
  struct dev_crypto_rq_s *aes_rq = dev_crypto_rq_from_kr(kr);
  struct vulpis_context_s *ctx = vulpis_context_s_from_aes_rq(aes_rq);

  logk_trace("%s %i", __func__, aes_rq->error);

  if (aes_rq->error)
    {
      ctx->error = aes_rq->error;
      kroutine_exec(&ctx->kr);
      return;
    }

  uint_fast8_t len = ctx->len;
  uint_fast8_t hmac_data_len = 6 + len;
  uint_fast8_t s = (1 + (hmac_data_len > 16)) << 4;

  memcpy(ctx->frame + 6 + len, ctx->hmac + s - 16, ctx->hmac_len);

  vulpis_crc16(ctx->frame + 6 + len + ctx->hmac_len, ctx->frame, 6 + len + ctx->hmac_len);

  ctx->enc_frame[0] = 0xaa;
  ctx->enc_frame[1] = 0xaa;

  vulpis_set_state(ctx, VULPIS_ST_SEND1);
  vulpis_send_once(ctx);
}

static KROUTINE_EXEC(vulpis_rf_done)
{
  struct dev_rfpacket_rq_s *rf_rq = dev_rfpacket_rq_from_kr(kr);
  struct vulpis_context_s *ctx = vulpis_context_s_from_rf_rq(rf_rq);

  logk_trace("%s %i", __func__, rf_rq->error);

  if (rf_rq->error)
    {
      ctx->error = rf_rq->error;
      kroutine_exec(&ctx->kr);
      return;
    }

  switch (ctx->state)
    {
    case VULPIS_ST_SEND1:
      vulpis_set_state(ctx, VULPIS_ST_DELAY1);
      DEVICE_OP(&ctx->tm_dev, request, &ctx->tm_rq);
      break;
    case VULPIS_ST_SEND2:
      vulpis_set_state(ctx, VULPIS_ST_DELAY2);
      DEVICE_OP(&ctx->tm_dev, request, &ctx->tm_rq);
      break;
    case VULPIS_ST_SEND3:
      ctx->error = 0;
      kroutine_exec(&ctx->kr);
      vulpis_set_state(ctx, VULPIS_ST_IDLE);
      return;
    default:
      UNREACHABLE();
    }
}

static KROUTINE_EXEC(vulpis_tm_done)
{
  struct dev_timer_rq_s *tm_rq = dev_timer_rq_from_kr(kr);
  struct vulpis_context_s *ctx = vulpis_context_s_from_tm_rq(tm_rq);

  logk_trace("%s", __func__);

  switch (ctx->state)
    {
    case VULPIS_ST_DELAY1:
      vulpis_set_state(ctx, VULPIS_ST_SEND2);
      vulpis_send_once(ctx);
      break;
    case VULPIS_ST_DELAY2:
      vulpis_set_state(ctx, VULPIS_ST_SEND3);
      vulpis_send_once(ctx);
      break;
    default:
      UNREACHABLE();
    }
}

void vulpis_context_init(struct vulpis_context_s *ctx)
{
  logk_trace("%s", __func__);

  vulpis_set_state(ctx, VULPIS_ST_IDLE);

  memset(&ctx->aes_ctx, 0, sizeof(ctx->aes_ctx));
  ctx->aes_ctx.key_len = 16;
  ctx->aes_ctx.iv_len = 16;
  ctx->aes_ctx.mode = DEV_CRYPTO_MODE_CBC;

  ctx->aes_rq.ctx = &ctx->aes_ctx;
  ctx->aes_rq.op = DEV_CRYPTO_INIT;

  dev_crypto_rq_init(&ctx->aes_rq, vulpis_hmac_done);

  ctx->pn_chan = rand_64() & 2047;

  ctx->rf_cfg.base.drate = 100;

  memset(&ctx->rf_rq, 0, sizeof(ctx->rf_rq));
  ctx->rf_rq.tx_pwr = 19 * 8;
  ctx->rf_rq.type = DEV_RFPACKET_RQ_TX;
  ctx->rf_rq.pk_cfg = &vulpis_pkcfg.base;
  ctx->rf_rq.rf_cfg = &ctx->rf_cfg.base;

  dev_rfpacket_rq_init(&ctx->rf_rq, vulpis_rf_done);

  dev_timer_rq_init(&ctx->tm_rq, vulpis_tm_done);
  dev_timer_init_sec(&ctx->tm_dev, &ctx->tm_rq.delay, NULL, 500, 1000);
  ctx->tm_rq.deadline = 0;

  ctx->pr_rq.uid_offset = CONFIG_VULPIS_PERSIST_UID;
  kroutine_init_deferred(&ctx->pr_rq.kr, vulpis_persist_done);
}

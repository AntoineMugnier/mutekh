
#include <device/class/rfpacket.h>
#include <device/class/timer.h>
#include <device/class/crypto.h>
#include <persist/persist.h>

#ifndef _VULPIS_H_
#define _VULPIS_H_

enum vulpis_state_e
{
  VULPIS_ST_IDLE,
  VULPIS_ST_SEQINC,
  VULPIS_ST_ID,
  VULPIS_ST_HMAC,
  VULPIS_ST_SEND1,
  VULPIS_ST_DELAY1,
  VULPIS_ST_SEND2,
  VULPIS_ST_DELAY2,
  VULPIS_ST_SEND3,
};

struct vulpis_context_s
{
  struct device_rfpacket_s rf_dev;
  struct dev_rfpacket_rf_cfg_fsk_s rf_cfg;
  struct dev_rfpacket_rq_s rf_rq;

  struct device_timer_s tm_dev;
  struct dev_timer_rq_s tm_rq;

  struct device_crypto_s aes_dev;
  struct dev_crypto_context_s aes_ctx;
  struct dev_crypto_rq_s aes_rq;

  struct persist_context_s *pr_ctx;
  struct persist_rq_s pr_rq;

  struct kroutine_s kr;
  enum vulpis_state_e state;
  error_t error;

  uint8_t frame[22];
  union {
    uint8_t enc_frame[26];
    uint8_t hmac[32];
  };

  uint8_t hmac_len;
  uint8_t len;
  uint16_t pn_chan;
};

STRUCT_COMPOSE(vulpis_context_s, aes_rq);
STRUCT_COMPOSE(vulpis_context_s, rf_rq);
STRUCT_COMPOSE(vulpis_context_s, tm_rq);
STRUCT_COMPOSE(vulpis_context_s, pr_rq);

extern const struct persist_descriptor_s vulpis_persist_id_desc;

void vulpis_context_init(struct vulpis_context_s *ctx);

void vulpis_send(struct vulpis_context_s *ctx, const uint8_t *data, size_t bits);

#endif

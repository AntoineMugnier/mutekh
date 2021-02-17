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

   Copyright (c) 2017 Sebastien Cerdan <sebcerdan@gmail.com>

 */

#ifndef RADIO_H_
# define RADIO_H_

#include "common.h"

#define EFR32_RADIO_RFP_BUFFER_SIZE 256

// Rac irq types
enum efr32_rac_irq_type {
  EFR32_RAC_IRQ_TIMEOUT,
  EFR32_RAC_IRQ_TXRX,
};

struct radio_efr32_rfp_ctx_s {
  struct radio_efr32_ctx_s pv;
  uint8_t sg_buffer[EFR32_RADIO_RFP_BUFFER_SIZE];
  bool_t sleep;
  bool_t isSigfox;
  // Req status
  enum dev_rfpacket_status_s done_status;
  enum dev_rfpacket_status_s timeout_status;
  // RAC irq type
  enum efr32_rac_irq_type rac_irq_type;
  // RQ_RX_TIMEOUT time request struct
  struct dev_timer_rq_s rx_cont_trq;
  // Generic rfpacket context struct
  struct dev_rfpacket_ctx_s gctx;
  // Current config
  const struct dev_rfpacket_rf_cfg_s *rf_cfg;
  const struct dev_rfpacket_pk_cfg_s *pk_cfg;
  // Timeout kroutine
  struct kroutine_s kr;
  // Config values
  uint32_t curr_rx_pb_len;
  uint32_t curr_freq;
  uint32_t curr_drate;
  int32_t synth_ratio;
  // local cristal error
  uint16_t osc_ppb;
#ifdef CONFIG_DRIVER_EFR32_RFPACKET_LDC
  // LDC values
  uint32_t ldc_rx_start;
  uint32_t ldc_rx_end;
  bool ldc_canceled;
#endif
};

DRIVER_PV(struct radio_efr32_rfp_ctx_s);
STRUCT_COMPOSE(radio_efr32_rfp_ctx_s, kr);
STRUCT_COMPOSE(radio_efr32_rfp_ctx_s, pv);

// Efr32 radio config functions
int32_t efr32_calc_synth_ratio(uint32_t freq);
error_t efr32_set_tx_power(struct radio_efr32_rfp_ctx_s *ctx, struct dev_rfpacket_rq_s *rq);
void efr32_rfp_set_cca_threshold(struct radio_efr32_rfp_ctx_s *ctx, struct dev_rfpacket_rq_s *rq);
error_t efr32_build_rf_config(struct radio_efr32_rfp_ctx_s *ctx, struct dev_rfpacket_rq_s *rq);
error_t efr32_build_pkt_config(struct radio_efr32_rfp_ctx_s *ctx, struct dev_rfpacket_rq_s *rq);
#ifndef CONFIG_DEVICE_RFPACKET_STATIC_RF_CONFIG
error_t efr32_rfp_fsk_init(struct radio_efr32_rfp_ctx_s *ctx);
#endif

#endif
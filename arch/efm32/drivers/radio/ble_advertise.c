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

#include <ble/protocol/advertise.h>
#include <ble/protocol/gap.h>
#include <mutek/buffer_pool.h>

#include "common.h"

#define EFR32_RADIO_BUFFER_SIZE 4096

#define EFR32_RX_IRQ_FRC_MSK      (EFR32_FRC_IF_RXDONE     |      \
                                   EFR32_FRC_IF_RXOF       |      \
                                   EFR32_FRC_IF_ABORTED    |      \
                                   EFR32_FRC_IF_BLOCKERROR |      \
                                   EFR32_FRC_IF_FRAMEERROR)

enum efr32_radio_ble_state
{
  EFR32_RADIO_STATE_IDLE,
  EFR32_RADIO_STATE_ADV,
  EFR32_RADIO_STATE_SCAN_RQ,
  EFR32_RADIO_STATE_SCAN_RSP,
};

DRIVER_PV(struct radio_efr32_ble_ctx_s
{
  struct radio_efr32_ctx_s      pv;

  uint8_t                       chan_idx;
  struct ble_addr_s             ble_addr;
  struct buffer_s *             adv_packet;
  struct buffer_s *             scan_rsp_packet;
  struct buffer_s *             tx_buffer;
  struct buffer_s *             ble_rx_buffer;
  struct buffer_pool_s          pool;
  enum efr32_radio_ble_state    state;
});

STRUCT_COMPOSE(radio_efr32_ble_ctx_s, pv);

static inline void efr32_radio_fsm(struct radio_efr32_ble_ctx_s *pv);;
static inline void efr32_radio_change_adv_channel(struct radio_efr32_ble_ctx_s *pv);
static error_t efr32_radio_reset(struct radio_efr32_ble_ctx_s *ctx);

static void efr32_radio_set_state(struct radio_efr32_ble_ctx_s *pv, enum efr32_radio_ble_state state)
{
  efr32_radio_printk("drv: st %d\n", state);
  pv->state = state;
}

static KROUTINE_EXEC(efr32_radio_kr_op)
{
  struct radio_efr32_ctx_s *pv = radio_efr32_ctx_s_from_kr(kr);

  /* Protimer init */
  efr32_protimer_init(&pv->pti);
  efr32_protimer_start_counter(&pv->pti);

  dev_timer_value_t value = efr32_protimer_get_value(&pv->pti);
  efr32_protimer_request_start(&pv->pti, value, value + 10000000, 0);
}

static void efr32_radio_start_rx(struct radio_efr32_ble_ctx_s *ctx)
{
  /* Clear buffer */
  cpu_mem_write_32(EFR32_BUFC_ADDR + EFR32_BUFC_CMD_ADDR(1), EFR32_BUFC_CMD_CLEAR); 

  ctx->ble_rx_buffer = buffer_pool_alloc(&ctx->pool);

  cpu_mem_write_32(EFR32_BUFC_ADDR + EFR32_BUFC_ADDR_ADDR(1), (uint32_t)ctx->ble_rx_buffer->data);
  cpu_mem_write_32(EFR32_BUFC_ADDR + EFR32_BUFC_WRITEOFFSET_ADDR(1), 0);
  cpu_mem_write_32(EFR32_BUFC_ADDR + EFR32_BUFC_READOFFSET_ADDR(1), buffer_size(ctx->ble_rx_buffer));

  /* Check RAC state */
  assert(EFR32_RAC_STATUS_STATE_GET(endian_le32(cpu_mem_read_32(EFR32_RAC_ADDR + EFR32_RAC_STATUS_ADDR))) == EFR32_RAC_STATUS_STATE_OFF);
  assert(ctx->ble_rx_buffer);

  /* Enable RX */
  uint32_t x = cpu_mem_read_32(EFR32_RAC_ADDR + EFR32_RAC_RXENSRCEN_ADDR);
  x |= 0x2;
  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_RXENSRCEN_ADDR, x);
}

static void efr32_radio_start_tx(struct radio_efr32_ble_ctx_s *ctx, struct buffer_s * buffer)
{
  assert(buffer);

  /* Check RAC state */
  assert(EFR32_RAC_STATUS_STATE_GET(endian_le32(cpu_mem_read_32(EFR32_RAC_ADDR + EFR32_RAC_STATUS_ADDR))) == EFR32_RAC_STATUS_STATE_OFF);

  /* Clear buffer */
  cpu_mem_write_32(EFR32_BUFC_ADDR + EFR32_BUFC_CMD_ADDR(0), EFR32_BUFC_CMD_CLEAR); 

  cpu_mem_write_32(EFR32_BUFC_ADDR + EFR32_BUFC_ADDR_ADDR(0), (uint32_t)buffer->data);
  cpu_mem_write_32(EFR32_BUFC_ADDR + EFR32_BUFC_WRITEOFFSET_ADDR(0), buffer->data[buffer->begin + 1] + 2); 
  cpu_mem_write_32(EFR32_BUFC_ADDR + EFR32_BUFC_READOFFSET_ADDR(0), 0);

  cpu_mem_write_32(EFR32_BUFC_ADDR + EFR32_BUFC_CMD_ADDR(0), EFR32_BUFC_CMD_PREFETCH);
  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_CMD_ADDR, EFR32_RAC_CMD_TXEN);
}

static inline void efr32_radio_fsm(struct radio_efr32_ble_ctx_s *ctx)
{
  assert(EFR32_RAC_STATUS_STATE_GET(endian_le32(cpu_mem_read_32(EFR32_RAC_ADDR + EFR32_RAC_STATUS_ADDR))) == EFR32_RAC_STATUS_STATE_OFF);

  switch (ctx->state)
    {
      case EFR32_RADIO_STATE_IDLE:
        if (ctx->pv.dev->start_count == 0)
        {
          ctx->chan_idx = 37;
          break;
        }
        cpu_mem_write_32(EFR32_RADIO_SEQ_RAM_ADDR + 0x1F00, 0);
        efr32_radio_set_state(ctx, EFR32_RADIO_STATE_ADV);
        efr32_radio_start_tx(ctx, ctx->adv_packet);
        break;
      case EFR32_RADIO_STATE_ADV:
        efr32_radio_set_state(ctx, EFR32_RADIO_STATE_SCAN_RQ);
        efr32_radio_start_rx(ctx);
        break;
      case EFR32_RADIO_STATE_SCAN_RQ:
        efr32_radio_set_state(ctx, EFR32_RADIO_STATE_SCAN_RSP);
        efr32_radio_start_tx(ctx, ctx->scan_rsp_packet);
        break;
      case EFR32_RADIO_STATE_SCAN_RSP:
        efr32_radio_set_state(ctx, EFR32_RADIO_STATE_IDLE);
        efr32_radio_printk("*%d", ctx->chan_idx);
        efr32_radio_change_adv_channel(ctx);
        break;
    }
}

static inline void efr32_radio_tx_irq(struct radio_efr32_ble_ctx_s *ctx, uint32_t irq)
{
  assert(irq & EFR32_FRC_IF_TXDONE);
  assert(ctx->ble_rx_buffer == NULL);
  return efr32_radio_fsm(ctx);
}

static inline void efr32_radio_change_adv_channel(struct radio_efr32_ble_ctx_s *ctx)
{
  struct radio_efr32_ctx_s *pv = &ctx->pv;

  if (ctx->chan_idx == 39)
    ctx->chan_idx = 37;
  else
    ctx->chan_idx++;

  uint8_t chan_number = 39;
  if (ctx->chan_idx == 37)
    chan_number = 0; 
  else if (ctx->chan_idx == 38)
    chan_number = 12; 

  cpu_mem_write_32(EFR32_FRC_ADDR + EFR32_FRC_WHITEINIT_ADDR, 0x40 | ctx->chan_idx);
  cpu_mem_write_32(EFR32_SYNTH_ADDR + EFR32_SYNTH_CHCTRL_ADDR, chan_number);

  efr32_radio_set_state(ctx, EFR32_RADIO_STATE_IDLE);

  dev_timer_value_t value = efr32_protimer_get_value(&pv->pti);
  efr32_protimer_request_start(&pv->pti, value, value + 480000,  0);
}

static void efr32_protimer_irq(struct device_s *dev)
{
  struct radio_efr32_ble_ctx_s *ctx = dev->drv_pv;

  while (1)
    {
      uint32_t irq = endian_le32(cpu_mem_read_32(EFR32_PROTIMER_ADDR + EFR32_PROTIMER_IF_ADDR));
      irq &= endian_le32(cpu_mem_read_32(EFR32_PROTIMER_ADDR + EFR32_PROTIMER_IEN_ADDR));

      if (!irq)
        break;

      cpu_mem_write_32(EFR32_PROTIMER_ADDR + EFR32_PROTIMER_IFC_ADDR, endian_le32(irq));

      /* Timer class interrupts */
      if (!(irq & (EFR32_PROTIMER_IF_CC(EFR32_PROTIMER_CHANNEL) | EFR32_PROTIMER_IF_WRAPCNTOF)))
        break;

#if EFR32_PROTIMER_HW_WIDTH < 64
      /* Compare channel interrupt */ 
      if (irq & EFR32_PROTIMER_IF_CC(EFR32_PROTIMER_CHANNEL))
         efr32_protimer_disable_compare(&pv->pti, EFR32_PROTIMER_CHANNEL);

      /* Update the software part of the counter */
      if (irq & EFR32_PROTIMER_IF_WRAPCNTOF)
        pv->pti.swvalue++;
#endif

      /* CC channel irq */
      efr32_radio_fsm(ctx);
    }
}

static inline void efr32_radio_rx_irq(struct radio_efr32_ble_ctx_s *ctx, uint32_t irq)
{
  assert(ctx->ble_rx_buffer);

  if (irq != EFR32_FRC_IF_RXDONE)
    goto rx_error;

  ctx->ble_rx_buffer->begin = 0;
  ctx->ble_rx_buffer->end = cpu_mem_read_32(EFR32_BUFC_ADDR + EFR32_BUFC_WRITEOFFSET_ADDR(1)) - 2;

  struct ble_addr_s addr;
  ble_advertise_packet_rxaddr_get(ctx->ble_rx_buffer, &addr);

  /* CRC failed or not a scan request or bad address */
  if (!ctx->ble_rx_buffer->data[ctx->ble_rx_buffer->end + 1] ||
       ble_advertise_packet_type_get(ctx->ble_rx_buffer) != BLE_SCAN_REQ ||
       ble_addr_cmp(&ctx->ble_addr, &addr))
    goto rx_error;

  buffer_refdec(ctx->ble_rx_buffer);
  ctx->ble_rx_buffer = NULL;
  return efr32_radio_fsm(ctx);

rx_error:
  cpu_mem_write_32(EFR32_BUFC_ADDR + EFR32_BUFC_CMD_ADDR(1), EFR32_BUFC_CMD_CLEAR);

  buffer_refdec(ctx->ble_rx_buffer);
  ctx->ble_rx_buffer = NULL;

  return efr32_radio_change_adv_channel(ctx);
}

static DEV_IRQ_SRC_PROCESS(efr32_radio_irq)
{
  struct device_s *dev = ep->base.dev;
  struct radio_efr32_ble_ctx_s *ctx = dev->drv_pv;
  struct radio_efr32_ctx_s *pv = &ctx->pv;

  lock_spin(&dev->lock);

  uint32_t irq = 0;

  switch (ep - pv->irq_ep)
  {
    case 0:
      irq = cpu_mem_read_32(EFR32_MODEM_ADDR + EFR32_MODEM_IF_ADDR);
      irq &= cpu_mem_read_32(EFR32_MODEM_ADDR + EFR32_MODEM_IEN_ADDR);
      efr32_radio_printk("modem irq: 0x%x\n", irq);
    case 1:
    case 2:
      efr32_radio_printk("Rac irq: 0x%x\n", cpu_mem_read_32(EFR32_RAC_ADDR + EFR32_RAC_IF_ADDR));
      break;
    case 3:
      irq = cpu_mem_read_32(EFR32_BUFC_ADDR + EFR32_BUFC_IF_ADDR);
      irq &= cpu_mem_read_32(EFR32_BUFC_ADDR + EFR32_BUFC_IEN_ADDR);
      cpu_mem_write_32(EFR32_BUFC_ADDR + EFR32_BUFC_IFC_ADDR, irq);
      break;
    case 7:
      efr32_protimer_irq(dev);
      break;
    case 8:
        while(1)
        {
          irq = cpu_mem_read_32(EFR32_FRC_ADDR + EFR32_FRC_IF_ADDR);
          irq &= cpu_mem_read_32(EFR32_FRC_ADDR + EFR32_FRC_IEN_ADDR);

          /* Clear irqs */
          cpu_mem_write_32(EFR32_FRC_ADDR + EFR32_FRC_IFC_ADDR, irq);
       
          if (irq == 0)
            break;
       
          if (irq & EFR32_TX_IRQ_FRC_MSK)
            efr32_radio_tx_irq(ctx, irq);

          if (irq & EFR32_RX_IRQ_FRC_MSK)
            efr32_radio_rx_irq(ctx, irq);
        }
      break;
    default:
      efr32_radio_printk("irq: %d\n", ep - pv->irq_ep);
      abort();
      break;
  }

  lock_release(&dev->lock);
}

static DEV_USE(efr32_radio_use)
{
  struct device_accessor_s *accessor = param;

  switch (op)
    {
    case DEV_USE_START: {
      struct device_s *dev = accessor->dev;
      struct radio_efr32_ble_ctx_s *ctx = dev->drv_pv;
      if (dev->start_count == 0)
        kroutine_exec(&ctx->pv.kr);
      return 0;
    }

    case DEV_USE_STOP: {
      return 0;
    }

    default:
      return dev_use_generic(param, op);
    }
}

static DEV_INIT(efr32_radio_init);
static DEV_CLEANUP(efr32_radio_cleanup);

DRIVER_DECLARE(efr32_radio_ble_drv, 0, "EFR32 BLE radio", efr32_radio, NULL);
DRIVER_REGISTER(efr32_radio_ble_drv);

static SLAB_GROW(efr32_grow)
{
  return 42;
}

static DEV_INIT(efr32_radio_init)
{
  struct radio_efr32_ble_ctx_s *ctx;

  /* allocate private driver data */
  ctx = mem_alloc(sizeof(*ctx), (mem_scope_sys));

  if (!ctx)
    return -ENOMEM;

  memset(ctx, 0, sizeof(*ctx));

  dev->drv_pv = ctx;
  ctx->pv.dev = dev;

  buffer_pool_init(&ctx->pool, 128, efr32_grow, mem_scope_sys);

  ctx->ble_addr.type = BLE_ADDR_RANDOM;
  ctx->ble_addr.addr[0] = 0x11;
  ctx->ble_addr.addr[1] = 0x22;
  ctx->ble_addr.addr[2] = 0x33;
  ctx->ble_addr.addr[3] = 0x44;
  ctx->ble_addr.addr[4] = 0x55;
  ctx->ble_addr.addr[5] = 0x66;

  ctx->chan_idx = 37;

  ctx->adv_packet = buffer_pool_alloc(&ctx->pool);
  ctx->scan_rsp_packet = buffer_pool_alloc(&ctx->pool);

  ble_adv_ind_set(ctx->adv_packet, &ctx->ble_addr); 
  ble_adv_data_append(ctx->adv_packet, BLE_GAP_FLAGS, (const uint8_t []){ BLE_GAP_FLAGS_GENERAL_ADV | BLE_GAP_FLAGS_BREDR_NOT_SUPPORTED }, 1);
  ble_adv_data_append(ctx->adv_packet, BLE_GAP_APPEARANCE, (const uint8_t []){
    BLE_GAP_APPEARANCE_GENERIC_MEDIA_PLAYER & 0xff,
    BLE_GAP_APPEARANCE_GENERIC_MEDIA_PLAYER >> 8,
    }, 2);

  ble_adv_scan_rsp_set(ctx->scan_rsp_packet, &ctx->ble_addr); 
  ble_adv_data_string_append(ctx->scan_rsp_packet, BLE_GAP_COMPLETE_LOCAL_NAME, "MutekH on EFR32"); 

  if (efr32_radio_reset(ctx))
    goto err_mem;

  ctx->pv.freq.num = EFR32_RADIO_HFXO_CLK;
  ctx->pv.freq.denom = 1;

  device_get_res_freq(dev, &ctx->pv.freq, 0);

  assert(ctx->pv.freq.denom == 1);

  device_irq_source_init(dev, ctx->pv.irq_ep, EFR32_RADIO_IRQ_COUNT,
      &efr32_radio_irq);

  if (device_irq_source_link(dev, ctx->pv.irq_ep, EFR32_RADIO_IRQ_COUNT, -1))
    goto err_mem;

#ifdef CONFIG_DRIVER_EFR32_DEBUG
  efr32_radio_debug_init(&ctx->pv);
#endif

  return 0;

err_mem:
  mem_free(ctx);
  return -1;
}

static DEV_CLEANUP(efr32_radio_cleanup)
{
  struct radio_efr32_ble_ctx_s *ctx = dev->drv_pv;

  cpu_mem_write_32(EFR32_PROTIMER_ADDR + EFR32_PROTIMER_CMD_ADDR, endian_le32(EFR32_PROTIMER_CMD_STOP));

  buffer_pool_cleanup(&ctx->pool);
  device_irq_source_unlink(dev, ctx->pv.irq_ep, EFR32_RADIO_IRQ_COUNT);

  mem_free(ctx);

  return 0;
}

static error_t efr32_radio_reset(struct radio_efr32_ble_ctx_s *ctx)
{
  struct radio_efr32_ctx_s *pv = &ctx->pv;


  cpu_mem_write_32(EFR32_FRC_ADDR + EFR32_FRC_DFLCTRL_ADDR, EFR32_FRC_DFLCTRL_DFLMODE(SINGLEBYTE) |
                                                      EFR32_FRC_DFLCTRL_DFLOFFSET(1) |
                                                      EFR32_FRC_DFLCTRL_DFLBITS(8) |
                                                      EFR32_FRC_DFLCTRL_MINLENGTH(1));

  cpu_mem_write_32(EFR32_FRC_ADDR + EFR32_FRC_MAXLENGTH_ADDR, EFR32_FRC_MAXLENGTH_MAXLENGTH(256));

  cpu_mem_write_32(EFR32_FRC_ADDR + EFR32_FRC_WCNTCMP1_ADDR, EFR32_FRC_WCNTCMP1_LENGTHFIELDLOC(1));

  /* Whitenning  */
  cpu_mem_write_32(EFR32_FRC_ADDR + EFR32_FRC_WHITEPOLY_ADDR, EFR32_FRC_WHITEPOLY_POLY(68));
  cpu_mem_write_32(EFR32_FRC_ADDR + EFR32_FRC_WHITEINIT_ADDR, EFR32_FRC_WHITEINIT_WHITEINIT(101));

  cpu_mem_write_32(EFR32_FRC_ADDR + EFR32_FRC_FECCTRL_ADDR, EFR32_FRC_FECCTRL_BLOCKWHITEMODE(WHITE) |
                                                      EFR32_FRC_FECCTRL_CONVMODE(DISABLE) |
                                                      EFR32_FRC_FECCTRL_INTERLEAVEMODE(DISABLE));

  cpu_mem_write_32(EFR32_FRC_ADDR + EFR32_FRC_CTRL_ADDR, EFR32_FRC_CTRL_BITORDER(LSB) |
                                                   EFR32_FRC_CTRL_TXFCDMODE(FCDMODE2) |
                                                   EFR32_FRC_CTRL_RXFCDMODE(FCDMODE2) |
                                                   EFR32_FRC_CTRL_BITSPERWORD(7));

  cpu_mem_write_32(EFR32_FRC_ADDR + EFR32_FRC_RXCTRL_ADDR, EFR32_FRC_RXCTRL_BUFRESTOREFRAMEERROR | EFR32_FRC_RXCTRL_BUFRESTORERXABORTED);
  cpu_mem_write_32(EFR32_FRC_ADDR + EFR32_FRC_TRAILRXDATA_ADDR, EFR32_FRC_TRAILRXDATA_RSSI | EFR32_FRC_TRAILRXDATA_CRCOK);

  cpu_mem_write_32(EFR32_FRC_ADDR + EFR32_FRC_FCD_ADDR(0), EFR32_FRC_FCD_WORDS(255) |
                                                     EFR32_FRC_FCD_INCLUDECRC |
                                                     EFR32_FRC_FCD_CALCCRC);

  cpu_mem_write_32(EFR32_FRC_ADDR + EFR32_FRC_FCD_ADDR(1), EFR32_FRC_FCD_WORDS(255) |
                                                     EFR32_FRC_FCD_INCLUDECRC |
                                                     EFR32_FRC_FCD_CALCCRC |
                                                     EFR32_FRC_FCD_SKIPWHITE);

  cpu_mem_write_32(EFR32_FRC_ADDR + EFR32_FRC_FCD_ADDR(2), EFR32_FRC_FCD_WORDS(255) |
                                                     EFR32_FRC_FCD_BUFFER(1) |
                                                     EFR32_FRC_FCD_INCLUDECRC |
                                                     EFR32_FRC_FCD_CALCCRC);

  cpu_mem_write_32(EFR32_FRC_ADDR + EFR32_FRC_FCD_ADDR(3), EFR32_FRC_FCD_WORDS(255) |
                                                     EFR32_FRC_FCD_BUFFER(1) |
                                                     EFR32_FRC_FCD_INCLUDECRC |
                                                     EFR32_FRC_FCD_CALCCRC |
                                                     EFR32_FRC_FCD_SKIPWHITE);

  cpu_mem_write_32(EFR32_FRC_ADDR + EFR32_FRC_MIRRORIF_ADDR, EFR32_FRC_MIRRORIF_TXDONEM | EFR32_FRC_MIRRORIF_IFMIRRORCLEAR);

  /* CRC configuration */
  cpu_mem_write_32(EFR32_CRC_ADDR + EFR32_CRC_CTRL_ADDR, EFR32_CRC_CTRL_CRCWIDTH(CRCWIDTH24) |
                              EFR32_CRC_CTRL_INPUTBITORDER(LSB) |
                              EFR32_CRC_CTRL_BYTEREVERSE(MSB) |
                              EFR32_CRC_CTRL_BITREVERSE(MSB) |
                              EFR32_CRC_CTRL_BITSPERWORD(7));

  cpu_mem_write_32(EFR32_CRC_ADDR + EFR32_CRC_INIT_ADDR, EFR32_CRC_INIT_INIT(0xaaaaaa));
  cpu_mem_write_32(EFR32_CRC_ADDR + EFR32_CRC_POLY_ADDR, EFR32_CRC_POLY_POLY(0xda6000));

  /* Frequency */
  cpu_mem_write_32(EFR32_SYNTH_ADDR + EFR32_SYNTH_CALCTRL_ADDR, EFR32_SYNTH_CALCTRL_NUMCYCLES(1) |
                                                        EFR32_SYNTH_CALCTRL_CAPCALCYCLEWAIT(CYCLES1) |
                                                        EFR32_SYNTH_CALCTRL_STARTUPTIMING(10) |
                                                        EFR32_SYNTH_CALCTRL_AUXCALCYCLES(4) |
                                                        EFR32_SYNTH_CALCTRL_AUXCALCYCLEWAIT(CYCLES64));

  cpu_mem_write_32(EFR32_SYNTH_ADDR + EFR32_SYNTH_VCDACCTRL_ADDR, EFR32_SYNTH_VCDACCTRL_VCDACVAL(35));

  cpu_mem_write_32(EFR32_SYNTH_ADDR + EFR32_SYNTH_FREQ_ADDR, EFR32_SYNTH_FREQ_FREQ(32795306));

  cpu_mem_write_32(EFR32_SYNTH_ADDR + EFR32_SYNTH_IFFREQ_ADDR, EFR32_SYNTH_IFFREQ_IFFREQ(14563) |
                                                       EFR32_SYNTH_IFFREQ_LOSIDE);

  cpu_mem_write_32(EFR32_SYNTH_ADDR + EFR32_SYNTH_DIVCTRL_ADDR, EFR32_SYNTH_DIVCTRL_LODIVFREQCTRL(LODIV1) |
                                                        EFR32_SYNTH_DIVCTRL_AUXLODIVFREQCTRL(LODIV1));

  /* Channel configuration */
  cpu_mem_write_32(EFR32_SYNTH_ADDR + EFR32_SYNTH_CHSP_ADDR, EFR32_SYNTH_CHSP_CHSP(27306));
  cpu_mem_write_32(EFR32_SYNTH_ADDR + EFR32_SYNTH_CHCTRL_ADDR, 0);

  cpu_mem_write_32(EFR32_SYNTH_ADDR + EFR32_SYNTH_VCOGAIN_ADDR, EFR32_SYNTH_VCOGAIN_VCOGAIN(42));
  cpu_mem_write_32(EFR32_SYNTH_ADDR + EFR32_SYNTH_AUXVCDACCTRL_ADDR, EFR32_SYNTH_AUXVCDACCTRL_VALUE(7));
  cpu_mem_write_32(EFR32_SYNTH_ADDR + EFR32_SYNTH_CAPCALCYCLECNT_ADDR, EFR32_SYNTH_CAPCALCYCLECNT_CAPCALCYCLECNT(127));

  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_CTRL_ADDR, EFR32_RAC_CTRL_ACTIVEPOL |
                                                   EFR32_RAC_CTRL_PAENPOL |
                                                   EFR32_RAC_CTRL_LNAENPOL);

  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_IEN_ADDR, EFR32_RAC_IF_SEQ(235));
  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_LVDSCTRL_ADDR, EFR32_RAC_LVDSCTRL_LVDSCURR(3));
  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_LVDSIDLESEQ_ADDR, EFR32_RAC_LVDSIDLESEQ_LVDSIDLESEQ(188));

  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_HFXORETIMECTRL_ADDR, EFR32_RAC_HFXORETIMECTRL_LIMITH(6) |
                                                             EFR32_RAC_HFXORETIMECTRL_LIMITL(7));

  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_WAITSNSH_ADDR, EFR32_RAC_WAITSNSH_WAITSNSH(1));

  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_SEQCTRL_ADDR, EFR32_RAC_SEQCTRL_COMPACT |
                                                      EFR32_RAC_SEQCTRL_COMPINVALMODE(NEVER));

  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_PRESC_ADDR, EFR32_RAC_PRESC_STIMER(7));

  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_SYNTHREGCTRL_ADDR, EFR32_RAC_SYNTHREGCTRL_MMDLDOAMPCURR(3) |
                                                           EFR32_RAC_SYNTHREGCTRL_MMDLDOVREFTRIM(3) |
                                                           EFR32_RAC_SYNTHREGCTRL_VCOLDOAMPCURR(3) |
                                                           EFR32_RAC_SYNTHREGCTRL_VCOLDOVREFTRIM(3) |
                                                           EFR32_RAC_SYNTHREGCTRL_CHPLDOAMPCURR(3) |
                                                           EFR32_RAC_SYNTHREGCTRL_CHPLDOVREFTRIM(3));

  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_VCOCTRL_ADDR, EFR32_RAC_VCOCTRL_VCOAMPLITUDE(10) |
                                                      EFR32_RAC_VCOCTRL_VCODETAMPLITUDE(7) |
                                                      EFR32_RAC_VCOCTRL_VCODETEN |
                                                      EFR32_RAC_VCOCTRL_VCODETMODE |
                                                      EFR32_RAC_VCOCTRL_VCOAREGCURR(1) |
                                                      EFR32_RAC_VCOCTRL_VCOCREGCURR(2) |
                                                      EFR32_RAC_VCOCTRL_VCODIVCURR(15));

  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_MMDCTRL_ADDR, EFR32_RAC_MMDCTRL_MMDDIVDCDC(85) |
                                                      EFR32_RAC_MMDCTRL_MMDDIVRSDCDC(1) |
                                                      EFR32_RAC_MMDCTRL_MMDDIVRSDIG(2) |
                                                      EFR32_RAC_MMDCTRL_MMDENRSDIG);

  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_CHPCTRL_ADDR, EFR32_RAC_CHPCTRL_CHPBIAS(6) |
                                                      EFR32_RAC_CHPCTRL_CHPCURR(5));

  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_CHPCAL_ADDR, EFR32_RAC_CHPCAL_PSRC(4) |
                                                     EFR32_RAC_CHPCAL_NSRC(4));

  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_LPFCTRL_ADDR, EFR32_RAC_LPFCTRL_LPFINPUTCAP(3) |
                                                      EFR32_RAC_LPFCTRL_LPFSWITCHINGEN |
                                                      EFR32_RAC_LPFCTRL_LPFGNDSWITCHINGEN |
                                                      EFR32_RAC_LPFCTRL_LPFBWTX(8));

  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_AUXCTRL_ADDR, EFR32_RAC_AUXCTRL_CHPCURR(1) |
                                                      EFR32_RAC_AUXCTRL_LDOAMPCURR(3) |
                                                      EFR32_RAC_AUXCTRL_LDOVREFTRIM(3) |
                                                      EFR32_RAC_AUXCTRL_LPFRES(1));

  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_RFENCTRL_ADDR, EFR32_RAC_RFENCTRL_DEMEN);

  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_RFENCTRL0_ADDR, EFR32_RAC_RFENCTRL0_CASCODEDIS |
                                                        EFR32_RAC_RFENCTRL0_STRIPESLICEDIS);
  
  /* Power Amplifier */

  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_PACTRL0_ADDR, 0x5f3fffdc);
  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_PAPKDCTRL_ADDR, 0x104d701);
  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_PABIASCTRL0_ADDR, 0x400485);
  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_PABIASCTRL1_ADDR, 0x24525);

  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_RFBIASCTRL_ADDR, EFR32_RAC_RFBIASCTRL_LDOVREF(4) |
                                                         EFR32_RAC_RFBIASCTRL_LDOAMPCURR(3));

  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_RFBIASCAL_ADDR, EFR32_RAC_RFBIASCAL_VREF(22) |
                                                        EFR32_RAC_RFBIASCAL_BIAS(28) |
                                                        EFR32_RAC_RFBIASCAL_TEMPCO(48));

  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_LNAMIXCTRL1_ADDR, EFR32_RAC_LNAMIXCTRL1_TRIMAUXPLLCLK(E0) |
                                                          EFR32_RAC_LNAMIXCTRL1_TRIMTRSWGATEV(3) |
                                                          EFR32_RAC_LNAMIXCTRL1_TRIMVCASLDO |
                                                          EFR32_RAC_LNAMIXCTRL1_TRIMVREGMIN(1));

  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_IFPGACTRL_ADDR, EFR32_RAC_IFPGACTRL_VLDO(3) |
                                                        EFR32_RAC_IFPGACTRL_BANDSEL(2P4) |
                                                        EFR32_RAC_IFPGACTRL_CASCBIAS(7) |
                                                        EFR32_RAC_IFPGACTRL_TRIMVCASLDO |
                                                        EFR32_RAC_IFPGACTRL_TRIMVCM(3) |
                                                        EFR32_RAC_IFPGACTRL_TRIMVREGMIN(1));

  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_IFPGACAL_ADDR, EFR32_RAC_IFPGACAL_IRAMP(2) |
                                                       EFR32_RAC_IFPGACAL_IRPHASE(10) |
                                                       EFR32_RAC_IFPGACAL_OFFSETI(64));

  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_IFFILTCTRL_ADDR, EFR32_RAC_IFFILTCTRL_BANDWIDTH(13) |
                                                         EFR32_RAC_IFFILTCTRL_CENTFREQ(3) |
                                                         EFR32_RAC_IFFILTCTRL_VCM(2) |
                                                         EFR32_RAC_IFFILTCTRL_VREG(4));

  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_IFADCCTRL_ADDR, EFR32_RAC_IFADCCTRL_VLDOSERIES(3) |
                                                        EFR32_RAC_IFADCCTRL_VLDOSERIESCURR(3) |
                                                        EFR32_RAC_IFADCCTRL_VLDOSHUNT(2) |
                                                        EFR32_RAC_IFADCCTRL_VLDOCLKGEN(3) |
                                                        EFR32_RAC_IFADCCTRL_VCM(E2) |
                                                        EFR32_RAC_IFADCCTRL_OTA1CURRENT(2) |
                                                        EFR32_RAC_IFADCCTRL_OTA2CURRENT(2) |
                                                        EFR32_RAC_IFADCCTRL_OTA3CURRENT(2) |
                                                        EFR32_RAC_IFADCCTRL_REGENCLKDELAY(3) |
                                                        EFR32_RAC_IFADCCTRL_INVERTCLK);

  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_PACTUNECTRL_ADDR, EFR32_RAC_PACTUNECTRL_PACTUNERX(4) |
                                                           EFR32_RAC_PACTUNECTRL_SGPACTUNERX(4));
  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_RCTUNE_ADDR, EFR32_RAC_RCTUNE_IFADCRCTUNE(34) |
                                                           EFR32_RAC_RCTUNE_IFFILT(34));
  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_APC_ADDR, EFR32_RAC_APC_AMPCONTROLLIMITSW(255));
  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_CHPCTRL1_ADDR, EFR32_RAC_CHPCTRL1_BYPREPLDORX |
                                                           EFR32_RAC_CHPCTRL1_TRIMREPLDO(1));
  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_MMDCTRL1_ADDR, EFR32_RAC_MMDCTRL1_BYPREPLDORX |
                                                           EFR32_RAC_MMDCTRL1_TRIMREPLDO(1));

  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_FREQOFFEST_ADDR, EFR32_MODEM_FREQOFFEST_CORRVAL(111) |
                                                           EFR32_MODEM_FREQOFFEST_SOFTVAL(198));

  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_MIXCTRL_ADDR, EFR32_MODEM_MIXCTRL_MODE(NORMAL) |
                                                        EFR32_MODEM_MIXCTRL_DIGIQSWAPEN);

  /* Modulation */
  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_CTRL0_ADDR, EFR32_MODEM_CTRL0_MAPFSK(MAP0) |
                                                      EFR32_MODEM_CTRL0_CODING(NRZ) |
                                                      EFR32_MODEM_CTRL0_MODFORMAT(FSK2) |
                                                      EFR32_MODEM_CTRL0_DSSSSHIFTS(NOSHIFT) |
                                                      EFR32_MODEM_CTRL0_DSSSDOUBLE(DIS) |
                                                      EFR32_MODEM_CTRL0_DIFFENCMODE(DIS) |
                                                      EFR32_MODEM_CTRL0_SHAPING(ASYMMETRIC) |
                                                      EFR32_MODEM_CTRL0_DEMODRAWDATASEL(DIS) |
                                                      EFR32_MODEM_CTRL0_FRAMEDETDEL(DEL32));

  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_CTRL1_ADDR, EFR32_MODEM_CTRL1_SYNCBITS(31) |
                                                      EFR32_MODEM_CTRL1_COMPMODE(NOLOCK) |
                                                      EFR32_MODEM_CTRL1_RESYNCPER(1) |
                                                      EFR32_MODEM_CTRL1_PHASEDEMOD(BDD));

  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_CTRL2_ADDR, EFR32_MODEM_CTRL2_SQITHRESH(200) |
                                                      EFR32_MODEM_CTRL2_TXPINMODE(OFF) |
                                                      EFR32_MODEM_CTRL2_DATAFILTER(LEN7) |
                                                      EFR32_MODEM_CTRL2_RATESELMODE(NOCHANGE) |
                                                      EFR32_MODEM_CTRL2_DMASEL(SOFT));

  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_CTRL3_ADDR, EFR32_MODEM_CTRL3_PRSDINSEL(PRSCH0) |
                                                      EFR32_MODEM_CTRL3_ANTDIVMODE(ANTENNA0) |
                                                      EFR32_MODEM_CTRL3_TSAMPMODE(OFF) |
                                                      EFR32_MODEM_CTRL3_TSAMPDEL(2));

  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_CTRL4_ADDR, EFR32_MODEM_CTRL4_ADCSATLEVEL(CONS64));

  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_TXBR_ADDR, EFR32_MODEM_TXBR_TXBRNUM(24) |
                                                     EFR32_MODEM_TXBR_TXBRDEN(5));

  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_RXBR_ADDR, EFR32_MODEM_RXBR_RXBRNUM(1) |
                                                     EFR32_MODEM_RXBR_RXBRDEN(2) |
                                                     EFR32_MODEM_RXBR_RXBRINT(3));

  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_CF_ADDR, EFR32_MODEM_CF_DEC0(DF3) |
                                                   EFR32_MODEM_CF_DEC1(1) |
                                                   EFR32_MODEM_CF_CFOSR(CF7) |
                                                   EFR32_MODEM_CF_DEC1GAIN(ADD0));

  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_PRE_ADDR, EFR32_MODEM_PRE_BASE(2) |
                                                    EFR32_MODEM_PRE_BASEBITS(1) |
                                                    EFR32_MODEM_PRE_PREERRORS(1) |
                                                    EFR32_MODEM_PRE_TXBASES(4));

  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_SYNC0_ADDR, EFR32_MODEM_SYNC0_SYNC0(2391391958));

  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_TIMING_ADDR, EFR32_MODEM_TIMING_TIMTHRESH(140) |
                                                       EFR32_MODEM_TIMING_FDM0THRESH(3) |
                                                       EFR32_MODEM_TIMING_FASTRESYNC(DIS));

  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_MODINDEX_ADDR, EFR32_MODEM_MODINDEX_MODINDEXM(20) |
                                                         EFR32_MODEM_MODINDEX_FREQGAINE(2) |
                                                         EFR32_MODEM_MODINDEX_FREQGAINM(3));

  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_SHAPING0_ADDR, EFR32_MODEM_SHAPING0_COEFF0(1) |
                                                         EFR32_MODEM_SHAPING0_COEFF1(4) |
                                                         EFR32_MODEM_SHAPING0_COEFF2(11) |
                                                         EFR32_MODEM_SHAPING0_COEFF3(25));

  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_SHAPING1_ADDR, EFR32_MODEM_SHAPING1_COEFF4(44) |
                                                         EFR32_MODEM_SHAPING1_COEFF5(64) |
                                                         EFR32_MODEM_SHAPING1_COEFF6(76) |
                                                         EFR32_MODEM_SHAPING1_COEFF7(89));

  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_SHAPING2_ADDR, EFR32_MODEM_SHAPING2_COEFF8(96) |
                                                         EFR32_MODEM_SHAPING2_COEFF9(91) |
                                                         EFR32_MODEM_SHAPING2_COEFF10(83) |
                                                         EFR32_MODEM_SHAPING2_COEFF11(71));

  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_SHAPING3_ADDR, EFR32_MODEM_SHAPING3_COEFF12(54) |
                                                         EFR32_MODEM_SHAPING3_COEFF13(37) |
                                                         EFR32_MODEM_SHAPING3_COEFF14(27) |
                                                         EFR32_MODEM_SHAPING3_COEFF15(17));

  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_SHAPING4_ADDR, EFR32_MODEM_SHAPING4_COEFF17(10) |
                                                         EFR32_MODEM_SHAPING4_COEFF18(13) |
                                                         EFR32_MODEM_SHAPING4_COEFF19(15) |
                                                         EFR32_MODEM_SHAPING4_COEFF20(14) |
                                                         EFR32_MODEM_SHAPING4_COEFF21(12));

  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_SHAPING5_ADDR, EFR32_MODEM_SHAPING5_COEFF22(9) |
                                                         EFR32_MODEM_SHAPING5_COEFF23(7) |
                                                         EFR32_MODEM_SHAPING5_COEFF24(6) |
                                                         EFR32_MODEM_SHAPING5_COEFF25(5) |
                                                         EFR32_MODEM_SHAPING5_COEFF26(4) |
                                                         EFR32_MODEM_SHAPING5_COEFF27(3) |
                                                         EFR32_MODEM_SHAPING5_COEFF28(2) |
                                                         EFR32_MODEM_SHAPING5_COEFF29(2));

  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_SHAPING6_ADDR, EFR32_MODEM_SHAPING6_COEFF30(1) |
                                                         EFR32_MODEM_SHAPING6_COEFF31(1) |
                                                         EFR32_MODEM_SHAPING6_COEFF32(1) |
                                                         EFR32_MODEM_SHAPING6_COEFF33(1) |
                                                         EFR32_MODEM_SHAPING6_COEFF34(1));

  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_RAMPCTRL_ADDR, EFR32_MODEM_RAMPCTRL_RAMPRATE2(4));

  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_RAMPLEV_ADDR, EFR32_MODEM_RAMPLEV_RAMPLEV2(180));

  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_DCCOMP_ADDR, EFR32_MODEM_DCCOMP_DCESTIEN |
                                                       EFR32_MODEM_DCCOMP_DCCOMPEN |
                                                       EFR32_MODEM_DCCOMP_DCCOMPGEAR(3) |
                                                       EFR32_MODEM_DCCOMP_DCLIMIT(FULLSCALE));

  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_DCESTI_ADDR, EFR32_MODEM_DCESTI_DCCOMPESTIVALI(32650) |
                                                       EFR32_MODEM_DCESTI_DCCOMPESTIVALQ(236));

  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_SRCCHF_ADDR, EFR32_MODEM_SRCCHF_SRCRATIO1(143) |
                                                       EFR32_MODEM_SRCCHF_SRCENABLE1 |
                                                       EFR32_MODEM_SRCCHF_SRCRATIO2(838) |
                                                       EFR32_MODEM_SRCCHF_SRCENABLE2 |
                                                       EFR32_MODEM_SRCCHF_BWSEL(2) |
                                                       EFR32_MODEM_SRCCHF_INTOSR);

  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_DSATHD0_ADDR, EFR32_MODEM_DSATHD0_SPIKETHD(50) |
                                                        EFR32_MODEM_DSATHD0_UNMODTHD(4) |
                                                        EFR32_MODEM_DSATHD0_FDEVMINTHD(8) |
                                                        EFR32_MODEM_DSATHD0_FDEVMAXTHD(100));

  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_DSATHD1_ADDR, EFR32_MODEM_DSATHD1_POWABSTHD(2000) |
                                                        EFR32_MODEM_DSATHD1_POWRELTHD(MODE3) |
                                                        EFR32_MODEM_DSATHD1_DSARSTCNT(2) |
                                                        EFR32_MODEM_DSATHD1_RSSIJMPTHD(6) |
                                                        EFR32_MODEM_DSATHD1_PWRFLTBYP |
                                                        EFR32_MODEM_DSATHD1_FREQSCALE);

  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_DSACTRL_ADDR, EFR32_MODEM_DSACTRL_DSAMODE(ENABLED) |
                                                        EFR32_MODEM_DSACTRL_ARRTHD(4) |
                                                        EFR32_MODEM_DSACTRL_ARRTOLERTHD0(2) |
                                                        EFR32_MODEM_DSACTRL_ARRTOLERTHD1(4) |
                                                        EFR32_MODEM_DSACTRL_FREQAVGSYM |
                                                        EFR32_MODEM_DSACTRL_DSARSTON);

  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_VITERBIDEMOD_ADDR, EFR32_MODEM_VITERBIDEMOD_VTDEMODEN |
                                                             EFR32_MODEM_VITERBIDEMOD_VITERBIKSI1(62) |
                                                             EFR32_MODEM_VITERBIDEMOD_VITERBIKSI2(42) |
                                                             EFR32_MODEM_VITERBIDEMOD_VITERBIKSI3(28));

  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_VTCORRCFG0_ADDR, EFR32_MODEM_VTCORRCFG0_EXPECTPATT(355197) |
                                                           EFR32_MODEM_VTCORRCFG0_EXPSYNCLEN(126) |
                                                           EFR32_MODEM_VTCORRCFG0_BUFFHEAD(12));

  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_DIGMIXCTRL_ADDR, EFR32_MODEM_DIGMIXCTRL_DIGMIXFREQ(87377) |
                                                           EFR32_MODEM_DIGMIXCTRL_DIGMIXMODE);

  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_VTCORRCFG1_ADDR, EFR32_MODEM_VTCORRCFG1_CORRSHFTLEN(48) |
                                                           EFR32_MODEM_VTCORRCFG1_VTFRQLIM(110));

  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_VTTRACK_ADDR, EFR32_MODEM_VTTRACK_FREQTRACKMODE(MODE1) |
                                                           EFR32_MODEM_VTTRACK_TIMTRACKTHD(2) |
                                                           EFR32_MODEM_VTTRACK_TIMEACQUTHD(238) |
                                                           EFR32_MODEM_VTTRACK_TIMEOUTMODE |
                                                           EFR32_MODEM_VTTRACK_TIMGEAR(GEAR0));

  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_BREST_ADDR, EFR32_MODEM_BREST_BRESTINT(7));

  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_CGCLKSTOP_ADDR, EFR32_MODEM_CGCLKSTOP_FORCEOFF(57343));

  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_POE_ADDR, EFR32_MODEM_POE_POEI(511));

  cpu_mem_write_32(EFR32_AGC_ADDR + EFR32_AGC_RSSI_ADDR, EFR32_AGC_RSSI_RSSIFRAC(2) |
                                                   EFR32_AGC_RSSI_RSSIINT(209));

  cpu_mem_write_32(EFR32_AGC_ADDR + EFR32_AGC_FRAMERSSI_ADDR, EFR32_AGC_FRAMERSSI_FRAMERSSIFRAC(1) |
                                                           EFR32_AGC_FRAMERSSI_FRAMERSSIINT(195));

  cpu_mem_write_32(EFR32_AGC_ADDR + EFR32_AGC_CTRL0_ADDR, EFR32_AGC_CTRL0_PWRTARGET(248) |
                                                    EFR32_AGC_CTRL0_MODE(CONT) |
                                                    EFR32_AGC_CTRL0_RSSISHIFT(78));

  cpu_mem_write_32(EFR32_AGC_ADDR + EFR32_AGC_CTRL1_ADDR, EFR32_AGC_CTRL1_RSSIPERIOD(3));

  cpu_mem_write_32(EFR32_AGC_ADDR + EFR32_AGC_CTRL2_ADDR, EFR32_AGC_CTRL2_HYST(3) |
                                                    EFR32_AGC_CTRL2_FASTLOOPDEL(5) |
                                                    EFR32_AGC_CTRL2_CFLOOPDEL(29) |
                                                    EFR32_AGC_CTRL2_ADCRSTFASTLOOP(GAINREDUCTION) |
                                                    EFR32_AGC_CTRL2_ADCRSTSTARTUP);

  cpu_mem_write_32(EFR32_AGC_ADDR + EFR32_AGC_IFPEAKDET_ADDR, EFR32_AGC_IFPEAKDET_PKDTHRESH1(2) |
                                                           EFR32_AGC_IFPEAKDET_PKDTHRESH2(8));
  cpu_mem_write_32(EFR32_AGC_ADDR + EFR32_AGC_MANGAIN_ADDR, EFR32_AGC_MANGAIN_MANGAINLNAATTEN(12));
  cpu_mem_write_32(EFR32_AGC_ADDR + EFR32_AGC_RFPEAKDET_ADDR, EFR32_AGC_RFPEAKDET_RFPKDTHRESH1(5) |
                                                           EFR32_AGC_RFPEAKDET_RFPKDTHRESH2(13));
  cpu_mem_write_32(EFR32_AGC_ADDR + EFR32_AGC_GAINRANGE_ADDR, EFR32_AGC_GAINRANGE_MAXGAIN(60) |
                                                           EFR32_AGC_GAINRANGE_MINGAIN(122));
  cpu_mem_write_32(EFR32_AGC_ADDR + EFR32_AGC_GAININDEX_ADDR, EFR32_AGC_GAININDEX_NUMINDEXPGA(12) |
                                                        EFR32_AGC_GAININDEX_NUMINDEXDEGEN(3) |
                                                        EFR32_AGC_GAININDEX_NUMINDEXSLICES(6) |
                                                        EFR32_AGC_GAININDEX_NUMINDEXATTEN(12));

  cpu_mem_write_32(EFR32_AGC_ADDR + EFR32_AGC_SLICECODE_ADDR, EFR32_AGC_SLICECODE_SLICECODEINDEX0(3) |
                                                        EFR32_AGC_SLICECODE_SLICECODEINDEX1(4) |
                                                        EFR32_AGC_SLICECODE_SLICECODEINDEX2(5) |
                                                        EFR32_AGC_SLICECODE_SLICECODEINDEX3(6) |
                                                        EFR32_AGC_SLICECODE_SLICECODEINDEX4(8) |
                                                        EFR32_AGC_SLICECODE_SLICECODEINDEX5(10) |
                                                        EFR32_AGC_SLICECODE_SLICECODEINDEX6(12));

  cpu_mem_write_32(EFR32_AGC_ADDR + EFR32_AGC_ATTENCODE1_ADDR, EFR32_AGC_ATTENCODE1_ATTENCODEINDEX1(1) |
                                                         EFR32_AGC_ATTENCODE1_ATTENCODEINDEX2(2) |
                                                         EFR32_AGC_ATTENCODE1_ATTENCODEINDEX3(3) |
                                                         EFR32_AGC_ATTENCODE1_ATTENCODEINDEX4(4) |
                                                         EFR32_AGC_ATTENCODE1_ATTENCODEINDEX5(5) |
                                                         EFR32_AGC_ATTENCODE1_ATTENCODEINDEX6(6));

  cpu_mem_write_32(EFR32_AGC_ADDR + EFR32_AGC_ATTENCODE2_ADDR, EFR32_AGC_ATTENCODE2_ATTENCODEINDEX7(7) |
                                                         EFR32_AGC_ATTENCODE2_ATTENCODEINDEX8(8) |
                                                         EFR32_AGC_ATTENCODE2_ATTENCODEINDEX9(9) |
                                                         EFR32_AGC_ATTENCODE2_ATTENCODEINDEX10(10) |
                                                         EFR32_AGC_ATTENCODE2_ATTENCODEINDEX11(11) |
                                                         EFR32_AGC_ATTENCODE2_ATTENCODEINDEX12(12));

  cpu_mem_write_32(EFR32_AGC_ADDR + EFR32_AGC_ATTENCODE3_ADDR, EFR32_AGC_ATTENCODE3_ATTENCODEINDEX13(13) |
                                                         EFR32_AGC_ATTENCODE3_ATTENCODEINDEX14(14) |
                                                         EFR32_AGC_ATTENCODE3_ATTENCODEINDEX15(15) |
                                                         EFR32_AGC_ATTENCODE3_ATTENCODEINDEX16(16) |
                                                         EFR32_AGC_ATTENCODE3_ATTENCODEINDEX17(17) |
                                                         EFR32_AGC_ATTENCODE3_ATTENCODEINDEX18(18));

  cpu_mem_write_32(EFR32_AGC_ADDR + EFR32_AGC_GAINSTEPLIM_ADDR, EFR32_AGC_GAINSTEPLIM_FASTSTEPDOWN(5) |
                                                          EFR32_AGC_GAINSTEPLIM_FASTSTEPUP(2) |
                                                          EFR32_AGC_GAINSTEPLIM_CFLOOPSTEPMAX(1) |
                                                          EFR32_AGC_GAINSTEPLIM_ADCATTENMODE(DISABLE));

  cpu_mem_write_32(EFR32_AGC_ADDR + EFR32_AGC_LOOPDEL_ADDR, EFR32_AGC_LOOPDEL_PKDWAIT(15) |
                                                      EFR32_AGC_LOOPDEL_IFPGADEL(7) |
                                                      EFR32_AGC_LOOPDEL_LNASLICESDEL(7));

  cpu_mem_write_32(EFR32_AGC_ADDR + EFR32_AGC_MININDEX_ADDR, EFR32_AGC_MININDEX_INDEXMINATTEN(6) |
                                                       EFR32_AGC_MININDEX_INDEXMINDEGEN(30) |
                                                       EFR32_AGC_MININDEX_INDEXMINPGA(18));

  /* Sequencer code initialisaton */
  efr32_radio_seq_init(pv, seqcode, 4 * seqcode_size);

  /* Clear buffer */
  for (uint8_t i = 0; i < 4; i++)
    cpu_mem_write_32(EFR32_BUFC_ADDR + EFR32_BUFC_CMD_ADDR(i), EFR32_BUFC_CMD_CLEAR);

  // TX/RX buffers initialisation 

  uint32_t x = bit_ctz32(EFR32_RADIO_BUFFER_SIZE) - 6;

  cpu_mem_write_32(EFR32_BUFC_ADDR + EFR32_BUFC_ADDR_ADDR(2), (uint32_t)pv->rx_length_buffer);
  cpu_mem_write_32(EFR32_BUFC_ADDR + EFR32_BUFC_CTRL_ADDR(2), 0);

  /* Enable irq */
  x =  cpu_mem_read_32(EFR32_FRC_ADDR + EFR32_FRC_IEN_ADDR);
  x |= 0x1FF;
  cpu_mem_write_32(EFR32_FRC_ADDR + EFR32_FRC_IFC_ADDR, x); 
  cpu_mem_write_32(EFR32_FRC_ADDR + EFR32_FRC_IEN_ADDR, x); 

  efr32_radio_set_state(ctx, EFR32_RADIO_STATE_IDLE);

  kroutine_init_deferred(&pv->kr, &efr32_radio_kr_op);

  return 0;
}


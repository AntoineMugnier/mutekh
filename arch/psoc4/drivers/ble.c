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

  Copyright (c) Nicolas Pouillon <nipo@ssji.net>, 2016
*/

#include <hexo/types.h>
#include <hexo/endian.h>
#include <hexo/iospace.h>
#include <hexo/bit.h>
#include <hexo/interrupt.h>

#include <mutek/mem_alloc.h>
#include <mutek/printk.h>

#include <device/resources.h>
#include <device/device.h>
#include <device/class/timer.h>
#include <device/class/crypto.h>
#include <device/clock.h>
#include <device/irq.h>

#include <arch/psoc4/bless.h>
#include <arch/psoc4/blell.h>
#include <arch/psoc4/blerd.h>
#include <arch/psoc4/sflash.h>
#include <arch/psoc4/variant.h>

#define BLESS(a) (PSOC4_BLESS_ADDR + BLESS_##a##_ADDR)
#define BLERD(a) (PSOC4_BLERD_ADDR + BLERD_##a##_ADDR)
#define BLELL(a) (PSOC4_BLELL_ADDR + BLELL_##a##_ADDR)
#define SFLASH(a) (PSOC4_SFLASH_ADDR + SFLASH_##a##_ADDR)

#define BLESSn(a, n) (PSOC4_BLESS_ADDR + BLESS_##a##_ADDR(n))
#define BLERDn(a, n) (PSOC4_BLERD_ADDR + BLERD_##a##_ADDR(n))
#define BLELLn(a, n) (PSOC4_BLELL_ADDR + BLELL_##a##_ADDR(n))

//#define dprintk printk
#ifndef dprintk
# define dwritek(...) do{}while(0)
# define dprintk(...) do{}while(0)
#else
# define dwritek writek
#endif

enum start_bit_e
{
  START_AES,
  START_BLE,
};

struct psoc4_ble_private_s
{
  struct dev_clock_sink_ep_s sink[PSOC4_BLE_CLK_SINK_COUNT];
  struct dev_freq_s sleep_freq;
  struct dev_irq_src_s irq_ep;

  dev_request_queue_root_t aes_q;

  uint8_t bb_clk_mhz;
  bool_t ll_powered;
  bool_t adv;
  bool_t aes_busy;
};

DRIVER_PV(struct psoc4_ble_private_s);

static DEVCRYPTO_INFO(psoc4_ble_aes_info)
{
  memset(info, 0, sizeof(*info));

  info->name = "aes";
  info->modes_mask = 0
    | bit(DEV_CRYPTO_MODE_ECB)
    ;
  info->cap = DEV_CRYPTO_CAP_INPLACE
    | DEV_CRYPTO_CAP_NOTINPLACE
    | DEV_CRYPTO_CAP_STATEFUL
    | DEV_CRYPTO_CAP_128BITS_KEY
    ;
  info->state_size = 0;
  info->block_len = 16;

  return 0;
};

static void psoc4_ble_stop(struct device_s *dev, uint32_t clear_mask)
{
  dprintk("%s %x %d\n", __FUNCTION__, clear_mask, dev->start_count);

  dev->start_count &= ~clear_mask;

  device_sleep_schedule(dev);
}

static bool_t psoc4_ble_start(struct device_s *dev, uint32_t set_mask)
{
  struct psoc4_ble_private_s *pv = dev->drv_pv;

  dprintk("%s %x %d\n", __FUNCTION__, set_mask, dev->start_count);

  dev->start_count |= set_mask;

  if (dev->start_count >> CONFIG_DEVICE_START_LOG2INC)
    dev_clock_sink_gate(&pv->sink[PSOC4_BLE_CLK_SINK_LFCLK], DEV_CLOCK_EP_CLOCK);
  
  if (dev->start_count & (bit(START_BLE) | bit(START_AES)))
    dev_clock_sink_gate(&pv->sink[PSOC4_BLE_CLK_SINK_LL], DEV_CLOCK_EP_ANY);

  return ((pv->sink[PSOC4_BLE_CLK_SINK_LL].src->flags
           & DEV_CLOCK_EP_ANY) == DEV_CLOCK_EP_ANY
          && (pv->sink[PSOC4_BLE_CLK_SINK_LFCLK].src->flags
              & DEV_CLOCK_EP_CLOCK));
}

static void psoc4_ble_aes_exec_first(struct device_s *dev)
{
  struct psoc4_ble_private_s *pv = dev->drv_pv;
  struct dev_crypto_rq_s *rq
    = dev_crypto_rq_s_cast(dev_request_queue_head(&pv->aes_q));

  dprintk("%s\n", __FUNCTION__);

  if (!rq) {
    dprintk(" -> no rq\n");
    cpu_mem_write_16(BLELL(COMMAND), BLELL_COMMAND_COMMAND_CLK_ENC_OFF);
    return psoc4_ble_stop(dev, bit(START_AES));
  }

  if (!psoc4_ble_start(dev, bit(START_AES))) {
    dprintk(" -> no clock\n");
    return;
  }

  if (pv->aes_busy) {
    dprintk(" -> busy\n");
    return;
  }

  struct dev_crypto_context_s *ctx = rq->ctx;

  cpu_mem_write_16(BLELL(COMMAND), BLELL_COMMAND_COMMAND_CLK_ENC_ON);

  dprintk(" -> starting %d\n", ctx->mode);

  pv->aes_busy = 1;

  switch ((int)ctx->mode) {
  case DEV_CRYPTO_MODE_ECB:
    cpu_mem_write_16(BLELL(ENC_INTR), 0
                     | BLELL_ENC_INTR_IN_DATA_CLEAR
                     );

    cpu_mem_write_16(BLELL(ENC_INTR_EN), 0
                     | BLELL_ENC_INTR_EN_ECB
                     );

    cpu_mem_write_16(BLELL(ENC_PARAMS), 0
                     );

    cpu_mem_write_16(BLELL(ENC_CONFIG), 0
                     | BLELL_ENC_CONFIG_MODE(ECB)
                     );

    for (uint_fast8_t i = 0; i < 8; i++)
      cpu_mem_write_16(BLELLn(ENC_KEY, i),
                       endian_le16_na_load((const uint8_t *)rq->ctx->key_data + i * 2));

    for (uint_fast8_t i = 0; i < 8; i++)
      cpu_mem_write_16(BLELLn(DATA, i),
                       endian_le16_na_load((const uint8_t *)rq->in + i * 2));

    cpu_mem_write_16(BLELL(ENC_CONFIG), 0
                     | BLELL_ENC_CONFIG_MODE(ECB)
                     | BLELL_ENC_CONFIG_START_PROC
                     );

    break;

  default:
    assert(0);
  }
}

static DEVCRYPTO_REQUEST(psoc4_ble_aes_request)
{
  struct device_s *dev = accessor->dev;
  struct psoc4_ble_private_s *pv = dev->drv_pv;
  struct dev_crypto_context_s *ctx = rq->ctx;
  bool_t empty;

  switch ((int)ctx->mode) {
  case DEV_CRYPTO_MODE_ECB:
    if (rq->op & DEV_CRYPTO_INVERSE)
      goto nosup;

    empty = dev_request_queue_isempty(&pv->aes_q);
    dev_request_queue_pushback(&pv->aes_q, &rq->rq);
    if (empty)
      psoc4_ble_aes_exec_first(dev);
    break;

  default:
  nosup:
    rq->err = -ENOTSUP;
    kroutine_exec(&rq->rq.kr);
    break;
  }
}

static
void blell_enc_intr(struct device_s *dev)
{
  struct psoc4_ble_private_s *pv = dev->drv_pv;
  struct dev_crypto_rq_s *rq;
  uint32_t tmp;

  dprintk("%s %d\n", __FUNCTION__, pv->aes_busy);

  if (!pv->aes_busy)
    goto again;

  pv->aes_busy = 0;

  tmp = cpu_mem_read_16(BLELL(ENC_INTR));
  tmp &= ~BLELL_ENC_INTR_IN_DATA_CLEAR;
  cpu_mem_write_16(BLELL(ENC_INTR), tmp);

  dprintk("%s intr = %x\n", __FUNCTION__, tmp);

  rq = dev_crypto_rq_s_cast(dev_request_queue_pop(&pv->aes_q));
  if (!rq)
    goto again;

  if (tmp & BLELL_ENC_INTR_ECB) {
    switch ((int)rq->ctx->mode) {
    case DEV_CRYPTO_MODE_ECB:
      for (uint_fast8_t i = 0; i < 8; i++)
        endian_le16_na_store((uint8_t *)rq->out + i * 2,
                             cpu_mem_read_16(BLELLn(DATA, i)));
      rq->err = 0;
      kroutine_exec(&rq->rq.kr);
      break;
    }
  }

 again:
  psoc4_ble_aes_exec_first(dev);
}

static
void psoc4_blerd_init(struct psoc4_ble_private_s *pv)
{
  uint32_t tmp;
  uint32_t bb_bump2, ldo;

  dprintk("%s\n", __FUNCTION__);

  cpu_mem_write_16(BLELL(COMMAND), BLELL_COMMAND_COMMAND_CLK_CORE_ON);

  /* Disable Auto LPM. */
  // TODO check whether it belongs here
  CPU_INTERRUPT_SAVESTATE_DISABLE;
  tmp = cpu_mem_read_32(BLESS(WCO_CONFIG));
  tmp &= ~BLESS_WCO_CONFIG_LPM_EN;
  tmp &= ~BLESS_WCO_CONFIG_LPM_AUTO;
  cpu_mem_write_32(BLESS(WCO_CONFIG), tmp);
  CPU_INTERRUPT_RESTORESTATE;

  // IRQ control, nothing to do here
  cpu_mem_write_32(BLESS(LL_DSM_CTRL), 0 // 0xe
//                   | BLESS_LL_DSM_CTRL_DSM_ENTERED_INTR_EN
//                   | BLESS_LL_DSM_CTRL_DSM_EXITED_INTR_EN
//                   | BLESS_LL_DSM_CTRL_XTAL_ON_INTR_EN
                   );

  cpu_mem_write_32(BLERD(BB_XO_CAPTRIM), 0 // 0x6555
                   | BLERD_BB_XO_CAPTRIM_X1(101) // 13.82pF
                   | BLERD_BB_XO_CAPTRIM_X2(85) // 12.215pF
                   );

  // From CyComponentLibrary Boot code
  bb_bump2 = cpu_mem_read_16(SFLASH(BLESS_BB_BUMP2));
  ldo = cpu_mem_read_16(SFLASH(BLESS_LDO));

  if ((ldo == 0x0b58 && bb_bump2 == 0x0007)
      || (ldo == 0x0d40 && bb_bump2 == 0x0004)) {
    ldo = 0x0d58;
    bb_bump2 = 0x0004;
  }

  cpu_mem_write_32(BLERD(LDO), ldo);
  cpu_mem_write_32(BLERD(BB_BUMP2), bb_bump2);

  tmp = cpu_mem_read_16(SFLASH(BLESS_SY_BUMP1));
  if (tmp == 0x0f0f || tmp == 0x0505 || tmp == 0x0005)
    tmp = 0x0f05;
  cpu_mem_write_32(BLERD(SY_BUMP1), tmp);


  cpu_mem_write_32(BLERD(CFG1), 0
                   | BLERD_CFG1_LNA_GAIN(VHG)
                   | BLERD_CFG1_CBPF_GAIN(15DB)
                   | BLERD_CFG1_ADC_DC_CAPTURE_EN
                   | BLERD_CFG1_ADC_IQ_INVERSE
                   );

  cpu_mem_write_32(BLERD(CFG2), 0
                   | BLERD_CFG2_DAC_REG_DATA(2)
                   );

  tmp = cpu_mem_read_32(BLERD(DBUS));
  tmp |= BLERD_DBUS_ISOLATE_N;
  cpu_mem_write_32(BLERD(DBUS), tmp);

  // From test code sent by Cypress
  // Magic constants from original code have been translated
  // back to defines.
  cpu_mem_write_32(BLERD(SY_BUMP2), 0 // 0x20
                   | BLERD_SY_BUMP2_FCAL_BIAS_SEL(0)
                   | BLERD_SY_BUMP2_ACAP_BIAS_SEL(630MV)
                   | BLERD_SY_BUMP2_ICP_XFACTOR(2)
                   | BLERD_SY_BUMP2_ICP_OFFSET(0)
                   | BLERD_SY_BUMP2_CLKNC_MODE(ON_SYNC)
                   | BLERD_SY_BUMP2_RST_DLY(500PS)
                   | BLERD_SY_BUMP2_PDCP_OFFSET(38)
                   );

  cpu_mem_write_32(BLERD(ADC_BUMP2), 0 // 0x14
                   | BLERD_ADC_BUMP2_DUTCYCLE(25)
                   | BLERD_ADC_BUMP2_IBUMP
                   | BLERD_ADC_BUMP2_RETURN_SKEW(0)
                   | BLERD_ADC_BUMP2_PREAMP_BWCTRL(0)
                   );
  cpu_mem_write_32(BLERD(CTR1), 0 // 0x01C0
                   | BLERD_CTR1_VCO_WARMUP_TIME(5US)
                   | BLERD_CTR1_PLL_SETTLING_TIME(25US)
                   | BLERD_CTR1_TX_FREEZE_TIME(30US)
                   | BLERD_CTR1_TX_PREDRV_TIME(AFTER_KVCAL)
                   | BLERD_CTR1_TX_MODSTART_US(3)
                   | BLERD_CTR1_TX_DF2_SEL(242KHZ)
                   | BLERD_CTR1_DBG_SELECT(DEMOD)
                   | BLERD_CTR1_AGC_RST_DLY(0)
                   );
  cpu_mem_write_32(BLERD(THRSHD1), 0 // 0x3F2F
                   | BLERD_THRSHD1_AGC60_66(47)
                   | BLERD_THRSHD1_AGC66_60(63)
                   );
  cpu_mem_write_32(BLERD(THRSHD2), 0 // 0x012E
                   | BLERD_THRSHD2_AGC48_60(46)
                   | BLERD_THRSHD2_AGC60_48(1)
                   );
  cpu_mem_write_32(BLERD(THRSHD3), 0 // 0x3F30
                   | BLERD_THRSHD3_AGC36_48(48)
                   | BLERD_THRSHD3_AGC48_36(63)
                   );
  cpu_mem_write_32(BLERD(THRSHD4), 0 // 0x0231
                   | BLERD_THRSHD4_AGC18_36(49)
                   | BLERD_THRSHD4_AGC36_18(2)
                   );
  cpu_mem_write_32(BLERD(THRSHD5), 0 // 0x0332
                   | BLERD_THRSHD5_AGC0_18(50)
                   | BLERD_THRSHD5_AGC18_0(3)
                   );
  cpu_mem_write_32(BLERD(RCCAL), 0 // 0x5210
                   | BLERD_RCCAL_CODE_TX(16)
                   | BLERD_RCCAL_CODE_RX(16)
                   | BLERD_RCCAL_SOFTRST_POWER_DIFF(6DB)
                   | BLERD_RCCAL_SOFTRST_EN_TOSTR
                   | BLERD_RCCAL_AGC_GAIN_INC_TIMES_THRES(2ND)
                   );
  cpu_mem_write_32(BLERD(MODEM), 0
                   | BLERD_MODEM_NARROW_SPD(X1)
                   | BLERD_MODEM_RST_CNT2_SEL(32BITS)
                   | BLERD_MODEM_LOAD_PREV_GAIN_EN
                   | BLERD_MODEM_READ_DC_OFFSET_SEL(DIGITAL)
                   | BLERD_MODEM_ADCDFT_SEL(BEFORE)
                   | BLERD_MODEM_ADC_FULL_SWING_DETECT_EN
                   | BLERD_MODEM_DC_PARAM(430K)
                   | BLERD_MODEM_RESET2_EN
                   // 0x16EC
                   | BLERD_MODEM_DC_SCALING_EN
                   | BLERD_MODEM_WIDE_SPD(X14)
                   // 0x12E4
                   //| BLERD_MODEM_WIDE_SPD(X10)
                   );
  cpu_mem_write_32(BLERD(AGC), 0 // 0x12FA
                   | BLERD_AGC_START_WAIT_US(0)
                   | BLERD_AGC_SAT_CHK_TIM(3_12MHZ_CLOCK)
                   | BLERD_AGC_GAIN_STABLE_TIM(18)
                   | BLERD_AGC_PWR_MEAS_TIM(1US)
                   | BLERD_AGC_GAIN_SAT_THRES(3)
                   | BLERD_AGC_GAIN_MAPPING_MODE(REV_A)
                   | BLERD_AGC_CHECK_SAT_EN(QUICK)
                   );

#if 1
  // 89dBm RX sensitivity/0dBm output power
  cpu_mem_write_32(BLERD(TX_BUMP1), 0 // 0xD031
                   | BLERD_TX_BUMP1_SY_DIVN_TXPOWERSAVE(D1_D2_BUF)
                   | BLERD_TX_BUMP1_TX_VTXREF(4) // 300mV
                   | BLERD_TX_BUMP1_TX_LPF_REF(400MV)
                   | BLERD_TX_BUMP1_TX_LPF_BIAS(10U)
                   | BLERD_TX_BUMP1_SY_RST_DLY_TX(300PS)
                   | BLERD_TX_BUMP1_TX_DRIVER(1)
                   );
  cpu_mem_write_32(BLERD(TX_BUMP2), 0 // 0xE33A
                   | BLERD_TX_BUMP2_SY_CP_TXPOWERSAVE(BUFFER)
                   | BLERD_TX_BUMP2_DAC_RES(8) // 0%
                   | BLERD_TX_BUMP2_SY_ICP_OFFSET_TX(25PS)
                   | BLERD_TX_BUMP2_DRV_VCASCH(0V)
                   | BLERD_TX_BUMP2_DRV_AB_VBIAS(718)
                   );
  cpu_mem_write_32(BLERD(RX_BUMP1), 0 // 0xC800
                   | BLERD_RX_BUMP1_LNA_CASCODE
                   | BLERD_RX_BUMP1_LNA_PULL_DOWN
                   | BLERD_RX_BUMP1_MIXER(0_2V)
                   );

  cpu_mem_write_32(BLELL(ADV_CH_TX_POWER), 0 // 0x0007
                   | BLELL_ADV_CH_TX_POWER_POWER(0_DBM)
                   );
#else
  // 92dBm RX sensitivity/3dBm TX power
  cpu_mem_write_32(BLERD(TX_BUMP1), 0 // 0xD033
                   | BLERD_TX_BUMP1_SY_DIVN_TXPOWERSAVE(D1_D2_BUF)
                   | BLERD_TX_BUMP1_TX_VTXREF(4) // 300mV
                   | BLERD_TX_BUMP1_TX_LPF_REF(400MV)
                   | BLERD_TX_BUMP1_TX_LPF_BIAS(10U)
                   | BLERD_TX_BUMP1_SY_RST_DLY_TX(300PS)
                   | BLERD_TX_BUMP1_TX_DRIVER(3)
                   );
  cpu_mem_write_32(BLERD(TX_BUMP2), 0 // 0xE33F
                   | BLERD_TX_BUMP2_SY_CP_TXPOWERSAVE(BUFFER)
                   | BLERD_TX_BUMP2_DAC_RES(8) // 0%
                   | BLERD_TX_BUMP2_SY_ICP_OFFSET_TX(25PS)
                   | BLERD_TX_BUMP2_DRV_VCASCH(0V)
                   | BLERD_TX_BUMP2_DRV_AB_VBIAS(769)
                   );
  cpu_mem_write_32(BLERD(RX_BUMP1), 0 // 0xE800
                   | BLERD_RX_BUMP1_LNA_CASCODE
                   | BLERD_RX_BUMP1_LNA_PULL_DOWN
                   | BLERD_RX_BUMP1_LNA_BOOST
                   | BLERD_RX_BUMP1_MIXER_SET(0_2V)
                   );
#endif

  cpu_mem_write_32(BLERDn(AGC_GAIN_COMP, 0), 0 // 0x5699
                   | BLERD_AGC_GAIN_COMP_GAIN(0, 25) // Gain5
                   | BLERD_AGC_GAIN_COMP_GAIN(1, 20) // Gain4
                   | BLERD_AGC_GAIN_COMP_GAIN(2, 21) // Gain3
                   );
  cpu_mem_write_32(BLERDn(AGC_GAIN_COMP, 1), 0 // 0x0FF8
                   | BLERD_AGC_GAIN_COMP_GAIN(0, 24) // Gain2
                   | BLERD_AGC_GAIN_COMP_GAIN(1, 31) // Gain1
                   | BLERD_AGC_GAIN_COMP_GAIN(2, 3) // Gain0
                   );
  cpu_mem_write_32(BLERD(PA_RSSI), 0 // 0x0000
                   );

  cpu_mem_write_16(BLELL(RECEIVE_TRIG_CTRL), 0
                   | BLELL_RECEIVE_TRIG_CTRL_ACC_TRIGGER_THRESHOLD(32)
                   | BLELL_RECEIVE_TRIG_CTRL_ACC_TRIGGER_TIMEOUT(144)
                   );

  cpu_mem_write_16(BLELL(EVENT_ENABLE), 0
                   | BLELL_EVENT_ENABLE_ADV
                   | BLELL_EVENT_ENABLE_ENC
                   );
}

static void blell_adv_intr(struct device_s *dev)
{
  struct psoc4_ble_private_s *pv = dev->drv_pv;
  uint32_t tmp;
  static const char *names[] = {
    "New advertising event started",
    "Current advertising event closed",
    "ADV packet transmitted",
    "Scan response packet transmitted",
    "Scan request packet received",
    "Connect request packet is received",
    "Connection is created as slave",
    "Directed advertising event timed out",
    "Advertiser procedure is ON in hardware",
  };

  tmp = cpu_mem_read_16(BLELL(ADV_INTR));

  dprintk("%s ADV_INTR: %03x, adv next instant: %08x\n", __FUNCTION__,
         tmp,
         cpu_mem_read_16(BLELL(ADV_NEXT_INSTANT)));

  for (uint_fast8_t i = 0; i < ARRAY_SIZE(names); ++i)
    if (bit_get(tmp, i))
      dprintk(" - %s\n", names[i]);
  cpu_mem_write_16(BLELL(ADV_INTR), tmp);

  if ((tmp & BLELL_ADV_INTR_ADV_ON)
      && (tmp & BLELL_ADV_INTR_EVENT_CLOSED)
      && !pv->adv) {
    cpu_mem_write_16(BLELL(COMMAND), BLELL_COMMAND_COMMAND_ADV_STOP);
    cpu_mem_write_16(BLELL(COMMAND), BLELL_COMMAND_COMMAND_CLK_ADV_OFF);
  }

  if (!(tmp & BLELL_ADV_INTR_ADV_ON) && !pv->adv)
    psoc4_ble_stop(dev, bit(START_BLE));
}

static DEV_IRQ_SRC_PROCESS(psoc4_bless_irq)
{
  struct device_s *dev = ep->base.dev;
  uint32_t tmp;

  tmp = cpu_mem_read_32(BLESS(LL_DSM_INTR_STAT));
  if (tmp & BLESS_LL_DSM_INTR_STAT_DSM_ENTERED_INTR)
    dprintk("BLESS DSM entered\n");
  if (tmp & BLESS_LL_DSM_INTR_STAT_DSM_EXITED_INTR)
    dprintk("BLESS DSM exited\n");
  if (tmp & BLESS_LL_DSM_INTR_STAT_XTAL_ON_INTR)
    dprintk("BLESS DSM xtal on\n");
  cpu_mem_write_32(BLESS(LL_DSM_INTR_STAT), tmp);

  tmp = cpu_mem_read_16(BLELL(EVENT_INTR));
  if (tmp & BLELL_EVENT_INTR_DSM)
    dprintk("BLELL DSM exited\n");
  if (tmp & BLELL_EVENT_INTR_SM)
    dprintk("BLELL SM exited\n");
  cpu_mem_write_16(BLELL(EVENT_INTR), tmp);

  if (tmp & BLELL_EVENT_INTR_ADV)
    blell_adv_intr(dev);

  if (tmp & BLELL_EVENT_INTR_ENC)
    blell_enc_intr(dev);

}

static DEV_TIMER_GET_VALUE(psoc4_ble_timer_get_value)
{
  return -ENOTSUP;
}

#define psoc4_ble_timer_config (dev_timer_config_t*)dev_driver_notsup_fcn

#define psoc4_ble_timer_request (dev_timer_request_t*)dev_driver_notsup_fcn
#define psoc4_ble_timer_cancel (dev_timer_cancel_t*)dev_driver_notsup_fcn

static void lfclk_source_update(struct psoc4_ble_private_s *pv)
{
  uint16_t tmp;

  if (!pv->ll_powered)
    return;

  for (uint_fast8_t i = 0; i < 32; ++i)
    asm volatile("");

  dprintk("%s\n", __FUNCTION__);

  cpu_mem_write_16(BLELL(TX_RX_ON_DELAY), 0 // 0x8990
                   | BLELL_TX_RX_ON_DELAY_TXON_DELAY(0x89)
                   | BLELL_TX_RX_ON_DELAY_RXON_DELAY(0x90)
                   );
  cpu_mem_write_16(BLELL(TX_RX_SYNTH_DELAY), 0 // 0x1840
                   | BLELL_TX_RX_SYNTH_DELAY_RX_EN_DELAY(0x40)
                   | BLELL_TX_RX_SYNTH_DELAY_TX_EN_DELAY(0x18)
                   );
  cpu_mem_write_16(BLELL(TX_EN_EXT_DELAY), 3);
  cpu_mem_write_16(BLELL(DPLL_CONFIG), 0x0FFF);
  cpu_mem_write_16(BLELL(RECEIVE_TRIG_CTRL), 0 // 0x9020
                   | BLELL_RECEIVE_TRIG_CTRL_ACC_TRIGGER_THRESHOLD(0x20)
                   | BLELL_RECEIVE_TRIG_CTRL_ACC_TRIGGER_TIMEOUT(0x90)
                   );

  cpu_mem_write_16(BLELL(WAKEUP_CONFIG), 0
                   | BLELL_WAKEUP_CONFIG_OSC_STARTUP_SLOTS(1)
                   | BLELL_WAKEUP_CONFIG_OSC_STARTUP_16KHZ(6)
                   );

  cpu_mem_write_16(BLELL(CLOCK_CONFIG), 0
                   | BLELL_CLOCK_CONFIG_DEEP_SLEEP_MODE_EN
                   | BLELL_CLOCK_CONFIG_SLEEP_MODE_EN
                   | BLELL_CLOCK_CONFIG_SM_INTR_EN
                   | BLELL_CLOCK_CONFIG_SM_AUTO_WKUP_EN
                   | BLELL_CLOCK_CONFIG_LPO_SEL_EXTERNAL
                   | BLELL_CLOCK_CONFIG_LPO_CLK_FREQ_SEL(32_768KHZ)
                   | BLELL_CLOCK_CONFIG_SYSCLK_GATE_EN
                   | BLELL_CLOCK_CONFIG_CORECLK_GATE_EN
                   | BLELL_CLOCK_CONFIG_INIT_CLK_GATE_EN
                   | BLELL_CLOCK_CONFIG_SCAN_CLK_GATE_EN
                   | BLELL_CLOCK_CONFIG_ADV_CLK_GATE_EN
                   );

  cpu_mem_write_16(BLELL(OFFSET_TO_FIRST_INSTANT), 1);
}

static void llclk_source_update(struct psoc4_ble_private_s *pv, bool_t powered)
{
  pv->ll_powered = powered;

  if (!powered)
    return;

  dprintk("%s\n", __FUNCTION__);

  cpu_mem_write_32(BLESS(LL_CLK_EN), 0
                   | BLESS_LL_CLK_EN_CLK_EN
                   //| BLESS_LL_CLK_EN_CY_CORREL_EN
                   );

  for (uint_fast8_t i = 0; i < 32; ++i)
    asm volatile("");

  cpu_mem_write_32(BLELL(DPLL_CONFIG), 0x92e0);

  cpu_mem_write_16(BLELL(TIM_CONTROL), 0
                   | BLELL_TIM_CONTROL_BB_CLK_FREQ_MINUS_1(pv->bb_clk_mhz - 1)
                   );

//+0x402e1a10:	0x00020002
//+0x402e1a14:	0x20642064
//+0x402e1a18:	0x00100010

}

static void adv_start(struct device_s *dev)
{
  //  struct psoc4_ble_private_s *pv = dev->drv_pv;
  uint32_t tmp;

  dprintk("%s\n", __FUNCTION__);

  if (!psoc4_ble_start(dev, bit(START_BLE)))
    return;

  tmp = cpu_mem_read_16(BLELL(ADV_INTR));

  if (tmp & BLELL_ADV_INTR_ADV_ON)
    return;

  cpu_mem_write_16(BLELL(COMMAND), BLELL_COMMAND_COMMAND_CLK_ADV_ON);

  // Put some easy to spot addresses
  cpu_mem_write_16(BLELL(DEVICE_RAND_ADDR_L), 0x0505);
  cpu_mem_write_16(BLELL(DEVICE_RAND_ADDR_M), 0x0505);
  cpu_mem_write_16(BLELL(DEVICE_RAND_ADDR_H), 0x0505);
  cpu_mem_write_16(BLELL(DEV_PUB_ADDR_L), 0x5050);
  cpu_mem_write_16(BLELL(DEV_PUB_ADDR_M), 0x5050);
  cpu_mem_write_16(BLELL(DEV_PUB_ADDR_H), 0x5050);

  cpu_mem_write_16(BLELL(ADV_PARAMS), 0
                   | BLELL_ADV_PARAMS_TX_ADDR(PUBLIC)
                   | BLELL_ADV_PARAMS_ADV_TYPE(CONN)
                   | BLELL_ADV_PARAMS_ADV_CHANNEL_37
                   | BLELL_ADV_PARAMS_ADV_CHANNEL_38
                   | BLELL_ADV_PARAMS_ADV_CHANNEL_39
                   );

  cpu_mem_write_16(BLELL(ADV_CONFIG), 0
                   | BLELL_ADV_CONFIG_ADV_PKT_INTERVAL(4)
                   | BLELL_ADV_CONFIG_ADV_STRT_EN
                   | BLELL_ADV_CONFIG_ADV_CLS_EN
                   | BLELL_ADV_CONFIG_ADV_TX_EN
                   | BLELL_ADV_CONFIG_SCN_RSP_TX_EN
                   | BLELL_ADV_CONFIG_ADV_SCN_REQ_RX_EN
                   | BLELL_ADV_CONFIG_ADV_TIMEOUT_EN
                   );

  cpu_mem_write_16(BLELL(ADV_INTERVAL_TIMEOUT), 32);

  // Flush fifo
  for (uint_fast8_t i = 0; i < 16; ++i)
    cpu_mem_read_16(BLELL(ADV_TX_DATA_FIFO));

  char ad[] = {0x07, // 7 total AD bytes
               0x06, 0x09, // Complete name, 5 bytes AD payload
               'h', 'e', 'l', 'l', 'o'
  };
  for (uint_fast8_t i = 0; i < sizeof(ad); i += 2)
    cpu_mem_write_16(BLELL(ADV_TX_DATA_FIFO), endian_le16_na_load(ad + i));

  cpu_mem_write_16(BLELL(COMMAND), BLELL_COMMAND_COMMAND_ADV_START);
}

static DEV_USE(psoc4_ble_use)
{
  switch (op) {
#ifdef CONFIG_DEVICE_SLEEP
  case DEV_USE_SLEEP: {
    struct device_s *dev = param;
    struct psoc4_ble_private_s *pv = dev->drv_pv;

    dprintk("%s DEV_USE_SLEEP %x\n", __FUNCTION__, dev->start_count);

    if (!dev->start_count)
      dev_clock_sink_gate(&pv->sink[PSOC4_BLE_CLK_SINK_LFCLK], 0);
  
    if (!(dev->start_count & bit(START_BLE)))
      dev_clock_sink_gate(&pv->sink[PSOC4_BLE_CLK_SINK_LL], 0);

    return 0;
  }
#endif

  case DEV_USE_CLOCK_SINK_FREQ_CHANGED: {
    struct dev_clock_notify_s *notify = param;
    struct dev_clock_sink_ep_s *sink = notify->sink;
    struct device_s *dev = sink->dev;
    struct psoc4_ble_private_s *pv = dev->drv_pv;
    uint_fast8_t id = sink - pv->sink;

    switch (id) {
    case PSOC4_BLE_CLK_SINK_LFCLK: {
      dprintk("%s LFCLK freq changed %d/%d\n", __FUNCTION__,
              (uint32_t)notify->freq.num, (uint32_t)notify->freq.denom);
      pv->sleep_freq = notify->freq;
      lfclk_source_update(pv);
      break;
    }

    case PSOC4_BLE_CLK_SINK_LL:
      dprintk("%s LLCLK changed freq ???\n", __FUNCTION__);
      break;
    }

    return 0;
  }

  case DEV_USE_CLOCK_SINK_GATE_DONE: {
    struct dev_clock_sink_ep_s *sink = param;
    struct device_s *dev = sink->dev;
    struct psoc4_ble_private_s *pv = dev->drv_pv;
    uint_fast8_t id = sink - pv->sink;

    switch (id) {
    case PSOC4_BLE_CLK_SINK_LFCLK:
      dprintk("%s LFCLK gated\n", __FUNCTION__);
      break;

    case PSOC4_BLE_CLK_SINK_LL: {
      bool_t was_powered = pv->ll_powered;
      bool_t powered = (sink->src->flags & DEV_CLOCK_EP_ANY) == DEV_CLOCK_EP_ANY;

      llclk_source_update(pv, powered);
      lfclk_source_update(pv);

      if (powered && !was_powered) {
        psoc4_blerd_init(pv);
      }

      break;
    }

    default:
      break;
    }

    if (pv->adv)
      adv_start(dev);

    if (bit_get(dev->start_count, START_AES))
      psoc4_ble_aes_exec_first(dev);

    return 0;
  }

  case DEV_USE_START: {
    struct device_accessor_s *accessor = param;
    struct device_s *dev = accessor->dev;
    struct psoc4_ble_private_s *pv = dev->drv_pv;

    pv->adv = 1;
    adv_start(dev);
    return 0;
  }

  case DEV_USE_STOP: {
    struct device_accessor_s *accessor = param;
    struct device_s *dev = accessor->dev;
    struct psoc4_ble_private_s *pv = dev->drv_pv;

    pv->adv = 0;
    return 0;
  }

  default:
    return dev_use_generic(param, op);
  }
}

static DEV_INIT(psoc4_ble_init)
{
  struct psoc4_ble_private_s *pv = dev->drv_pv;
  error_t err;
  struct dev_freq_s ll_freq;

  pv = mem_alloc(sizeof (*pv), (mem_scope_sys));
  if (!pv)
    return -ENOMEM;

  memset(pv, 0, sizeof (*pv));
  dev->drv_pv = pv;

  err = dev_drv_clock_init(dev, &pv->sink[PSOC4_BLE_CLK_SINK_LFCLK],
                           PSOC4_BLE_CLK_SINK_LFCLK,
                           DEV_CLOCK_EP_FREQ_NOTIFY, &pv->sleep_freq);
  if (err) {
    dprintk("%s: LFCLK error %d\n", __FUNCTION__, err);
    return err;
  }

  err = dev_drv_clock_init(dev, &pv->sink[PSOC4_BLE_CLK_SINK_LL],
                           PSOC4_BLE_CLK_SINK_LL,
                           DEV_CLOCK_EP_FREQ_NOTIFY, &ll_freq);
  if (err) {
    dprintk("%s: LLCLK error %d\n", __FUNCTION__, err);
    return err;
  }

  uint32_t mhz = ll_freq.num;
  if (ll_freq.denom != 1)
    mhz /= ll_freq.denom;
  mhz /= 1000000;

  dprintk("%s LLCLK freq: %d MHz\n", __FUNCTION__, mhz);
  struct dev_freq_ratio_s scale = {.num = 1, .denom = 1,};
  while (!(mhz & 1) && scale.denom < 4) {
    mhz >>= 1;
    scale.denom <<= 1;
  }

  printk("%s LLCLK / %d, Freq = %d MHz\n", __FUNCTION__, scale.denom, mhz);

  dev_clock_sink_scaler_set(&pv->sink[PSOC4_BLE_CLK_SINK_LL], &scale);
  pv->bb_clk_mhz = mhz;

  device_irq_source_init(dev, &pv->irq_ep, 1, psoc4_bless_irq);
  if (device_irq_source_link(dev, &pv->irq_ep, 1, -1))
    goto err_mem;

  dev_request_queue_init(&pv->aes_q);
  cpu_mem_write_32(BLESS(LL_DSM_CTRL), 0);

  return 0;

 err_mem:
  dev->drv_pv = NULL;
  mem_free(pv);
  return -1;
}

static DEV_CLEANUP(psoc4_ble_cleanup)
{
  struct psoc4_ble_private_s *pv = dev->drv_pv;

  device_irq_source_unlink(dev, &pv->irq_ep, 1);

  mem_free(pv);

  return 0;
}

DRIVER_DECLARE(psoc4_ble_drv, 0,
               "PSoC4 BLE", psoc4_ble,
               DRIVER_CRYPTO_METHODS(psoc4_ble_aes),
               DRIVER_TIMER_METHODS(psoc4_ble_timer));

DRIVER_REGISTER(psoc4_ble_drv);

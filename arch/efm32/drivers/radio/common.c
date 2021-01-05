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

#include "common.h"

#ifdef CONFIG_DRIVER_EFR32_DEBUG

#define EFR32_RADIO_DEBUG_SIZE 0x1000
#define EFR32_RADIO_DEBUG_ADDR (CONFIG_LOAD_ROM_RW_ADDR + 0x40000 - EFR32_RADIO_DEBUG_SIZE)

void efr32_radio_debug_init(struct radio_efr32_ctx_s *pv)
{
  uint32_t *p = (void *)EFR32_RADIO_DEBUG_ADDR;

  memset(p, 0, EFR32_RADIO_DEBUG_SIZE);

  p[0] = EFR32_RADIO_DEBUG_ADDR;
  uint32_t *base = (void *)EFR32_RADIO_DEBUG_ADDR;
  pv->pdbg = base + 1;

  efr32_radio_debug_port(pv, 0xF);
}

void efr32_radio_print_debug(char *p, struct radio_efr32_ctx_s *pv)
{
  efr32_radio_printk(p);

  uint32_t *base = (void *)EFR32_RADIO_DEBUG_ADDR;
  uint32_t *end = (uint32_t *)base[0];

  if (pv->pdbg == NULL)
    pv->pdbg = base + 1;

  efr32_radio_printk("start 0x%x\n", pv->pdbg);
  efr32_radio_printk("end   0x%x\n", end);

  if (end < pv->pdbg)
    return;

  while(1)
  {
    if (pv->pdbg == (uint32_t *)(EFR32_RADIO_DEBUG_ADDR + EFR32_RADIO_DEBUG_SIZE))
      pv->pdbg = base + 1;

    efr32_radio_printk("0x%x\n", *pv->pdbg);

    pv->pdbg += 1;

    if (pv->pdbg >= end)
      break;
  }
}

void debug_toggle_pin(void)
{
#if CONFIG_EFM32_ARCHREV == EFM32_ARCHREV_EFR_XG12
  uint32_t o = 2;
  uint32_t i = 4;
#elif CONFIG_EFM32_ARCHREV == EFM32_ARCHREV_EFR_XG14
  uint32_t o = 5;
  uint32_t i = 3;
#else
  #error
#endif
  cpu_mem_write_32(0x4000a018 + o * 0x30, (1 << i));
}

void efr32_radio_debug_port(struct radio_efr32_ctx_s *pv, uint8_t val)
{
  uint8_t bitidx = 0;
  uint8_t bitcnt = 4;

  assert(val < 16);

  uint32_t msk = (1 << bitcnt) - 1;

#if CONFIG_EFM32_ARCHREV == EFM32_ARCHREV_EFR_XG12
  /* Set PC4 to output */
  uintptr_t a = 0x4000a004 + 2 * 0x30;
  uint32_t x = cpu_mem_read_32(a);
  cpu_mem_write_32(a, x | (0x4 << 16));

  /* Set PB6 to PB9 in output */
  a = 0x4000a004 + 1 * 0x30;
  x = cpu_mem_read_32(a);
  cpu_mem_write_32(a, x | (0x44 << 24));

  a = 0x4000a008 + 1 * 0x30;
  x = cpu_mem_read_32(a);
  cpu_mem_write_32(a, x | 0x44);

  a = 0x4000a000 + 0xC + 1 * 0x30;

#elif CONFIG_EFM32_ARCHREV == EFM32_ARCHREV_EFR_XG14
  /* Set PF3 to PF7 in output */
  uintptr_t a = 0x4000a004 + 5 * 0x30;
  uint32_t x = cpu_mem_read_32(a);
  cpu_mem_write_32(a, x | (0x44444 << 12));

  a = 0x4000a000 + 0xC + 5 * 0x30;
#else
  #error
#endif

  x = (cpu_mem_read_32(a) & ~(msk << bitidx)) | ((val & msk) << bitidx);

  cpu_mem_write_32(a, x);
}
#endif

void efr32_radio_seq_init(struct radio_efr32_ctx_s *pv, const uint8_t *seq, size_t count)
{
  assert(count < 0xFF6C);

  /* Check RAC state */
  assert(EFR32_RAC_STATUS_STATE_GET(endian_le32(cpu_mem_read_32(EFR32_RAC_ADDR + EFR32_RAC_STATUS_ADDR))) == EFR32_RAC_STATUS_STATE_OFF);

  /* Sequencer Interrupt Vector Base Address */
  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_VECTADDR_ADDR, EFR32_RADIO_SEQ_RAM_ADDR);

  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_SEQCTRL_ADDR, 0x1);

  uintptr_t p = (uintptr_t)EFR32_RADIO_SEQ_RAM_ADDR;

  memcpy((uint8_t *)p, seq, count);

  /* Set register R6 = sp of sequencer */
  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_R_ADDR(6), EFR32_SEQ_STACK_POINTER_ADDR);

  /* Init end of Seq RAM with 0 */
  memset((uint8_t *)(p + 0x1F00), 0, 0x100);
}

void set_cw(void)
{
  uint32_t x = cpu_mem_read_32(EFR32_RAC_ADDR + EFR32_RAC_LPFCTRL_ADDR);
  EFR32_RAC_LPFCTRL_LPFBWTX_SET(x, 0);
  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_LPFCTRL_ADDR, x);

  x = cpu_mem_read_32(EFR32_MODEM_ADDR + EFR32_MODEM_CTRL0_ADDR);
  uint8_t mod = EFR32_MODEM_CTRL0_MODFORMAT_GET(x);

  if (mod == EFR32_MODEM_CTRL0_MODFORMAT_DBPSK)
    {
      EFR32_MODEM_CTRL0_MODFORMAT_SET(x, BPSK);
      cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_CTRL0_ADDR, x);
    }
  else
    cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_MODINDEX_ADDR, 0);

  x = cpu_mem_read_32(EFR32_MODEM_ADDR + EFR32_MODEM_PRE_ADDR);
  EFR32_MODEM_PRE_TXBASES_SET(x, 0xFFFF);
  EFR32_MODEM_PRE_BASE_SETVAL(x, 0xF);
  EFR32_MODEM_PRE_BASEBITS_SET(x, 0);
  cpu_mem_write_32(EFR32_MODEM_ADDR + EFR32_MODEM_PRE_ADDR, x);

  cpu_mem_write_32(EFR32_FRC_ADDR + EFR32_FRC_DFLCTRL_ADDR, EFR32_FRC_DFLCTRL_DFLMODE_INFINITE);

  x = cpu_mem_read_32(EFR32_FRC_ADDR + EFR32_FRC_CTRL_ADDR);
  x |= EFR32_FRC_CTRL_RANDOMTX;
  cpu_mem_write_32(EFR32_FRC_ADDR + EFR32_FRC_CTRL_ADDR, x);

  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_CMD_ADDR, EFR32_RAC_CMD_TXEN);
}

void stoptx(void)
{
  /* Stop Tx */
  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_CMD_ADDR, EFR32_RAC_CMD_TXDIS);
}

void set_pn9(void)
{
  uint32_t x = cpu_mem_read_32(EFR32_FRC_ADDR + EFR32_FRC_FCD_ADDR(0));
  x &= ~EFR32_FRC_FCD_SKIPWHITE;
  cpu_mem_write_32(EFR32_FRC_ADDR + EFR32_FRC_FCD_ADDR(0), x);

  x = cpu_mem_read_32(EFR32_FRC_ADDR + EFR32_FRC_FCD_ADDR(1));
  x &= ~EFR32_FRC_FCD_SKIPWHITE;
  cpu_mem_write_32(EFR32_FRC_ADDR + EFR32_FRC_FCD_ADDR(1), x);

  cpu_mem_write_32(EFR32_FRC_ADDR + EFR32_FRC_FECCTRL_ADDR, EFR32_FRC_FECCTRL_BLOCKWHITEMODE_WHITE);

  x = EFR32_FRC_WHITECTRL_SHROUTPUTSEL(0);
  EFR32_FRC_WHITECTRL_XORFEEDBACK_SET(x, XOR);
  EFR32_FRC_WHITECTRL_FEEDBACKSEL_SET(x, BIT4);
  cpu_mem_write_32(EFR32_FRC_ADDR + EFR32_FRC_WHITECTRL_ADDR, x);

  cpu_mem_write_32(EFR32_FRC_ADDR + EFR32_FRC_WHITEPOLY_ADDR, 0x100);
  cpu_mem_write_32(EFR32_FRC_ADDR + EFR32_FRC_WHITEINIT_ADDR, 0x138);
  cpu_mem_write_32(EFR32_FRC_ADDR + EFR32_FRC_DFLCTRL_ADDR, EFR32_FRC_DFLCTRL_DFLMODE(INFINITE));

  x = cpu_mem_read_32(EFR32_FRC_ADDR + EFR32_FRC_CTRL_ADDR);
  x |= EFR32_FRC_CTRL_RANDOMTX;
  cpu_mem_write_32(EFR32_FRC_ADDR + EFR32_FRC_CTRL_ADDR, x);

  cpu_mem_write_32(EFR32_RAC_ADDR + EFR32_RAC_CMD_ADDR, EFR32_RAC_CMD_TXEN);
}

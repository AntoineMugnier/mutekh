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

void efr32_radio_debug_port(struct radio_efr32_ctx_s *pv, uint8_t val)
{
  uint32_t x, a;

  uint8_t bitidx = 0;
  uint8_t bitcnt = 4;

  assert(val < 16);

  uint32_t msk = (1 << bitcnt) - 1;

  a = 0x4000a000 + 0xC + 1 * 0x30;

  x = (cpu_mem_read_32(a) & ~(msk << bitidx)) | ((val & msk) << bitidx);

  cpu_mem_write_32(a, x);
}

void efr32_radio_debug_init(struct radio_efr32_ctx_s *pv)
{
  uint32_t *p = (void *)EFR32_RADIO_DEBUG_ADDR;

  memset(p, 0, EFR32_RADIO_DEBUG_SIZE);

  p[0] = EFR32_RADIO_DEBUG_ADDR;
  uint32_t *base = (void *)EFR32_RADIO_DEBUG_ADDR;
  pv->pdbg = base + 1;

  /* Set PB6 to PB9 in output */
  uintptr_t a = 0x4000a004 + 1 * 0x30;
  uint32_t x = cpu_mem_read_32(a);
  cpu_mem_write_32(a, x | (0x44 << 24));

  a = 0x4000a008 + 1 * 0x30;
  x = cpu_mem_read_32(a);
  cpu_mem_write_32(a, x | 0x44);

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



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


#ifndef COMMON_H_
# define COMMON_H_

#include <hexo/iospace.h>

#include <mutek/printk.h>
#include <mutek/mem_alloc.h>

#include <device/device.h>
#include <device/resources.h>
#include <device/irq.h>
#include <device/class/iomux.h>
#include <device/class/cmu.h>
#include <device/class/timer.h>
#include <device/class/dma.h>
#include <device/class/rfpacket.h>

#include <arch/efm32/dma_source.h>
#include <arch/efm32/irq.h>
#include <arch/efm32/pin.h>
#include <arch/efm32/gpio.h>
#include <arch/efm32/devaddr.h>
#include <arch/efm32/cmu.h>
#include <arch/efm32/emu.h>
#include <arch/efm32/prs.h>
#include <arch/efm32/rtc.h>

#include <arch/efm32/efr/frc.h>
#include <arch/efm32/efr/rac.h>
#include <arch/efm32/efr/bufc.h>
#include <arch/efm32/efr/synth.h>
#include <arch/efm32/efr/agc.h>
#include <arch/efm32/efr/modem.h>
#include <arch/efm32/efr/crc.h>

#include "protimer.h"

#ifdef CONFIG_DRIVER_EFR32_DEBUG
# define efr32_radio_printk(...) do { printk(__VA_ARGS__); } while(0)
#else
# define efr32_radio_printk(...) do { } while(0)

#define EFR32_RADIO_DEBUG_SIZE 0x1000
#define EFR32_RADIO_DEBUG_ADDR (CONFIG_LOAD_ROM_RW_ADDR + 0x40000 - EFR32_RADIO_DEBUG_SIZE)

#endif

#define EFR32_TX_IRQ_FRC_MSK      (EFR32_FRC_IF_TXDONE     |      \
                                   EFR32_FRC_IF_TXUF       |      \
                                   EFR32_FRC_IF_TXABORTED  |      \
                                   EFR32_FRC_IF_TXAFTERFRAMEDONE)

#define EFR32_SEQ_DEADLINE_ADDR 0x21001F00
#define EFR32_RADIO_SEQ_RAM_ADDR 0x21000000
#define EFR32_SEQ_STACK_POINTER_ADDR 0x21001F80

#ifdef CONFIG_DRIVER_EFM32_RFPACKET_RTCC
  #define EFR32_RADIO_IRQ_COUNT 10
  #define EFR32_RADIO_CLK_EP_COUNT 10
#else
  #define EFR32_RADIO_IRQ_COUNT 9
  #define EFR32_RADIO_CLK_EP_COUNT 7
#endif

#define EFR32_RADIO_HFXO_CLK 38400000L

extern const unsigned char seqcode[];
extern const size_t seqcode_size;

struct radio_efr32_rf_cfg_s {
  uint32_t frequency;
  uint32_t drate;
  uint32_t config_size;
  uint32_t config_data[];
};

struct radio_efr32_pk_cfg_s {
  uint32_t config_size;
  uint32_t config_data[];
};

struct radio_efr32_ctx_s
{
  struct device_s               *dev;
  struct dev_irq_src_s          irq_ep[EFR32_RADIO_IRQ_COUNT];
  // Clock Endpoint
  struct dev_clock_sink_ep_s    clk_ep[EFR32_RADIO_CLK_EP_COUNT];
  struct dev_freq_s             freq;
  uint8_t                       rx_length_buffer[64];
  uint32_t*                     pdbg;
  // Used for memory copy
  struct kroutine_s             kr;
  struct efr32_protimer_s       pti;
};

STRUCT_COMPOSE(radio_efr32_ctx_s, kr);

void efr32_radio_print_debug(char *p, struct radio_efr32_ctx_s *pv);
void efr32_radio_dump_registers(struct radio_efr32_ctx_s *pv);
void efr32_radio_dump_range(struct radio_efr32_ctx_s *pv, char * str, uintptr_t start, size_t size);
void efr32_radio_debug_port(struct radio_efr32_ctx_s *pv, uint8_t val);
void efr32_radio_debug_init(struct radio_efr32_ctx_s *pv);
void debug_toggle_pin(void);
void efr32_radio_seq_init(struct radio_efr32_ctx_s *pv, const uint8_t *seq, size_t count);
void set_cw(void);
void set_pn9(void);
void stoptx(void);


#endif /* !COMMON_H_ */

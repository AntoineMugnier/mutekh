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

#include "protimer.h"

void efr32_protimer_disable_compare(struct efr32_protimer_s *pv, uint8_t channel)
{
  cpu_mem_write_32(EFR32_PROTIMER_ADDR + EFR32_PROTIMER_CC_CTRL_ADDR(channel), 0);
}

dev_timer_value_t efr32_protimer_get_value(struct efr32_protimer_s *pv)
{
  uint32_t basepre = endian_le32(cpu_mem_read_32(EFR32_PROTIMER_ADDR + EFR32_PROTIMER_BASEPRE_ADDR));
  uint64_t wrap = endian_le32(cpu_mem_read_32(EFR32_PROTIMER_ADDR + EFR32_PROTIMER_LWRAPCNT_ADDR));
#if EFR32_PROTIMER_HW_WIDTH < 64
  uint16_t pre  = basepre & EFR32_PRECNT_MASK;
  uint32_t base = ((basepre >> 16) & EFR32_BASECNT_MASK) << EFR32_PRECNT_WIDTH;
  basepre = base | pre;
#endif

  dev_timer_value_t value = (wrap << (EFR32_PRECNT_WIDTH + EFR32_BASECNT_WIDTH)) | basepre;

#if EFR32_PROTIMER_HW_WIDTH < 64
  if (value < EFR32_PROTIMER_HW_MASK / 2)      /* check if a wrap just occured */
    {
      uint32_t x = endian_le32(cpu_mem_read_32(EFR32_PROTIMER_ADDR + EFR32_PROTIMER_IF_ADDR));
      if (x & EFR32_PROTIMER_IF_WRAPCNTOF)
        value += 1ULL << EFR32_PROTIMER_HW_WIDTH;
    }
  value += (pv->swvalue << EFR32_PROTIMER_HW_WIDTH);
#endif
  return value;
}

bool_t efr32_protimer_request_start(struct efr32_protimer_s *pv,
                                    dev_timer_value_t value,
                                    dev_timer_value_t deadline,
                                    uint8_t channel)
{
#if EFR32_PROTIMER_HW_WIDTH < 64
  /* enable hw comparator if software part of the counter match */
  if (((deadline ^ value) & EFR32_PROTIMER_SW_MASK))
    return 0;
#endif

  /* Disable channel compare */
  efr32_protimer_disable_compare(pv, channel);

  /* Write value for comparaison */
  uint32_t v = deadline & EFR32_PRECNT_MASK;
  cpu_mem_write_32(EFR32_PROTIMER_ADDR + EFR32_PROTIMER_CC_PRE_ADDR(channel),
                   endian_le32(v));
  
  v = (deadline >> EFR32_PRECNT_WIDTH) & EFR32_BASECNT_MASK;
  cpu_mem_write_32(EFR32_PROTIMER_ADDR + EFR32_PROTIMER_CC_BASE_ADDR(channel),
                   endian_le32(v));

  v = deadline >> (EFR32_PRECNT_WIDTH + EFR32_BASECNT_WIDTH); 
  cpu_mem_write_32(EFR32_PROTIMER_ADDR + EFR32_PROTIMER_CC_WRAP_ADDR(channel),
                   endian_le32(v));

  v = EFR32_PROTIMER_CC_CTRL_ENABLE |
      EFR32_PROTIMER_CC_CTRL_CCMODE(COMPARE) |
      EFR32_PROTIMER_CC_CTRL_PREMATCHEN |
      EFR32_PROTIMER_CC_CTRL_BASEMATCHEN |
      EFR32_PROTIMER_CC_CTRL_WRAPMATCHEN ;

  cpu_mem_write_32(EFR32_PROTIMER_ADDR + EFR32_PROTIMER_CC_CTRL_ADDR(channel),
                   endian_le32(v));

  /* hw compare for == only, check for race condition */
  if (deadline <= efr32_protimer_get_value(pv))
    {
      uint32_t x = EFR32_PROTIMER_IF_CC(channel);
      cpu_mem_write_32(EFR32_PROTIMER_ADDR + EFR32_PROTIMER_IFS_ADDR, x);
      return 1;
    } 

  return 0;
}

void efr32_protimer_stop_counter(struct efr32_protimer_s *pv)
{
  cpu_mem_write_32(EFR32_PROTIMER_ADDR + EFR32_PROTIMER_CMD_ADDR, endian_le32(EFR32_PROTIMER_CMD_STOP));
#ifdef CONFIG_DEVICE_CLOCK_GATING
  if (endian_le32(cpu_mem_read_32(EFR32_PROTIMER_ADDR + EFR32_PROTIMER_IF_ADDR))
      & (EFR32_PROTIMER_IF_CC(EFR32_PROTIMER_CHANNEL) | EFR32_PROTIMER_IF_WRAPCNTOF))
    return;
  dev_clock_sink_gate(&pv->clk_ep, DEV_CLOCK_EP_POWER);
#endif
}

void efr32_protimer_start_counter(struct efr32_protimer_s *pv)
{
#ifdef CONFIG_DEVICE_CLOCK_GATING
  dev_clock_sink_gate(&pv->clk_ep, DEV_CLOCK_EP_POWER_CLOCK);
#endif

  uint32_t x = cpu_mem_read_32(EFR32_PROTIMER_ADDR + EFR32_PROTIMER_CTRL_ADDR);

  EFR32_PROTIMER_CTRL_PRECNTSRC_SET(x, CLOCK);
  EFR32_PROTIMER_CTRL_BASECNTSRC_SET(x, PRECNTOF);
  EFR32_PROTIMER_CTRL_WRAPCNTSRC_SET(x, BASECNTOF);

  cpu_mem_write_32(EFR32_PROTIMER_ADDR + EFR32_PROTIMER_CTRL_ADDR, x);

  cpu_mem_write_32(EFR32_PROTIMER_ADDR + EFR32_PROTIMER_PRECNT_ADDR, 0);
  cpu_mem_write_32(EFR32_PROTIMER_ADDR + EFR32_PROTIMER_BASECNT_ADDR, 0);
  cpu_mem_write_32(EFR32_PROTIMER_ADDR + EFR32_PROTIMER_WRAPCNT_ADDR, 0);

  cpu_mem_write_32(EFR32_PROTIMER_ADDR + EFR32_PROTIMER_PRECNTTOP_ADDR, EFR32_PRECNT_MASK << 8);
  cpu_mem_write_32(EFR32_PROTIMER_ADDR + EFR32_PROTIMER_BASECNTTOP_ADDR, EFR32_BASECNT_MASK);
  cpu_mem_write_32(EFR32_PROTIMER_ADDR + EFR32_PROTIMER_WRAPCNTTOP_ADDR, 0xFFFFFFFF);

  /* Start counter */
  cpu_mem_write_32(EFR32_PROTIMER_ADDR + EFR32_PROTIMER_CMD_ADDR, endian_le32(EFR32_PROTIMER_CMD_START));
}

void efr32_protimer_init(struct efr32_protimer_s *pv)
{
  /* Timer init */

  pv->cap = DEV_TIMER_CAP_HIGHRES | 
            DEV_TIMER_CAP_REQUEST |
            DEV_TIMER_CAP_TICKLESS |
            DEV_TIMER_CAP_STOPPABLE |
            DEV_TIMER_CAP_KEEPVALUE;

# ifdef CONFIG_DEVICE_CLOCK_VARFREQ
  if (pv->clk_ep.flags & DEV_CLOCK_EP_VARFREQ)
    pv->cap |= DEV_TIMER_CAP_VARFREQ | DEV_TIMER_CAP_CLKSKEW;
# endif

  /* Stop timer */
  cpu_mem_write_32(EFR32_PROTIMER_ADDR + EFR32_PROTIMER_CMD_ADDR, EFR32_PROTIMER_CMD_STOP);

  /* Enable interrupts */
  cpu_mem_write_32(EFR32_PROTIMER_ADDR + EFR32_PROTIMER_IFC_ADDR, EFR32_PROTIMER_IFC_MASK);

  uint32_t x = EFR32_PROTIMER_IF_CC(EFR32_PROTIMER_CHANNEL);
#if EFR32_PROTIMER_HW_WIDTH < 64
  x |= EFR32_PROTIMER_IF_WRAPCNTOF;
#endif
  cpu_mem_write_32(EFR32_PROTIMER_ADDR + EFR32_PROTIMER_IEN_ADDR, x);

#if EFR32_PROTIMER_HW_WIDTH < 64
  pv->swvalue = 0;
#endif

#ifdef CONFIG_DEVICE_CLOCK_GATING
  dev_clock_sink_gate(&pv->clk_ep, DEV_CLOCK_EP_POWER);
#endif
}

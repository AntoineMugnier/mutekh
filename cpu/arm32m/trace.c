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

    Copyright Nicolas Pouillon <nipo@ssji.net> (c) 2016
*/

#include <mutek/startup.h>

#include <hexo/iospace.h>

#include <cpu/arm32m/etm.h>
#include <cpu/arm32m/itm.h>
#include <cpu/arm32m/tpiu.h>
#include <cpu/arm32m/dwt.h>
#include <cpu/arm32m/coredebug.h>

#if defined(CONFIG_CPU_ARM32M_TRACE)
#define RATE_DIVISOR ((CONFIG_CPU_ARM32M_TRACE_CLKIN_RATE + CONFIG_CPU_ARM32M_TRACE_RATE / 2) \
                      / CONFIG_CPU_ARM32M_TRACE_RATE)

STATIC_ASSERT(bad_rate_divisor, RATE_DIVISOR <= (1<<12) && RATE_DIVISOR > 0);
#endif

/**
   ARM documentation is a mess.

   Most useful documents for register info:
   - About ITM, DWT, TPIU: DDI0337 / DDI0439 and DDI0403
   - About ETM-M4: DDI0440

   Most useful documents for frame format:
   - About TPIU Frame Format: IHI0029 (Chapter D4)
   - About ITM Frame Format: DDI0314 (Chapter 12.1.1)
*/


void arm32m_coresight_init(void)
{
  cpu_mem_write_32(COREDEBUG_DEMCR_ADDR, COREDEBUG_DEMCR_TRACEENA);

  /*
    First, stop TPIU, flush fifos
   */

#if defined(CONFIG_CPU_ARM32M_TRACE)
  // See DDI0314, 8.6.10
  cpu_mem_write_32(TPIU_FFCR_ADDR, 0
    | TPIU_FFCR_STOPFL
    | TPIU_FFCR_FONMAN
    );
  while (cpu_mem_read_32(TPIU_FFCR_ADDR) & TPIU_FFCR_FONMAN)
    ;
  while (!(cpu_mem_read_32(TPIU_FFSR_ADDR) & (TPIU_FFSR_FTSTOPPED | TPIU_FFSR_FTNONSTOP)))
    ;

  cpu_mem_write_32(TPIU_ACPR_ADDR, TPIU_ACPR_PRESCALER(RATE_DIVISOR - 1));

# if CONFIG_CPU_ARM32M_TRACE_PARALLEL == 0
  cpu_mem_write_32(TPIU_SPPR_ADDR, TPIU_SPPR_TXMODE(NRZ));
  cpu_mem_write_32(TPIU_CSPSR_ADDR, TPIU_CSPSR_CWIDTH(0));
# else
  cpu_mem_write_32(TPIU_SPPR_ADDR, TPIU_SPPR_TXMODE(PARALLEL));
  cpu_mem_write_32(TPIU_CSPSR_ADDR, TPIU_CSPSR_CWIDTH(CONFIG_CPU_ARM32M_TRACE_PARALLEL - 1));
# endif
  cpu_mem_write_32(TPIU_FFCR_ADDR, 0
    | TPIU_FFCR_TRIGIN
# if defined(CONFIG_CPU_ARM32M_TRACE_FORMATTING)
    | TPIU_FFCR_ENFCONT
# endif
    );

  while (!(cpu_mem_read_32(TPIU_FFSR_ADDR) & TPIU_FFSR_FTNONSTOP)
         && (cpu_mem_read_32(TPIU_FFSR_ADDR) & TPIU_FFSR_FTSTOPPED))
    ;
#endif

  /*
    TPIU is now running in the right format, rate, etc.
   */

  /*
    Configure ITM and DWT
   */

#if defined(CONFIG_CPU_ARM32M_ITM)
  cpu_mem_write_32(ITM_LAR_ADDR, ITM_LAR_ACCESS_ENABLE);
  cpu_mem_write_32(ITM_TPR_ADDR, ITM_TPR_PRIVMASK(0xf));
  cpu_mem_write_32(ITM_TCR_ADDR, 0);
#endif

#if defined(CONFIG_CPU_ARM32M_DWT)
  cpu_mem_write_32(DWT_CTRL_ADDR, 0
    //| DWT_CTRL_CYCTAP(EVERY_2_9)
# if defined(CONFIG_CPU_ARM32M_DWT_SYNC)
    | DWT_CTRL_SYNCTAP(EVERY_64M) // Mandatory for sync
# endif
    | DWT_CTRL_CYCCNTENA // Mandatory for sync
# if defined(CONFIG_CPU_ARM32M_ITM)
    //| DWT_CTRL_CPIEVTENA
    //| DWT_CTRL_LSUEVTENA
    //| DWT_CTRL_SLEEPEVTENA
    | DWT_CTRL_FOLDEVTENA
    //| DWT_CTRL_CYCEVTENA
    //| DWT_CTRL_EXCEVTENA
    //| DWT_CTRL_PCSAMPLENA
    //| DWT_CTRL_POSTINIT(0xf)
    //| DWT_CTRL_POSTPRESET(0xf)
    | DWT_CTRL_EXCTRCENA
# endif
    );
#endif

#if defined(CONFIG_CPU_ARM32M_ITM)
  cpu_mem_write_32(ITM_TCR_ADDR, 0
# if defined(CONFIG_CPU_ARM32M_DWT)
    | ITM_TCR_DWTENA // Mandatory for sync
# endif
    | ITM_TCR_SWOENA
    | ITM_TCR_ITMENA
# if defined(CONFIG_CPU_ARM32M_DWT_SYNC)
    | ITM_TCR_SYNCENA // Mandatory for sync
# endif
    | ITM_TCR_TSENA
# if defined(CONFIG_CPU_ARM32M_TRACE_FORMATTING)
    | ITM_TCR_TRACEBUSID(CONFIG_CPU_ARM32M_ITM_BUSID)
# endif
    //| ITM_TCR_GTSFREQ(ON_EMPTY)
    );
  cpu_mem_write_32(ITM_TER_ADDR, 0);
#endif

  cpu_mem_write_32(ETM_LAR_ADDR, ETM_LAR_ACCESS_ENABLE);

#if defined(CONFIG_CPU_ARM32M_ETM)
  cpu_mem_write_32(ETM_CR_ADDR, cpu_mem_read_32(ETM_CR_ADDR)
                   | ETM_CR_PROG);
  while (!(cpu_mem_read_32(ETM_SR_ADDR) & ETM_SR_PROGBIT))
    ;
  cpu_mem_write_32(ETM_CR_ADDR, cpu_mem_read_32(ETM_CR_ADDR)
                   | ETM_CR_EN
                   | ETM_CR_SP
                   | ETM_CR_TE
                   | ETM_CR_BO
                   );
# if defined(CONFIG_CPU_ARM32M_TRACE_FORMATTING)
  cpu_mem_write_32(ETM_TRACEIDR_ADDR, CONFIG_CPU_ARM32M_ETM_BUSID);
# else
  cpu_mem_write_32(ETM_TRACEIDR_ADDR, 0);
# endif
  cpu_mem_write_32(ETM_TECR1_ADDR, 0);
  cpu_mem_write_32(ETM_CR_ADDR, cpu_mem_read_32(ETM_CR_ADDR)
                   & ~ETM_CR_PROG);
  while ((cpu_mem_read_32(ETM_SR_ADDR) & ETM_SR_PROGBIT))
    ;
#endif

  uint32_t temp = 1 << 23;
  while (--temp)
    asm volatile("");
}

uint32_t arm32m_dwt_cycle_count(void);

uint32_t arm32m_dwt_cycle_count(void)
{
  return cpu_mem_read_32(DWT_CYCCNT_ADDR);
}

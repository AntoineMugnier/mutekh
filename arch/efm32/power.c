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

    Copyright Alexandre Becoulet <alexandre.becoulet@free.fr> (c) 2015

*/

#include <hexo/power.h>
#include <hexo/iospace.h>
#include <cpu/arm32m/v7m.h>
#include <mutek/startup.h>

#include <arch/efm32/devaddr.h>

#if EFM32_SERIES(CONFIG_EFM32_CFAMILY) == 0
# include <arch/efm32/rmu.h>
# include <arch/efm32/emu.h>
#endif

error_t power_reboot()
{
  cpu_mem_write_32(ARMV7M_AIRCR_ADDR, ARMV7M_AIRCR_VECTKEY(KEY)
                   | ARMV7M_AIRCR_SYSRESETREQ);
  while (1)
    asm volatile("dmb");
}

error_t power_shutdown()
{
  return ENOTSUP;
}

uint16_t efm32_reset_cause = 0;

enum power_reset_cause_e power_reset_cause(void)
{
#if EFM32_SERIES(CONFIG_EFM32_CFAMILY) == 0
  if (efm32_reset_cause & EFM32_RMU_RSTCAUSE_PORST)
    return POWER_RESET_CAUSE_POWERUP;
  if (efm32_reset_cause & EFM32_RMU_RSTCAUSE_EXTRST)
    return POWER_RESET_CAUSE_HARD;
  if (efm32_reset_cause & EFM32_RMU_RSTCAUSE_LOCKUPRST)
    return POWER_RESET_CAUSE_FAULT;
  if (efm32_reset_cause & EFM32_RMU_RSTCAUSE_SYSREQRST)
    return POWER_RESET_CAUSE_SOFT;
  if (efm32_reset_cause & (EFM32_RMU_RSTCAUSE_BODUNREGRST | EFM32_RMU_RSTCAUSE_BODREGRST))
    return POWER_RESET_CAUSE_BROWNOUT;
  if (efm32_reset_cause & EFM32_RMU_RSTCAUSE_WDOGRST)
    return POWER_RESET_CAUSE_WATCHDOG;
#endif
  return POWER_RESET_CAUSE_UNKNOWN;
}

void efm32_copy_reset_cause()
{
#if EFM32_SERIES(CONFIG_EFM32_CFAMILY) == 0
  efm32_reset_cause = cpu_mem_read_32(EFM32_RMU_ADDR + EFM32_RMU_RSTCAUSE_ADDR);

  /* clear causes */
  cpu_mem_write_32(EFM32_RMU_ADDR + EFM32_RMU_CMD_ADDR, EFM32_RMU_CMD_RCCLR);

  cpu_mem_write_32(EFM32_EMU_ADDR + EFM32_EMU_AUXCTRL_ADDR, EFM32_EMU_AUXCTRL_HRCCLR);
  cpu_mem_write_32(EFM32_EMU_ADDR + EFM32_EMU_AUXCTRL_ADDR, 0);
#endif
}

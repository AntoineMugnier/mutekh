#include <mutek/printk.h>

#if defined(CONFIG_DEVICE)
# include <device/resources.h>
# include <device/irq.h>
# include <device/class/iomux.h>
# include <device/class/cmu.h>
# include <device/class/dma.h>
# include <device/class/usbdev.h>
# include <device/class/i2c.h>
#endif

#include <mutek/shell.h>

#include <hexo/ordering.h>
#include <hexo/iospace.h>
#include <arch/efm32/clock.h>

#include "dac.h"

static TERMUI_CON_COMMAND_PROTOTYPE(dac_set)
{
  uint_fast16_t r = strto_uintl32(argv[0], NULL, 0);

  cpu_mem_write_32(EFM32_DAC0_ADDR + EFM32_DAC_CH0DATA, r);

  return 0;
}

static TERMUI_CON_GROUP_DECL(dac_subgroup) =
{

  TERMUI_CON_ENTRY(dac_set, "set",
    TERMUI_CON_ARGS(1, 1)
  )

  TERMUI_CON_LIST_END
};

MUTEK_SHELL_ROOT_GROUP(dac_subgroup, "dac")

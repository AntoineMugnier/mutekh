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

    Copyright Alexandre Becoulet <alexandre.becoulet@free.fr> (c) 2014

*/

#include <hexo/types.h>
#include <hexo/endian.h>
#include <hexo/iospace.h>

#include <mutek/mem_alloc.h>
#include <arch/efm32_cmu.h>
#include <arch/efm32_devaddr.h>
#include <arch/efm32_clock.h>

#include <device/resources.h>
#include <device/device.h>
#include <device/irq.h>
#include <device/class/clock.h>

/* FIXME uint32_t would be better when EFM32_CLOCK_count <= 32 */
typedef uint64_t efm32_clock_mask_t;
#define EFM32_CLK_MASK(x) (1ULL << (x))

#define EFM32_CLOCK_HFCORECLK_CHILDMASK \
  (((1ULL << (EFM32_CLOCK_HFCORECLK_last - EFM32_CLOCK_HFCORECLK_first + 1)) - 1) \
   << EFM32_CLOCK_HFCORECLK_first)

#define EFM32_CLOCK_HFPERCLK_CHILDMASK \
  (((1ULL << (EFM32_CLOCK_HFPERCLK_last - EFM32_CLOCK_HFPERCLK_first + 1)) - 1) \
   << EFM32_CLOCK_HFPERCLK_first)

#define EFM32_CLOCK_LFACLK_CHILDMASK \
  (((1ULL << (EFM32_CLOCK_LFACLK_last - EFM32_CLOCK_LFACLK_first + 1)) - 1) \
   << EFM32_CLOCK_LFACLK_first)

#define EFM32_CLOCK_LFBCLK_CHILDMASK \
  (((1ULL << (EFM32_CLOCK_LFBCLK_last - EFM32_CLOCK_LFBCLK_first + 1)) - 1) \
   << EFM32_CLOCK_LFBCLK_first)

#ifdef CONFIG_DRIVER_EFM32_RECMU_NAMES
static const char * const efm32_clock_names[EFM32_CLOCK_count] = {
    [EFM32_CLOCK_HFXO]       = "hfxo",
    [EFM32_CLOCK_HFRCO]      = "hfrco",
# ifdef EFM32_CLOCK_AUXHFRCO
    [EFM32_CLOCK_AUXHFRCO]   = "auxhfrco",
# endif
    [EFM32_CLOCK_HFCLK]      = "hfclk",     
    [EFM32_CLOCK_HFCLKDIV]   = "hfclkdiv",
    [EFM32_CLOCK_HFCORECLK]  = "hfcoreclk",
    [EFM32_CLOCK_HFPERCLK]   = "hfperclk",
    [EFM32_CLOCK_LE]         = "le",        
    [EFM32_CLOCK_LFACLK]     = "lfaclk",    
    [EFM32_CLOCK_LFBCLK]     = "lfbclk",    
    [EFM32_CLOCK_LFXO]       = "lfxo",      
    [EFM32_CLOCK_LFRCO]      = "lfrco",     
    [EFM32_CLOCK_ULFRCO]     = "ulfrco",    
    [EFM32_CLOCK_OUT0]       = "out0",      
    [EFM32_CLOCK_OUT1]       = "out1",      
    [EFM32_CLOCK_CPU]        = "cpu",
# ifdef EFM32_CLOCK_AES
    [EFM32_CLOCK_AES]        = "aes",
# endif
# ifdef EFM32_CLOCK_DMA
    [EFM32_CLOCK_DMA]        = "dma",
# endif
# ifdef EFM32_CLOCK_EBI
    [EFM32_CLOCK_EBI]        = "ebi",
# endif
# ifdef EFM32_CLOCK_USB
    [EFM32_CLOCK_USB]        = "usb",
# endif
# ifdef EFM32_CLOCK_USBC
    [EFM32_CLOCK_USBC]       = "usbc",
# endif
# ifdef EFM32_CLOCK_TIMER0
    [EFM32_CLOCK_TIMER0]     = "timer0",
# endif
# ifdef EFM32_CLOCK_TIMER1
    [EFM32_CLOCK_TIMER1]     = "timer1",
# endif
# ifdef EFM32_CLOCK_TIMER2
    [EFM32_CLOCK_TIMER2]     = "timer2",
# endif
# ifdef EFM32_CLOCK_TIMER3
    [EFM32_CLOCK_TIMER3]     = "timer3",
# endif
# ifdef EFM32_CLOCK_USART0
    [EFM32_CLOCK_USART0]     = "usart0",
# endif
# ifdef EFM32_CLOCK_USART1
    [EFM32_CLOCK_USART1]     = "usart1",
# endif
# ifdef EFM32_CLOCK_USART2
    [EFM32_CLOCK_USART2]     = "usart2",
# endif
# ifdef EFM32_CLOCK_I2C0
    [EFM32_CLOCK_I2C0]       = "i2c0",
# endif
# ifdef EFM32_CLOCK_I2C1
    [EFM32_CLOCK_I2C1]       = "i2c1",
# endif
# ifdef EFM32_CLOCK_GPIO
    [EFM32_CLOCK_GPIO]       = "gpio",
# endif
# ifdef EFM32_CLOCK_ADC0
    [EFM32_CLOCK_ADC0]       = "adc0",
# endif
# ifdef EFM32_CLOCK_VCMP
    [EFM32_CLOCK_VCMP]       = "vcmp",
# endif
# ifdef EFM32_CLOCK_PRS
    [EFM32_CLOCK_PRS]        = "prs",
# endif
# ifdef EFM32_CLOCK_ACMP0
    [EFM32_CLOCK_ACMP0]      = "acmp0",
# endif
# ifdef EFM32_CLOCK_ACMP1
    [EFM32_CLOCK_ACMP1]      = "acmp1",
# endif
# ifdef EFM32_CLOCK_LESENSE
    [EFM32_CLOCK_LESENSE]    = "lesense",
# endif
# ifdef EFM32_CLOCK_RTC
    [EFM32_CLOCK_RTC]        = "rtc",
# endif
# ifdef EFM32_CLOCK_LETIMER
    [EFM32_CLOCK_LETIMER]    = "letimer",
# endif
# ifdef EFM32_CLOCK_LCD
    [EFM32_CLOCK_LCD]        = "lcd",
# endif
# ifdef EFM32_CLOCK_LEUART0
    [EFM32_CLOCK_LEUART0]    = "leuart0",
# endif
# ifdef EFM32_CLOCK_LEUART1
    [EFM32_CLOCK_LEUART1]    = "leuart1",
# endif
# ifdef EFM32_CLOCK_UART0
    [EFM32_CLOCK_UART0]      = "uart0",
# endif
# ifdef EFM32_CLOCK_UART1
    [EFM32_CLOCK_UART1]      = "uart1",
# endif
};
#endif

static const uint8_t efm32_en_bits[EFM32_CLOCK_count] = {
    [EFM32_CLOCK_LE]       = EFM32_CMU_HFCORECLKEN0_LE_SHIFT | 0x80,
    [EFM32_CLOCK_HFPERCLK] = EFM32_CMU_HFPERCLKDIV_HFPERCLKEN_SHIFT | 0x80,
#ifdef EFM32_CLOCK_AES
    [EFM32_CLOCK_AES]      = EFM32_CMU_HFCORECLKEN0_AES_SHIFT | 0x80,
#endif
#ifdef EFM32_CLOCK_DMA
    [EFM32_CLOCK_DMA]      = EFM32_CMU_HFCORECLKEN0_DMA_SHIFT | 0x80,
#endif
#ifdef EFM32_CLOCK_EBI
    [EFM32_CLOCK_EBI]      = EFM32_CMU_HFCORECLKEN0_EBI_SHIFT | 0x80,
#endif
#ifdef EFM32_CLOCK_USB
    [EFM32_CLOCK_USB]      = EFM32_CMU_HFCORECLKEN0_USB_SHIFT | 0x80,
#endif
#ifdef EFM32_CLOCK_TIMER0
    [EFM32_CLOCK_TIMER0]   = EFM32_CMU_HFPERCLKEN0_TIMER0_SHIFT | 0x80,
#endif
#ifdef EFM32_CLOCK_TIMER1
    [EFM32_CLOCK_TIMER1]   = EFM32_CMU_HFPERCLKEN0_TIMER1_SHIFT | 0x80,
#endif
#ifdef EFM32_CLOCK_TIMER2
    [EFM32_CLOCK_TIMER2]   = EFM32_CMU_HFPERCLKEN0_TIMER2_SHIFT | 0x80,
#endif
#ifdef EFM32_CLOCK_TIMER3
    [EFM32_CLOCK_TIMER3]   = EFM32_CMU_HFPERCLKEN0_TIMER3_SHIFT | 0x80,
#endif
#ifdef EFM32_CLOCK_USART0
    [EFM32_CLOCK_USART0]   = EFM32_CMU_HFPERCLKEN0_USART0_SHIFT | 0x80,
#endif
#ifdef EFM32_CLOCK_USART1
    [EFM32_CLOCK_USART1]   = EFM32_CMU_HFPERCLKEN0_USART1_SHIFT | 0x80,
#endif
#ifdef EFM32_CLOCK_USART2
    [EFM32_CLOCK_USART2]   = EFM32_CMU_HFPERCLKEN0_USART2_SHIFT | 0x80,
#endif
#ifdef EFM32_CLOCK_I2C0
    [EFM32_CLOCK_I2C0]     = EFM32_CMU_HFPERCLKEN0_I2C0_SHIFT | 0x80,
#endif
#ifdef EFM32_CLOCK_I2C1
    [EFM32_CLOCK_I2C1]     = EFM32_CMU_HFPERCLKEN0_I2C1_SHIFT | 0x80,
#endif
#ifdef EFM32_CLOCK_GPIO
    [EFM32_CLOCK_GPIO]     = EFM32_CMU_HFPERCLKEN0_GPIO_SHIFT | 0x80,
#endif
#ifdef EFM32_CLOCK_ADC0
    [EFM32_CLOCK_ADC0]     = EFM32_CMU_HFPERCLKEN0_ADC0_SHIFT | 0x80,
#endif
#ifdef EFM32_CLOCK_VCMP
    [EFM32_CLOCK_VCMP]     = EFM32_CMU_HFPERCLKEN0_VCMP_SHIFT | 0x80,
#endif
#ifdef EFM32_CLOCK_PRS
    [EFM32_CLOCK_PRS]      = EFM32_CMU_HFPERCLKEN0_PRS_SHIFT | 0x80,
#endif
#ifdef EFM32_CLOCK_ACMP0
    [EFM32_CLOCK_ACMP0]    = EFM32_CMU_HFPERCLKEN0_ACMP0_SHIFT | 0x80,
#endif
#ifdef EFM32_CLOCK_ACMP1
    [EFM32_CLOCK_ACMP1]    = EFM32_CMU_HFPERCLKEN0_ACMP1_SHIFT | 0x80,
#endif
#ifdef EFM32_CLOCK_LESENSE
    [EFM32_CLOCK_LESENSE]  = EFM32_CMU_LFACLKEN0_LESENSE_SHIFT | 0x80,
#endif
#ifdef EFM32_CLOCK_RTC
    [EFM32_CLOCK_RTC]      = EFM32_CMU_LFACLKEN0_RTC_SHIFT | 0x80,
#endif
#ifdef EFM32_CLOCK_LETIMER
    [EFM32_CLOCK_LETIMER]  = EFM32_CMU_LFACLKEN0_LETIMER0_SHIFT | 0x80,
#endif
#ifdef EFM32_CLOCK_LCD
    [EFM32_CLOCK_LCD]      = EFM32_CMU_LFACLKEN0_LCD_SHIFT | 0x80,
#endif
#ifdef EFM32_CLOCK_LEUART0
    [EFM32_CLOCK_LEUART0]  = EFM32_CMU_LFBCLKEN0_LEUART0_SHIFT | 0x80,
#endif
#ifdef EFM32_CLOCK_LEUART1
    [EFM32_CLOCK_LEUART1]  = EFM32_CMU_LFBCLKEN0_LEUART1_SHIFT | 0x80,
#endif
#ifdef EFM32_CLOCK_UART0
    [EFM32_CLOCK_UART0]  = EFM32_CMU_HFPERCLKEN0_UART0_SHIFT | 0x80,
#endif
#ifdef EFM32_CLOCK_UART1
    [EFM32_CLOCK_UART1]  = EFM32_CMU_HFPERCLKEN0_UART1_SHIFT | 0x80,
#endif
};

struct efm32_recmu_private_s
{
  struct dev_clock_src_ep_s src[EFM32_CLOCK_EP_COUNT];

  struct dev_freq_s hfxo_freq;
  struct dev_freq_accuracy_s hfxo_acc;
  struct dev_freq_s lfxo_freq;
  struct dev_freq_accuracy_s lfxo_acc;

  enum efm32_clock_node_e hfclk_parent:8;
  enum efm32_clock_node_e hfclk_new_parent:8;

  enum efm32_clock_node_e lfaclk_parent:8;
  enum efm32_clock_node_e lfaclk_new_parent:8;

  enum efm32_clock_node_e lfbclk_parent:8;
  enum efm32_clock_node_e lfbclk_new_parent:8;

#ifdef EFM32_CLOCK_USBC
  enum efm32_clock_node_e usbcclk_parent:8;
  enum efm32_clock_node_e usbcclk_new_parent:8;
#endif

  bool_t busy;

  efm32_clock_mask_t chg_mask;   /* what clock signal have their config changed */

  efm32_clock_mask_t use_mask;   /* what is enabled by direct use */
  efm32_clock_mask_t dep_mask;   /* what is enabled, including dependencies */
  efm32_clock_mask_t wait_mask;
  efm32_clock_mask_t notify_mask; /* source ep with notification enabled */

  uint32_t lfclksel;            /* mux value of lfclksel (without gating) */

  /** registers used used for new config */
  uint32_t r_ctrl;
  uint32_t r_hfcoreclkdiv;
  uint32_t r_hfperclkdiv;
  uint32_t r_hfrcoctrl;
  uint32_t r_lfrcoctrl;
  uint32_t r_auxhfrcoctrl;
  uint32_t r_cmd;
  uint32_t r_lfclksel;
  uint32_t r_lfapresc0;
  uint32_t r_lfbpresc0;
};

static error_t
efm32_recmu_get_node_freq(struct efm32_recmu_private_s *pv,
                          struct dev_freq_s *freq,
                          struct dev_freq_accuracy_s *acc,
                          dev_clock_node_id_t node)
{
  uint32_t div;

  /* get divider and parent of lower nodes */
  switch (node)
    {
      /* LFACLK childs */
#ifdef EFM32_CLOCK_RTC
    case EFM32_CLOCK_RTC:
      node = EFM32_CLOCK_LFACLK;
      div = 1 << EFM32_CMU_LFAPRESC0_RTC_GET(endian_le32(
        cpu_mem_read_32(EFM32_CMU_ADDR + EFM32_CMU_LFAPRESC0_ADDR)));
      break;
#endif

#ifdef EFM32_CLOCK_LCD
    case EFM32_CLOCK_LCD:
      node = EFM32_CLOCK_LFACLK;
      div = 16 << EFM32_CMU_LFAPRESC0_LCD_GET(endian_le32(
        cpu_mem_read_32(EFM32_CMU_ADDR + EFM32_CMU_LFAPRESC0_ADDR)));
      break;
#endif

#ifdef EFM32_CLOCK_LETIMER
    case EFM32_CLOCK_LETIMER:
      node = EFM32_CLOCK_LFACLK;
      div = 1 << EFM32_CMU_LFAPRESC0_LETIMER0_GET(endian_le32(
        cpu_mem_read_32(EFM32_CMU_ADDR + EFM32_CMU_LFAPRESC0_ADDR)));
      break;
#endif

#ifdef EFM32_CLOCK_LESENSE
    case EFM32_CLOCK_LESENSE:
      node = EFM32_CLOCK_LFACLK;
      div = 1 << EFM32_CMU_LFAPRESC0_LESENSE_GET(endian_le32(
        cpu_mem_read_32(EFM32_CMU_ADDR + EFM32_CMU_LFAPRESC0_ADDR)));
      break;
#endif

      /* LFBCLK childs */
#ifdef EFM32_CLOCK_LEUART0
    case EFM32_CLOCK_LEUART0:
      node = EFM32_CLOCK_LFBCLK;
      div = 1 << EFM32_CMU_LFBPRESC0_LEUART0_GET(endian_le32(
        cpu_mem_read_32(EFM32_CMU_ADDR + EFM32_CMU_LFBPRESC0_ADDR)));
      break;
#endif

#ifdef EFM32_CLOCK_LEUART1
    case EFM32_CLOCK_LEUART1:
      node = EFM32_CLOCK_LFBCLK;
      div = 1 << EFM32_CMU_LFBPRESC0_LEUART1_GET(endian_le32(
        cpu_mem_read_32(EFM32_CMU_ADDR + EFM32_CMU_LFBPRESC0_ADDR)));
      break;
#endif

#ifdef EFM32_CLOCK_USBC
    case EFM32_CLOCK_USBC:
      node = pv->usbcclk_parent;
      div = 1;
      break;
#endif

    default:
      /* HFCORECLK & childs */
      if (EFM32_CLK_MASK(node) & (EFM32_CLOCK_HFCORECLK_CHILDMASK |
               EFM32_CLK_MASK(EFM32_CLOCK_LE) | EFM32_CLK_MASK(EFM32_CLOCK_HFCORECLK)))
        {
          node = EFM32_CLOCK_HFCLKDIV;
          div = 1 << EFM32_CMU_HFCORECLKDIV_HFCORECLKDIV_GET(endian_le32(
            cpu_mem_read_32(EFM32_CMU_ADDR + EFM32_CMU_HFCORECLKDIV_ADDR)));
        }

      /* HFPERCLK & childs */
      else if (EFM32_CLK_MASK(node) & (EFM32_CLOCK_HFPERCLK_CHILDMASK | EFM32_CLK_MASK(EFM32_CLOCK_HFPERCLK)))
        {
          node = EFM32_CLOCK_HFCLKDIV;
          div = 1 << EFM32_CMU_HFPERCLKDIV_HFPERCLKDIV_GET(endian_le32(
            cpu_mem_read_32(EFM32_CMU_ADDR + EFM32_CMU_HFPERCLKDIV_ADDR)));
        }

      /* not a lower node */
      else
        {
          div = 1;
        }
      break;
    }

  /* get divider and parent of intermediate nodes */
  switch (node)
    {
    case EFM32_CLOCK_HFCLKDIV:
# if defined(CONFIG_EFM32_LEOPARD_GECKO) \
  || defined(CONFIG_EFM32_WONDER_GECKO) \
  || defined(CONFIG_EFM32_GIANT_GECKO)
      div *= EFM32_CMU_CTRL_HFCLKDIV_GET(endian_le32(
        cpu_mem_read_32(EFM32_CMU_ADDR + EFM32_CMU_CTRL_ADDR))) + 1;
# endif
    case EFM32_CLOCK_HFCLK:
      node = pv->hfclk_parent;
      break;

    case EFM32_CLOCK_LFACLK:
      node = pv->lfaclk_parent;
      break;
    case EFM32_CLOCK_LFBCLK:
      node = pv->lfbclk_parent;
      break;

    default:
      break;
    }

  /* compute frequency of root nodes */
  switch (node)
    {
    case EFM32_CLOCK_LE:
      if (efm32_recmu_get_node_freq(pv, freq, acc, EFM32_CLOCK_HFCORECLK))
        return -EINVAL;
# if defined(CONFIG_EFM32_LEOPARD_GECKO) \
  || defined(CONFIG_EFM32_WONDER_GECKO) \
  || defined(CONFIG_EFM32_GIANT_GECKO) \
  || defined(CONFIG_EFM32_ZERO_GECKO)
      div *= 2 << EFM32_CMU_HFCORECLKDIV_HFCORECLKLEDIV_GET(endian_le32(
        cpu_mem_read_32(EFM32_CMU_ADDR + EFM32_CMU_HFCORECLKDIV_ADDR)));
# else
      div *= 2;
# endif
      break;

    case EFM32_CLOCK_HFXO:
      *freq = pv->hfxo_freq;
      *acc = pv->hfxo_acc;
      break;

    case EFM32_CLOCK_LFXO:
      *freq = pv->lfxo_freq;
      *acc = pv->lfxo_acc;
      break;

    case EFM32_CLOCK_HFRCO: {
      static const uint8_t hfrcoband[8] = { 1, 7, 11, 14, 21, 28 };
      freq->denom = 1;
      freq->num = hfrcoband[EFM32_CMU_HFRCOCTRL_BAND_GET(endian_le32(
        cpu_mem_read_32(EFM32_CMU_ADDR + EFM32_CMU_HFRCOCTRL_ADDR)))] * 1000000;
      *acc = DEV_FREQ_ACC(4, 27); /* 1% */
      break;
    }

#ifdef EFM32_CLOCK_AUXHFRCO
    case EFM32_CLOCK_AUXHFRCO: {
      static const uint8_t auxfrcoband[8] = { 14, 11, 7, 1, 0, 0, 28, 21 };
      freq->denom = 1;
      freq->num = auxfrcoband[EFM32_CMU_AUXHFRCOCTRL_BAND_GET(endian_le32(
        cpu_mem_read_32(EFM32_CMU_ADDR + EFM32_CMU_AUXHFRCOCTRL_ADDR)))] * 1000000;
      *acc = DEV_FREQ_ACC(4, 27); /* 1% */
      break;
    }
#endif

    case EFM32_CLOCK_LFRCO:
      freq->denom = 1;
      freq->num = 32768;
      *acc = DEV_FREQ_ACC(4, 27); /* 1% */
      break;

    case EFM32_CLOCK_ULFRCO:
      freq->denom = 1;
      freq->num = 1000;
      *acc = DEV_FREQ_ACC(4, 27); /* 1% */
      break;

    default:
      return -ENOTSUP;
    }

  freq->denom *= div;

  if (!freq->num || !freq->denom)
    return -EINVAL;

  /* FIXME simplify fraction */

  return 0;
}

static DEV_CLOCK_CONFIG_NODE(efm32_recmu_config_node)
{
  struct device_s *dev = accessor->dev;
  struct efm32_recmu_private_s *pv = dev->drv_pv;

  if (pv->busy)
    return -EBUSY;

  if (node_id >= EFM32_CLOCK_count)
    return -ENOENT;

  /* oscilator nodes */
  switch (node_id)
    {
    case EFM32_CLOCK_HFRCO: {
      switch (value->freq.denom)
        {
        case 1:
          break;
        case 0:
          return 0;
        default:
          return -ENOTSUP;
        }
      switch (value->freq.num)
        {
        case 0:
          break;
        case 1000000:
          EFM32_CMU_HFRCOCTRL_BAND_SET(pv->r_hfrcoctrl, 1MHZ);
          break;
        case 7000000:
          EFM32_CMU_HFRCOCTRL_BAND_SET(pv->r_hfrcoctrl, 7MHZ);
          break;
        case 11000000:
          EFM32_CMU_HFRCOCTRL_BAND_SET(pv->r_hfrcoctrl, 11MHZ);
          break;
        case 14000000:
          EFM32_CMU_HFRCOCTRL_BAND_SET(pv->r_hfrcoctrl, 14MHZ);
          break;
        case 21000000:
          EFM32_CMU_HFRCOCTRL_BAND_SET(pv->r_hfrcoctrl, 21MHZ);
          break;
# if defined(CONFIG_EFM32_LEOPARD_GECKO) \
  || defined(CONFIG_EFM32_WONDER_GECKO) \
  || defined(CONFIG_EFM32_GIANT_GECKO) \
  || defined(CONFIG_EFM32_GECKO)
        case 28000000:
          EFM32_CMU_HFRCOCTRL_BAND_SET(pv->r_hfrcoctrl, 28MHZ);
          break;
# endif
        default:
          return -ENOTSUP;
        }
      pv->chg_mask |= EFM32_CLK_MASK(node_id);
      return 0;
    }

# if defined(CONFIG_EFM32_LEOPARD_GECKO) \
  || defined(CONFIG_EFM32_WONDER_GECKO) \
  || defined(CONFIG_EFM32_GIANT_GECKO) \
  || defined(CONFIG_EFM32_ZERO_GECKO)
    case EFM32_CLOCK_AUXHFRCO: {
      switch (value->freq.denom)
        {
        case 1:
          break;
        case 0:
          return 0;
        default:
          return -ENOTSUP;
        }
      switch (value->freq.num)
        {
        case 0:
          break;
        case 1000000:
          EFM32_CMU_AUXHFRCOCTRL_BAND_SET(pv->r_auxhfrcoctrl, 1MHZ);
          break;
        case 7000000:
          EFM32_CMU_AUXHFRCOCTRL_BAND_SET(pv->r_auxhfrcoctrl, 7MHZ);
          break;
        case 11000000:
          EFM32_CMU_AUXHFRCOCTRL_BAND_SET(pv->r_auxhfrcoctrl, 11MHZ);
          break;
        case 14000000:
          EFM32_CMU_AUXHFRCOCTRL_BAND_SET(pv->r_auxhfrcoctrl, 14MHZ);
          break;
        case 21000000:
          EFM32_CMU_AUXHFRCOCTRL_BAND_SET(pv->r_auxhfrcoctrl, 21MHZ);
          break;
# if defined(CONFIG_EFM32_LEOPARD_GECKO) \
  || defined(CONFIG_EFM32_WONDER_GECKO) \
  || defined(CONFIG_EFM32_GIANT_GECKO)
        case 28000000:
          EFM32_CMU_AUXHFRCOCTRL_BAND_SET(pv->r_auxhfrcoctrl, 28MHZ);
          break;
# endif
        default:
          return -ENOTSUP;
        }
      pv->chg_mask |= EFM32_CLK_MASK(node_id);
      return 0;
    }
#endif

    case EFM32_CLOCK_HFXO:
      if (DEV_FREQ_IS_VALID(value->freq))
        pv->hfxo_freq = value->freq;
      if (DEV_FREQ_ACC_IS_VALID(value->acc))
        pv->hfxo_acc = value->acc;
      pv->chg_mask |= EFM32_CLK_MASK(node_id);
      return 0;

    case EFM32_CLOCK_LFXO:
      if (DEV_FREQ_IS_VALID(value->freq))
        pv->lfxo_freq = value->freq;
      if (DEV_FREQ_ACC_IS_VALID(value->acc))
        pv->lfxo_acc = value->acc;
      pv->chg_mask |= EFM32_CLK_MASK(node_id);
      return 0;

    case EFM32_CLOCK_LFRCO:
    case EFM32_CLOCK_ULFRCO:
      switch (value->freq.denom)
        {
        case 0:
          return 0;
        default:
          return -ENOTSUP;
        }

    default:
      break;
    }

  /* other nodes */

  if (value != NULL && (value->ratio.num != 1 ||
                        (value->ratio.denom & (value->ratio.denom-1))))
    return -EINVAL;

  switch (node_id)
    {
    case EFM32_CLOCK_HFCLK: {
      if (value != NULL && (value->freq.denom != 1))
        return -ENOTSUP;
      switch (parent_id)
        {
        case EFM32_CLOCK_HFRCO:
          EFM32_CMU_CMD_HFCLKSEL_SET(pv->r_cmd, HFRCO);
          break;
        case EFM32_CLOCK_HFXO:
          EFM32_CMU_CMD_HFCLKSEL_SET(pv->r_cmd, HFXO);
          break;
        case EFM32_CLOCK_LFRCO:
          EFM32_CMU_CMD_HFCLKSEL_SET(pv->r_cmd, LFRCO);
          break;
        case EFM32_CLOCK_LFXO:
          EFM32_CMU_CMD_HFCLKSEL_SET(pv->r_cmd, HFRCO);
          break;
        default:
          return -ENOTSUP;
        }
      pv->hfclk_new_parent = parent_id;
      break;
    }

# if defined(CONFIG_EFM32_LEOPARD_GECKO) \
  || defined(CONFIG_EFM32_WONDER_GECKO) \
  || defined(CONFIG_EFM32_GIANT_GECKO)
    case EFM32_CLOCK_HFCLKDIV:
      if (parent_id != EFM32_CLOCK_HFCLK)
        return -ENOTSUP;
      if (value != NULL)
        {
          uint32_t d = value->ratio.denom;
          if (d > 8)
            return -ENOTSUP;
          EFM32_CMU_CTRL_HFCLKDIV_SET(pv->r_ctrl, d + 1);
        }
      break;
#endif

    case EFM32_CLOCK_HFCORECLK: {
      if (parent_id != EFM32_CLOCK_HFCLKDIV)
        return -ENOTSUP;
      if (value != NULL)
        {
          uint32_t d = value->ratio.denom;
          if (d > 512)
            return -ENOTSUP;
          EFM32_CMU_HFCORECLKDIV_HFCORECLKDIV_SETVAL(pv->r_hfcoreclkdiv, __FFS(d) - 1);
        }
      break;
    }

    case EFM32_CLOCK_HFPERCLK: {
      if (parent_id != EFM32_CLOCK_HFCLKDIV)
        return -ENOTSUP;
      if (value != NULL)
        {
          uint32_t d = value->ratio.denom;
          if (d > 512)
            return -ENOTSUP;
          EFM32_CMU_HFPERCLKDIV_HFPERCLKDIV_SETVAL(pv->r_hfperclkdiv, __FFS(d) - 1);
        }
      break;
    }

    case EFM32_CLOCK_LE: {
      if (parent_id != EFM32_CLOCK_HFCORECLK)
        return -ENOTSUP;
      if (value != NULL)
        {
          uint32_t d = value->ratio.denom;
# if defined(CONFIG_EFM32_LEOPARD_GECKO) \
  || defined(CONFIG_EFM32_WONDER_GECKO) \
  || defined(CONFIG_EFM32_GIANT_GECKO) \
  || defined(CONFIG_EFM32_ZERO_GECKO)
          if (d != 2 && d != 4)
            return -ENOTSUP;
          EFM32_CMU_HFCORECLKDIV_HFCORECLKLEDIV_SETVAL(pv->r_hfcoreclkdiv, d == 4);
#else
          if (d != 2)
            return -ENOTSUP;
#endif
        }
      break;
    }

#ifdef EFM32_CLOCK_USBC
    case EFM32_CLOCK_USBC: {
      if (value != NULL && value->freq.denom != 1)
        return -ENOTSUP;
      switch (parent_id)
        {
        case EFM32_CLOCK_LFXO:
          EFM32_CMU_CMD_USBCCLKSEL_SET(pv->r_cmd, LFXO);
          break;
        case EFM32_CLOCK_LFRCO:
          EFM32_CMU_CMD_USBCCLKSEL_SET(pv->r_cmd, LFRCO);
          break;
        case EFM32_CLOCK_HFCLK:
          EFM32_CMU_CMD_USBCCLKSEL_SET(pv->r_cmd, HFCLKNODIV);
          break;
        default:
          return -ENOTSUP;
        }
      pv->usbcclk_new_parent = parent_id;
      break;
    }
#endif

    case EFM32_CLOCK_LFACLK: {
      if (value != NULL && value->freq.denom != 1)
        return -ENOTSUP;
      switch (parent_id)
        {
        case EFM32_CLOCK_LE:
          EFM32_CMU_LFCLKSEL_LFA_SET(pv->r_lfclksel, HFCORECLKLEDIV2);
          break;
        case EFM32_CLOCK_LFXO:
          EFM32_CMU_LFCLKSEL_LFA_SET(pv->r_lfclksel, LFXO);
          break;
        case EFM32_CLOCK_LFRCO:
          EFM32_CMU_LFCLKSEL_LFA_SET(pv->r_lfclksel, LFRCO);
          break;
# if defined(CONFIG_EFM32_LEOPARD_GECKO) \
  || defined(CONFIG_EFM32_WONDER_GECKO) \
  || defined(CONFIG_EFM32_GIANT_GECKO) \
  || defined(CONFIG_EFM32_ZERO_GECKO)
        case EFM32_CLOCK_ULFRCO:
          EFM32_CMU_LFCLKSEL_LFA_SET(pv->r_lfclksel, DISABLED_OR_ULFRCO);
          EFM32_CMU_LFCLKSEL_LFAE_SET(pv->r_lfclksel, ULFRCO);
          break;
# endif
        default:
          return -ENOTSUP;
        }
      pv->lfaclk_new_parent = parent_id;
      break;
    }

    case EFM32_CLOCK_LFBCLK: {
      if (value != NULL && value->freq.denom != 1)
        return -ENOTSUP;
      switch (parent_id)
        {
        case EFM32_CLOCK_LE:
          EFM32_CMU_LFCLKSEL_LFB_SET(pv->r_lfclksel, HFCORECLKLEDIV2);
          break;
        case EFM32_CLOCK_LFXO:
          EFM32_CMU_LFCLKSEL_LFB_SET(pv->r_lfclksel, LFXO);
          break;
        case EFM32_CLOCK_LFRCO:
          EFM32_CMU_LFCLKSEL_LFB_SET(pv->r_lfclksel, LFRCO);
          break;
# if defined(CONFIG_EFM32_LEOPARD_GECKO) \
  || defined(CONFIG_EFM32_WONDER_GECKO) \
  || defined(CONFIG_EFM32_GIANT_GECKO) \
  || defined(CONFIG_EFM32_ZERO_GECKO)
        case EFM32_CLOCK_ULFRCO:
          EFM32_CMU_LFCLKSEL_LFB_SET(pv->r_lfclksel, DISABLED_OR_ULFRCO);
          EFM32_CMU_LFCLKSEL_LFBE_SET(pv->r_lfclksel, ULFRCO);
          break;
# endif
        default:
          return -ENOTSUP;
        }
      pv->lfbclk_new_parent = parent_id;
      break;
    }

#ifdef EFM32_CLOCK_LESENSE
    case EFM32_CLOCK_LESENSE:
      if (parent_id != EFM32_CLOCK_LFACLK)
        return -ENOTSUP;
      if (value != NULL)
        {
          uint32_t d = value->ratio.denom;
          if (d > 8)
            return -ENOTSUP;
          EFM32_CMU_LFAPRESC0_LESENSE_SETVAL(pv->r_lfapresc0, __FFS(d) - 1);
        }
      break;
#endif

#ifdef EFM32_CLOCK_RTC
    case EFM32_CLOCK_RTC:
      if (parent_id != EFM32_CLOCK_LFACLK)
        return -ENOTSUP;
      if (value != NULL)
        {
          uint32_t d = value->ratio.denom;
          if (d > 32768)
            return -ENOTSUP;
          EFM32_CMU_LFAPRESC0_RTC_SETVAL(pv->r_lfapresc0, __FFS(d) - 1);
        }
      break;
#endif

#ifdef EFM32_CLOCK_LETIMER
    case EFM32_CLOCK_LETIMER:
      if (parent_id != EFM32_CLOCK_LFACLK)
        return -ENOTSUP;
      if (value != NULL)
        {
          uint32_t d = value->ratio.denom;
          if (d > 32768)
            return -ENOTSUP;
          EFM32_CMU_LFAPRESC0_LETIMER0_SETVAL(pv->r_lfapresc0, __FFS(d) - 1);
        }
      break;
#endif

#ifdef EFM32_CLOCK_LCD
    case EFM32_CLOCK_LCD:
      if (parent_id != EFM32_CLOCK_LFACLK)
        return -ENOTSUP;
      if (value != NULL)
        {
          uint32_t d = value->ratio.denom;
          if (d < 16)
            return -ENOTSUP;
          EFM32_CMU_LFAPRESC0_LCD_SETVAL(pv->r_lfapresc0, __FFS(d) - 5);
        }
      break;
#endif

#ifdef EFM32_CLOCK_LEUART0
    case EFM32_CLOCK_LEUART0:
      if (parent_id != EFM32_CLOCK_LFBCLK)
        return -ENOTSUP;
      if (value != NULL)
        {
          uint32_t d = value->ratio.denom;
          if (d > 8)
            return -ENOTSUP;
          EFM32_CMU_LFBPRESC0_LEUART0_SETVAL(pv->r_lfbpresc0, __FFS(d) - 1);
        }
      break;
#endif

#ifdef EFM32_CLOCK_LEUART1
    case EFM32_CLOCK_LEUART1:
      if (parent_id != EFM32_CLOCK_LFBCLK)
        return -ENOTSUP;
      if (value != NULL)
        {
          uint32_t d = value->ratio.denom;
          if (d > 8)
            return -ENOTSUP;
          EFM32_CMU_LFBPRESC0_LEUART1_SETVAL(pv->r_lfbpresc0, __FFS(d) - 1);
        }
      break;
#endif

    default:
      return -ENOTSUP;
    }

  pv->chg_mask |= EFM32_CLK_MASK(node_id);
  return 0;
}

static void efm32_recmu_clock_wait(struct efm32_recmu_private_s *pv)
{
  uint32_t x = 0;

#ifdef EFM32_CLOCK_AUXHFRCO
  if (pv->wait_mask & EFM32_CLK_MASK(EFM32_CLOCK_AUXHFRCO))
    x |= EFM32_CMU_STATUS_AUXHFRCORDY;
#endif
  if (pv->wait_mask & EFM32_CLK_MASK(EFM32_CLOCK_HFXO))
    x |= EFM32_CMU_STATUS_HFXORDY;
  if (pv->wait_mask & EFM32_CLK_MASK(EFM32_CLOCK_LFXO))
    x |= EFM32_CMU_STATUS_LFXORDY;
  if (pv->wait_mask & EFM32_CLK_MASK(EFM32_CLOCK_HFRCO))
    x |= EFM32_CMU_STATUS_HFRCORDY;
  if (pv->wait_mask & EFM32_CLK_MASK(EFM32_CLOCK_LFRCO))
    x |= EFM32_CMU_STATUS_LFRCORDY;

  while ((endian_le32(cpu_mem_read_32(EFM32_CMU_ADDR +
                                      EFM32_CMU_STATUS_ADDR)) & x) != x)
    ;

  pv->wait_mask = 0;
}

static void efm32_recmu_clock_en(struct efm32_recmu_private_s *pv,
                                 efm32_clock_mask_t use_mask)
{
  struct lower_nodes_s
  {
    efm32_clock_mask_t child_mask;
    enum efm32_clock_node_e node;
    uintptr_t addr;
  };

  static const struct lower_nodes_s tm[4] = {
    { EFM32_CLOCK_LFACLK_CHILDMASK,    EFM32_CLOCK_LFACLK,    EFM32_CMU_LFACLKEN0_ADDR },
    { EFM32_CLOCK_LFBCLK_CHILDMASK,    EFM32_CLOCK_LFBCLK,    EFM32_CMU_LFBCLKEN0_ADDR },
    { EFM32_CLOCK_HFPERCLK_CHILDMASK,  EFM32_CLOCK_HFPERCLK,  EFM32_CMU_HFPERCLKEN0_ADDR },
    { EFM32_CLOCK_HFCORECLK_CHILDMASK, EFM32_CLOCK_HFCORECLK, EFM32_CMU_HFCORECLKEN0_ADDR },
  };

  efm32_clock_mask_t dep_mask;

  /* EFM32_CLOCK_HFCORECLK and EFM32_CLOCK_HFCLK always enabled */
  dep_mask = use_mask | EFM32_CLK_MASK(pv->hfclk_parent) |
    EFM32_CLK_MASK(EFM32_CLOCK_HFCLK) | EFM32_CLK_MASK(EFM32_CLOCK_HFCORECLK) |
    EFM32_CLK_MASK(EFM32_CLOCK_CPU);

  efm32_clock_mask_t m = dep_mask ^ pv->dep_mask;

  /***** lower nodes */

  const struct lower_nodes_s *t = tm;
  for (; t < tm + 4; t++)
    {
      /* update deps */
      if (dep_mask & t->child_mask)
        dep_mask |= EFM32_CLK_MASK(t->node);

      efm32_clock_mask_t q = m & t->child_mask;
      if (!q)
        continue;

      /* enable/disable peripherals clocks */
      uint32_t x = cpu_mem_read_32(EFM32_CMU_ADDR + t->addr);
      while (q)
        {
          dev_clock_node_id_t id = __FFS(q) - 1;
          uint8_t s = efm32_en_bits[id];
          if (s & 0x80)
            {
              s &= 0x7f;
              if ((dep_mask >> id) & 1)
                x |= endian_le32(1 << s);
              else
                x &= ~endian_le32(1 << s);
            }

          q = q & (q - 1);  /* clear rightmost bit set */
        }
      cpu_mem_write_32(EFM32_CMU_ADDR + t->addr, x);
    }

  /***** intermediate nodes */

  m = dep_mask ^ pv->dep_mask;

  /* enable/disable EFM32_CLOCK_HFPERCLK */
  if (m & EFM32_CLK_MASK(EFM32_CLOCK_HFPERCLK))
    {
      uint32_t x = endian_le32(cpu_mem_read_32(EFM32_CMU_ADDR + EFM32_CMU_HFPERCLKDIV_ADDR));
      EFM32_CMU_HFPERCLKDIV_HFPERCLKEN_SET(x, (dep_mask >> EFM32_CLOCK_HFPERCLK) & 1);
      cpu_mem_write_32(EFM32_CMU_ADDR + EFM32_CMU_HFPERCLKDIV_ADDR, endian_le32(x));
    }

#ifdef EFM32_CLOCK_USBC
  /* enable/disable EFM32_CLOCK_USBC */
  if (m & EFM32_CLK_MASK(EFM32_CLOCK_USBC))
    {
      bool_t en = (dep_mask >> EFM32_CLOCK_USBC) & 1;
      uint32_t x = endian_le32(cpu_mem_read_32(EFM32_CMU_ADDR + EFM32_CMU_HFCORECLKEN0_ADDR));
      EFM32_CMU_HFCORECLKEN0_USBC_SET(x, en);
      cpu_mem_write_32(EFM32_CMU_ADDR + EFM32_CMU_HFCORECLKEN0_ADDR, endian_le32(x));
      if (en)
        dep_mask |= EFM32_CLK_MASK(pv->usbcclk_parent);
    }
#endif

  /* enable/disable EFM32_CLOCK_LF*CLK */
  {
    uint32_t x = 0;

    if (dep_mask & EFM32_CLK_MASK(EFM32_CLOCK_LFACLK))
      {
        x |= pv->lfclksel & 0x00010003;
        dep_mask |= EFM32_CLK_MASK(pv->lfaclk_parent) | EFM32_CLK_MASK(EFM32_CLOCK_LE);
      }

    if (dep_mask & EFM32_CLK_MASK(EFM32_CLOCK_LFBCLK))
      {
        x |= pv->lfclksel & 0x0010000c;
        dep_mask |= EFM32_CLK_MASK(pv->lfbclk_parent) | EFM32_CLK_MASK(EFM32_CLOCK_LE);
      }

    cpu_mem_write_32(EFM32_CMU_ADDR + EFM32_CMU_LFCLKSEL_ADDR, endian_le32(x));
  }

  /***** root nodes */

  m = dep_mask ^ pv->dep_mask;

  /* enable/disable EFM32_CLOCK_LE */
  if (m & EFM32_CLK_MASK(EFM32_CLOCK_LE))
    {
      uint32_t x = endian_le32(cpu_mem_read_32(EFM32_CMU_ADDR + EFM32_CMU_HFCORECLKEN0_ADDR));
      EFM32_CMU_HFCORECLKEN0_LE_SET(x, (dep_mask >> EFM32_CLOCK_LE) & 1);
      cpu_mem_write_32(EFM32_CMU_ADDR + EFM32_CMU_HFCORECLKEN0_ADDR, endian_le32(x));
    }

  /* enable/disable oscillators */
  static const efm32_clock_mask_t o =
      EFM32_CLK_MASK(EFM32_CLOCK_LFXO) | EFM32_CLK_MASK(EFM32_CLOCK_HFXO)
    | EFM32_CLK_MASK(EFM32_CLOCK_LFRCO) | EFM32_CLK_MASK(EFM32_CLOCK_HFRCO)
#ifdef EFM32_CLOCK_AUXHFRCO
    | EFM32_CLK_MASK(EFM32_CLOCK_AUXHFRCO)
#endif
    ;

  if (m & o)
    {
      efm32_clock_mask_t en = o & m & dep_mask;
      efm32_clock_mask_t dis = o & m & ~dep_mask;
      uint32_t cmd = 0;
#ifdef EFM32_CLOCK_AUXHFRCO
      EFM32_CMU_OSCENCMD_AUXHFRCOEN_SET(cmd,  (en  >> EFM32_CLOCK_AUXHFRCO) & 1);
      EFM32_CMU_OSCENCMD_AUXHFRCODIS_SET(cmd, (dis >> EFM32_CLOCK_AUXHFRCO) & 1);
#endif
      EFM32_CMU_OSCENCMD_LFXOEN_SET(cmd,   (en  >> EFM32_CLOCK_LFXO) & 1);
      EFM32_CMU_OSCENCMD_LFXODIS_SET(cmd,  (dis >> EFM32_CLOCK_LFXO) & 1);
      EFM32_CMU_OSCENCMD_LFRCOEN_SET(cmd,  (en  >> EFM32_CLOCK_LFRCO) & 1);
      EFM32_CMU_OSCENCMD_LFRCODIS_SET(cmd, (dis >> EFM32_CLOCK_LFRCO) & 1);
      EFM32_CMU_OSCENCMD_HFXOEN_SET(cmd,   (en  >> EFM32_CLOCK_HFXO) & 1);
      EFM32_CMU_OSCENCMD_HFXODIS_SET(cmd,  (dis >> EFM32_CLOCK_HFXO) & 1);
      EFM32_CMU_OSCENCMD_HFRCOEN_SET(cmd,  (en  >> EFM32_CLOCK_HFRCO) & 1);
      EFM32_CMU_OSCENCMD_HFRCODIS_SET(cmd, (dis >> EFM32_CLOCK_HFRCO) & 1);
      cpu_mem_write_32(EFM32_CMU_ADDR + EFM32_CMU_OSCENCMD_ADDR, endian_le32(cmd));

      pv->wait_mask |= en;
    }

  pv->dep_mask = dep_mask;
}


static DEV_CLOCK_COMMIT(efm32_recmu_commit)
{
  struct device_s *dev = accessor->dev;
  struct efm32_recmu_private_s *pv = dev->drv_pv;

  if (pv->busy)
    return -EBUSY;
  pv->busy = 1;

  pv->lfaclk_parent = pv->lfaclk_new_parent;
  pv->lfbclk_parent = pv->lfbclk_new_parent;
  pv->hfclk_parent = pv->hfclk_new_parent;

  /* Enable clocks from new config. Keep clocks from old config
     enabled. */
  efm32_recmu_clock_en(pv, pv->dep_mask);
  efm32_recmu_clock_wait(pv);

  /* Write configuration to device registers */
  cpu_mem_write_32(EFM32_CMU_ADDR + EFM32_CMU_CTRL_ADDR,
                  endian_le32(pv->r_ctrl));
  cpu_mem_write_32(EFM32_CMU_ADDR + EFM32_CMU_HFCORECLKDIV_ADDR,
                  endian_le32(pv->r_hfcoreclkdiv));
  cpu_mem_write_32(EFM32_CMU_ADDR + EFM32_CMU_HFPERCLKDIV_ADDR,
                  endian_le32(pv->r_hfperclkdiv));
  cpu_mem_write_32(EFM32_CMU_ADDR + EFM32_CMU_HFRCOCTRL_ADDR,
                  endian_le32(pv->r_hfrcoctrl));
  cpu_mem_write_32(EFM32_CMU_ADDR + EFM32_CMU_LFRCOCTRL_ADDR,
                  endian_le32(pv->r_lfrcoctrl));
  cpu_mem_write_32(EFM32_CMU_ADDR + EFM32_CMU_AUXHFRCOCTRL_ADDR,
                  endian_le32(pv->r_auxhfrcoctrl));

    /* lf clocks use the config mux for gating, register will be
       updated in efm32_recmu_clock_en. */
  pv->lfclksel = pv->r_lfclksel;

  cpu_mem_write_32(EFM32_CMU_ADDR + EFM32_CMU_LFAPRESC0_ADDR,
                  endian_le32(pv->r_lfapresc0));
  cpu_mem_write_32(EFM32_CMU_ADDR + EFM32_CMU_LFBPRESC0_ADDR,
                  endian_le32(pv->r_lfbpresc0));

  cpu_mem_write_32(EFM32_CMU_ADDR + EFM32_CMU_CMD_ADDR,
                  endian_le32(pv->r_cmd));
  pv->r_cmd = 0;

  /* Disable unused clocks */
  efm32_recmu_clock_en(pv, pv->use_mask);
  efm32_recmu_clock_wait(pv);

  /* propagate changes in tree */
  if ((pv->chg_mask >> pv->hfclk_new_parent) & 1)
    pv->chg_mask |= EFM32_CLK_MASK(EFM32_CLOCK_HFCLK)
      | EFM32_CLK_MASK(EFM32_CLOCK_HFCLKDIV)
      | EFM32_CLK_MASK(EFM32_CLOCK_HFPERCLK)
      | EFM32_CLK_MASK(EFM32_CLOCK_HFCORECLK)
      | EFM32_CLK_MASK(EFM32_CLOCK_LE)
      ;

  if ((pv->chg_mask >> pv->lfaclk_new_parent) & 1)
    pv->chg_mask |= EFM32_CLK_MASK(EFM32_CLOCK_LFACLK);

  if ((pv->chg_mask >> pv->lfbclk_new_parent) & 1)
    pv->chg_mask |= EFM32_CLK_MASK(EFM32_CLOCK_LFBCLK);

  if ((pv->chg_mask >> EFM32_CLOCK_HFCORECLK) & 1)
    pv->chg_mask |= EFM32_CLOCK_HFCORECLK_CHILDMASK;

  if ((pv->chg_mask >> EFM32_CLOCK_HFPERCLK) & 1)
    pv->chg_mask |= EFM32_CLOCK_HFPERCLK_CHILDMASK;

  if ((pv->chg_mask >> EFM32_CLOCK_LFACLK) & 1)
    pv->chg_mask |= EFM32_CLOCK_LFACLK_CHILDMASK;

  if ((pv->chg_mask >> EFM32_CLOCK_LFBCLK) & 1)
    pv->chg_mask |= EFM32_CLOCK_LFBCLK_CHILDMASK;

  /* iterate over changed source ep */
  efm32_clock_mask_t m = (pv->chg_mask & pv->notify_mask) >> EFM32_CLOCK_FIRST_EP;

  while (m)
    {
      dev_clock_node_id_t id = __FFS(m) - 1;
      struct dev_clock_src_ep_s *src = pv->src + id;
      assert(src->flags & DEV_CLOCK_SRC_EP_NOTIFY);
      id += EFM32_CLOCK_FIRST_EP;

      struct dev_freq_s freq;
      struct dev_freq_accuracy_s acc;
      if (efm32_recmu_get_node_freq(pv, &freq, &acc, id))
        abort();
      dev_clock_src_changed(accessor, src, &freq, &acc);

      m = m & (m - 1);          /* clear lsb */
    }

  pv->chg_mask = 0;

  pv->busy = 0;

  return 0;
}

static void efm32_recmu_read_config(struct efm32_recmu_private_s *pv)
{
  pv->r_ctrl = endian_le32(cpu_mem_read_32(EFM32_CMU_ADDR + EFM32_CMU_CTRL_ADDR));
  pv->r_hfcoreclkdiv = endian_le32(cpu_mem_read_32(EFM32_CMU_ADDR + EFM32_CMU_HFPERCLKDIV_ADDR));
  pv->r_hfperclkdiv = endian_le32(cpu_mem_read_32(EFM32_CMU_ADDR + EFM32_CMU_HFPERCLKDIV_ADDR));
  pv->r_hfrcoctrl = endian_le32(cpu_mem_read_32(EFM32_CMU_ADDR + EFM32_CMU_HFRCOCTRL_ADDR));
  pv->r_lfrcoctrl = endian_le32(cpu_mem_read_32(EFM32_CMU_ADDR + EFM32_CMU_LFRCOCTRL_ADDR));
  pv->r_auxhfrcoctrl = endian_le32(cpu_mem_read_32(EFM32_CMU_ADDR + EFM32_CMU_AUXHFRCOCTRL_ADDR));
  pv->r_cmd = 0;
  pv->r_lfclksel = pv->lfclksel;
  pv->r_lfapresc0 = endian_le32(cpu_mem_read_32(EFM32_CMU_ADDR + EFM32_CMU_LFAPRESC0_ADDR));
  pv->r_lfbpresc0 = endian_le32(cpu_mem_read_32(EFM32_CMU_ADDR + EFM32_CMU_LFBPRESC0_ADDR));

  pv->chg_mask = 0;
  pv->lfaclk_new_parent = pv->lfaclk_parent;
  pv->lfbclk_new_parent = pv->lfbclk_parent;
  pv->hfclk_new_parent = pv->hfclk_parent;
#ifdef EFM32_CLOCK_USBC
  pv->usbcclk_new_parent = pv->usbcclk_parent;
#endif
}

static DEV_CLOCK_ROLLBACK(efm32_recmu_rollback)
{
  struct device_s *dev = accessor->dev;
  struct efm32_recmu_private_s *pv = dev->drv_pv;

  if (pv->busy)
    return -EBUSY;

  efm32_recmu_read_config(pv);

  return 0;
}

static DEV_CLOCK_NODE_INFO(efm32_recmu_node_info)
{
  struct device_s *dev = accessor->dev;
  struct efm32_recmu_private_s *pv = dev->drv_pv;

  if (node_id >= EFM32_CLOCK_count)
    return -EINVAL;

  if (*mask & (DEV_CLOCK_INFO_FREQ | DEV_CLOCK_INFO_ACCURACY))
    if (efm32_recmu_get_node_freq(pv, &info->freq, &info->acc, node_id))
      *mask &= ~(DEV_CLOCK_INFO_FREQ | DEV_CLOCK_INFO_ACCURACY);

#ifdef CONFIG_DRIVER_EFM32_RECMU_NAMES
  info->name = efm32_clock_names[node_id];
#else
  *mask &= ~DEV_CLOCK_INFO_NAME;  
#endif

  info->running = (pv->dep_mask >> node_id) & 1;

  if (*mask & DEV_CLOCK_INFO_PARENT)
    switch (node_id)
      {
      case EFM32_CLOCK_HFCLK:
        info->parent_id = pv->hfclk_parent;
        break;
      case EFM32_CLOCK_LFACLK:
        info->parent_id = pv->lfaclk_parent;
        break;
      case EFM32_CLOCK_LFBCLK:
        info->parent_id = pv->lfbclk_parent;
        break;
#ifdef EFM32_CLOCK_USBC
      case EFM32_CLOCK_USBC:
        info->parent_id = pv->usbcclk_parent;
        break;
#endif
      default:
        *mask ^= DEV_CLOCK_INFO_PARENT;
        break;
      }

  if (*mask & (DEV_CLOCK_INFO_SRC | DEV_CLOCK_INFO_SINK))
    {
      info->src = node_id > EFM32_CLOCK_FIRST_EP ?
        pv->src + node_id - EFM32_CLOCK_FIRST_EP : NULL;
      *mask &= ~DEV_CLOCK_INFO_SINK;
    }

  return 0;
}

static DEV_CLOCK_SRC_USE(efm32_recmu_ep_use)
{
  struct device_s *dev = src->dev;
  struct efm32_recmu_private_s *pv = dev->drv_pv;

  dev_clock_node_id_t id = src - pv->src + EFM32_CLOCK_FIRST_EP;
  if (id >= EFM32_CLOCK_count)
    return -EINVAL;

  switch (action)
    {
      case DEV_CLOCK_SRC_USE_HOLD:
        pv->use_mask |= EFM32_CLK_MASK(id);
        src->flags |= DEV_CLOCK_SRC_EP_RUNNING;
        break;
      case DEV_CLOCK_SRC_USE_RELEASE:
        pv->use_mask &= ~EFM32_CLK_MASK(id);
        src->flags &= ~DEV_CLOCK_SRC_EP_RUNNING;
        break;
      case DEV_CLOCK_SRC_USE_NOTIFY:
        pv->notify_mask |= EFM32_CLK_MASK(id);
        return 0;
      case DEV_CLOCK_SRC_USE_IGNORE:
        pv->notify_mask &= ~EFM32_CLK_MASK(id);
        return 0;
      }

  efm32_recmu_clock_en(pv, pv->use_mask);
  efm32_recmu_clock_wait(pv);

  return 0;
}

static DEV_INIT(efm32_recmu_init);
static DEV_CLEANUP(efm32_recmu_cleanup);
#define efm32_recmu_use dev_use_generic

DRIVER_DECLARE(efm32_recmu_drv, DRIVER_FLAGS_EARLY_INIT,
               "EFM32 Reset, Energy and Clock management units", efm32_recmu,
               DRIVER_CLOCK_METHODS(efm32_recmu));

DRIVER_REGISTER(efm32_recmu_drv);

static DEV_INIT(efm32_recmu_init)
{
  struct efm32_recmu_private_s *pv;

  __unused__ uintptr_t addr = 0;
  assert(device_res_get_uint(dev, DEV_RES_MEM, 0, &addr, NULL) == 0 &&
         EFM32_RMU_ADDR == addr);

  assert(device_res_get_uint(dev, DEV_RES_MEM, 1, &addr, NULL) == 0 &&
         EFM32_EMU_ADDR == addr);

  assert(device_res_get_uint(dev, DEV_RES_MEM, 2, &addr, NULL) == 0 &&
         EFM32_CMU_ADDR == addr);

  dev->status = DEVICE_DRIVER_INIT_FAILED;

  pv = mem_alloc(sizeof (*pv), (mem_scope_sys));
  if (!pv)
    return -ENOMEM;

  memset(pv, 0, sizeof (*pv));

  pv->lfxo_acc = DEV_FREQ_ACC(4, 17); /* default to 100ppm */
  pv->hfxo_acc = DEV_FREQ_ACC(4, 17);
  dev->drv_pv = pv;

  /* find nodes which can have multiple clock sources with the
     defined set of resources. */
  uint_fast8_t i;
  efm32_clock_mask_t once, mult = 0, o;
  do {
    o = mult;
    once = 0;

    DEVICE_RES_FOREACH(dev, r, {
        switch (r->type)
          {
          case DEV_RES_CLOCK_RTE:
            i = r->u.clock_rte.node;
            if ((mult >> r->u.clock_rte.parent) & 1)
              mult |= EFM32_CLK_MASK(i);
            break;
          case DEV_RES_CLOCK_OSC:
            i = r->u.clock_osc.node;
            break;
          default:
            continue;
          }
        mult |= once & EFM32_CLK_MASK(i);
        once |= EFM32_CLK_MASK(i);
    });

    if ((mult >> EFM32_CLOCK_HFCLK) & 1)
      mult |= EFM32_CLOCK_HFCORECLK_CHILDMASK | EFM32_CLOCK_HFPERCLK_CHILDMASK;
    if ((mult >> EFM32_CLOCK_LFACLK) & 1)
      mult |= EFM32_CLOCK_LFACLK_CHILDMASK;
    if ((mult >> EFM32_CLOCK_LFBCLK) & 1)
      mult |= EFM32_CLOCK_LFBCLK_CHILDMASK;
  } while (o != mult);

  /* init oscilator nodes */
  for (i = 0; i < EFM32_CLOCK_EP_COUNT; i++)
    {
      dev_clock_source_init(dev, &pv->src[i], &efm32_recmu_ep_use);
      if ((mult >> (i + EFM32_CLOCK_FIRST_EP)) & 1)
        pv->src[i].flags |= DEV_CLOCK_SRC_EP_VARFREQ;
    }

  pv->lfclksel = EFM32_CMU_LFCLKSEL_LFA(LFRCO) | EFM32_CMU_LFCLKSEL_LFB(LFRCO);
  pv->hfclk_parent = EFM32_CLOCK_HFRCO;
  pv->lfaclk_parent = EFM32_CLOCK_LFRCO;
  pv->lfbclk_parent = EFM32_CLOCK_LFRCO;
#ifdef EFM32_CLOCK_USBC
  pv->usbcclk_parent = EFM32_CLOCK_HFCLK;
#endif

  efm32_recmu_read_config(pv);

#ifdef CONFIG_DRIVER_EFM32_LEUART_PRINTK
  /* hack to keep leuart clock enabled for early console before the
     leuart driver is loaded. */
# if defined(EFM32_CLOCK_LEUART0) && CONFIG_MUTEK_PRINTK_ADDR == 0x40084000
    pv->use_mask |= EFM32_CLK_MASK(EFM32_CLOCK_LEUART0);
# elif defined(EFM32_CLOCK_LEUART1) && CONFIG_MUTEK_PRINTK_ADDR == 0x40084400
    pv->use_mask |= EFM32_CLK_MASK(EFM32_CLOCK_LEUART1);
# endif
#endif

#ifdef CONFIG_DRIVER_EFM32_USART_PRINTK
  /* hack to keep usart clock enabled for early console before the
     usart driver is loaded. */
# if defined(EFM32_CLOCK_UART0) && CONFIG_MUTEK_PRINTK_ADDR == 0x4000e000
    pv->use_mask |= EFM32_CLK_MASK(EFM32_CLOCK_UART0);
# elif defined(EFM32_CLOCK_UART1) && CONFIG_MUTEK_PRINTK_ADDR == 0x4000e400
    pv->use_mask |= EFM32_CLK_MASK(EFM32_CLOCK_UART1);
# elif defined(EFM32_CLOCK_USART0) && CONFIG_MUTEK_PRINTK_ADDR == 0x4000c000
    pv->use_mask |= EFM32_CLK_MASK(EFM32_CLOCK_USART0);
# elif defined(EFM32_CLOCK_USART1) && CONFIG_MUTEK_PRINTK_ADDR == 0x4000c400
    pv->use_mask |= EFM32_CLK_MASK(EFM32_CLOCK_USART1);
# elif defined(EFM32_CLOCK_USART2) && CONFIG_MUTEK_PRINTK_ADDR == 0x4000c800
    pv->use_mask |= EFM32_CLK_MASK(EFM32_CLOCK_USART2);
# endif
#endif

  dev->drv = &efm32_recmu_drv;
  dev->status = DEVICE_DRIVER_INIT_DONE;

  return 0;

 err_mem:
  mem_free(pv);
  return -1;
}

static DEV_CLEANUP(efm32_recmu_cleanup)
{
  struct efm32_recmu_private_s *pv = dev->drv_pv;

  mem_free(pv);
}


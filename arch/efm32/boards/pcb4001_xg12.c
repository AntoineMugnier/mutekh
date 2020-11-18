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

    Copyright Sebastien Cerdan <sebcerdan@gmail.com> (c) 2016

*/

#if defined(CONFIG_DEVICE)
# include <device/resources.h>
# include <device/irq.h>
# include <device/class/iomux.h>
# include <device/class/cmu.h>
# include <device/class/timer.h>
# include <device/class/dma.h>
# include <device/resource/uart.h>
# include <device/class/i2c.h>
#endif

#include <hexo/iospace.h>
#include <arch/efm32/irq.h>
#include <arch/efm32/pin.h>
#include <arch/efm32/gpio.h>
#include <arch/efm32/cmu.h>
#ifdef CONFIG_DEVICE_CLOCK
  #include <arch/efm32/clock.h>
#endif
#include <arch/efm32/dma_source.h>
#include <arch/efm32/devaddr.h>
#include <arch/efm32/cmu.h>
#include <arch/efm32/emu.h>
#include <mutek/startup.h>

#define HFXO_FREQ  38400000

static void efm32_wait_button_released()
{
  uint32_t button_pin = EFM32_PF6;
  uint32_t x;

  /* wait for button to be released */
  uint32_t bank = button_pin / 16;
  uint32_t h = (button_pin >> 1) & 4;

  x = cpu_mem_read_32(EFM32_GPIO_ADDR + EFM32_GPIO_MODEL_ADDR(bank) + h);
  EFM32_GPIO_MODEL_MODE_SET(button_pin % 8, x, INPUT);
  cpu_mem_write_32(EFM32_GPIO_ADDR + EFM32_GPIO_MODEL_ADDR(bank) + h, x);

  while (!(cpu_mem_read_32(EFM32_GPIO_ADDR + EFM32_GPIO_DIN_ADDR(bank))
           & EFM32_GPIO_DIN_DIN(button_pin % 16)))
    ;
}

void efm32_board_init_dcdc()
{
  uint32_t x;
  /* unlock registers */
  cpu_mem_write_32(EFM32_EMU_ADDR + EFR32_EMU_PWRLOCK_ADDR, 0xADE8);

  x = cpu_mem_read_32(EFM32_EMU_ADDR + EFR32_EMU_PWRCFG_ADDR);
  EFR32_EMU_PWRCFG_PWRCFG_SET(x, DCDCTODVDD);
  cpu_mem_write_32(EFM32_EMU_ADDR + EFR32_EMU_PWRCFG_ADDR, x);

  x = cpu_mem_read_32(EFM32_EMU_ADDR + EFR32_EMU_PWRCFG_ADDR);
  assert((x & EFR32_EMU_PWRCFG_MASK) == EFR32_EMU_PWRCFG_PWRCFG_DCDCTODVDD);

  x = EFR32_EMU_PWRCTRL_ANASW |
      EFR32_EMU_PWRCTRL_REGPWRSEL;
  cpu_mem_write_32(EFM32_EMU_ADDR + EFR32_EMU_PWRCTRL_ADDR, x);

  x = EFR32_EMU_DCDCLNFREQCTRL_RCOBAND(4) |
      EFR32_EMU_DCDCLNFREQCTRL_RCOTRIM(0x10);
  cpu_mem_write_32(EFM32_EMU_ADDR + EFR32_EMU_DCDCLNFREQCTRL_ADDR, x);

  while (cpu_mem_read_32(EFM32_EMU_ADDR + EFR32_EMU_DCDCSYNC_ADDR) & 1);

  x = EFR32_EMU_DCDCCTRL_DCDCMODE(LOWPOWER) |
      EFR32_EMU_DCDCCTRL_DCDCMODEEM23 |
      EFR32_EMU_DCDCCTRL_DCDCMODEEM4;  
  cpu_mem_write_32(EFM32_EMU_ADDR + EFR32_EMU_DCDCCTRL_ADDR, x);

  x = EFR32_EMU_DCDCMISCCTRL_PFETCNT(3) |
      EFR32_EMU_DCDCMISCCTRL_NFETCNT(3) |
      EFR32_EMU_DCDCMISCCTRL_LNFORCECCM | 
      EFR32_EMU_DCDCMISCCTRL_LPCMPHYSDIS |
      EFR32_EMU_DCDCMISCCTRL_LPCMPHYSHI |
      EFR32_EMU_DCDCMISCCTRL_LNFORCECCM |
      EFR32_EMU_DCDCMISCCTRL_LPCLIMILIMSEL(1) |
      EFR32_EMU_DCDCMISCCTRL_LNCLIMILIMSEL(7) |
      EFR32_EMU_DCDCMISCCTRL_LPCMPBIASEM234H(BIAS0);
 
  cpu_mem_write_32(EFM32_EMU_ADDR + EFR32_EMU_DCDCMISCCTRL_ADDR, x);
}

void efm32_board_init()
{
  uint32_t x;

  efm32_board_init_dcdc();

  /* Enable GPIO clock */
  
  x = EFM32_CMU_HFBUSCLKEN0_GPIO;
  cpu_mem_write_32(EFM32_CMU_ADDR + EFM32_CMU_HFBUSCLKEN0_ADDR, x);

  /* Wait button to be released */
  efm32_wait_button_released();

  /* Set  CTUNE field */
  x = cpu_mem_read_32(EFM32_CMU_ADDR + EFM32_CMU_HFXOSTEADYSTATECTRL_ADDR);
  EFM32_CMU_HFXOSTEADYSTATECTRL_CTUNE_SET(x, 0x150);
  cpu_mem_write_32(EFM32_CMU_ADDR + EFM32_CMU_HFXOSTEADYSTATECTRL_ADDR, x);

  /* Select HFXO as HF clock */
  x = EFM32_CMU_OSCENCMD_HFXOEN;
  cpu_mem_write_32(EFM32_CMU_ADDR + EFM32_CMU_OSCENCMD_ADDR, x);

  while (!(cpu_mem_read_32(EFM32_CMU_ADDR + EFM32_CMU_STATUS_ADDR) & EFM32_CMU_STATUS_HFXORDY))
    ;

  cpu_mem_write_32(EFM32_CMU_ADDR + EFM32_CMU_HFCLKSEL_ADDR, EFM32_CMU_HFCLKSEL_HF(HFXO));

  x = cpu_mem_read_32(EFM32_CMU_ADDR + EFM32_CMU_HFCLKSTATUS_ADDR);
  assert(EFM32_CMU_HFCLKSTATUS_SELECTED_GET(x) == EFM32_CMU_HFCLKSTATUS_SELECTED_HFXO);

#ifndef CONFIG_DEVICE_CLOCK_GATING
  /* Set PA5 high for enabling VCOM for USART0 */
  x = cpu_mem_read_32(EFM32_GPIO_ADDR + EFM32_GPIO_MODEL_ADDR(0));
  EFM32_GPIO_MODEL_MODE_SET(5, x, PUSHPULL);
  cpu_mem_write_32(EFM32_GPIO_ADDR + EFM32_GPIO_MODEL_ADDR(0), x);

  x = EFM32_GPIO_DOUT_DOUT(5);
  cpu_mem_write_32(EFM32_GPIO_ADDR + EFM32_GPIO_DOUT_ADDR(0) + 0x06000000, x);

  /* Switch leds on */
  x = cpu_mem_read_32(EFM32_GPIO_ADDR + EFM32_GPIO_MODEL_ADDR(5));
  EFM32_GPIO_MODEL_MODE_SET(4, x, PUSHPULL);
  EFM32_GPIO_MODEL_MODE_SET(5, x, PUSHPULL);
  cpu_mem_write_32(EFM32_GPIO_ADDR + EFM32_GPIO_MODEL_ADDR(5), x);
  
  x = EFM32_GPIO_DOUT_DOUT(4) | EFM32_GPIO_DOUT_DOUT(5);
  cpu_mem_write_32(EFM32_GPIO_ADDR + EFM32_GPIO_DOUT_ADDR(5) + 0x06000000, x);
#endif

#if defined(CONFIG_DRIVER_EFR32_RADIO)
  x = cpu_mem_read_32(EFM32_CMU_ADDR + EFM32_CMU_CTRL_ADDR);
  x |= EFM32_CMU_CTRL_HFRADIOCLKEN;
  cpu_mem_write_32(EFM32_CMU_ADDR + EFM32_CMU_CTRL_ADDR, x);

  cpu_mem_write_32(EFM32_CMU_ADDR + EFM32_CMU_HFRADIOCLKEN0_ADDR, EFM32_CMU_HFRADIOCLKEN0_MASK);
 #if (CONFIG_EFM32_ARCHREV == EFM32_ARCHREV_EFR_XG12) ||\
     (CONFIG_EFM32_ARCHREV == EFM32_ARCHREV_EFR_XG14)
  cpu_mem_write_32(EFM32_CMU_ADDR + EFM32_CMU_HFRADIOALTCLKEN0_ADDR, EFM32_CMU_HFRADIOALTCLKEN0_MASK);
 #endif
  x = cpu_mem_read_32(EFM32_CMU_ADDR + EFM32_CMU_HFXOCTRL_ADDR);
  x |= EFM32_CMU_HFXOCTRL_AUTOSTARTRDYSELRAC;
  cpu_mem_write_32(EFM32_CMU_ADDR + EFM32_CMU_HFXOCTRL_ADDR, x);

#endif 

}

#if defined(CONFIG_DRIVER_CPU_ARM32M)

DEV_DECLARE_STATIC(cpu_dev, "cpu", DEVICE_FLAG_CPU, arm32m_drv,
                   DEV_STATIC_RES_ID(0, 0),
# ifdef CONFIG_CPU_ARM32M_CLOCK
                   DEV_STATIC_RES_CLK_SRC("/recmu", EFM32_CLOCK_CPU, 0)
# else
                   DEV_STATIC_RES_FREQ(HFXO_FREQ, 1),
# endif
                   );

#endif

#if defined(CONFIG_DRIVER_EFM32_RECMU)

DEV_DECLARE_STATIC(recmu_dev, "recmu", 0, efm32_recmu_drv,
                   DEV_STATIC_RES_MEM(0x400e5000, 0x400e5400), /* RMU */
                   DEV_STATIC_RES_MEM(0x400e3000, 0x400e3400), /* EMU */
                   DEV_STATIC_RES_MEM(0x400e4000, 0x400e4400), /* CMU */

                   DEV_STATIC_RES_DEV_ICU("/cpu"),
                   DEV_STATIC_RES_IRQ(0, EFM32_IRQ_CMU, DEV_IRQ_SENSE_RISING_EDGE, 0, 1),

                   /* Common to all config */
                   DEV_STATIC_RES_CMU_OSC(EFM32_CLOCK_LFXO, 0b11, 32768, 1),
                   DEV_STATIC_RES_CMU_MUX(EFM32_CLOCK_LFXO, EFM32_CLOCK_LFACLK, 0b11, 1, 1),
                   DEV_STATIC_RES_CMU_MUX(EFM32_CLOCK_LFXO, EFM32_CLOCK_LFBCLK, 0b11, 1, 1),
                   DEV_STATIC_RES_CMU_MUX(EFM32_CLOCK_LFXO, EFM32_CLOCK_LFECLK, 0b11, 1, 1),

                   /* config 0: run on HFRCO @ 38Mhz, LFXO @ 32Khz */
                   DEV_STATIC_RES_CMU_MUX(EFM32_CLOCK_HFRCO, EFM32_CLOCK_HFCLK,  0b1, 1, 1),

                   /* config 1: run on crystals HFXO @ 38Mhz, LFXO @ 32Khz */
                   DEV_STATIC_RES_CMU_MUX(EFM32_CLOCK_HFXO, EFM32_CLOCK_HFCLK,  0b10, 1, 1),
                   );

#endif

#if defined(CONFIG_DRIVER_EFM32_USART_CHAR)

DEV_DECLARE_STATIC(uart0_dev, "usart0", 0, efm32_usart_drv,
                   DEV_STATIC_RES_MEM(0x40010000, 0x40010400),
# ifdef CONFIG_DEVICE_CLOCK
                   DEV_STATIC_RES_CLK_SRC("/recmu", EFM32_CLOCK_USART0, 0),
# else
                   DEV_STATIC_RES_FREQ(HFXO_FREQ, 1),
#endif

                   DEV_STATIC_RES_DEV_ICU("/cpu"),
                   DEV_STATIC_RES_IRQ(0, EFM32_IRQ_USART0_RX, DEV_IRQ_SENSE_RISING_EDGE, 0, 1),
                   DEV_STATIC_RES_IRQ(1, EFM32_IRQ_USART0_TX, DEV_IRQ_SENSE_RISING_EDGE, 0, 1),

                   DEV_STATIC_RES_DEV_IOMUX("/gpio"),
                   DEV_STATIC_RES_IOMUX("rx", EFM32_LOC0, EFM32_PA1, 0, 0),
                   DEV_STATIC_RES_IOMUX("tx", EFM32_LOC0, EFM32_PA0, 0, 0),
                   DEV_STATIC_RES_UART(115200, 8, 0, 0, 0)
                   );

#endif

#ifdef CONFIG_DRIVER_EFM32_LEUART_CHAR

DEV_DECLARE_STATIC(leuart0_dev, "leuart0", 0, efm32_leuart_drv,
                   DEV_STATIC_RES_MEM(0x4004a000, 0x4004a400),
# ifdef CONFIG_DEVICE_CLOCK
                   DEV_STATIC_RES_CLK_SRC("/recmu", EFM32_CLOCK_LEUART0, 0),
# else
                   DEV_STATIC_RES_FREQ(32768, 1),
#endif

                   DEV_STATIC_RES_DEV_ICU("/cpu"),
                   DEV_STATIC_RES_IRQ(0, EFM32_IRQ_LEUART0, DEV_IRQ_SENSE_RISING_EDGE, 0, 1),

                   DEV_STATIC_RES_DEV_IOMUX("/gpio"),
                   DEV_STATIC_RES_IOMUX("tx",  EFM32_LOC2, EFM32_PA2, 0, 0),
                   DEV_STATIC_RES_IOMUX("rx",  EFM32_LOC2, EFM32_PA3, 0, 0),

                  DEV_STATIC_RES_UART(9600, 8, 0, 0, 0)
                   );

#endif

#if defined(CONFIG_DRIVER_EFM32_I2C)

DEV_DECLARE_STATIC(i2c0_dev, "i2c0", 0, efm32_i2c_drv,
                   DEV_STATIC_RES_MEM(0x4000c000, 0x4000c400),
# ifdef CONFIG_DEVICE_CLOCK
                   DEV_STATIC_RES_CLK_SRC("/recmu", EFM32_CLOCK_I2C0, 0),
# else
                   DEV_STATIC_RES_FREQ(HFXO_FREQ, 1),
#endif

                   DEV_STATIC_RES_DEV_ICU("/cpu"),
                   DEV_STATIC_RES_IRQ(0, EFM32_IRQ_I2C0, DEV_IRQ_SENSE_RISING_EDGE, 0, 1),

                   DEV_STATIC_RES_DEV_IOMUX("/gpio"),
                   DEV_STATIC_RES_IOMUX("sda", EFM32_LOC16, EFM32_PC11, 0, 0),
                   DEV_STATIC_RES_IOMUX("scl", EFM32_LOC14, EFM32_PC10, 0, 0),
                   DEV_STATIC_RES_I2C_BITRATE(100000),
                   );

#elif defined(CONFIG_DRIVER_EFM32_I2C_SLAVE)

DEV_DECLARE_STATIC(i2c0_dev, "i2cs0", 0, efm32_i2c_slave_drv,
                   DEV_STATIC_RES_MEM(0x4000c000, 0x4000c400),
# ifdef CONFIG_DEVICE_CLOCK
                   DEV_STATIC_RES_CLK_SRC("/recmu", EFM32_CLOCK_I2C0, 0),
# else
                   DEV_STATIC_RES_FREQ(HFXO_FREQ, 1),
#endif

                   DEV_STATIC_RES_DEV_ICU("/cpu"),
                   DEV_STATIC_RES_IRQ(0, EFM32_IRQ_I2C0, DEV_IRQ_SENSE_RISING_EDGE, 0, 1),

                   DEV_STATIC_RES_DEV_IOMUX("/gpio"),
                   DEV_STATIC_RES_IOMUX("sda", EFM32_LOC16, EFM32_PC11, 0, 0),
                   DEV_STATIC_RES_IOMUX("scl", EFM32_LOC14, EFM32_PC10, 0, 0),
                   DEV_STATIC_RES_I2C_BITRATE(100000),
                   );

#endif

#ifdef CONFIG_DRIVER_EFM32_GPIO

DEV_DECLARE_STATIC(gpio_dev, "gpio", 0, efm32_gpio_drv,
                   DEV_STATIC_RES_MEM(0x4000a000, 0x4000b000),
# ifdef CONFIG_DEVICE_CLOCK
                   DEV_STATIC_RES_CLK_SRC("/recmu", EFM32_CLOCK_GPIO, 0),
# else
                   DEV_STATIC_RES_FREQ(HFXO_FREQ, 1),
#endif
                   DEV_STATIC_RES_DEV_ICU("/cpu"),
                   DEV_STATIC_RES_IRQ(0, EFM32_IRQ_GPIO_EVEN, DEV_IRQ_SENSE_RISING_EDGE, 0, 1),
                   DEV_STATIC_RES_IRQ(1, EFM32_IRQ_GPIO_ODD, DEV_IRQ_SENSE_RISING_EDGE, 0, 1)
                   );

#endif

#ifdef CONFIG_DRIVER_EFM32_RTCC

DEV_DECLARE_STATIC(rtcc_dev, "rtcc", 0, efm32_rtcc_drv,
                   DEV_STATIC_RES_MEM(0x40042000, 0x40042400),
# ifdef CONFIG_DEVICE_CLOCK
                   DEV_STATIC_RES_CLK_SRC("/recmu", EFM32_CLOCK_RTCC, 0),
# else
                   DEV_STATIC_RES_FREQ(32768, 1),
#endif
                   DEV_STATIC_RES_DEV_ICU("/cpu"),
                   DEV_STATIC_RES_IRQ(0, EFM32_IRQ_RTCC, DEV_IRQ_SENSE_RISING_EDGE, 0, 1),
                   );

#endif

#ifdef CONFIG_DRIVER_EFM32_TIMER

DEV_DECLARE_STATIC(timer0_dev, "timer0", 0, efm32_timer_drv,
                   DEV_STATIC_RES_MEM(0x40018000, 0x40018400),
# ifdef CONFIG_DEVICE_CLOCK
                   DEV_STATIC_RES_CLK_SRC("/recmu", EFM32_CLOCK_TIMER0, 0),
# else
                   DEV_STATIC_RES_FREQ(HFXO_FREQ, 1),
#endif
                   DEV_STATIC_RES_DEV_ICU("/cpu"),
                   DEV_STATIC_RES_IRQ(0, EFM32_IRQ_TIMER0, DEV_IRQ_SENSE_RISING_EDGE, 0, 1),
                   );

#endif

#if defined(CONFIG_DRIVER_EFR32_DMA)

DEV_DECLARE_STATIC(dma_dev, "dma", 0, efm32_dma_drv,
                   DEV_STATIC_RES_MEM(0x400e2000, 0x400e3000),
# ifdef CONFIG_DEVICE_CLOCK
                   DEV_STATIC_RES_CLK_SRC("/recmu", EFM32_CLOCK_LDMA, 0),
# else
                   DEV_STATIC_RES_FREQ(HFXO_FREQ, 1),
#endif
                   DEV_STATIC_RES_DEV_ICU("/cpu"),
                   DEV_STATIC_RES_IRQ(0, EFM32_IRQ_DMA, DEV_IRQ_SENSE_RISING_EDGE, 0, 1)
                   );

#endif

#if defined(CONFIG_DRIVER_EFM32_USART_SPI)

DEV_DECLARE_STATIC(usart_dev, "spi", 0, efm32_usart_spi_drv,

  #if (CONFIG_EFM32_ARCHREV == EFM32_ARCHREV_EFR_XG1) ||\
      (CONFIG_EFM32_ARCHREV == EFM32_ARCHREV_EFR_XG14)

                   DEV_STATIC_RES_MEM(0x40010400, 0x40010800),
# ifdef CONFIG_DEVICE_CLOCK
                   DEV_STATIC_RES_CLK_SRC("/recmu", EFM32_CLOCK_USART1, 0),
# else
                   DEV_STATIC_RES_FREQ(HFXO_FREQ, 1),
#endif

                   DEV_STATIC_RES_DEV_ICU("/cpu"),
                   DEV_STATIC_RES_IRQ(0, EFM32_IRQ_USART1_RX, DEV_IRQ_SENSE_RISING_EDGE, 0, 1),
    #if defined(CONFIG_DRIVER_EFR32_DMA)
                   DEV_STATIC_RES_DEV_PARAM("dma", "/dma"),
                   DEV_STATIC_RES_DMA((1 << 0), EFM32_DMA_SOURCE_USART1),
                   DEV_STATIC_RES_DMA((1 << 1), EFM32_DMA_SOURCE_USART1),
    #endif

                   DEV_STATIC_RES_DEV_IOMUX("/gpio"),
                   DEV_STATIC_RES_IOMUX("clk",  EFM32_LOC11, EFM32_PC8, 0, 0),
                   DEV_STATIC_RES_IOMUX("miso", EFM32_LOC11, EFM32_PC7, 0, 0),
                   DEV_STATIC_RES_IOMUX("mosi", EFM32_LOC11, EFM32_PC6, 0, 0),

  #elif (CONFIG_EFM32_ARCHREV == EFM32_ARCHREV_EFR_XG12)

                   DEV_STATIC_RES_MEM(0x40010400, 0x40010800),
# ifdef CONFIG_DEVICE_CLOCK
                   DEV_STATIC_RES_CLK_SRC("/recmu", EFM32_CLOCK_USART2, 0),
# else
                   DEV_STATIC_RES_FREQ(HFXO_FREQ, 1),
#endif

                   DEV_STATIC_RES_DEV_ICU("/cpu"),
                   DEV_STATIC_RES_IRQ(0, EFM32_IRQ_USART2_RX, DEV_IRQ_SENSE_RISING_EDGE, 0, 1),
    #if defined(CONFIG_DRIVER_EFR32_DMA)
                   DEV_STATIC_RES_DEV_PARAM("dma", "/dma"),
                   DEV_STATIC_RES_DMA((1 << 0), EFM32_DMA_SOURCE_USART2),
                   DEV_STATIC_RES_DMA((1 << 1), EFM32_DMA_SOURCE_USART2),
    #endif

                   DEV_STATIC_RES_DEV_IOMUX("/gpio"),
                   DEV_STATIC_RES_IOMUX("clk",  EFM32_LOC1, EFM32_PA8, 0, 0),
                   DEV_STATIC_RES_IOMUX("miso", EFM32_LOC1, EFM32_PA7, 0, 0),
                   DEV_STATIC_RES_IOMUX("mosi", EFM32_LOC1, EFM32_PA6, 0, 0),
  #else
   #error
  #endif

  #ifdef CONFIG_DRIVER_EFM32_RTCC
                   DEV_STATIC_RES_DEV_TIMER("/rtcc")
  #elif defined(CONFIG_DRIVER_EFM32_TIMER)
                   DEV_STATIC_RES_DEV_TIMER("/timer0")
  #endif
                   );
#endif


#if defined(CONFIG_DRIVER_EFR32_RADIO)
DEV_DECLARE_STATIC(radio_dev, "rfpacket", 0, efr32_radio_drv,
# ifdef CONFIG_DEVICE_CLOCK
                   DEV_STATIC_RES_CLK_SRC("/recmu", EFM32_CLOCK_PROTIMER, 0),
                   DEV_STATIC_RES_CLK_SRC("/recmu", EFM32_CLOCK_MODEM, 1),
                   DEV_STATIC_RES_CLK_SRC("/recmu", EFM32_CLOCK_RAC, 2),
                   DEV_STATIC_RES_CLK_SRC("/recmu", EFM32_CLOCK_BUFC, 3),
                   DEV_STATIC_RES_CLK_SRC("/recmu", EFM32_CLOCK_FRC, 4),
                   DEV_STATIC_RES_CLK_SRC("/recmu", EFM32_CLOCK_SYNTH, 5),
                   DEV_STATIC_RES_CLK_SRC("/recmu", EFM32_CLOCK_RFSENSE, 6),
                   DEV_STATIC_RES_CLK_SRC("/recmu", EFM32_CLOCK_AGC, 7),
# ifdef CONFIG_DRIVER_EFM32_RFPACKET_RTCC
                   DEV_STATIC_RES_CLK_SRC("/recmu", EFM32_CLOCK_PRS, 8),
                   DEV_STATIC_RES_CLK_SRC("/recmu", EFM32_CLOCK_RTCC, 9),
# endif
# else
                   DEV_STATIC_RES_FREQ(HFXO_FREQ, 1),
#endif

#if defined(CONFIG_DRIVER_EFR32_RFPACKET_ANT_DIV)
                   DEV_STATIC_RES_DEV_IOMUX("/gpio"),
                   DEV_STATIC_RES_IOMUX("sel",  EFM32_LOC11, EFM32_PC9, 0, 0),
                   DEV_STATIC_RES_IOMUX("nsel", EFM32_LOC11, EFM32_PC10, 0, 0),
#endif

                   DEV_STATIC_RES_DEV_ICU("/cpu"),
                   DEV_STATIC_RES_IRQ(0, EFM32_IRQ_MODEM, DEV_IRQ_SENSE_RISING_EDGE, 0, 1),
                   DEV_STATIC_RES_IRQ(1, EFM32_IRQ_RAC_SEQ, DEV_IRQ_SENSE_RISING_EDGE, 0, 1),
                   DEV_STATIC_RES_IRQ(2, EFM32_IRQ_RAC_RSM, DEV_IRQ_SENSE_RISING_EDGE, 0, 1),
                   DEV_STATIC_RES_IRQ(3, EFM32_IRQ_BUFC, DEV_IRQ_SENSE_RISING_EDGE, 0, 1),
                   DEV_STATIC_RES_IRQ(4, EFM32_IRQ_AGC, DEV_IRQ_SENSE_RISING_EDGE, 0, 1),
                   DEV_STATIC_RES_IRQ(5, EFM32_IRQ_SYNTH, DEV_IRQ_SENSE_RISING_EDGE, 0, 1),
                   DEV_STATIC_RES_IRQ(6, EFM32_IRQ_RFSENSE, DEV_IRQ_SENSE_RISING_EDGE, 0, 1),
                   DEV_STATIC_RES_IRQ(7, EFM32_IRQ_PROTIMER, DEV_IRQ_SENSE_RISING_EDGE, 0, 1),
                   DEV_STATIC_RES_IRQ(8, EFM32_IRQ_FRC, DEV_IRQ_SENSE_RISING_EDGE, 0, 1),
#ifdef CONFIG_DRIVER_EFM32_RFPACKET_RTCC
                   DEV_STATIC_RES_IRQ(9, EFM32_IRQ_RTCC, DEV_IRQ_SENSE_RISING_EDGE, 0, 1),
#endif


                   );
#endif


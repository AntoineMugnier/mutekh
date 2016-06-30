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

#if defined(CONFIG_DEVICE)
# include <device/resources.h>
# include <device/irq.h>
# include <device/class/iomux.h>
# include <device/class/cmu.h>
# include <device/class/dma.h>
# include <device/class/usbdev.h>
# include <device/class/i2c.h>
#endif

#include <arch/efm32/irq.h>
#include <arch/efm32/pin.h>
#include <arch/efm32/clock.h>
#include <arch/efm32/dma_source.h>

#if defined(CONFIG_DRIVER_CPU_ARM32M)

DEV_DECLARE_STATIC(cpu_dev, "cpu", DEVICE_FLAG_CPU, arm32m_drv,
                   DEV_STATIC_RES_ID(0, 0),
# ifdef CONFIG_CPU_ARM32M_CLOCK
                   DEV_STATIC_RES_CLK_SRC("/recmu", EFM32_CLOCK_CPU, 0)
# else
                   DEV_STATIC_RES_FREQ(14000000, 1),
# endif
                   );

#endif

#ifdef CONFIG_DRIVER_EFM32_RECMU

DEV_DECLARE_STATIC(recmu_dev, "recmu", 0, efm32_recmu_drv,
                   DEV_STATIC_RES_MEM(0x400ca000, 0x400ca400), /* RMU */
                   DEV_STATIC_RES_MEM(0x400c6000, 0x400c6400), /* EMU */
                   DEV_STATIC_RES_MEM(0x400c8000, 0x400c8400), /* CMU */

                   DEV_STATIC_RES_DEV_ICU("/cpu"),
                   DEV_STATIC_RES_IRQ(0, EFM32_IRQ_CMU, DEV_IRQ_SENSE_RISING_EDGE, 0, 1),

                   /* config 0: run on HFRCO @ 14Mhz */
                   DEV_STATIC_RES_CMU_MUX(EFM32_CLOCK_LFRCO, EFM32_CLOCK_LFACLK, 0b0111, 1, 1),
                   DEV_STATIC_RES_CMU_MUX(EFM32_CLOCK_HFRCO, EFM32_CLOCK_HFCLK,  0b0011, 1, 1),
                   DEV_STATIC_RES_CMU_OSC(EFM32_CLOCK_HFRCO, 0b0001, 14000000, 1),
                   /* config 1: run on HFRCO @ 28Mhz */
                   DEV_STATIC_RES_CMU_OSC(EFM32_CLOCK_HFRCO, 0b0010, 28000000, 1),
                   /* config 2: run on LFRCO @ 32khz */
                   DEV_STATIC_RES_CMU_MUX(EFM32_CLOCK_LFRCO, EFM32_CLOCK_HFCLK,  0b0100, 1, 1),

                   /* config 3: run on crystals HFXO @ 48Mhz, LFXO @ 32Khz */
                   DEV_STATIC_RES_CMU_OSC(EFM32_CLOCK_LFXO, 0b1000, 32768, 1),
                   DEV_STATIC_RES_CMU_OSC(EFM32_CLOCK_HFXO, 0b1000, 48000000, 1),
                   DEV_STATIC_RES_CMU_MUX(EFM32_CLOCK_LFXO, EFM32_CLOCK_LFACLK, 0b1000, 1, 1),
                   DEV_STATIC_RES_CMU_MUX(EFM32_CLOCK_HFXO, EFM32_CLOCK_HFCLK,  0b1000, 1, 1)
                   );

#endif


#ifdef CONFIG_DRIVER_EFM32_MSC

DEV_DECLARE_STATIC(msc_dev, "mem", 0, efm32_msc_drv,
                   DEV_STATIC_RES_MEM(0x400c0000, 0x400c0400)
                   );

#endif

#if defined(CONFIG_DRIVER_EFM32_DMA)

DEV_DECLARE_STATIC(dma_dev, "dma", 0, efm32_dma_drv,
                   DEV_STATIC_RES_MEM(0x400c2000, 0x400c4000),
# ifdef CONFIG_DEVICE_CLOCK
                   DEV_STATIC_RES_CLK_SRC("/recmu", EFM32_CLOCK_DMA, 0),
# else
                   DEV_STATIC_RES_FREQ(14000000, 1),
# endif

                   DEV_STATIC_RES_DEV_ICU("/cpu"),
                   DEV_STATIC_RES_IRQ(0, EFM32_IRQ_DMA, DEV_IRQ_SENSE_RISING_EDGE, 0, 1)
                   );

#endif

#if defined(CONFIG_DRIVER_EFM32_USART_SPI)

DEV_DECLARE_STATIC(usart1_dev, "spi1", 0, efm32_usart_spi_drv,

                   DEV_STATIC_RES_MEM(0x4000c400, 0x4000c800),
# ifdef CONFIG_DEVICE_CLOCK
                   DEV_STATIC_RES_CLK_SRC("/recmu", EFM32_CLOCK_USART1, 0),
# else
                   DEV_STATIC_RES_FREQ(14000000, 1),
# endif

                   DEV_STATIC_RES_DEV_ICU("/cpu"),
                   DEV_STATIC_RES_IRQ(0, EFM32_IRQ_USART1_RX, DEV_IRQ_SENSE_RISING_EDGE, 0, 1),

#if defined(CONFIG_DRIVER_EFM32_DMA)
                   DEV_STATIC_RES_DMA("/dma", CONFIG_DRIVER_EFM32_DMA_CHANNEL_COUNT - 1, \
                                      (EFM32_DMA_SOURCE_USART1 << 16) | EFM32_DMA_SIGNAL_USART1RXDATAV),
                   DEV_STATIC_RES_DMA("/dma", CONFIG_DRIVER_EFM32_DMA_CHANNEL_COUNT - 2, \
                                      (EFM32_DMA_SOURCE_USART1 << 16) | EFM32_DMA_SIGNAL_USART1TXEMPTY),
#endif

                   DEV_STATIC_RES_DEV_IOMUX("/gpio"),
                   DEV_STATIC_RES_IOMUX("clk",  EFM32_LOC1, EFM32_PD2, 0, 0),
                   DEV_STATIC_RES_IOMUX("miso", EFM32_LOC1, EFM32_PD1, 0, 0),
                   DEV_STATIC_RES_IOMUX("mosi", EFM32_LOC1, EFM32_PD0, 0, 0),
#if 0
                   DEV_STATIC_RES_IOMUX("cs",   EFM32_LOC1, EFM32_PD3, 0, 0),
#endif

#ifdef CONFIG_DRIVER_EFM32_RTC
                   DEV_STATIC_RES_DEV_TIMER("/rtc")
#endif
                   );

#endif


#if defined(CONFIG_DRIVER_EFM32_USART_CHAR)

DEV_DECLARE_STATIC(uart0_dev, "uart0", 0, efm32_usart_drv,
                   DEV_STATIC_RES_MEM(0x4000e000, 0x4000e400),
# ifdef CONFIG_DEVICE_CLOCK
                   DEV_STATIC_RES_CLK_SRC("/recmu", EFM32_CLOCK_UART0, 0),
# else
                   DEV_STATIC_RES_FREQ(14000000, 1),
# endif

                   DEV_STATIC_RES_DEV_ICU("/cpu"),
                   DEV_STATIC_RES_IRQ(0, EFM32_IRQ_UART0_RX, DEV_IRQ_SENSE_RISING_EDGE, 0, 1),
                   DEV_STATIC_RES_IRQ(1, EFM32_IRQ_UART0_TX, DEV_IRQ_SENSE_RISING_EDGE, 0, 1),

                   DEV_STATIC_RES_DEV_IOMUX("/gpio"),
                   DEV_STATIC_RES_IOMUX("rx", EFM32_LOC1, EFM32_PE1, 0, 0),
                   DEV_STATIC_RES_IOMUX("tx", EFM32_LOC1, EFM32_PE0, 0, 0)
                   );

#endif


#ifdef CONFIG_DRIVER_EFM32_LEUART_CHAR

DEV_DECLARE_STATIC(leuart0_dev, "leuart0", 0, efm32_leuart_drv,
                   DEV_STATIC_RES_MEM(0x40084000, 0x40084400),
# ifdef CONFIG_DEVICE_CLOCK
                   DEV_STATIC_RES_CLK_SRC("/recmu", EFM32_CLOCK_LEUART0, 0),
# else
                   DEV_STATIC_RES_FREQ(32768, 1),
# endif
                   DEV_STATIC_RES_DEV_ICU("/cpu"),
                   DEV_STATIC_RES_IRQ(0, EFM32_IRQ_LEUART0, DEV_IRQ_SENSE_RISING_EDGE, 0, 1),

                   DEV_STATIC_RES_DEV_IOMUX("/gpio"),
                   DEV_STATIC_RES_IOMUX("tx",  EFM32_LOC0, EFM32_PD4, 0, 0),
                   DEV_STATIC_RES_IOMUX("rx",  EFM32_LOC0, EFM32_PD5, 0, 0)
                   );

#endif

#ifdef CONFIG_DRIVER_EFM32_TIMER

DEV_DECLARE_STATIC(timer1_dev, "timer1", 0, efm32_timer_drv,
                   DEV_STATIC_RES_MEM(0x40010400, 0x40010800),
# ifdef CONFIG_DEVICE_CLOCK
                   DEV_STATIC_RES_CLK_SRC("/recmu", EFM32_CLOCK_TIMER1, 0),
# else
                   DEV_STATIC_RES_FREQ(14000000, 1),
# endif

                   DEV_STATIC_RES_DEV_ICU("/cpu"),
                   DEV_STATIC_RES_IRQ(0, EFM32_IRQ_TIMER1, DEV_IRQ_SENSE_RISING_EDGE, 0, 1),
                   );

#endif



#ifdef CONFIG_DRIVER_EFM32_RTC

DEV_DECLARE_STATIC(rtc_dev, "rtc", 0, efm32_rtc_drv,
                   DEV_STATIC_RES_MEM(0x40080000, 0x40080400),
# ifdef CONFIG_DEVICE_CLOCK
                   DEV_STATIC_RES_CLK_SRC("/recmu", EFM32_CLOCK_RTC, 0),
# else
                   DEV_STATIC_RES_FREQ(32768, 1),
# endif

                   DEV_STATIC_RES_DEV_ICU("/cpu"),
                   DEV_STATIC_RES_IRQ(0, EFM32_IRQ_RTC, DEV_IRQ_SENSE_RISING_EDGE, 0, 1)
                   );

#endif


#ifdef CONFIG_DRIVER_EFM32_GPIO

DEV_DECLARE_STATIC(gpio_dev, "gpio", 0, efm32_gpio_drv,
                   DEV_STATIC_RES_MEM(0x40006000, 0x40007000),
# ifdef CONFIG_DEVICE_CLOCK
                   DEV_STATIC_RES_CLK_SRC("/recmu", EFM32_CLOCK_GPIO, 0),
# endif
                   DEV_STATIC_RES_DEV_ICU("/cpu"),
                   DEV_STATIC_RES_IRQ(0, EFM32_IRQ_GPIO_EVEN, DEV_IRQ_SENSE_RISING_EDGE, 0, 1),
                   DEV_STATIC_RES_IRQ(1, EFM32_IRQ_GPIO_ODD, DEV_IRQ_SENSE_RISING_EDGE, 0, 1)
                   );

#endif

#ifdef CONFIG_DRIVER_EFM32_I2C

DEV_DECLARE_STATIC(i2c_dev, "i2c1", 0, efm32_i2c_drv,
                   DEV_STATIC_RES_MEM(0x4000a400, 0x4000a800),
# ifdef CONFIG_DEVICE_CLOCK
                   DEV_STATIC_RES_CLK_SRC("/recmu", EFM32_CLOCK_I2C1, 0),
# else
                   DEV_STATIC_RES_FREQ(14000000, 1),
# endif
                   DEV_STATIC_RES_I2C_BITRATE(100000),
                   DEV_STATIC_RES_DEV_TIMER("/rtc"),

                   DEV_STATIC_RES_DEV_ICU("/cpu"),
                   DEV_STATIC_RES_IRQ(0, EFM32_IRQ_I2C1, DEV_IRQ_SENSE_RISING_EDGE, 0, 1),

                   DEV_STATIC_RES_DEV_IOMUX("/gpio"),

                   DEV_STATIC_RES_IOMUX("scl", EFM32_LOC0, EFM32_PC5, 0, 0),
                   DEV_STATIC_RES_IOMUX("sda", EFM32_LOC0, EFM32_PC4, 0, 0)
                   );

#endif

#ifdef CONFIG_DRIVER_EFM32_PWM

DEV_DECLARE_STATIC(pwm_dev, "pwm3", 0, efm32_pwm_drv,
                   DEV_STATIC_RES_MEM(0x40010c00, 0x40011000),
# ifdef CONFIG_DEVICE_CLOCK
                   DEV_STATIC_RES_CLK_SRC("/recmu", EFM32_CLOCK_TIMER3, 0),
# else
                   DEV_STATIC_RES_FREQ(14000000, 1),
# endif

                   DEV_STATIC_RES_DEV_IOMUX("/gpio"),
                   /* led0 */
                   DEV_STATIC_RES_IOMUX("cc2", EFM32_LOC1, EFM32_PE2, 0, 0)
                   );

#endif

#ifdef CONFIG_DRIVER_EFM32_AES

DEV_DECLARE_STATIC(aes_dev, "aes", 0, efm32_aes_drv,
                   DEV_STATIC_RES_MEM(0x400e0000, 0x400e0400),
# ifdef CONFIG_DEVICE_CLOCK
                   DEV_STATIC_RES_CLK_SRC("/recmu", EFM32_CLOCK_AES, 0),
# endif
                   DEV_STATIC_RES_DEV_ICU("/cpu"),
                   DEV_STATIC_RES_IRQ(0, EFM32_IRQ_AES, DEV_IRQ_SENSE_RISING_EDGE, 0, 1)
                   );
#endif

#ifdef CONFIG_DRIVER_USB_SYNOPSYS_EFM32
DEV_DECLARE_STATIC(usb_dev, "usb", 0, efm32_usbdev_drv,
                   DEV_STATIC_RES_MEM(0x400c4000, 0x400c4400),
                   DEV_STATIC_RES_CLK_SRC("/recmu", EFM32_CLOCK_USB, 0),
                   DEV_STATIC_RES_CLK_SRC("/recmu", EFM32_CLOCK_USBC, 1),
                   DEV_STATIC_RES_DEV_PARAM("icu", "/cpu"),
                   DEV_STATIC_RES_IRQ(0, EFM32_IRQ_USB, DEV_IRQ_SENSE_RISING_EDGE, 0, 1),
                   DEV_STATIC_RES_DEV_PARAM("iomux", "/gpio"),
                   DEV_STATIC_RES_IOMUX("vbusen",  EFM32_LOC0, EFM32_PF5, 0, 0)
                   );
#endif

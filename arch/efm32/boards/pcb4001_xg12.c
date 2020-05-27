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

void efm32_board_init()
{
  uint32_t x;
  /* unlock registers */
  cpu_mem_write_32(EFM32_EMU_ADDR + EFR32_EMU_PWRLOCK_ADDR, 0xADE8);

  x = cpu_mem_read_32(EFM32_EMU_ADDR + EFR32_EMU_PWRCFG_ADDR);
  EFR32_EMU_PWRCFG_PWRCFG_SET(x, DCDCTODVDD);
  cpu_mem_write_32(EFM32_EMU_ADDR + EFR32_EMU_PWRCFG_ADDR, x);

  x = cpu_mem_read_32(EFM32_EMU_ADDR + EFR32_EMU_PWRCFG_ADDR);
  assert((x & EFR32_EMU_PWRCFG_MASK) == EFR32_EMU_PWRCFG_PWRCFG_DCDCTODVDD);

  x = cpu_mem_read_32(EFM32_EMU_ADDR + EFR32_EMU_PWRCTRL_ADDR);
  x |= EFR32_EMU_PWRCTRL_ANASW |
       EFR32_EMU_PWRCTRL_REGPWRSEL;
  cpu_mem_write_32(EFM32_EMU_ADDR + EFR32_EMU_PWRCTRL_ADDR, x);

  x = cpu_mem_read_32(EFM32_EMU_ADDR + EFR32_EMU_DCDCLNFREQCTRL_ADDR);
  EFR32_EMU_DCDCLNFREQCTRL_RCOBAND_SET(x, 4);
  cpu_mem_write_32(EFM32_EMU_ADDR + EFR32_EMU_DCDCLNFREQCTRL_ADDR, x);

  while (cpu_mem_read_32(EFM32_EMU_ADDR + EFR32_EMU_DCDCSYNC_ADDR) & 1);

  x = cpu_mem_read_32(EFM32_EMU_ADDR + EFR32_EMU_DCDCCTRL_ADDR);  
  EFR32_EMU_DCDCCTRL_DCDCMODE_SETVAL(x, 1);
  cpu_mem_write_32(EFM32_EMU_ADDR + EFR32_EMU_DCDCCTRL_ADDR, x);

  x = cpu_mem_read_32(EFM32_EMU_ADDR + EFR32_EMU_DCDCMISCCTRL_ADDR);
  EFR32_EMU_DCDCMISCCTRL_PFETCNT_SET(x, 7);
  EFR32_EMU_DCDCMISCCTRL_NFETCNT_SET(x, 7);
  cpu_mem_write_32(EFM32_EMU_ADDR + EFR32_EMU_DCDCMISCCTRL_ADDR, x);

  /* Enable GPIO clock */
  x = cpu_mem_read_32(EFM32_CMU_ADDR + EFM32_CMU_HFBUSCLKEN0_ADDR);
  x |= EFM32_CMU_HFBUSCLKEN0_GPIO;
  cpu_mem_write_32(EFM32_CMU_ADDR + EFM32_CMU_HFBUSCLKEN0_ADDR, x);

  /* Wait button to be released */
  efm32_wait_button_released();

  /* Select HFXO as HF clock */
  x = EFM32_CMU_OSCENCMD_HFXOEN;
  cpu_mem_write_32(EFM32_CMU_ADDR + EFM32_CMU_OSCENCMD_ADDR, x);

  while (!(cpu_mem_read_32(EFM32_CMU_ADDR + EFM32_CMU_STATUS_ADDR) & EFM32_CMU_STATUS_HFXORDY))
    ;

  cpu_mem_write_32(EFM32_CMU_ADDR + EFM32_CMU_HFCLKSEL_ADDR, EFM32_CMU_HFCLKSEL_HF(HFXO));

  x = cpu_mem_read_32(EFM32_CMU_ADDR + EFM32_CMU_HFCLKSTATUS_ADDR);
  assert(EFM32_CMU_HFCLKSTATUS_SELECTED_GET(x) == EFM32_CMU_HFCLKSTATUS_SELECTED_HFXO);

  /* Set PA5 high for enabling VCOM */
  x = cpu_mem_read_32(EFM32_GPIO_ADDR + EFM32_GPIO_MODEL_ADDR(0));
  EFM32_GPIO_MODEL_MODE_SET(5, x, PUSHPULL);
  cpu_mem_write_32(EFM32_GPIO_ADDR + EFM32_GPIO_MODEL_ADDR(0), x);

  x = EFM32_GPIO_DOUT_DOUT(5);
  cpu_mem_write_32(EFM32_GPIO_ADDR + EFM32_GPIO_DOUT_ADDR(0) + 0x06000000, x);

  /* Set PF4 and PF5 high for led */
  x = cpu_mem_read_32(EFM32_GPIO_ADDR + EFM32_GPIO_MODEL_ADDR(5));
  EFM32_GPIO_MODEL_MODE_SET(4, x, PUSHPULL);
  EFM32_GPIO_MODEL_MODE_SET(5, x, PUSHPULL);
  cpu_mem_write_32(EFM32_GPIO_ADDR + EFM32_GPIO_MODEL_ADDR(5), x);
  
  x = EFM32_GPIO_DOUT_DOUT(4) | EFM32_GPIO_DOUT_DOUT(5);
  cpu_mem_write_32(EFM32_GPIO_ADDR + EFM32_GPIO_DOUT_ADDR(5) + 0x06000000, x);

#if defined(CONFIG_DRIVER_EFR32_RADIO)
  x = cpu_mem_read_32(EFM32_CMU_ADDR + EFM32_CMU_CTRL_ADDR);
  x |= EFM32_CMU_CTRL_HFRADIOCLKEN;
  cpu_mem_write_32(EFM32_CMU_ADDR + EFM32_CMU_CTRL_ADDR, x);

  cpu_mem_write_32(EFM32_CMU_ADDR + EFM32_CMU_HFRADIOCLKEN0_ADDR, EFM32_CMU_HFRADIOCLKEN0_MASK);
 #if (CONFIG_EFM32_ARCHREV == EFM32_ARCHREV_EFR_XG12) ||\
     (CONFIG_EFM32_ARCHREV == EFM32_ARCHREV_EFR_XG14)
  cpu_mem_write_32(EFM32_CMU_ADDR + EFM32_CMU_HFRADIOALTCLKEN0_ADDR, EFM32_CMU_HFRADIOALTCLKEN0_MASK);
 #endif
#endif 

}

#if defined(CONFIG_DRIVER_CPU_ARM32M)

DEV_DECLARE_STATIC(cpu_dev, "cpu", DEVICE_FLAG_CPU, arm32m_drv,
                   DEV_STATIC_RES_ID(0, 0),
                   DEV_STATIC_RES_FREQ(HFXO_FREQ, 1),
                   );

#endif

#if defined(CONFIG_DRIVER_EFM32_USART_CHAR)

DEV_DECLARE_STATIC(uart0_dev, "usart0", 0, efm32_usart_drv,
                   DEV_STATIC_RES_MEM(0x40010000, 0x40010400),
                   DEV_STATIC_RES_FREQ(HFXO_FREQ, 1),

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
                   DEV_STATIC_RES_FREQ(32768, 1),

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
                   DEV_STATIC_RES_FREQ(HFXO_FREQ, 1),

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
                   DEV_STATIC_RES_FREQ(HFXO_FREQ, 1),

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
                   DEV_STATIC_RES_DEV_ICU("/cpu"),
                   DEV_STATIC_RES_IRQ(0, EFM32_IRQ_GPIO_EVEN, DEV_IRQ_SENSE_RISING_EDGE, 0, 1),
                   DEV_STATIC_RES_IRQ(1, EFM32_IRQ_GPIO_ODD, DEV_IRQ_SENSE_RISING_EDGE, 0, 1)
                   );

#endif

#ifdef CONFIG_DRIVER_EFM32_RTCC

DEV_DECLARE_STATIC(rtcc_dev, "rtcc", 0, efm32_rtcc_drv,
                   DEV_STATIC_RES_MEM(0x40042000, 0x40042400),
                   DEV_STATIC_RES_FREQ(32768, 1),
                   DEV_STATIC_RES_DEV_ICU("/cpu"),
                   DEV_STATIC_RES_IRQ(0, EFM32_IRQ_RTCC, DEV_IRQ_SENSE_RISING_EDGE, 0, 1),
                   );

#endif

#ifdef CONFIG_DRIVER_EFM32_TIMER

DEV_DECLARE_STATIC(timer1_dev, "timer0", 0, efm32_timer_drv,
                   DEV_STATIC_RES_MEM(0x40018000, 0x40018400),
                   DEV_STATIC_RES_FREQ(HFXO_FREQ, 1),
                   DEV_STATIC_RES_DEV_ICU("/cpu"),
                   DEV_STATIC_RES_IRQ(0, EFM32_IRQ_TIMER0, DEV_IRQ_SENSE_RISING_EDGE, 0, 1),
                   );

#endif

#if defined(CONFIG_DRIVER_EFR32_DMA)

DEV_DECLARE_STATIC(dma_dev, "dma", 0, efm32_dma_drv,
                   DEV_STATIC_RES_MEM(0x400e2000, 0x400e3000),
                   DEV_STATIC_RES_FREQ(HFXO_FREQ, 1),
                   DEV_STATIC_RES_DEV_ICU("/cpu"),
                   DEV_STATIC_RES_IRQ(0, EFM32_IRQ_DMA, DEV_IRQ_SENSE_RISING_EDGE, 0, 1)
                   );

#endif

#if defined(CONFIG_DRIVER_EFM32_USART_SPI)

DEV_DECLARE_STATIC(usart_dev, "spi", 0, efm32_usart_spi_drv,

  #if (CONFIG_EFM32_ARCHREV == EFM32_ARCHREV_EFR_XG1) ||\
      (CONFIG_EFM32_ARCHREV == EFM32_ARCHREV_EFR_XG14)

                   DEV_STATIC_RES_MEM(0x40010400, 0x40010800),
                   DEV_STATIC_RES_FREQ(HFXO_FREQ, 1),

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

                   DEV_STATIC_RES_MEM(0x40010800, 0x40010C00),
                   DEV_STATIC_RES_FREQ(HFXO_FREQ, 1),

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

#if defined(CONFIG_DRIVER_EFR32_SIGFOX)

static const uint32_t efr32_rf_sigfox_cfg[] = 
  0x64, // Datarate
  0x9C, // Config size
  0x40083008, 0x100ac3f,
  0x4008300c, 0x42801,
  0x40083024, 0x23,
  0x40083030, 0x10231c,
  0x40083034, 0x03,
  0x40083044, 0x7b,
  0x40083050, 0x22,
  0x4008305c, 0x07,
  0x40083064, 0x7f,
  0x4008400c, 0x380,
  0x40084024, 0x0c,
  0x40084028, 0xbc,
  0x40084030, 0x540,
  0x4008406c, 0x01,
  0x40084070, 0x01,
  0x40084074, 0x0d,
  0x40084078, 0x4a45,
  0x4008407c, 0x21000000,
  0x40084080, 0x01,
  0x40084084, 0x07,
  0x40084088, 0x40010049,
  0x4008409c, 0x3636d80,
  0x400840a0, 0xf0027aa,
  0x400840a4, 0x1147b,
  0x400840b0, 0x2e,
  0x400840b4, 0x24,
  0x400840b8, 0xf3c000,
  0x400840c0, 0x1360010,
  0x400840f4, 0x1ff057,
  0x400840f8, 0x3000000,
  0x40084110, 0x3000000,
  0x40084114, 0x186db00,
  0x40084118, 0x423fffe8,
  0x4008411c, 0x108d700,
  0x40084120, 0x7000445,
  0x40084124, 0x84523,
  0x4008412c, 0x35,
  0x40084130, 0x301e15,
  0x40084134, 0x880,
  0x40084138, 0x87f6,
  0x4008413c, 0x44400246,
  0x40084140, 0x880020,
  0x40084144, 0x4d52e6c1,
  0x40084154, 0x220022,
  0x40084158, 0xff000000,
  0x40084168, 0x06,
  0x4008416c, 0x06,
  0x40086008, 0x2a8c0000,
  0x40086014, 0x10,
  0x40086018, 0x20000c0,
  0x4008601c, 0x12c80c,
  0x40086028, 0x13000000,
  0x4008602c, 0x600,
  0x40086050, 0x1bb80,
  0x40086054, 0x841,
  0x40086058, 0x8010eb4,
  0x4008605c, 0x92,
  0x40086060, 0x1ac0,
  0x40086078, 0x22c004b,
  0x40086080, 0x2f4,
  0x4008608c, 0x3b261100,
  0x40086090, 0x7b726350,
  0x40086094, 0x7f,
  0x400860b8, 0xf00,
  0x400860bc, 0x960000,
  0x400860d8, 0x33,
  0x400860e0, 0x3f7e00a3,
  0x400860e4, 0xcc15088b,
  0x400860ec, 0x7830464,
  0x400860f0, 0x3ac81388,
  0x400860f4, 0x62090,
  0x400860f8, 0x206100,
  0x400860fc, 0x208556b7,
  0x40086104, 0x10bb3f,
  0x40086108, 0x3020,
  0x4008610c, 0xbb88,
  0x40086110, 0x02,
  0x40086124, 0x1ff,
  0x4008302c, 0xb4dc44,
  0x4008303c, 0x01,
}

static const uint32_t efr32_pk_sigfox_cfg[] = {
  0x12, // Config size
  0x40080010, 0x6f,
  0x4008002c, 0x80,
  0x40080030, 0xff,
  0x40080034, 0x02,
  0x400800a0, 0xff,
  0x400800a8, 0x1ff,
  0x40080040, 0x704,
  0x40080004, 0x00,
  0x40080008, 0xff,
}

#endif

DEV_DECLARE_STATIC(radio_dev, "rfpacket0", 0, efr32_radio_drv,
                  DEV_STATIC_RES_FREQ(HFXO_FREQ, 1),

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
                  // static config
              #if defined(CONFIG_DRIVER_EFR32_SIGFOX)
                  DEV_STATIC_RES_BLOB_PARAM("rf_sigfox", efr32_rf_sigfox_cfg),
                  DEV_STATIC_RES_BLOB_PARAM("pk_sigfox", efr32_pk_sigfox_cfg),
              #endif
                  );
#endif

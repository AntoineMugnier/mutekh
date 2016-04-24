#include <device/resources.h>
#include <device/class/iomux.h>
#include <device/class/uart.h>
#include <device/class/gpio.h>
#include <device/class/cmu.h>
#include <device/valio/adc.h>
#include <device/irq.h>
#include <arch/nrf5x/adc.h>
#include <arch/nrf5x/ids.h>

/*
  Full design available on http://nipo.ssji.net/hardware/2016-04-08-nrf51-barcode/
*/

DEV_DECLARE_STATIC(clock_dev, "clock", 0, nrf5x_clock_drv,
  // Muxes
  DEV_STATIC_RES_CMU_MUX(NRF_CLOCK_OSC_HFXO, NRF_CLOCK_SRC_HFCLK, 0b100, 1, 1),
  DEV_STATIC_RES_CMU_MUX(NRF_CLOCK_OSC_LFXO, NRF_CLOCK_SRC_LFCLK, 0b110, 1, 1),
  DEV_STATIC_RES_CMU_MUX(NRF_CLOCK_OSC_LFRC, NRF_CLOCK_SRC_LFCLK, 0b001, 1, 1),
  DEV_STATIC_RES_CMU_MUX(NRF_CLOCK_OSC_HFRC, NRF_CLOCK_SRC_HFCLK, 0b011, 1, 1),

  // Oscillators
  DEV_STATIC_RES_CMU_OSC_ACC(NRF_CLOCK_OSC_LFXO, 0b111, 32768, 1, 2, 15), // 20ppm
  DEV_STATIC_RES_CMU_OSC_ACC(NRF_CLOCK_OSC_LFRC, 0b111, 32768, 1, 2, 25), // 2%
  DEV_STATIC_RES_CMU_OSC_ACC(NRF_CLOCK_OSC_HFXO, 0b111, 32000000, 1, 7, 15), // 31ppm
  DEV_STATIC_RES_CMU_OSC_ACC(NRF_CLOCK_OSC_HFRC, 0b111, 16000000, 1, 2, 24), // 1%

  NRF_STATIC_RES_PERIPHERAL_MEM(NRF5X_CLOCK),
  DEV_STATIC_RES_DEV_ICU("/cpu"),
  DEV_STATIC_RES_IRQ(0, NRF5X_CLOCK, DEV_IRQ_SENSE_HIGH_LEVEL, 0, 1),
  );

// Barcode header IOs
DEV_DECLARE_STATIC(uart_dev, "uart", 0, nrf5x_uart_drv,
                   NRF_STATIC_RES_PERIPHERAL_MEM(NRF5X_UART0),
                   DEV_STATIC_RES_DEV_ICU("/cpu"),
                   DEV_STATIC_RES_IRQ(0, NRF5X_UART0, DEV_IRQ_SENSE_HIGH_LEVEL, 0, 1),
                   DEV_STATIC_RES_DEV_IOMUX("/gpio"),
                   DEV_STATIC_RES_UART(38400, 8, DEV_UART_PARITY_NONE, 1, 0, 0),
                   DEV_STATIC_RES_IOMUX("tx", 0, 15, 0, 0),
                   DEV_STATIC_RES_IOMUX("rx", 0, 16, 0, 0)
                   );

#if 0
// Debug port IOs
DEV_DECLARE_STATIC(uart_dev, "uart", 0, nrf5x_uart_drv,
                   NRF_STATIC_RES_PERIPHERAL_MEM(NRF5X_UART0),
                   DEV_STATIC_RES_DEV_ICU("/cpu"),
                   DEV_STATIC_RES_IRQ(0, NRF5X_UART0, DEV_IRQ_SENSE_HIGH_LEVEL, 0, 1),
                   DEV_STATIC_RES_DEV_IOMUX("/gpio"),
                   DEV_STATIC_RES_UART(19200, 8, DEV_UART_PARITY_NONE, 1, 0, 0),
                   DEV_STATIC_RES_IOMUX("tx", 0, 17, 0, 0),
                   DEV_STATIC_RES_IOMUX("rx", 0, 18, 0, 0)
                   );
#endif

DEV_DECLARE_STATIC(keyboard_dev, "keyboard", 0, button_set_drv,
                   DEV_STATIC_RES_DEV_GPIO("/gpio"),
                   DEV_STATIC_RES_DEV_ICU("/gpio"),
                   DEV_STATIC_RES_IRQ(0, NRF_GPIO_RANGE_IRQ_ID, DEV_IRQ_SENSE_HIGH_LEVEL, 0, 1),
                   DEV_STATIC_RES_IO(7, 7),
                   DEV_STATIC_RES_BLOB_PARAM("mask", dev_gpio_mask1),
                   DEV_STATIC_RES_UINT_PARAM("active", 0),
                   );

#if defined(CONFIG_NRF5X_DRIVER_GPIO_PWM)

DEV_DECLARE_STATIC(pwm_dev, "leds", 0, nrf5x_gpio_pwm_drv,
                   NRF_STATIC_RES_PERIPHERAL_MEM(NRF5X_TIMER2),
                   DEV_STATIC_RES_DEV_ICU("/cpu"),
                   DEV_STATIC_RES_IRQ(0, NRF5X_TIMER2, DEV_IRQ_SENSE_HIGH_LEVEL, 0, 1),
                   DEV_STATIC_RES_DEV_IOMUX("/gpio"),
                   DEV_STATIC_RES_IOMUX("_p0", 0, 9, 0, 0),
                   DEV_STATIC_RES_IOMUX("_p1", 0, 10, 0, 0),
                   DEV_STATIC_RES_IOMUX("_p2", 0, 11, 0, 0),
                   );
#endif

DEV_DECLARE_STATIC(adc_dev, "adc", 0, nrf5x_adc_drv,
                   NRF_STATIC_RES_PERIPHERAL_MEM(NRF5X_ADC),
                   DEV_STATIC_RES_DEV_ICU("/cpu"),
                   DEV_STATIC_RES_IRQ(0, NRF5X_ADC, DEV_IRQ_SENSE_HIGH_LEVEL, 0, 1),
                   DEV_STATIC_RES_UINT_ARRAY_PARAM("config",
                                                   0,
                                                   0,
                                                   0,
                                                   0,
                                                   0,
                                                   // Input is ISense * 1800 / 400.
                                                   // So, with 1.2V reference and 1/3 prescaling, we have .78mA per unit
                                                   NRF_ADC_CONFIG_RES_10BIT
                                                   | NRF_ADC_CONFIG_INPSEL_1_3
                                                   | NRF_ADC_CONFIG_REFSEL_1_2V,
                                                   0,
                                                   // Input is VBat * .196
                                                   // So, with 1.2V reference and no prescaling,
                                                   // we have 0 -> 3.7V in 0 -> 619 range, and 3.4V = 569
                                                   // then ((ADC_OUT - 569) * 2) maps 100 percents from 3.4V to 3.7V
                                                   NRF_ADC_CONFIG_RES_10BIT
                                                   | NRF_ADC_CONFIG_INPSEL_RAW
                                                   | NRF_ADC_CONFIG_REFSEL_1_2V),
                   );

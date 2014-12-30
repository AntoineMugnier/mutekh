objs-$(CONFIG_DRIVER_NRF51_PPI) += ppi.o
objs-$(CONFIG_DRIVER_NRF51_GPIO) += gpio.o
objs-$(CONFIG_DRIVER_NRF51_UART) += uart.o
objs-$(CONFIG_DRIVER_NRF51_PRINTK) += printk.o
objs-$(CONFIG_DRIVER_NRF51_SPI) += spi.o
objs-$(CONFIG_DRIVER_NRF51_I2C) += i2c.o
objs-$(CONFIG_DRIVER_NRF51_RTC) += rtc.o
ifneq ($(CONFIG_DRIVER_NRF51_NVMC)$(CONFIG_DRIVER_NRF51_PERSIST),undefinedundefined)
objs += nvmc.o
endif
objs-$(CONFIG_DRIVER_NRF51_RAM) += ram.o
objs-$(CONFIG_DRIVER_NRF51_TIMER) += timer.o
objs += clock.o
objs-$(CONFIG_DRIVER_NRF51_BLE_RADIO) += ble_radio.o ble_radio_handler.o
objs-$(CONFIG_DRIVER_NRF51_AES) += aes.o
objs-$(CONFIG_DRIVER_NRF51_RNG) += rng.o
objs-$(CONFIG_DRIVER_NRF51_ADC) += adc.o

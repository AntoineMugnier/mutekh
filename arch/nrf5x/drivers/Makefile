objs += clock.o
objs-$(CONFIG_DRIVER_NRF5X_GPIO) += gpio.o
objs-$(CONFIG_DRIVER_NRF5X_GPIO_PWM) += gpio_pwm.o
objs-$(CONFIG_DRIVER_NRF5X_UART) += uart.o
objs-$(CONFIG_DRIVER_NRF5X_PRINTK) += printk.o
objs-$(CONFIG_DRIVER_NRF5X_SPI) += spi.o
objs-$(CONFIG_DRIVER_NRF5X_I2C) += i2c.o
objs-$(CONFIG_DRIVER_NRF5X_RTC) += rtc.o
objs += nvmc.o
objs-$(CONFIG_DRIVER_NRF5X_RAM) += ram.o
objs-$(CONFIG_DRIVER_NRF5X_TIMER) += timer.o
objs-$(CONFIG_DRIVER_NRF5X_BLE_RADIO) += ble_radio.o ble_radio_handler.o
objs-$(CONFIG_DRIVER_NRF5X_RADIO) += radio.o
objs-$(CONFIG_DRIVER_NRF5X_AES) += aes.o
objs-$(CONFIG_DRIVER_NRF5X_RNG) += rng.o
objs-$(CONFIG_DRIVER_NRF5X_ADC) += adc.o

ifeq ($(CONFIG_DRIVER_NRF5X_BLE), defined)
objs += ble.o ble_primitives.o
objs-$(CONFIG_BLE_PERIPHERAL) += ble_advertiser.o ble_slave.o
objs-$(CONFIG_BLE_CENTRAL) += ble_master.o
endif

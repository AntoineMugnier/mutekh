ifeq ($(CONFIG_DRIVER_NRF5X_CLOCK), defined)
objs += clock.o
else
objs += clock_nodrv.o
endif

objs-$(CONFIG_DRIVER_NRF5X_GPIO) += gpio.o
objs-$(CONFIG_DRIVER_NRF5X_GPIO_PWM) += gpio_pwm.o
objs-$(CONFIG_DRIVER_NRF5X_UART) += uart.o
objs-$(CONFIG_DRIVER_NRF52_UARTE) += uarte.o
objs-$(CONFIG_DRIVER_NRF5X_PRINTK) += printk.o
objs-$(CONFIG_DRIVER_NRF5X_SPI) += spi.o
objs-$(CONFIG_DRIVER_NRF52_SPIM) += spim.o
objs-$(CONFIG_DRIVER_NRF5X_I2C) += i2c.o
objs-$(CONFIG_DRIVER_NRF5X_RTC) += rtc.o
objs-$(CONFIG_DRIVER_NRF5X_NVMC) += nvmc.o
objs-$(CONFIG_DRIVER_NRF5X_TIMER) += timer.o
objs-$(CONFIG_DRIVER_NRF5X_AES) += aes.o
objs-$(CONFIG_DRIVER_NRF5X_RNG) += rng.o
objs-$(CONFIG_DRIVER_NRF5X_ADC) += adc.o
objs-$(CONFIG_DRIVER_NRF52_SAADC) += saadc.o
objs-$(CONFIG_DRIVER_NRF52_PDM) += pdm.o
objs-$(CONFIG_DRIVER_NRF52_IR_BLASTER) += ir_blaster.o

#local
objs-$(CONFIG_DRIVER_NRF52_NFCT) += nfct.o

#distant (merged default)
objs-$(CONFIG_DRIVER_NRF52_USBD) += usbd.o

ifeq ($(CONFIG_DRIVER_NRF5X_BLE), defined)
objs += ble.o ble_primitives.o
objs-$(CONFIG_BLE_PERIPHERAL) += ble_advertiser.o ble_slave.o
objs-$(CONFIG_BLE_CENTRAL) += ble_scanner.o ble_master.o
endif

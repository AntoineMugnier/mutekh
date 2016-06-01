objs += imo.o
ifeq ($(CONFIG_DRIVER_PSOC4_CLOCK),defined)
objs += clock.o
else
objs += clock_nodrv.o
endif
objs-$(CONFIG_DRIVER_PSOC4_PERI) += peri.o
objs-$(CONFIG_DRIVER_PSOC4_PCLK) += pclk.o
objs-$(CONFIG_DRIVER_PSOC4_PRINTK) += printk.o
objs-$(CONFIG_DRIVER_PSOC4_UART) += uart.o
objs-$(CONFIG_DRIVER_PSOC4_GPIO) += gpio.o
objs-$(CONFIG_DRIVER_PSOC4_RTC) += rtc.o
objs-$(CONFIG_DRIVER_PSOC4_BLE) += ble.o

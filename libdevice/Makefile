
DIR_CFLAGS = -D_HEXO_INTTYPES_DEPRECATED -DLOGK_MODULE_ID='"dev "'

enum_headers = include/device/types.h \
               include/device/driver.h \
               include/device/device.h \
               include/device/class/mem.h \
               include/device/class/pwm.h \
               include/device/class/crypto.h \
               include/device/class/timer.h \
               include/device/class/rfpacket.h \
               include/device/class/smi.h \
               include/device/class/spi.h \
               include/device/class/usbdev.h \
               include/device/usb/usb.h \
               include/device/resource/uart.h \
               include/device/valio/hwclock.h

objs = driver.o device.o resources.o init.o request.o

objs-$(CONFIG_DEVICE_IRQ) += irq.o
objs-$(CONFIG_DEVICE_SLEEP) += sleep.o

objs-$(CONFIG_DEVICE_CPU) += device_cpu.o

ifeq ($(CONFIG_DEVICE_ENUM), defined)
objs += device_enum.o
objs-$(CONFIG_DEVICE_SHELL) += shell_enum.o
endif

ifeq ($(CONFIG_DEVICE_CHAR), defined)
objs += device_char.o
objs-$(CONFIG_DEVICE_SHELL_CHAR) += shell_char.o
endif

ifeq ($(CONFIG_DEVICE_TIMER), defined)
objs += device_timer.o
objs-$(CONFIG_DEVICE_SHELL_TIMER) += shell_timer.o
endif

ifeq ($(CONFIG_DEVICE_I2C), defined)
objs += device_i2c.o
objs-$(CONFIG_DEVICE_SHELL_I2C) += shell_i2c.o
endif

ifeq ($(CONFIG_DEVICE_SPI), defined)
objs += device_spi.o
objs-$(CONFIG_DEVICE_SHELL_SPI) += shell_spi.o
endif

ifeq ($(CONFIG_DEVICE_SMI), defined)
objs += device_smi.o
objs-$(CONFIG_DEVICE_SHELL_SMI) += shell_smi.o
endif

objs-$(CONFIG_DEVICE_IOMUX) += device_iomux.o

ifeq ($(CONFIG_DEVICE_RFPACKET), defined)
objs += device_rfpacket.o
objs-$(CONFIG_DEVICE_SHELL_RFPACKET) += shell_rfpacket.o
endif

ifeq ($(CONFIG_DEVICE_GPIO), defined)
objs += device_gpio.o
objs-$(CONFIG_DEVICE_SHELL_GPIO) += shell_gpio.o
endif

ifeq ($(CONFIG_DEVICE_CLOCK), defined)
objs += device_clock.o
objs-$(CONFIG_DEVICE_SHELL_CLOCK) += shell_clock.o
endif

ifeq ($(CONFIG_DEVICE_MEM), defined)
objs += device_mem.o
objs-$(CONFIG_DEVICE_SHELL_MEM) += shell_mem.o
endif

objs-$(CONFIG_DEVICE_SHELL) += shell.o

ifeq ($(CONFIG_DEVICE_PWM), defined)
objs += device_pwm.o
objs-$(CONFIG_DEVICE_SHELL_PWM) += shell_pwm.o
endif

ifeq ($(CONFIG_DEVICE_CRYPTO), defined)
objs += device_crypto.o
objs-$(CONFIG_DEVICE_SHELL_CRYPTO) += shell_crypto.o
endif

objs-$(CONFIG_DEVICE_USBDEV) += device_usbdev.o device_usbdev_helpers.o

ifeq ($(CONFIG_DEVICE_VALIO), defined)
objs += device_valio.o
endif

objs += device_uart.o

ifeq ($(CONFIG_DEVICE_VALIO_UART_CONFIG), defined)
objs-$(CONFIG_DEVICE_SHELL) += shell_uart.o
endif

ifeq ($(CONFIG_DEVICE_VALIO_HWCLOCK), defined)
objs += device_hwclock.o
objs-$(CONFIG_DEVICE_SHELL) += shell_hwclock.o
endif

ifeq ($(CONFIG_DEVICE_VALIO_LED), defined)
objs-$(CONFIG_DEVICE_SHELL) += shell_led.o
endif

ifeq ($(CONFIG_DEVICE_VALIO_MOTION_SENSOR), defined)
objs-$(CONFIG_DEVICE_SHELL) += shell_motion_sensor.o
endif


DIR_CFLAGS = -D_HEXO_INTTYPES_DEPRECATED

enum_headers = include/device/types.h \
               include/device/driver.h \
               include/device/device.h \
               include/device/class/mem.h \
               include/device/class/pwm.h \
               include/device/class/crypto.h \
               include/device/class/timer.h \
               include/device/class/rfpacket.h \
               include/device/class/persist.h \
               include/device/class/uart.h \
               include/device/class/spi.h

objs = driver.o device.o resources.o init.o request.o

objs-$(CONFIG_DEVICE_IRQ) += irq.o
objs-$(CONFIG_DEVICE_SLEEP) += sleep.o

ifeq ($(CONFIG_DEVICE_ENUM), defined)
objs += device_enum.o
objs-$(CONFIG_DEVICE_SHELL) += shell_enum.o
endif

ifeq ($(CONFIG_DEVICE_CHAR), defined)
objs += device_char.o
objs-$(CONFIG_DEVICE_SHELL) += shell_char.o
endif

ifeq ($(CONFIG_DEVICE_TIMER), defined)
objs += device_timer.o
objs-$(CONFIG_DEVICE_SHELL) += shell_timer.o
endif

objs-$(CONFIG_DEVICE_DMA) += device_dma.o
objs-$(CONFIG_DEVICE_BLOCK) += device_block.o

ifeq ($(CONFIG_DEVICE_I2C), defined)
objs += device_i2c.o
objs-$(CONFIG_DEVICE_SHELL) += shell_i2c.o
endif

ifeq ($(CONFIG_DEVICE_SPI), defined)
objs += device_spi.o
objs-$(CONFIG_DEVICE_SHELL) += shell_spi.o
endif

objs-$(CONFIG_DEVICE_LCD) += device_lcd.o
objs-$(CONFIG_DEVICE_IOMUX) += device_iomux.o

ifeq ($(CONFIG_DEVICE_RFPACKET), defined)
objs += device_rfpacket.o
objs-$(CONFIG_DEVICE_SHELL) += shell_rfpacket.o
endif

ifeq ($(CONFIG_DEVICE_GPIO), defined)
objs += device_gpio.o
objs-$(CONFIG_DEVICE_SHELL) += shell_gpio.o
endif

ifeq ($(CONFIG_DEVICE_CLOCK), defined)
objs += device_clock.o
objs-$(CONFIG_DEVICE_SHELL) += shell_clock.o
endif

ifeq ($(CONFIG_DEVICE_MEM), defined)
objs += device_mem.o
objs-$(CONFIG_DEVICE_SHELL) += shell_mem.o
endif

objs-$(CONFIG_DEVICE_SHELL) += shell.o

ifeq ($(CONFIG_DEVICE_PWM), defined)
objs += device_pwm.o
objs-$(CONFIG_DEVICE_SHELL) += shell_pwm.o
endif

ifeq ($(CONFIG_DEVICE_CRYPTO), defined)
objs += device_crypto.o
objs-$(CONFIG_DEVICE_SHELL) += shell_crypto.o
endif

ifeq ($(CONFIG_DEVICE_UART), defined)
objs += device_uart.o
objs-$(CONFIG_DEVICE_SHELL) += shell_uart.o
endif

ifeq ($(CONFIG_DEVICE_PERSIST), defined)
objs += device_persist.o
endif

objs-$(CONFIG_DEVICE_USBDEV) += device_usbdev.o device_usbdev_helpers.o

ifeq ($(CONFIG_DEVICE_VALIO), defined)
objs += device_valio.o
endif


DIR_CFLAGS = -D_HEXO_INTTYPES_DEPRECATED

doc_headers =  device/device.h  device/driver.h device/irq.h device/resources.h \
	device/class/block.h device/class/char.h			\
	device/class/enum.h device/class/gpio.h			\
	device/class/icu.h device/class/net.h device/class/cpu.h	\
	device/class/timer.h device/class/dma.h \
    device/class/i2c.h device/class/spi.h \
    device/class/uart.h device/class/pwm.h \
    device/class/mem.h \
    device/class/valio.h

enum_headers = device/driver.h device/device.h device/class/mem.h \
               device/class/pwm.h device/class/crypto.h device/class/timer.h

objs = driver.o device.o resources.o init.o request.o

objs-$(CONFIG_DEVICE_IRQ) += irq.o
objs-$(CONFIG_DEVICE_TREE) += enum_root.o
objs-$(CONFIG_DEVICE_CHAR) += device_char.o

ifeq ($(CONFIG_DEVICE_TIMER), defined)
objs += device_timer.o
objs-$(CONFIG_DEVICE_SHELL) += shell_timer.o
endif

objs-$(CONFIG_DEVICE_DMA) += device_dma.o
objs-$(CONFIG_DEVICE_BLOCK) += device_block.o
objs-$(CONFIG_DEVICE_I2C) += device_i2c.o
objs-$(CONFIG_DEVICE_SPI) += device_spi.o
objs-$(CONFIG_DEVICE_LCD) += device_lcd.o
objs-$(CONFIG_DEVICE_GPIO) += device_gpio.o
objs-$(CONFIG_DEVICE_IOMUX) += device_iomux.o
objs-$(CONFIG_DEVICE_RFPACKET) += device_rfpacket.o

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


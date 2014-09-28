
DIR_CFLAGS = -D_HEXO_INTTYPES_DEPRECATED

doc_headers =  device/device.h  device/driver.h device/irq.h device/resources.h \
	device/class/block.h device/class/char.h			\
	device/class/enum.h device/class/gpio.h			\
	device/class/icu.h device/class/net.h device/class/cpu.h	\
	device/class/timer.h device/class/dma.h

objs = driver.o device.o dump.o resources.o init.o

ifeq ($(CONFIG_DEVICE_IRQ), defined)
 objs += irq.o
endif

ifeq ($(CONFIG_DEVICE_TREE), defined)
 objs += enum_root.o
endif

ifeq ($(CONFIG_DEVICE_CHAR), defined)
 objs += device_char.o
endif

ifeq ($(CONFIG_DEVICE_TIMER), defined)
objs += device_timer.o
ifeq ($(CONFIG_DEVICE_SHELL), defined)
objs += shell_timer.o
endif
endif

ifeq ($(CONFIG_DEVICE_DMA), defined)
 objs += device_dma.o
endif

ifeq ($(CONFIG_DEVICE_BLOCK), defined)
objs += device_block.o
endif

ifeq ($(CONFIG_DEVICE_I2C), defined)
objs += device_i2c.o
endif

ifeq ($(CONFIG_DEVICE_SPI), defined)
objs += device_spi.o
endif

ifeq ($(CONFIG_DEVICE_LCD), defined)
objs += device_lcd.o
endif

ifeq ($(CONFIG_DEVICE_GPIO), defined)
objs += device_gpio.o
endif

ifeq ($(CONFIG_DEVICE_IOMUX), defined)
objs += device_iomux.o
endif

ifeq ($(CONFIG_DEVICE_CLOCK), defined)
objs += device_clock.o
ifeq ($(CONFIG_DEVICE_SHELL), defined)
objs += shell_clock.o
endif
endif

ifeq ($(CONFIG_DEVICE_SHELL), defined)
objs += shell.o
endif



DIR_CFLAGS = -D_HEXO_INTTYPES_DEPRECATED

doc_headers =  device/device.h  device/driver.h device/irq.h \
	device/class/block.h device/class/char.h			\
	device/class/enum.h device/class/fb.h device/class/gpio.h device/class/i2c.h			\
	device/class/icu.h device/class/input.h device/class/lcd.h device/class/mem.h device/class/net.h	\
	device/class/sound.h device/class/spi.h device/class/timer.h

objs = driver.o device.o dump.o resources.o

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


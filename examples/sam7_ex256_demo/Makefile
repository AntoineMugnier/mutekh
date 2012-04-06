
objs = main.o bitmap.o block.o joystick.o i2c.o

ifeq ($(CONFIG_DEVICE_LCD),defined)
objs += lcd.o
endif

ifeq ($(CONFIG_MUTEK_TIMER),defined)
objs += timer.o
endif


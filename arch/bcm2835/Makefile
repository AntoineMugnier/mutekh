objs = arch_init.o
objs-$(CONFIG_MUTEK_PRINTK) += early_console.o

ifeq ($(findstring RASPBERRY_,$(CONFIG_ARCH_BCM2835_BOARD)), RASPBERRY_)
objs += dev_raspberry.o
endif

meta = ldscript

subdirs = drivers



objs = arch_init.o device_ids.o

ifneq ($(CONFIG_GAISLER_PRINTK),undefined)
objs += early_console.o
endif

ifeq ($(LINKING),1)
meta = ldscript
endif

subdirs = drivers

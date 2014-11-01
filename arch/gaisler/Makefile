objs = arch_init.o device_ids.o
objs-$(CONFIG_GAISLER_PRINTK) += early_console.o

ifeq ($(LINKING),1)
meta = ldscript
endif

subdirs = drivers

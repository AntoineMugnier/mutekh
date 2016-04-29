objs = arch_init.o device_ids.o power.o
objs-$(CONFIG_GAISLER_PRINTK) += printk.o

ifeq ($(LINKING),1)
meta = ldscript
endif

subdirs = drivers
enum_headers = include/arch/gaisler/device_ids.h


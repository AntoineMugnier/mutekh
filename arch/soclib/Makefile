
objs = 											\
arch_init.o										\
power.o

ifneq ($(CONFIG_SOCLIB_PRINTK),undefined)
objs += early_console.o
endif

ifeq ($(CONFIG_HEXO_MMU), defined)
objs += mmu.o
endif

ifeq ($(LINKING),1)
meta = ldscript
endif

subdirs = drivers

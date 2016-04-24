
objs = 											\
arch_init.o										\
power.o

objs-$(CONFIG_SOCLIB_PRINTK) += printk.o
objs-$(CONFIG_HEXO_MMU) += mmu.o

ifeq ($(LINKING),1)
meta = ldscript
endif

subdirs = drivers

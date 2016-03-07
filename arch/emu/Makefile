objs =											\
arch_init.o										\
power.o \
syscalls.o

objs-$(CONFIG_ARCH_EMU_DARWIN) += darwin_before.o darwin_after.o
objs-$(CONFIG_HEXO_IRQ) += interrupts.o
subdirs = include/arch/hexo drivers

ifeq ($(LINKING),1)
meta = ldscript
endif


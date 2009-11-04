
objs =											\
interrupts.o										\
boot.o											\
cpu_init.o										\
cpu_context.o

ifeq ($(CONFIG_HEXO_MMU), defined)
objs += mmu.o
endif

subdirs = libc libnetwork libelf

copy = ldscript

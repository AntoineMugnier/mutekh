
objs =											\
interrupts.o										\
boot.o											\
cpu_init.o										\
cpu_context.o

ifeq ($(CONFIG_HEXO_VMEM), defined)
objs += vmem.o
endif

subdirs = libc libnetwork


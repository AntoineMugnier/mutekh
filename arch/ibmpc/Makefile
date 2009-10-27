
objs =											\
arch_init.o										\
mem_alloc.o \
hw_init.o

ifeq ($(CONFIG_ARCH_IBMPC_DMA), defined)
objs += dma-8237.o
endif

meta = ldscript

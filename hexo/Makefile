
objs =						\
cpu.o						\
init.o						\
context.o					\
mem_alloc.o					\
device.o					

ifeq ($(CONFIG_COMPILE_INSTRUMENT), defined)
objs += instrument.o
endif

ifeq ($(CONFIG_HEXO_IPI), defined)
objs += ipi.o
endif


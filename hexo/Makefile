
objs =											\
context.o										\
mem_alloc.o										\
device.o

ifeq ($(CONFIG_HEXO_VMEM), defined)
objs += vmem_paddr.o
endif

ifeq ($(CONFIG_COMPILE_INSTRUMENT), defined)
objs += instrument.o
endif

ifeq ($(CONFIG_HEXO_SCHED), defined)
objs += scheduler.o
endif




objs = timer.o

ifeq ($(CONFIG_MUTEK_MAIN), defined)
 objs += main.o
endif

ifeq ($(CONFIG_MUTEK_LOGO), defined)
 objs += mutek_logo_320x200.o
endif

ifeq ($(CONFIG_VMEM_PHYS_ALLOC), defined)
objs += page_alloc.o
endif

ifeq ($(CONFIG_MUTEK_SCHEDULER), defined)
objs += scheduler.o
endif

ifeq ($(CONFIG_VMEM_KERNEL_ALLOC), defined)
objs += vmem_kalloc.o
endif

ifeq ($(CONFIG_MUTEK_RWLOCK), defined)
objs += rwlock.o
endif



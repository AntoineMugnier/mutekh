
doc_headers = mutek/mem_alloc.h mutek/page_alloc.h mutek/printf_arg.h	\
	mutek/printk.h mutek/rwlock.h mutek/scheduler.h mutek/timer.h	\
	mutek/vmem_kalloc.h mutek/semaphore.h

objs = timer.o
objs += printf_arg.o

ifeq ($(CONFIG_MUTEK_MEM_REGION), defined)
objs += mem_region.o					
endif

ifeq ($(CONFIG_MUTEK_MEMALLOC_SMART), defined)
objs += memory_allocator_smart.o					
endif

ifeq ($(CONFIG_MUTEK_MEMALLOC_SIMPLE), defined)
objs += memory_allocator_simple.o					
endif

ifeq ($(CONFIG_COMPILE_INSTRUMENT), defined)
objs += instrument.o
endif

ifeq ($(CONFIG_MUTEK_PRINTK), defined)
objs += printk.o
endif

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
objs += scheduler.o semaphore.o
endif

ifeq ($(CONFIG_VMEM_KERNEL_ALLOC), defined)
objs += vmem_kalloc.o
endif

ifeq ($(CONFIG_MUTEK_RWLOCK), defined)
objs += rwlock.o
endif



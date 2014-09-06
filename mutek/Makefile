
doc_headers = mutek/bytecode.h mutek/console.h mutek/fileops.h  \
        mutek/kroutine.h mutek/mem_alloc.h mutek/mem_region.h   \
        mutek/memory_allocator.h mutek/printk.h mutek/rwlock.h  \
        mutek/scheduler.h mutek/semaphore.h mutek/startup.h

DIR_CFLAGS = -D_HEXO_INTTYPES_DEPRECATED

objs += fault.o startup.o

ifeq ($(CONFIG_MUTEK_CONSOLE), defined)
objs += console.o
endif

ifeq ($(CONFIG_MUTEK_SHELL), defined)
objs += shell.o
endif

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

ifeq ($(CONFIG_MUTEK_LOGO), defined)
 objs += mutek_logo_320x200.o
endif

ifeq ($(CONFIG_VMEM_PHYS_ALLOC), defined)
objs += page_alloc.o
endif

ifeq ($(CONFIG_MUTEK_SCHEDULER), defined)
objs += scheduler.o

 ifeq ($(CONFIG_MUTEK_SEMAPHORE), defined)
  objs += semaphore.o
 endif

 ifeq ($(CONFIG_MUTEK_RWLOCK), defined)
  objs += rwlock.o
 endif
endif

ifeq ($(CONFIG_VMEM_KERNEL_ALLOC), defined)
objs += vmem_kalloc.o
endif

ifeq ($(CONFIG_MUTEK_SLAB), defined)
 objs += slab.o
endif

ifeq ($(CONFIG_MUTEK_BUFFER_POOL), defined)
 objs += buffer_pool.o
endif

ifeq ($(CONFIG_MUTEK_BYTECODE), defined)
 objs += bytecode.o
endif


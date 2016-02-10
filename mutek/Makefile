
DIR_CFLAGS = -D_HEXO_INTTYPES_DEPRECATED

objs += fault.o startup.o mem_alloc.o kroutine.o

objs-$(CONFIG_MUTEK_CONSOLE) += console.o
objs-$(CONFIG_MUTEK_SHELL) += shell.o shell_mem.o
objs-$(CONFIG_MUTEK_MEM_REGION) += mem_region.o
objs-$(CONFIG_MUTEK_MEMALLOC_SMART) += memory_allocator_smart.o
objs-$(CONFIG_MUTEK_MEMALLOC_SIMPLE) += memory_allocator_simple.o
objs-$(CONFIG_COMPILE_INSTRUMENT) += instrument.o
objs-$(CONFIG_MUTEK_PRINTK_HANDLER) += printk.o
objs-$(CONFIG_MUTEK_LOGO) += mutek_logo_320x200.o
objs-$(CONFIG_VMEM_PHYS_ALLOC) += page_alloc.o

objs-$(CONFIG_MUTEK_SCHED) += scheduler.o
objs-$(CONFIG_MUTEK_THREAD) += thread.o
objs-$(CONFIG_MUTEK_SEMAPHORE) += semaphore.o
objs-$(CONFIG_MUTEK_RWLOCK) += rwlock.o

objs-$(CONFIG_VMEM_KERNEL_ALLOC) += vmem_kalloc.o
objs-$(CONFIG_MUTEK_SLAB) += slab.o
objs-$(CONFIG_MUTEK_BUFFER_POOL) += buffer_pool.o
objs-$(CONFIG_MUTEK_BYTECODE) += bytecode.o

objs-$(CONFIG_MUTEK_INSTRUMENTATION) += instrumentation.o

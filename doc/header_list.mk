
HEXO_HEADER= -I hexo/include hexo/atomic.h hexo/cpu.h hexo/ipi.h	\
	hexo/mmu.h hexo/interrupt.h hexo/local.h hexo/error.h		\
	hexo/endian.h hexo/context.h hexo/iospace.h hexo/segment.h	\
	hexo/init.h hexo/lock.h hexo/types.h hexo/vmem.h hexo/alloc.h	\
	hexo/device.h

LIBC_HEADER= -I libc/include sys/stat.h fileops.h ctype.h unistd.h	\
	semaphore.h fcntl.h errno.h stdio.h math.h time.h stdarg.h	\
	stddef.h stdint.h stdlib.h setjmp.h string.h assert.h limits.h

PTHREAD_HEADER= -I libpthread/include pthread.h

MUTEK_HEADER= -I mutek/include mutek/timer.h mutek/page_alloc.h	\
	mutek/vmem_kalloc.h mutek/rwlock.h mutek/printk.h	\
	mutek/scheduler.h mutek/printf_arg.h

ARCH_HEADER= arch/hexo/alloc.h arch/hexo/atomic.h arch/hexo/lock.h	\
	arch/hexo/segment.h arch/hexo/types.h

CPU_HEADER= cpu/hexo/atomic.h cpu/hexo/iospace.h cpu/hexo/local.h	\
	cpu/hexo/types.h

#	-I doc/include
#	doc/include/cpu/hexo/types.h
#	doc/include/arch/hexo/types.h


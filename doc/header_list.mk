
BASE_MODULES += libsrl libfdt libpthread libm

ARCH_HEADER= arch/mem_alloc.h arch/hexo/atomic.h arch/hexo/lock.h	\
	arch/hexo/segment.h arch/hexo/types.h

CPU_HEADER= cpu/hexo/atomic.h cpu/hexo/iospace.h cpu/hexo/local.h	\
	cpu/hexo/types.h

#	-I doc/include
#	doc/include/cpu/hexo/types.h
#	doc/include/arch/hexo/types.h


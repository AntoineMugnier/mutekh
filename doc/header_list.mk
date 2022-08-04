
BASE_MODULES += libfdt libpthread libm libvfs hexo mutek libc   \
libdevice libnetwork libinet libble libpersist libgfx libvulpis

ARCH_HEADER= arch/hexo/atomic.h arch/hexo/lock.h	\
	arch/hexo/types.h

CPU_HEADER= cpu/hexo/atomic.h cpu/hexo/iospace.h cpu/hexo/local.h	\
	cpu/hexo/types.h

MKDOC_ARGS += -I arch/nrf5x/include

#	-I doc/include
#	doc/include/cpu/hexo/types.h
#	doc/include/arch/hexo/types.h


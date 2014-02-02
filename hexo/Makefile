
DIR_CFLAGS = -D_HEXO_INTTYPES_DEPRECATED

objs = cpu.o context.o interrupt.o init.o

ifeq ($(CONFIG_HEXO_IPI), defined)
objs += ipi.o
endif

doc_headers = hexo/asm.h hexo/atomic.h hexo/context.h hexo/cpu.h        \
        hexo/decls.h hexo/endian.h hexo/error.h hexo/interrupt.h        \
        hexo/iospace.h hexo/ipi.h hexo/local.h hexo/lock.h              \
        hexo/ordering.h hexo/types.h

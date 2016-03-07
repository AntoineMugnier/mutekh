
DIR_CFLAGS = -D_HEXO_INTTYPES_DEPRECATED

objs =						\
string.o					\
alloc.o						\
intconv.o					\
rand.o						\
error.o						\
qsort.o						\
stdlib.o					\
unistd.o


objs-$(CONFIG_LIBC_TIME) += time.o
objs-$(CONFIG_LIBC_ASSERT) += assert.o
objs-$(CONFIG_LIBC_STREAM) += stdio.o
objs-$(CONFIG_LIBC_PRINTF) += printf.o
objs-$(CONFIG_LIBC_SCANF) += scanf.o
objs-$(CONFIG_LIBC_FORMATTER) += formatter.o
objs-$(CONFIG_LIBC_FORMATTER_FLOAT) += formatter_float.o


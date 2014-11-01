
DIR_CFLAGS = -D_HEXO_INTTYPES_DEPRECATED

objs =						\
string.o					\
alloc.o						\
intconv.o					\
printf.o					\
scanf.o	        				\
rand.o						\
error.o						\
qsort.o						\
stdlib.o					\
unistd.o					\
formatter.o


objs-$(CONFIG_LIBC_TIME) += time.o
objs-$(CONFIG_LIBC_ASSERT) += assert.o
objs-$(CONFIG_LIBC_STREAM) += stdio.o
objs-$(CONFIG_LIBC_FORMATTER_FLOAT) += formatter_float.o

doc_headers = alloca.h assert.h ctype.h errno.h fcntl.h limits.h        \
	semaphore.h setjmp.h stdarg.h stdbool.h stddef.h stdint.h       \
	stdio.h stdlib.h string.h time.h unistd.h sys/stat.h            \
	libc/formatter.h

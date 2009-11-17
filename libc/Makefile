objs =						\
string.o					\
alloc.o						\
intconv.o					\
printf.o					\
rand.o						\
error.o						\
stdlib.o					\
unistd.o

ifeq ($(CONFIG_LIBC_ASSERT), defined)
objs += assert.o
endif

ifeq ($(CONFIG_LIBC_STREAM), defined)
objs += stdio.o
endif

ifeq ($(CONFIG_LIBC_STREAM_STD), defined)
objs += stdio_in_out_err.o
endif

doc_headers = \
	sys/stat.h fileops.h ctype.h unistd.h	\
	semaphore.h fcntl.h errno.h stdio.h time.h stdarg.h	\
	stddef.h stdint.h stdlib.h setjmp.h string.h assert.h limits.h

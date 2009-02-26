objs =						\
string.o					\
alloc.o						\
intconv.o					\
printf.o					\
rand.o						\
error.o						\
stdlib.o					\
unistd.o

ifeq ($(CONFIG_LIBC_SEMAPHORE), defined)
objs += sem.o
endif

ifeq ($(CONFIG_LIBC_ASSERT), defined)
objs += assert.o
endif

ifeq ($(CONFIG_LIBC_STREAM), defined)
objs += stdio.o
endif

ifeq ($(CONFIG_LIBC_STREAM_STD), defined)
objs += stdio_in_out_err.o
endif


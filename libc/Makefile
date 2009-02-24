objs = 											\
string.o										\
alloc.o											\
intconv.o										\
printf.o										\
rand.o											\
timer.o

ifeq ($(CONFIG_LIBC_SEMAPHORE), defined)
objs += sem.o
endif

ifeq ($(CONFIG_LIBC_ASSERT), defined)
objs += assert.o
endif

ifeq ($(CONFIG_LIBC_STREAM), defined)
objs += stream.o
endif

ifeq ($(CONFIG_LIBC_STREAM_STD), defined)
objs += stream_std.o
endif


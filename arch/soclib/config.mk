
ifeq ($(CONFIG_SOCLIB_MEMCHECK), defined)
# prevent the compiler from hidding bad memory accesses
CFLAGS += -fno-isolate-erroneous-paths-dereference
# disable load optimizations : prevent loading of undefined values
CFLAGS += -fno-gcse-lm -fno-gcse-sm -fno-sched-spec
endif

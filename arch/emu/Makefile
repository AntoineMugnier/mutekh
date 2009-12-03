
objs =											\
arch_init.o										\
mem_init.o \
hw_init.o

ifeq ($(CONFIG_ARCH_EMU_DARWIN),defined)
objs += darwin_before.o darwin_after.o
endif

subdirs = include/arch/hexo

copy = ldscript

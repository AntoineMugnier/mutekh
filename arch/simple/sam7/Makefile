objs=sam7.o

objs-$(CONFIG_ARCH_SIMPLE_OLIMEX_SAM7_EX256) += sam7_ex256_hw_init.o

ifeq ($(CONFIG_CPU_ARM_THUMB), defined)
sam7.o_CFLAGS = -mno-thumb
endif

subdirs = drivers

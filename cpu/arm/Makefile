
objs = 											\
cpu_init.o										\
cpu_context.o

ifeq ($(CONFIG_CPU_ARM_M_PROFILE),defined)
objs += boot_m.o
else
objs += boot.o
endif

copy = ldscript

subdirs = libc



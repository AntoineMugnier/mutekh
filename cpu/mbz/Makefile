
objs = 										\
cpu_init.o									\
context_switch.o								\
cpu_context.o                                                                   \
exception.o

ifeq ($(CONFIG_CPU_RESET_HANDLER),defined)
objs += boot.o
endif

copy = ldscript

subdirs = libc

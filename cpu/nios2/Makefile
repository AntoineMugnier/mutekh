
objs = 											\
interrupts.o										\
context_switch.o									\
cpu_init.o										\
cpu_context.o

ifeq ($(CONFIG_CPU_RESET_HANDLER),defined)
objs += boot.o
endif

copy = ldscript

subdirs = libc

ifeq ($(CONFIG_DRIVER_CPU_NIOS2),defined)
objs += driver.o
endif


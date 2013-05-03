
objs = interrupts.o context_switch.o cpu_context.o boot.o

copy = ldscript

subdirs = libc

ifeq ($(CONFIG_DRIVER_CPU_NIOS2),defined)
objs += driver.o
endif


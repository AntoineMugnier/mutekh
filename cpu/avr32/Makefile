
objs = interrupt.o cpu_context.o context_switch.o boot.o

copy = ldscript

subdirs = libc

ifeq ($(CONFIG_DRIVER_CPU_AVR32),defined)
objs += driver.o
endif

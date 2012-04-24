
objs = interrupt.o cpu_context.o context_switch.o

ifeq ($(CONFIG_CPU_RESET_HANDLER),defined)
objs += boot.o
endif

copy = ldscript

subdirs = libc

ifeq ($(CONFIG_DRIVER_CPU_SPARC),defined)
objs += driver.o
endif

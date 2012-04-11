
objs = interrupt.o context_switch.o cpu_init.o cpu_context.o

ifeq ($(CONFIG_CPU_RESET_HANDLER),defined)
objs += boot.o
endif

ifeq ($(CONFIG_DRIVER_CPU_MIPS),defined)
objs += driver.o
endif

copy = ldscript

subdirs = libc libelf


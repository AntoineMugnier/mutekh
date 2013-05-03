
objs = interrupt.o context_switch.o cpu_context.o boot.o

ifeq ($(CONFIG_DRIVER_CPU_MIPS),defined)
objs += driver.o
endif

ifeq ($(LINKING),1)
meta = ldscript
endif

subdirs = libc libelf



objs = interrupt.o context_switch.o cpu_context.o boot.o

objs-$(CONFIG_DRIVER_CPU_MIPS) += driver.o

ifeq ($(LINKING),1)
meta = ldscript
endif

subdirs = libc libelf


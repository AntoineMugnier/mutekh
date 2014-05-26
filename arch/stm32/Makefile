
objs =											\
	arch_init.o									\

ifeq ($(CONFIG_STM32_EARLY_CONSOLE_UART), defined)
objs += early_console.o
endif

meta = ldscript

subdirs = drivers


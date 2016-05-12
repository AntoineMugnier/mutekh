
enum_headers = include/hexo/power.h

DIR_CFLAGS = -D_HEXO_INTTYPES_DEPRECATED

objs = cpu.o context.o interrupt.o init.o enum.o

objs-$(CONFIG_HEXO_IPI) += ipi.o
objs-$(CONFIG_MUTEK_SHELL) += shell_power.o



objs = getline_lua.o mem.o

objs-$(CONFIG_VFS) += vfs_functions.o
objs-$(CONFIG_LIBELF_RTLD) += rtld_functions.o
objs-$(CONFIG_LIBDSRL) += dsrl_functions.o
objs-$(CONFIG_LIBCRYPTO_MD5) += crypto_functions.o

ifeq ($(CONFIG_ARCH_SIMPLE_OLIMEX_SAM7_EX256),defined)
objs-$(CONFIG_DEVICE_LCD) += lcd.o
endif

objs-$(CONFIG_MUTEK_TIMER) += timer.o

#DIR_CFLAGS=-O0 -ggdb

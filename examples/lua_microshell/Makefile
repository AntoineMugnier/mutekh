
objs = getline_lua.o shell_functions.o

ifeq ($(CONFIG_LIBELF_RTLD),defined)
objs += rtld_functions.o
endif

ifeq ($(CONFIG_LIBDSRL),defined)
objs += dsrl_functions.o
endif

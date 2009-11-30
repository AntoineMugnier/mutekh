
objs = getline_lua.o shell_functions.o

ifeq ($(CONFIG_ARCH_SOCLIB),defined)

ifeq ($(CONFIG_CPU_MIPS),defined)
 ifeq ($(CONFIG_CPU_CACHE_COHERENCY),defined)
  objs += platform-mipsel-coherent.o
 else
  objs += platform-mips.o
 endif
endif

ifeq ($(CONFIG_CPU_PPC),defined)
objs += platform-ppc.o
endif

ifeq ($(CONFIG_CPU_ARM),defined)
objs += platform-arm.o
endif

endif

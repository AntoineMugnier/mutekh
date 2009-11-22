
objs = hello.o

ifeq ($(CONFIG_ARCH_SOCLIB),defined)

ifeq ($(CONFIG_CPU_MIPS),defined)
objs += platform-mips-coherent.o
endif

ifeq ($(CONFIG_CPU_PPC),defined)
objs += platform-ppc.o
endif

ifeq ($(CONFIG_CPU_ARM),defined)
objs += platform-arm.o
endif

endif

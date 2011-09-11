CPUTOOLS=nios2-unknown-elf-
CPUCFLAGS= -mhw-mul -mhw-div

ifeq ($(CONFIG_CPU_NIOS_MUL), defined)
CPUCFLAGS+= -mhw-mul
endif

ifeq ($(CONFIG_CPU_NIOS_MULX), defined)
CPUCFLAGS+= -mhw-mulx
endif

ifeq ($(CONFIG_CPU_NIOS_DIV), defined)
CPUCFLAGS+= -mhw-div
endif

CPUCFLAGS+= -G0 


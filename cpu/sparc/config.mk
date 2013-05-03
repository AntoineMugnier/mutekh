CPUCFLAGS=-fsigned-char -mflat -Wa,-Av9 -Usparc
CPUTOOLS=sparc-unknown-elf-

ifeq ($(CONFIG_COMPILE_SOFTFLOAT), defined)
CPUCFLAGS += -msoft-float
endif

ifneq ($(CONFIG_CPU_SPARC_APP_REGS), defined)
CPUCFLAGS += -mno-app-regs
endif

ifeq ($(CONFIG_SOCLIB_MEMCHECK), defined)
# prevent filling branch delay slot with memory load instructions
# because some load may access non-initialized data
CPUCFLAGS+= -fno-delayed-branch
endif


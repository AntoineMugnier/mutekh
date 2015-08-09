CPUTOOLS=mipsel-unknown-elf-

ifeq ($(CONFIG_CPU_MIPS_VERSION), 32)
CPUCFLAGS=-mips32
  ifeq ($(CONFIG_MUTEK_BYTECODE_NATIVE), defined)
  BCFLAGS+= -b mips32
  endif
endif

ifeq ($(CONFIG_CPU_MIPS_VERSION), 322)
CPUCFLAGS=-mips32r2
  ifeq ($(CONFIG_MUTEK_BYTECODE_NATIVE), defined)
  BCFLAGS+= -b mips32
  endif
endif

ifeq ($(CONFIG_CPU_MIPS_VERSION), 4)
CPUCFLAGS=-mips4 -mno-branch-likely
endif

ifeq ($(CONFIG_CPU_MIPS_VERSION), 3)
CPUCFLAGS=-mips3 -mno-branch-likely
endif

ifeq ($(CONFIG_CPU_MIPS_VERSION), 2)
CPUCFLAGS=-mips2 -mno-branch-likely
endif

ifeq ($(CONFIG_CPU_MIPS_VERSION), 1)
CPUCFLAGS=-mips1 -mno-branch-likely
endif

ifeq ($(CONFIG_HEXO_FPU), defined)
  ifeq ($(CONFIG_CPU_MIPS_FPU), 32)
  CPUCFLAGS+=-mfp32
  endif

  ifeq ($(CONFIG_CPU_MIPS_FPU), 64)
  CPUCFLAGS+=-mfp64
  endif
endif

ifeq ($(CONFIG_CPU_ENDIAN_LITTLE), defined)
CPUCFLAGS+= -EL
CPULDFLAGS+= -EL
ifeq ($(CONFIG_MUTEK_BYTECODE_NATIVE), defined)
BCFLAGS+= -e little
endif
endif

ifeq ($(CONFIG_CPU_ENDIAN_BIG), defined)
CPUCFLAGS+= -EB
CPULDFLAGS+= -EB
ifeq ($(CONFIG_MUTEK_BYTECODE_NATIVE), defined)
BCFLAGS+= -e big
endif
endif

ifeq ($(CONFIG_COMPILE_SOFTFLOAT), defined)
CPUCFLAGS += -msoft-float
endif

ifeq ($(CONFIG_SOCLIB_MEMCHECK), defined)
# prevent filling branch delay slot with memory load instructions
# because some load may access non-initialized data
CPUCFLAGS+= -fno-delayed-branch -Wa,-O0
endif

CPUCFLAGS+= -G0 -Umips


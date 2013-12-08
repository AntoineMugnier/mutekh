CPUTOOLS=arm-mutekh-eabi-

CPUCFLAGS= -fsigned-char -mabi=aapcs
CPULDFLAGS=
CPUASFLAGS=
LIBGCC_DIR=.

ifeq ($(CONFIG_CPU_ARM_LITTLE), defined)
CPUCFLAGS+= -mlittle-endian
CPULDFLAGS+= -EL
else
CPUCFLAGS+= -mbig-endian
CPULDFLAGS+= -EB
LIBGCC_DIR += /be
endif

ifeq ($(CONFIG_COMPILE_SOFTFLOAT), defined)
CPUCFLAGS += -msoft-float -mfloat-abi=soft -mfpu=vfp
else
 ifeq ($(CONFIG_CPU_ARM_ARCH_PROFILE_M), defined)
CPUCFLAGS += -mhard-float -mfpu=fpv4-sp-d16
LIBGCC_DIR += /fpu/fpv4sp
 else
  ifeq ($(CONFIG_CPU_ARM_THUMB), defined)
CPUCFLAGS += -mhard-float -mfloat-abi=softfp
LIBGCC_DIR += /softfp
  else
CPUCFLAGS += -mhard-float -mfloat-abi=hard
LIBGCC_DIR += /fpu
  endif
  ifeq ($(CONFIG_CPU_ARM_NEON), defined)
CPUCFLAGS += -mfpu=neon
LIBGCC_DIR += /neon
  else
CPUCFLAGS += -mfpu=vfp
  endif
 endif
endif

ifeq ($(CONFIG_CPU_ARM_ARCH_PROFILE_A), defined)
 ifeq ($(CONFIG_CPU_ARM_ARCH_VERSION), 4)
CPUCFLAGS += -march=armv4t
 endif
 ifeq ($(CONFIG_CPU_ARM_ARCH_VERSION), 5)
CPUCFLAGS += -march=armv5
 endif
 ifeq ($(CONFIG_CPU_ARM_ARCH_VERSION), 6)
CPUCFLAGS += -march=armv6
 endif
 ifeq ($(CONFIG_CPU_ARM_ARCH_VERSION), 7)
CPUCFLAGS += -march=armv7-a
 endif
 ifeq ($(CONFIG_CPU_ARM_ARCH_VERSION), 8)
CPUCFLAGS += -march=armv8-a
LIBGCC_DIR += /v8a
 endif

 ifeq ($(CONFIG_CPU_ARM_THUMB), defined)
CPUCFLAGS += -mthumb
LIBGCC_DIR += /thumb
 endif
endif

ifeq ($(CONFIG_CPU_ARM_ARCH_PROFILE_M), defined)
 ifeq ($(CONFIG_CPU_ARM_ARCH_VERSION), 6)
CPUCFLAGS += -march=armv6-m -mthumb
LIBGCC_DIR += /v6m/thumb
 endif
 ifeq ($(CONFIG_CPU_ARM_ARCH_VERSION), 7)
CPUCFLAGS += -march=armv7-m -mthumb
LIBGCC_DIR += /v7m/thumb
 endif
endif

ifeq ($(CONFIG_CPU_ARM_BIG_BE8), defined)
LINK_LDFLAGS+= --be8
endif

ifeq ($(CONFIG_CPU_ARM_SOCLIB), defined)
CPUCFLAGS += -mno-unaligned-access
endif


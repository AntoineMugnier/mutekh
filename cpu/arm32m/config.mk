CPUTOOLS=arm-mutekh-eabi-

CPUCFLAGS= -fsigned-char -mabi=aapcs
CPULDFLAGS=
CPUASFLAGS=
LIBGCC_DIR=.

ifeq ($(CONFIG_CPU_ENDIAN_LITTLE), defined)
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
CPUCFLAGS += -mhard-float -mfpu=fpv4-sp-d16
LIBGCC_DIR += /fpu/fpv4sp
endif

CPUCFLAGS += -mthumb

ifeq ($(CONFIG_COMPILE_MCPU), undefined)
 ifeq ($(CONFIG_CPU_ARM32M_ARCH_VERSION), 6)
CPUCFLAGS += -march=armv6-m
 endif
 ifeq ($(CONFIG_CPU_ARM32M_ARCH_VERSION), 7)
CPUCFLAGS += -march=armv7-m
 endif
endif

ifeq ($(CONFIG_CPU_ARM32M_ARCH_VERSION), 6)
LIBGCC_DIR += /v6m/thumb
endif
ifeq ($(CONFIG_CPU_ARM32M_ARCH_VERSION), 7)
LIBGCC_DIR += /v7m/thumb
endif

ifeq ($(CONFIG_MUTEK_BYTECODE_NATIVE), defined)
BCFLAGS+= -b armv6m
endif


CPUTOOLS=arm-mutekh-eabi-

BCFLAGS+= -w 2

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
 ifeq ($(CONFIG_CPU_ARM32M_ARCH_VERSION), 8)
CPUCFLAGS += -march=armv8-m
 endif
endif

ifeq ($(CONFIG_CPU_ARM32M_ARCH_VERSION), 6)
LIBGCC_DIR += /v6m/thumb
  ifeq ($(CONFIG_MUTEK_BYTECODE_NATIVE), defined)
  BCFLAGS+= -b armv6m
  endif
endif

ifeq ($(CONFIG_CPU_ARM32M_ARCH_VERSION), 7)
LIBGCC_DIR += /v7m/thumb
  ifeq ($(CONFIG_MUTEK_BYTECODE_NATIVE), defined)
  BCFLAGS+= -b armv7m
  endif
endif

ifeq ($(CONFIG_CPU_ARM32M_ARCH_VERSION), 8)
#LIBGCC_DIR += /v8m/thumb
DECL_FILTER_CC = /opt/mutekh/bin/arm-mutekh-eabi-gcc
DECL_FILTER_REPLACE = cpu dbg
DECL_FILTER_REPLACE_cpu_from = -mcpu=$(CONFIG_COMPILE_MCPU)
DECL_FILTER_REPLACE_cpu_to = -mcpu=cortex-m4
DECL_FILTER_REPLACE_dbg_from = -gdwarf-4
DECL_FILTER_REPLACE_dbg_to = 
  ifeq ($(CONFIG_MUTEK_BYTECODE_NATIVE), defined)
  BCFLAGS+= -b armv7m
  endif
endif

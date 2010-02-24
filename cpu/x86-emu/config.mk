
ifeq ($(CONFIG_ARCH_EMU_DARWIN), defined)
CPUCFLAGS+= -fno-stack-protector -m32
else
CPUTOOLS=
endif

CPUCFLAGS=-mno-tls-direct-seg-refs -m32
CPULDFLAGS=-melf_i386


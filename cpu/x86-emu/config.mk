
CPUCFLAGS=-mno-tls-direct-seg-refs -m32
CPULDFLAGS=-m elf_i386
ifeq ($(CONFIG_ARCH_EMU_DARWIN), defined)
CPUTOOLS=i686-unknown-elf-
DEPCC=gcc
else
CPUTOOLS=
endif


ifeq ($(shell uname -s), Darwin)
CPUTOOLS=i686-unknown-elf-
DEPCC=gcc
else
CPUTOOLS=
endif

CPUCFLAGS=-mno-tls-direct-seg-refs


DIR_CFLAGS = -D_HEXO_INTTYPES_DEPRECATED

objs = elf.o 

objs-$(CONFIG_LIBELF_RTLD) += rtld.o user.o
objs-$(CONFIG_LIBELF_RTLD_TLS) += tls.o
objs-$(CONFIG_LIBELF_RTLD_KERNEL) += kernel.o

doc_headers = libelf/elf.h \
			  libelf/rtld.h

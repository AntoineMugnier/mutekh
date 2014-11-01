
DIR_CFLAGS = -D_HEXO_INTTYPES_DEPRECATED

objs-$(CONFIG_MWMR_PTHREAD) += mwmr_pthread.o

ifeq ($(CONFIG_MWMR_SOCLIB),defined)
ifeq ($(CONFIG_MWMR_LOCKFREE),defined)
objs += mwmr_soclib_lockfree.o
else
objs += mwmr_soclib.o
endif
endif

doc_headers = mwmr/mwmr.h

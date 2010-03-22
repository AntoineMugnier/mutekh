
objs += thread.o

ifeq ($(CONFIG_PTHREAD_MUTEX), defined)
objs += mutex.o
endif

ifeq ($(CONFIG_PTHREAD_COND), defined)
objs += cond.o
endif

ifeq ($(CONFIG_PTHREAD_BARRIER), defined)
objs += barrier.o
endif

doc_headers = \
	pthread.h

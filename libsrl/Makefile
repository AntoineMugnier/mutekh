
objs = main.o

ifeq ($(CONFIG_SRL_SOCLIB),defined)
objs += srl_log.o hw_init.o
endif

ifeq ($(CONFIG_PTHREAD),undefined)
objs += barrier.o sched_wait.o posix_wait_cycles.o
endif

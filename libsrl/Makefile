
objs = main.o

ifeq ($(CONFIG_SRL_SOCLIB),defined)
objs += srl_log.o hw_init.o
endif

ifeq ($(CONFIG_PTHREAD),defined)
objs += posix_wait_cycles.o
else
objs += barrier.o sched_wait.o
endif

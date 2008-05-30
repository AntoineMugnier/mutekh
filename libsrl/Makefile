
objs = main.o srl_log.o

ifeq ($(CONFIG_PTHREAD),undefined)
objs += barrier.o sched_wait.o
else
objs += posix_wait_cycles.o
endif

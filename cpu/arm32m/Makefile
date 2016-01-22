
subdirs = libc drivers

objs = context_switch.o cpu_context.o boot.o exception.o
objs-$(CONFIG_DRIVER_CPU_ARM32M) += driver.o
ifneq ($(CONFIG_CPU_ARM32M_TIMER_SYSTICK)_$(CONFIG_CPU_ARM32M_TIMER_DWTCYC), undefined_undefined)
objs += driver_timer.o
endif

objs-$(CONFIG_CPU_ARM32M_ITM_PRINTK) += itm_printk.o

copy = ldscript

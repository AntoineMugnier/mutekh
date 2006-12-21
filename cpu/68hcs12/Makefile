
objs = boot.o vector.o cpu_init.o cpu_context.o interrupts.o libgcc.o

subdirs =

ifeq ($(CONFIG_DRIVER_ICU_MC9S12NE64), defined)
 subdirs += icu-mc9s12ne64
endif

ifeq ($(CONFIG_DRIVER_CHAR_MC9S12NE64), defined)
 subdirs += uart-mc9s12ne64
endif

clean:

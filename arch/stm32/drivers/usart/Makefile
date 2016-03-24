
objs += usart.o

ifeq ($(CONFIG_STM32_FAMILY), 1)
objs-$(CONFIG_DRIVER_STM32_USART_PRINTK) += usart_printk_f1.o
endif

ifeq ($(CONFIG_STM32_FAMILY), 4)
objs-$(CONFIG_DRIVER_STM32_USART_PRINTK) += usart_printk_f4.o
endif


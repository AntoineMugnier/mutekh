
objs += usart.o

ifeq ($(CONFIG_STM32_FAMILY), 1)
objs-$(CONFIG_DRIVER_STM32_USART_PRINTK) += f1xx_usart_printk.o
endif

ifeq ($(CONFIG_STM32_FAMILY), 4)
objs-$(CONFIG_DRIVER_STM32_USART_PRINTK) += f4xx_usart_printk.o
endif


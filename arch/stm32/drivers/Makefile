subdirs-$(CONFIG_DRIVER_STM32_USART) += usart
subdirs-$(CONFIG_DRIVER_STM32_I2C) += i2c
subdirs-$(CONFIG_DRIVER_STM32_RCC) += rcc
subdirs-$(CONFIG_DRIVER_STM32_PWM) += pwm
subdirs-$(CONFIG_DRIVER_STM32_TIMER) += timer
subdirs-$(CONFIG_DRIVER_STM32_SPI) += spi

ifeq ($(CONFIG_STM32_FAMILY), 1)
subdirs-$(CONFIG_DRIVER_STM32_GPIO_F1) += gpio_f1
endif

ifeq ($(CONFIG_STM32_FAMILY), 4)
subdirs-$(CONFIG_DRIVER_STM32_GPIO_X4) += gpio_x4
endif

ifeq ($(CONFIG_STM32_FAMILY), L4)
subdirs-$(CONFIG_DRIVER_STM32_GPIO_X4) += gpio_x4
endif

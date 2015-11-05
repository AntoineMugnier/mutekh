#include <device/resources.h>
#include <device/device.h>
#include <device/irq.h>
#include <device/class/iomux.h>

#include <arch/cc26xx/irq.h>
#include <arch/cc26xx/dio.h>
#include <arch/cc26xx/ioc.h>

DEV_DECLARE_STATIC(cpu_dev, "cpu", DEVICE_FLAG_CPU, arm32m_drv,
                   DEV_STATIC_RES_ID(0, 0)
                   );


#ifdef CONFIG_DRIVER_CC26XX_UART
DEV_DECLARE_STATIC(uart0_dev, "uart0", 0, cc26xx_uart_drv,
                   DEV_STATIC_RES_MEM(0x40001000, 0x40002000),

                   DEV_STATIC_RES_DEV_PARAM("icu", "/cpu"),
                   DEV_STATIC_RES_IRQ(0, CC26XX_IRQ_UART0, DEV_IRQ_SENSE_HIGH_LEVEL, 0, 1),

                   DEV_STATIC_RES_DEV_PARAM("iomux", "/gpio"),
                   DEV_STATIC_RES_IOMUX("tx", 0, CC26XX_DIO3, CC26XX_IOC_IOCFG_PORT_ID_UART0_TX, 0),
                   DEV_STATIC_RES_IOMUX("rx", 0, CC26XX_DIO2, CC26XX_IOC_IOCFG_PORT_ID_UART0_RX, 0)
                   );
#endif

#ifdef CONFIG_DRIVER_CC26XX_GPIO
DEV_DECLARE_STATIC(gpio_dev, "gpio", 0, cc26xx_gpio_drv,
                   DEV_STATIC_RES_MEM(0x40022000, 0x40024000),

                   DEV_STATIC_RES_DEV_PARAM("icu", "/cpu"),
                   DEV_STATIC_RES_IRQ(0, CC26XX_IRQ_GPIO_EDGE_DETECT, DEV_IRQ_SENSE_HIGH_LEVEL, 0, 1)
                   );
#endif

#ifdef CONFIG_DRIVER_CC26XX_TIMER
DEV_DECLARE_STATIC(timer0_dev, "timer0", 0, cc26xx_timer_drv,
                   DEV_STATIC_RES_MEM(0x40010000, 0x40011000),
                   DEV_STATIC_RES_FREQ(48000000, 1),

                   DEV_STATIC_RES_DEV_PARAM("icu", "/cpu"),
                   DEV_STATIC_RES_IRQ(0, CC26XX_IRQ_GPTIMER_0A, DEV_IRQ_SENSE_HIGH_LEVEL, 0, 1)
                   );
#endif

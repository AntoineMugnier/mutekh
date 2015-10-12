#include <device/resources.h>
#include <device/device.h>
#include <device/irq.h>

#include <arch/cc26xx/irq.h>

DEV_DECLARE_STATIC(cpu_dev, "cpu", DEVICE_FLAG_CPU, arm32m_drv,
                   DEV_STATIC_RES_ID(0, 0)
                   );


DEV_DECLARE_STATIC(uart0_dev, "uart0", 0, cc26xx_uart_drv,
                   DEV_STATIC_RES_MEM(0x40001000, 0x40002000),

                   DEV_STATIC_RES_DEV_PARAM("icu", "/cpu"),
                   DEV_STATIC_RES_IRQ(0, CC26XX_IRQ_UART0, DEV_IRQ_SENSE_HIGH_LEVEL, 0, 1)
                   );


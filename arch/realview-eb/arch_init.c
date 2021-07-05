#if defined(CONFIG_DEVICE)
# include <device/driver.h>
# include <device/device.h>
# include <device/resources.h>
# include <device/irq.h>
# include <device/class/cpu.h>
#endif

#ifdef CONFIG_DRIVER_CPU_ARM32

DEV_DECLARE_STATIC(cpu_dev, "cpu", DEVICE_FLAG_CPU, arm32_drv,
                   DEV_STATIC_RES_ID(0, 0),
                   DEV_STATIC_RES_FREQ(100 * 1000 * 1000, 1)
                   );

#endif

#ifdef CONFIG_DRIVER_ICU_PL390

DEV_DECLARE_STATIC(icu_dev, "icu", 0, pl390_icu_drv,
                   DEV_STATIC_RES_MEM(0x10041000, 0x10042000),
                   DEV_STATIC_RES_MEM(0x10040000, 0x10040100),
                   DEV_STATIC_RES_DEV_ICU("/cpu"),
                   DEV_STATIC_RES_IRQ(0, 0, DEV_IRQ_SENSE_LOW_LEVEL, 0, 1),
                   );

#endif

#ifdef CONFIG_DRIVER_CHAR_PL011

DEV_DECLARE_STATIC(uart_dev, "uart", 0, pl011uart_drv,
                   DEV_STATIC_RES_MEM(0x10009000, 0x1000a000),
                   DEV_STATIC_RES_DEV_ICU("/icu"),
                   DEV_STATIC_RES_IRQ(0, 32 + 12, DEV_IRQ_SENSE_HIGH_LEVEL, 0, 1),
	);

#endif


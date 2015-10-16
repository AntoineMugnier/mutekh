#ifdef CONFIG_DEVICE
#include <device/resources.h>
#include <device/class/iomux.h>
#include <device/class/gpio.h>
#include <arch/nrf5x/ids.h>
#include <arch/nrf5x/adc.h>
#endif

#if defined(CONFIG_NRF5X_PCA10000) || defined(CONFIG_NRF5X_PCA10028)

DEV_DECLARE_STATIC(pwm_dev, "leds", 0, nrf5x_gpio_pwm_drv,
                   NRF_STATIC_RES_PERIPHERAL_MEM(NRF5X_TIMER2),
                   DEV_STATIC_RES_DEV_PARAM("icu", "/cpu"),
                   DEV_STATIC_RES_IRQ(0, NRF5X_TIMER2, DEV_IRQ_SENSE_HIGH_LEVEL, 0, 1),
                   DEV_STATIC_RES_DEV_PARAM("iomux", "/gpio"),
                   DEV_STATIC_RES_IOMUX("_p0", 0, 21, 0, 0),
                   DEV_STATIC_RES_IOMUX("_p1", 0, 22, 0, 0),
                   DEV_STATIC_RES_IOMUX("_p2", 0, 23, 0, 0),
                   );

#endif

#if defined(CONFIG_NRF5X_PCA10036)

DEV_DECLARE_STATIC(pwm_dev, "leds", 0, nrf5x_gpio_pwm_drv,
                   NRF_STATIC_RES_PERIPHERAL_MEM(NRF5X_TIMER2),
                   DEV_STATIC_RES_DEV_PARAM("icu", "/cpu"),
                   DEV_STATIC_RES_IRQ(0, NRF5X_TIMER2, DEV_IRQ_SENSE_HIGH_LEVEL, 0, 1),
                   DEV_STATIC_RES_DEV_PARAM("iomux", "/gpio"),
                   DEV_STATIC_RES_IOMUX("_p0", 0, 17, 0, 0),
                   DEV_STATIC_RES_IOMUX("_p1", 0, 18, 0, 0),
                   DEV_STATIC_RES_IOMUX("_p2", 0, 19, 0, 0),
                   );

#endif

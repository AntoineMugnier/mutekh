#ifdef CONFIG_DEVICE
#include <device/resources.h>
#include <device/class/iomux.h>
#include <device/class/gpio.h>
#include <device/class/icu.h>
#include <arch/nrf5x/ids.h>
#include <arch/nrf5x/adc.h>
#endif

#define board_pca10000 1
#define board_pca10028 1
#define board_pca10036 2
#define board_pca10040 2

#define _board_id(x) board_##x
#define board_id(x) _board_id(x)

#if board_id(CONFIG_NRF5X_BOARD_NAME) == 1

DEV_DECLARE_STATIC(pwm_dev, "leds", 0, nrf5x_gpio_pwm_drv,
                   NRF_STATIC_RES_PERIPHERAL_MEM(NRF5X_TIMER2),
                   DEV_STATIC_RES_DEV_ICU("/cpu"),
                   DEV_STATIC_RES_IRQ(0, NRF5X_TIMER2, DEV_IRQ_SENSE_HIGH_LEVEL, 0, 1),
                   DEV_STATIC_RES_DEV_IOMUX("/gpio"),
                   DEV_STATIC_RES_IOMUX("_p0", 0, 21, 0, 0),
                   DEV_STATIC_RES_IOMUX("_p1", 0, 22, 0, 0),
                   DEV_STATIC_RES_IOMUX("_p2", 0, 23, 0, 0),
                   );

#endif

#if board_id(CONFIG_NRF5X_BOARD_NAME) == 2

DEV_DECLARE_STATIC(pwm_dev, "leds", 0, nrf5x_gpio_pwm_drv,
                   NRF_STATIC_RES_PERIPHERAL_MEM(NRF5X_TIMER2),
                   DEV_STATIC_RES_DEV_ICU("/cpu"),
                   DEV_STATIC_RES_IRQ(0, NRF5X_TIMER2, DEV_IRQ_SENSE_HIGH_LEVEL, 0, 1),
                   DEV_STATIC_RES_DEV_IOMUX("/gpio"),
                   DEV_STATIC_RES_IOMUX("_p0", 0, 17, 0, 0),
                   DEV_STATIC_RES_IOMUX("_p1", 0, 18, 0, 0),
                   DEV_STATIC_RES_IOMUX("_p2", 0, 19, 0, 0),
                   );

#endif

#include <device/resources.h>
#include <device/class/iomux.h>
#include <device/class/uart.h>
#include <device/class/i2c.h>
#include <device/class/gpio.h>
#include <arch/nrf5x/ids.h>
#include <arch/nrf5x/adc.h>
#include <device/class/clock.h>

#if defined(CONFIG_NRF5X_PCA10036)
# if defined(CONFIG_DRIVER_NRF5X_I2C)

DEV_DECLARE_STATIC(i2c_dev, "i2c0", 0, nrf5x_i2c_drv,
                   NRF_STATIC_RES_PERIPHERAL_MEM(NRF5X_TWI0),
                   DEV_STATIC_RES_DEV_PARAM("icu", "/cpu"),
                   DEV_STATIC_RES_IRQ(0, NRF5X_TWI0, DEV_IRQ_SENSE_HIGH_LEVEL, 0, 1),
                   DEV_STATIC_RES_DEV_PARAM("iomux", "/gpio"),
                   DEV_STATIC_RES_IOMUX(",scl", 0, 29, 0, 0),
                   DEV_STATIC_RES_IOMUX(",sda", 0, 28, 0, 0),
                   );

#  if defined(CONFIG_DRIVER_MPU6505)

DEV_DECLARE_STATIC(accgyr0_dev, "accgyr0", 0, mpu6505_drv,
                   DEV_STATIC_RES_DEV_PARAM("bus", "/i2c0"),
                   DEV_STATIC_RES_I2C_ADDRESS(0x68),
                   DEV_STATIC_RES_DEV_PARAM("icu", "/gpio"),
                   DEV_STATIC_RES_IRQ(0, 30, DEV_IRQ_SENSE_HIGH_LEVEL, 0, 1),
                   DEV_STATIC_RES_DEV_PARAM("timer", "/rtc1"),
                   );

#  endif
# endif
#endif

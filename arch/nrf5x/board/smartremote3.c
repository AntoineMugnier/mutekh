/*
    This file is part of MutekH.

    MutekH is free software; you can redistribute it and/or modify it
    under the terms of the GNU Lesser General Public License as
    published by the Free Software Foundation; version 2.1 of the
    License.

    MutekH is distributed in the hope that it will be useful, but
    WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this program.  If not, see
    <http://www.gnu.org/licenses/>.

    Copyright Nicolas Pouillon <nipo@ssji.net> (c) 2015
*/

#include <device/resources.h>
#include <device/class/iomux.h>
#include <device/class/uart.h>
#include <device/class/gpio.h>
#include <device/class/cmu.h>
#include <device/class/i2c.h>
#include <arch/nrf5x/ids.h>

/*
PCA10040 on SmartRemote3 extension board (PCA63519)

Expected SB configuration:
Closed: SB1 SB4 SB6 SB8 SB9 SB12 SB19 SB24 SB27 SB28 SB30
Open: SB2 SB3 SB5 SB10 SB11 SB14 SB25 SB26 SB29
If not in list, use Schematic v1.0.0 default value.
*/

#if defined(CONFIG_DRIVER_CMU_GPIO_POWER_SYNC)
// VCC enabler on P15 (active high), powers:
// IR Led, ICM20608, Touchpad, ES8218

DEV_DECLARE_STATIC(vcc_dev, "vcc", 0, cmu_gpio_power_sync_drv,
                   DEV_STATIC_RES_DEV_GPIO("/gpio"),
                   DEV_STATIC_RES_GPIO("power", 15, 1),
                   DEV_STATIC_RES_UINT_PARAM("active", 1),
                   );
#endif

#if defined(CONFIG_DRIVER_NRF5X_I2C)
// One I2C bus with all IPs on it

DEV_DECLARE_STATIC(i2c_dev, "i2c0", 0, nrf5x_i2c_drv,
                   NRF_STATIC_RES_PERIPHERAL_MEM(NRF5X_TWI0),
                   DEV_STATIC_RES_DEV_ICU("/cpu"),
                   DEV_STATIC_RES_IRQ(0, NRF5X_TWI0, DEV_IRQ_SENSE_HIGH_LEVEL, 0, 1),
                   DEV_STATIC_RES_DEV_IOMUX("/gpio"),
                   DEV_STATIC_RES_DEV_TIMER("/timer1"),
                   DEV_STATIC_RES_DEV_GPIO("/gpio"),
                   DEV_STATIC_RES_I2C_BITRATE(100000),
                   DEV_STATIC_RES_IOMUX("scl", 0, 27, 0, 0),
                   DEV_STATIC_RES_IOMUX("sda", 0, 26, 0, 0)
                   );

# if defined(CONFIG_DRIVER_RMI4)
// Synaptics Touchpad, on demand
// Address: 0x20
// IRQ on P24

DEV_DECLARE_STATIC(touchpad0_dev, "touchpad0", 0, rmi4_drv,
                   DEV_STATIC_RES_I2C_ADDR("/i2c0", 0x20),
                   DEV_STATIC_RES_DEV_ICU("/gpio"),
                   DEV_STATIC_RES_IRQ(0, 24, DEV_IRQ_SENSE_FALLING_EDGE, 0, 1),
                   );
# endif

# if defined(CONFIG_DRIVER_LIS3D)
// LIS3D Accel, always on
// Address: 0x19
// INT1 on P25
// INT2 NC

DEV_DECLARE_STATIC(accgyr1_dev, "accgyr1", 0, lis3d_drv,
                   DEV_STATIC_RES_I2C_ADDR("/i2c0", 0x19),
                   DEV_STATIC_RES_DEV_ICU("/gpio"),
                   DEV_STATIC_RES_IRQ(0, 25, DEV_IRQ_SENSE_FALLING_EDGE, 0, 1),
                   );
# endif

# if defined(CONFIG_DRIVER_ICM20608)
// ICM20608 Gyro/Accel, on demand
// Address: 0x68
// IRQ on P20

DEV_DECLARE_STATIC(accgyr0_dev, "accgyr0", 0, icm20608_drv,
                   DEV_STATIC_RES_I2C_ADDR("/i2c0", 0x68),
                   DEV_STATIC_RES_CLK_SRC("/vcc", 0, 0),
                   DEV_STATIC_RES_DEV_ICU("/gpio"),
                   DEV_STATIC_RES_IRQ(0, 20, DEV_IRQ_SENSE_FALLING_EDGE, 0, 1),
                   );
# endif

# if defined(CONFIG_DRIVER_ES8218)
// ES8218 Audio Codec, on demand
// Address: 0x10

// Useless: with SB config, datapath is not connected
# endif

# if defined(CONFIG_DRIVER_SX1509_KBD)
// SX1509 Keyboard Driver, always on
// Address: 0x3e
// IRQ on P19
// Reset on P18

DEV_DECLARE_STATIC(keyboard_dev, "keyboard", 0, sx1509_kbd_drv,
                   DEV_STATIC_RES_I2C_ADDR("/i2c0", 0x3e),
                   DEV_STATIC_RES_DEV_ICU("/gpio"),
                   DEV_STATIC_RES_DEV_GPIO("/gpio"),
                   DEV_STATIC_RES_GPIO("resetn", 18, 1),
                   DEV_STATIC_RES_IRQ(0, 19, DEV_IRQ_SENSE_LOW_LEVEL, 0, 1),
                   DEV_STATIC_RES_UINT_PARAM("rows", 6),
                   DEV_STATIC_RES_UINT_PARAM("cols", 8),
                   );
# endif
#endif

#if defined(CONFIG_DRIVER_NRF52_IR_BLASTER)
// IR LED on P23, on demand

DEV_DECLARE_STATIC(ir0_dev, "ir0", 0, nrf52_ir_drv,
                   NRF_STATIC_RES_PERIPHERAL_MEM(NRF5X_PWM0),
                   DEV_STATIC_RES_CLK_SRC("/vcc", 0, 0),
                   DEV_STATIC_RES_DEV_ICU("/cpu"),
                   DEV_STATIC_RES_IRQ(0, NRF5X_PWM0, DEV_IRQ_SENSE_HIGH_LEVEL, 0, 1),
                   DEV_STATIC_RES_DEV_IOMUX("/gpio"),
                   DEV_STATIC_RES_IOMUX("out", 0, 23, 0, 0)
                   );
#endif

#if defined(CONFIG_DRIVER_NRF52_PDM)
// PDM microphones, on demand
// Clock: P4
// Data: P16

DEV_DECLARE_STATIC(pcm_dev, "pcm", 0, nrf52_pdm_drv,
                   NRF_STATIC_RES_PERIPHERAL_MEM(NRF5X_PDM),
                   DEV_STATIC_RES_DEV_ICU("/cpu"),
                   DEV_STATIC_RES_IRQ(0, NRF5X_PDM, DEV_IRQ_SENSE_HIGH_LEVEL, 0, 1),
                   DEV_STATIC_RES_CLK_SRC("/vcc", 0, 0),
                   DEV_STATIC_RES_DEV_IOMUX("/gpio"),
                   DEV_STATIC_RES_IOMUX("clk", 0, 4, 0, 0),
                   DEV_STATIC_RES_IOMUX("din", 0, 16, 0, 0)
                   );
#endif
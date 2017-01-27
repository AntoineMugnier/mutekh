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

    Copyright (c) 2014 Nicolas Pouillon <nipo@ssji.net>
*/
#ifndef MPU650X_REGS_H_
#define MPU650X_REGS_H_

#define REG_SELF_TEST_X_GYRO 0
#define REG_SELF_TEST_Y_GYRO 1
#define REG_SELF_TEST_Z_GYRO 2

#define REG6050_XA_OFFSET_H 6
#define REG6050_XA_OFFSET_L 7
#define REG6050_YA_OFFSET_H 8
#define REG6050_YA_OFFSET_L 9
#define REG6050_ZA_OFFSET_H 10
#define REG6050_ZA_OFFSET_L 11

#define REG_SELF_TEST_X_ACCEL 13
#define REG_SELF_TEST_Y_ACCEL 14
#define REG_SELF_TEST_Z_ACCEL 15
#define REG_XG_OFFSET_H       19
#define REG_XG_OFFSET_L       20
#define REG_YG_OFFSET_H       21
#define REG_YG_OFFSET_L       22
#define REG_ZG_OFFSET_H       23
#define REG_ZG_OFFSET_L       24
#define REG_SMPLRT_DIV        25
#define REG_CONFIG            26
#define   REG_CONFIG_FIFO_MODE_OVERFLOW  0x40
#define   REG_CONFIG_FIFO_MODE_OVERWRITE 0x00
#define   REG_CONFIG_DLPF_250HZ          0
#define   REG_CONFIG_DLPF_184HZ          1
#define   REG_CONFIG_DLPF_92HZ           2
#define   REG_CONFIG_DLPF_41HZ           3
#define   REG_CONFIG_DLPF_20HZ           4
#define   REG_CONFIG_DLPF_10HZ           5
#define   REG_CONFIG_DLPF_5HZ            6
#define   REG_CONFIG_DLPF_3600HZ         7

#define REG_GYRO_CONFIG 27
#define   REG_GYRO_CONFIG_X_ST_EN 0x80
#define   REG_GYRO_CONFIG_Y_ST_EN 0x40
#define   REG_GYRO_CONFIG_Z_ST_EN 0x20
#define   REG_GYRO_CONFIG_250DPS  0
#define   REG_GYRO_CONFIG_500DPS  0x08
#define   REG_GYRO_CONFIG_1000DPS 0x10
#define   REG_GYRO_CONFIG_2000DPS 0x18
#define   REG_GYRO_CONFIG_DLPF    0x00
#define   REG_GYRO_CONFIG_8800HZ  0x01
#define   REG_GYRO_CONFIG_3600HZ  0x02

#define REG_ACCEL_CONFIG 28
#define   REG_ACCEL_CONFIG_X_ST_EN 0x80
#define   REG_ACCEL_CONFIG_Y_ST_EN 0x40
#define   REG_ACCEL_CONFIG_Z_ST_EN 0x20
#define   REG_ACCEL_CONFIG_2G 0
#define   REG_ACCEL_CONFIG_4G 0x08
#define   REG_ACCEL_CONFIG_8G 0x10
#define   REG_ACCEL_CONFIG_16G 0x18

#define REG_ACCEL_CONFIG2 29
#define   REG_ACCEL_CONFIG2_1_13KHZ 0x08
#define   REG_ACCEL_CONFIG2_460HZ   0x0
#define   REG_ACCEL_CONFIG2_184HZ   0x1
#define   REG_ACCEL_CONFIG2_92HZ    0x2
#define   REG_ACCEL_CONFIG2_41HZ    0x3
#define   REG_ACCEL_CONFIG2_20HZ    0x4
#define   REG_ACCEL_CONFIG2_10HZ    0x5
#define   REG_ACCEL_CONFIG2_5HZ     0x6
#define   REG_ACCEL_CONFIG2_460HZ_  0x7

#define REG_LP_ACCEL_ODR 30
#define   REG_LP_ACCEL_ODR_0_24HZ  0
#define   REG_LP_ACCEL_ODR_0_49HZ  1
#define   REG_LP_ACCEL_ODR_0_98HZ  2
#define   REG_LP_ACCEL_ODR_1_95HZ  3
#define   REG_LP_ACCEL_ODR_3_91HZ  4
#define   REG_LP_ACCEL_ODR_7_81HZ  5
#define   REG_LP_ACCEL_ODR_15_63HZ 6
#define   REG_LP_ACCEL_ODR_31_65HZ 7
#define   REG_LP_ACCEL_ODR_62_5HZ  8
#define   REG_LP_ACCEL_ODR_125HZ   9
#define   REG_LP_ACCEL_ODR_250HZ   10
#define   REG_LP_ACCEL_ODR_500HZ   11

// In 4-mg units
#define REG_WOM_THR 31
#define   REG_WOM_THR_MG(x) ((x)/4)

#define REG_FIFO_EN 35
#define   REG_FIFO_EN_TEMP   0x80
#define   REG_FIFO_EN_GYRO_X 0x40
#define   REG_FIFO_EN_GYRO_Y 0x20
#define   REG_FIFO_EN_GYRO_Z 0x10
#define   REG_FIFO_EN_ACCEL  0x08

#define REG_INT_PIN_CFG 55
#define   REG_INT_PIN_CFG_ACTIVE_LOW    0x80
#define   REG_INT_PIN_CFG_ACTIVE_HIGH   0
#define   REG_INT_PIN_CFG_OPEN_DRAIN    0x40
#define   REG_INT_PIN_CFG_PUSHPULL      0
#define   REG_INT_PIN_CFG_LEVEL         0x20
#define   REG_INT_PIN_CFG_EDGE          0
#define   REG_INT_PIN_CLEAR_READ_ANY    0x10
#define   REG_INT_PIN_CLEAR_READ_STATUS 0
#define   REG_INT_PIN_FSYNC_IRQ_LOW     0x08
#define   REG_INT_PIN_FSYNC_IRQ_HIGH    0
#define   REG_INT_PIN_FSYNC_IRQ         0x04

#define REG_INT_ENABLE 56
#define   REG_INT_WOM           0x40
#define   REG_INT_FIFO_OVERFLOW 0x10
#define   REG_INT_FSYNC         0x08
#define   REG_INT_RAW_RDY       0x01

#define REG_INT_STATUS           58
#define REG_ACCEL_XOUT_H         59
#define REG_SENSOR_BLOCK_BEGIN   REG_ACCEL_XOUT_H
#define REG_ACCEL_XOUT_L         60
#define REG_ACCEL_YOUT_H         61
#define REG_ACCEL_YOUT_L         62
#define REG_ACCEL_ZOUT_H         63
#define REG_ACCEL_ZOUT_L         64
#define REG_TEMP_OUT_H           65
#define REG_TEMP_OUT_L           66
#define REG_GYRO_XOUT_H          67
#define REG_GYRO_XOUT_L          68
#define REG_GYRO_YOUT_H          69
#define REG_GYRO_YOUT_L          70
#define REG_GYRO_ZOUT_H          71
#define REG_GYRO_ZOUT_L          72
#define REG_SIGNAL_PATH_RESET    104
#define REG_ACCEL_INTEL_CTRL     105
#define   REG_ACCEL_INTEL_EN            0x80
#define   REG_ACCEL_INTEL_MODE_COMPARE  0x40
#define   REG_ACCEL_INTEL_MODE_NOT_USED 0x00

#define REG_USER_CTRL 106
#define   REG_USER_FIFO_EN      0x40
#define   REG_USER_I2C_IF_DIS   0x10
#define   REG_USER_FIFO_RST     0x04
#define   REG_USER_SIG_COND_RST 0x01

#define REG_PWR_MGMT_1 107
#define   REG_PWR_MGMT_1_H_RESET          0x80
#define   REG_PWR_MGMT_1_SLEEP            0x40
#define   REG_PWR_MGMT_1_CYCLE            0x20
#define   REG_PWR_MGMT_1_GYRO_STANDBY     0x10
#define   REG_PWR_MGMT_1_TEMP_DIS         0x08
#define   REG_PWR_MGMT_1_CLKSTEP_INTERNAL 0
#define   REG_PWR_MGMT_1_CLKSTEP_OFF      0x7

#define REG_PWR_MGMT_2 108
#define   REG_PWR_MGMT_2_LP_WAKE_CTRL_1_25_HZ 0x00
#define   REG_PWR_MGMT_2_LP_WAKE_CTRL_5_HZ    0x40
#define   REG_PWR_MGMT_2_LP_WAKE_CTRL_20_HZ   0x80
#define   REG_PWR_MGMT_2_LP_WAKE_CTRL_40_HZ   0xc0
#define   REG_PWR_MGMT_2_DISABLE_XA           0x20
#define   REG_PWR_MGMT_2_DISABLE_YA           0x10
#define   REG_PWR_MGMT_2_DISABLE_ZA           0x08
#define   REG_PWR_MGMT_2_DISABLE_XG           0x04
#define   REG_PWR_MGMT_2_DISABLE_YG           0x02
#define   REG_PWR_MGMT_2_DISABLE_ZG           0x01

#define REG_FIFO_COUNT_H 114
#define REG_FIFO_COUNT_L 115
#define REG_FIFO_R_W     116
#define REG_WHO_AM_I     117
#define REG_ACCEL_OFFSET 118
#define REG_XA_OFFSET_H  119
#define REG_XA_OFFSET_L  120
#define REG_YA_OFFSET_H  122
#define REG_YA_OFFSET_L  123
#define REG_ZA_OFFSET_H  125
#define REG_ZA_OFFSET_L  126

#endif

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

    Copyright (c) 2017 Nicolas Pouillon <nipo@ssji.net>
*/
#ifndef ICM20602_REGS_H_
#define ICM20602_REGS_H_

#define REG_XG_OFFSET_H 4
#define REG_XG_OFFSET_L 5
#define REG_YG_OFFSET_H 7
#define REG_YG_OFFSET_L 8
#define REG_ZG_OFFSET_H 10
#define REG_ZG_OFFSET_L 11

#define REG_XA_ST_DATA 13
#define REG_YA_ST_DATA 14
#define REG_ZA_ST_DATA 15

#define REG_XG_OFFSET_USR_H 119
#define REG_XG_OFFSET_USR_L 120
#define REG_YG_OFFSET_USR_H 121
#define REG_YG_OFFSET_USR_L 122
#define REG_ZG_OFFSET_USR_H 123
#define REG_ZG_OFFSET_USR_L 124

#define REG_SMPLRT_DIV 25

#define REG_CONFIG            26
#define   REG_CONFIG_FIFO_MODE_OVERFLOW  0x40
#define   REG_CONFIG_FIFO_MODE_OVERWRITE 0x00
#define   REG_CONFIG_FSYNC_DISABLED      (0 << 3)
#define   REG_CONFIG_FSYNC_TEMP_OUT_L    (1 << 3)
#define   REG_CONFIG_FSYNC_GYRO_XOUT_L   (2 << 3)
#define   REG_CONFIG_FSYNC_GYRO_YOUT_L   (3 << 3)
#define   REG_CONFIG_FSYNC_GYRO_ZOUT_L   (4 << 3)
#define   REG_CONFIG_FSYNC_ACCEL_XOUT_L  (5 << 3)
#define   REG_CONFIG_FSYNC_ACCEL_YOUT_L  (6 << 3)
#define   REG_CONFIG_FSYNC_ACCEL_ZOUT_L  (7 << 3)
#define   REG_CONFIG_DLPF_250HZ          0
#define   REG_CONFIG_DLPF_176HZ          1
#define   REG_CONFIG_DLPF_92HZ           2
#define   REG_CONFIG_DLPF_41HZ           3
#define   REG_CONFIG_DLPF_20HZ           4
#define   REG_CONFIG_DLPF_10HZ           5
#define   REG_CONFIG_DLPF_5HZ            6
#define   REG_CONFIG_DLPF_3281HZ         7

#define REG_GYRO_CONFIG 27
#define   REG_GYRO_CONFIG_X_ST_EN 0x80
#define   REG_GYRO_CONFIG_Y_ST_EN 0x40
#define   REG_GYRO_CONFIG_Z_ST_EN 0x20
#define   REG_GYRO_CONFIG_250DPS  0
#define   REG_GYRO_CONFIG_500DPS  0x08
#define   REG_GYRO_CONFIG_1000DPS 0x10
#define   REG_GYRO_CONFIG_2000DPS 0x18
#define   REG_GYRO_CONFIG_DLPF    0x00
#define   REG_GYRO_CONFIG_8173HZ  0x01
#define   REG_GYRO_CONFIG_3281HZ  0x02

#define REG_ACCEL_CONFIG 28
#define   REG_ACCEL_CONFIG_X_ST_EN 0x80
#define   REG_ACCEL_CONFIG_Y_ST_EN 0x40
#define   REG_ACCEL_CONFIG_Z_ST_EN 0x20
#define   REG_ACCEL_CONFIG_2G 0
#define   REG_ACCEL_CONFIG_4G 0x08
#define   REG_ACCEL_CONFIG_8G 0x10
#define   REG_ACCEL_CONFIG_16G 0x18

#define REG_ACCEL_CONFIG2 29
#define   REG_ACCEL_CONFIG2_DEC2_4  0x00
#define   REG_ACCEL_CONFIG2_DEC2_8  0x10
#define   REG_ACCEL_CONFIG2_DEC2_16 0x20
#define   REG_ACCEL_CONFIG2_DEC2_32 0x30
#define   REG_ACCEL_CONFIG2_1048HZ  0x08
#define   REG_ACCEL_CONFIG2_218HZ   0x0
#define   REG_ACCEL_CONFIG2_218HZ_2 0x1
#define   REG_ACCEL_CONFIG2_99HZ    0x2
#define   REG_ACCEL_CONFIG2_44HZ    0x3
#define   REG_ACCEL_CONFIG2_21HZ    0x4
#define   REG_ACCEL_CONFIG2_10HZ    0x5
#define   REG_ACCEL_CONFIG2_5HZ     0x6
#define   REG_ACCEL_CONFIG2_420HZ_  0x7

#define REG_LP_MODE_CFG 30
#define   REG_LP_MODE_CFG_GYRO_CYCLE   0x80
#define   REG_LP_MODE_CFG_G_AVGCFG(x)  ((x) << 4) // (1 << x) averages

// In 4-mg units
#define REG_WOM_THR_X 32
#define REG_WOM_THR_Y 33
#define REG_WOM_THR_Z 34
#define   REG_WOM_THR_MG(x) ((x)/4)

#define REG_FIFO_EN 35
#define   REG_FIFO_EN_TEMP_GYRO   0x10
#define   REG_FIFO_EN_TEMP_ACCEL  0x08

#define REG_FSYNC_INT 54
#define   REG_FSYNC_INT_GENERATED 0x80

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

#define REG_WM_INT_STATUS        57
#define   REG_WM_INT_GENERATED     0x40

#define REG_INT_STATUS           58
#define   REG_INT_STATUS_WOM_X     0x80
#define   REG_INT_STATUS_WOM_Y     0x40
#define   REG_INT_STATUS_WOM_Z     0x20
#define   REG_INT_STATUS_FIFO_OV   0x10
#define   REG_INT_STATUS_GDRIVE    0x04
#define   REG_INT_STATUS_DATA_RDY  0x01

#define REG_ACCEL_XOUT_H         59
#define REG_SENSOR_BLOCK_BEGIN   REG_ACCEL_XOUT_H
#define REG_ACCEL_XOUT_L         60
#define REG_ACCEL_YOUT_H         61
#define REG_ACCEL_YOUT_L         62
#define REG_ACCEL_ZOUT_H         63
#define REG_ACCEL_ZOUT_L         64
#define REG_TEMP_OUT_H           65 // 326.8 lsb/deg
#define REG_TEMP_OUT_L           66
#define REG_GYRO_XOUT_H          67 // 131 lsb/dps
#define REG_GYRO_XOUT_L          68
#define REG_GYRO_YOUT_H          69
#define REG_GYRO_YOUT_L          70
#define REG_GYRO_ZOUT_H          71
#define REG_GYRO_ZOUT_L          72

#define REG_XG_ST_DATA 80
#define REG_YG_ST_DATA 81
#define REG_ZG_ST_DATA 82

#define REG_FIFO_WM_TH_H 96
#define REG_FIFO_WM_TH_L 97

#define REG_SIGNAL_PATH_RESET    104
#define   REG_SIGNAL_PATH_RESET_ACCEL    0x02
#define   REG_SIGNAL_PATH_RESET_TEMP     0x01

#define REG_ACCEL_INTEL_CTRL     105
#define   REG_ACCEL_INTEL_EN              0x80
#define   REG_ACCEL_INTEL_MODE_COMPARE    0x40
#define   REG_ACCEL_INTEL_OUTPUT_NOLIMIT  0x02
#define   REG_ACCEL_INTEL_WOM_ANY         0x00
#define   REG_ACCEL_INTEL_WOM_ALL         0x01

#define REG_USER_CTRL 106
#define   REG_USER_FIFO_EN      0x40
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
#define   REG_PWR_MGMT_2_DISABLE_XA           0x20
#define   REG_PWR_MGMT_2_DISABLE_YA           0x10
#define   REG_PWR_MGMT_2_DISABLE_ZA           0x08
#define   REG_PWR_MGMT_2_DISABLE_XG           0x04
#define   REG_PWR_MGMT_2_DISABLE_YG           0x02
#define   REG_PWR_MGMT_2_DISABLE_ZG           0x01

#define REG_I2C_IF 112
#define   REG_I2C_IF_DISABLE 0x40

#define REG_FIFO_COUNT_H 114
#define REG_FIFO_COUNT_L 115
#define REG_FIFO_R_W     116

#define REG_WHO_AM_I     117
#define   REG_WHO_AM_I_ICM20602    0x12

#define REG_XA_OFFSET_H 119
#define REG_XA_OFFSET_L 120
#define REG_YA_OFFSET_H 121
#define REG_YA_OFFSET_L 122
#define REG_ZA_OFFSET_H 123
#define REG_ZA_OFFSET_L 124

#endif

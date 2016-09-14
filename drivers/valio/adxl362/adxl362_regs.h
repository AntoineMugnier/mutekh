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
    License along with MutekH; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
    02110-1301 USA.

    Copyright Sebastien Cerdan <sebcerdan@gmail.com> (c) 2014
    Copyright Vincent DEFILIPPI <vincentdefilippi@gmail.com> (c) 2016
*/

#ifndef _ADXL362_REGS_H_
#define _ADXL362_REGS_H_

#define ADXL362_REG_DEVID_AD                              0x00
#define   ADXL362_REG_DEVID_AD_MASK                       0xFF

#define ADXL362_REG_DEVID_MST                             0x01
#define   ADXL362_REG_DEVID_MST_MASK                      0xFF

#define ADXL362_REG_PARTID                                0x02
#define   ADXL362_REG_PARTID_MASK                         0xFF

#define ADXL362_REG_REVID                                 0x03
#define   ADXL362_REG_REVID_MASK                          0xFF

#define ADXL362_REG_XDATA                                 0x08
#define   ADXL362_REG_XDATA_MASK                          0xFF

#define ADXL362_REG_YDATA                                 0x09
#define   ADXL362_REG_YDATA_MASK                          0xFF

#define ADXL362_REG_ZDATA                                 0x0A
#define   ADXL362_REG_ZDATA_MASK                          0xFF

#define ADXL362_REG_STATUS                                0x0B
#define   ADXL362_REG_STATUS_ERR_USERS_MASK               0x80
#define   ADXL362_REG_STATUS_AWAKE_MASK                   0x40
#define   ADXL362_REG_STATUS_INACT_MASK                   0x20
#define   ADXL362_REG_STATUS_ACT_MASK                     0x10
#define   ADXL362_REG_STATUS_FIFO_OVERRUN_MASK            0x08
#define   ADXL362_REG_STATUS_FIFO_WATERMARK_MASK          0x04
#define   ADXL362_REG_STATUS_FIFO_READY_MASK              0x02
#define   ADXL362_REG_STATUS_DATA_READY_MASK              0x01

#define ADXL362_REG_FIFO_ENTRIES_L                        0x0C
#define   ADXL362_REG_FIFO_ENTRIES_L_MASK                 0xFF

#define ADXL362_REG_FIFO_ENTRIES_H                        0x0D
#define   ADXL362_REG_FIFO_ENTRIES_H_MASK                 0x03

#define ADXL362_REG_XDATA_L                               0x0E
#define   ADXL362_REG_XDATA_L_MASK                        0xFF

#define ADXL362_REG_XDATA_H                               0x0F
#define   ADXL362_REG_XDATA_H_MASK                        0xFF

#define ADXL362_REG_YDATA_L                               0x10
#define   ADXL362_REG_YDATA_L_MASK                        0xFF

#define ADXL362_REG_YDATA_H                               0x11
#define   ADXL362_REG_YDATA_H_MASK                        0xFF

#define ADXL362_REG_ZDATA_L                               0x12
#define   ADXL362_REG_ZDATA_L_MASK                        0xFF

#define ADXL362_REG_ZDATA_H                               0x13
#define   ADXL362_REG_ZDATA_H_MASK                        0xFF

#define ADXL362_REG_TEMP_L                                0x14
#define   ADXL362_REG_TEMP_L_MASK                         0xFF

#define ADXL362_REG_TEMP_H                                0x15
#define   ADXL362_REG_TEMP_H_MASK                         0xFF

#define ADXL362_REG_SOFT_RESET                            0x1F
#define   ADXL362_REG_SOFT_RESET_MASK                     0xFF

#define ADXL362_REG_THRESH_ACT_L                          0x20
#define   ADXL362_REG_THRESH_ACT_L_MASK                   0xFF

#define ADXL362_REG_THRESH_ACT_H                          0x21
#define   ADXL362_REG_THRESH_ACT_H_MASK                   0x07

#define ADXL362_REG_TIME_ACT                              0x22
#define   ADXL362_REG_TIME_ACT_MASK                       0xFF

#define ADXL362_REG_THRESH_INACT_L                        0x23
#define   ADXL362_REG_THRESH_INACT_L_MASK                 0xFF

#define ADXL362_REG_THRESH_INACT_H                        0x24
#define   ADXL362_REG_THRESH_INACT_H_MASK                 0x07

#define ADXL362_REG_TIME_INACT_L                          0x25
#define   ADXL362_REG_TIME_INACT_L_MASK                   0xFF

#define ADXL362_REG_TIME_INACT_H                          0x26
#define   ADXL362_REG_TIME_INACT_H_MASK                   0xFF

#define ADXL362_REG_ACT_INACT_CTL                         0x27
#define   ADXL362_REG_ACT_INACT_CTL_LINKLOOP_MASK         0x30
#define     ADXL362_REG_ACT_INACT_CTL_LINKLOOP(v) ((ADXL362_REG_ACT_INACT_CTL_LINKLOOP_##v) << 4)
#define     ADXL362_REG_ACT_INACT_CTL_LINKLOOP_DEFAULT    0x00
#define     ADXL362_REG_ACT_INACT_CTL_LINKLOOP_LINKED     0x01
#define     ADXL362_REG_ACT_INACT_CTL_LINKLOOP_LOOP       0x03
#define   ADXL362_REG_ACT_INACT_CTL_INACT_REF_MASK        0x08
#define   ADXL362_REG_ACT_INACT_CTL_INACT_EN_MASK         0x04
#define   ADXL362_REG_ACT_INACT_CTL_ACT_REF_MASK          0x02
#define   ADXL362_REG_ACT_INACT_CTL_ACT_EN_MASK           0x01

#define ADXL362_REG_FIFO_CONTROL                          0x28
#define   ADXL362_REG_FIFO_CONTROL_AH_MASK                0x08
#define   ADXL362_REG_FIFO_CONTROL_FIFO_TEMP_MASK         0x04
#define   ADXL362_REG_FIFO_CONTROL_FIFO_MODE_MASK         0x03
#define     ADXL362_REG_FIFO_CONTROL_FIFO_MODE(v) ((ADXL362_REG_FIFO_CONTROL_FIFO_MODE_##v) << 0)
#define     ADXL362_REG_FIFO_CONTROL_FIFO_MODE_DISABLED   0x00
#define     ADXL362_REG_FIFO_CONTROL_FIFO_MODE_OLDEST     0x00
#define     ADXL362_REG_FIFO_CONTROL_FIFO_MODE_STREAM     0x01
#define     ADXL362_REG_FIFO_CONTROL_FIFO_MODE_TRIGGERED  0x03

#define ADXL362_REG_FIFO_SAMPLES                          0x29
#define   ADXL362_REG_FIFO_SAMPLES_MASK                   0xFF

#define ADXL362_REG_INTMAP1                               0x2A
#define   ADXL362_REG_INTMAP1_INT_LOW_MASK                0x80
#define   ADXL362_REG_INTMAP1_AWAKE_MASK                  0x40
#define   ADXL362_REG_INTMAP1_INACT_MASK                  0x20
#define   ADXL362_REG_INTMAP1_ACT_MASK                    0x10
#define   ADXL362_REG_INTMAP1_FIFO_OVERRUN_MASK           0x08
#define   ADXL362_REG_INTMAP1_FIFO_WATERMARK_MASK         0x04
#define   ADXL362_REG_INTMAP1_FIFO_READY_MASK             0x02
#define   ADXL362_REG_INTMAP1_DATA_READY_MASK             0x01

#define ADXL362_REG_INTMAP2                               0x2B
#define   ADXL362_REG_INTMAP2_INT_LOW_MASK                0x80
#define   ADXL362_REG_INTMAP2_AWAKE_MASK                  0x40
#define   ADXL362_REG_INTMAP2_INACT_MASK                  0x20
#define   ADXL362_REG_INTMAP2_ACT_MASK                    0x10
#define   ADXL362_REG_INTMAP2_FIFO_OVERRUN_MASK           0x08
#define   ADXL362_REG_INTMAP2_FIFO_WATERMARK_MASK         0x04
#define   ADXL362_REG_INTMAP2_FIFO_READY_MASK             0x02
#define   ADXL362_REG_INTMAP2_DATA_READY_MASK             0x01

#define ADXL362_REG_FILTER_CTL                            0x2C
#define   ADXL362_REG_FILTER_CTL_RANGE_MASK               0xc0
#define     ADXL362_REG_FILTER_CTL_RANGE(v) ((ADXL362_REG_FILTER_CTL_RANGE_##v) << 6)
#define     ADXL362_REG_FILTER_CTL_RANGE_2G               0x00
#define     ADXL362_REG_FILTER_CTL_RANGE_4G               0x01
#define     ADXL362_REG_FILTER_CTL_RANGE_8G               0x02
#define   ADXL362_REG_FILTER_CTL_HALF_BW_MASK             0x10
#define   ADXL362_REG_FILTER_CTL_EXT_SAMPLE_MASK          0x08
#define   ADXL362_REG_FILTER_CTL_ODR_MASK                 0x07
#define     ADXL362_REG_FILTER_CTL_ODR(v) ((ADXL362_REG_FILTER_CTL_ODR_##v) << 0)
#define     ADXL362_REG_FILTER_CTL_ODR_12_5_HZ            0x00
#define     ADXL362_REG_FILTER_CTL_ODR_25_HZ              0x01
#define     ADXL362_REG_FILTER_CTL_ODR_50_HZ              0x02
#define     ADXL362_REG_FILTER_CTL_ODR_100_HZ             0x03
#define     ADXL362_REG_FILTER_CTL_ODR_200_HZ             0x04
#define     ADXL362_REG_FILTER_CTL_ODR_400_HZ             0x05

#define ADXL362_REG_POWER_CTL                             0x2D
#define   ADXL362_REG_POWER_CTL_EXT_CLK_MASK              0x40
#define   ADXL362_REG_POWER_CTL_LOW_NOISE_MASK            0x30
#define     ADXL362_REG_POWER_CTL_LOW_NOISE(v) ((ADXL362_REG_POWER_CTL_LOW_NOISE_##v) << 4)
#define     ADXL362_REG_POWER_CTL_LOW_NOISE_NORMAL        0x00
#define     ADXL362_REG_POWER_CTL_LOW_NOISE_LOW           0x01
#define     ADXL362_REG_POWER_CTL_LOW_NOISE_ULTRALOW      0x02
#define   ADXL362_REG_POWER_CTL_WAKEUP_MASK               0x08
#define   ADXL362_REG_POWER_CTL_AUTOSLEEP_MASK            0x04
#define   ADXL362_REG_POWER_CTL_MEASURE_MASK              0x03
#define     ADXL362_REG_POWER_CTL_MEASURE(v) ((ADXL362_REG_POWER_CTL_MEASURE_##v) << 0)
#define     ADXL362_REG_POWER_CTL_MEASURE_STANDBY         0x00
#define     ADXL362_REG_POWER_CTL_MEASURE_MEASUREMENT     0x02

#define ADXL362_REG_SELF_TEST                             0x2E
#define   ADXL362_REG_SELF_TEST_MASK                      0x01

#endif /* _ADXL362_REGS_H_ */

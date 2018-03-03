
#include <device/device.h>
#include <device/resources.h>
#include <device/class/icu.h>
#include <device/irq.h>

/* Declare an instance of the device_s struct. */
DEV_DECLARE_STATIC(
                   /* device global variable identifier */
                   my_dev0,

                   /* device name */
                   "mydev0",

                   /* flags */
                   DEVICE_FLAG_NO_AUTOINIT,

                   /* pointer to driver */
                   mydrv0_drv,

                   /* no device resource */
                   );


DEV_DECLARE_STATIC(my_dev1, "mydev1", DEVICE_FLAG_NO_AUTOINIT, mydrv1_drv, );

DEV_DECLARE_STATIC(my_dev2, "mydev2", DEVICE_FLAG_NO_AUTOINIT, mydrv2_drv, );

DEV_DECLARE_STATIC(my_dev3, "mydev3", DEVICE_FLAG_NO_AUTOINIT, mydrv3_drv,
                   /* address range of device memory mapped registers */
                   DEV_STATIC_RES_MEM(0xd0200000, 0xd0200010)
                   );

DEV_DECLARE_STATIC(my_dev4, "mydev4", DEVICE_FLAG_NO_AUTOINIT, mydrv4_drv,
                   DEV_STATIC_RES_MEM(0xd3200000, 0xd3200020)
                   );

DEV_DECLARE_STATIC(my_dev5, "mydev5", DEVICE_FLAG_NO_AUTOINIT, mydrv5_drv,
                   DEV_STATIC_RES_MEM(0xd3200000, 0xd3200020)
                   );

DEV_DECLARE_STATIC(my_dev6, "mydev6", DEVICE_FLAG_NO_AUTOINIT, mydrv6_drv,
                   DEV_STATIC_RES_MEM(0xd3200000, 0xd3200020),
                   DEV_STATIC_RES_DEV_ICU("/icu"),
                   DEV_STATIC_RES_IRQ(0, 2, DEV_IRQ_SENSE_HIGH_LEVEL, 0, 1)
                   );

DEV_DECLARE_STATIC(my_dev7, "mydev7", DEVICE_FLAG_NO_AUTOINIT, mydrv7_drv,
                   DEV_STATIC_RES_DEV_PARAM("myslave", "/timer")
                   );

DEV_DECLARE_STATIC(my_dev8, "mydev8", 0, mydrv8_drv,
                   DEV_STATIC_RES_MEM(0xd0200000, 0xd0200010),
                   DEV_STATIC_RES_DEV_ICU("/icu"),
                   DEV_STATIC_RES_IRQ(0, 0, DEV_IRQ_SENSE_HIGH_LEVEL, 0, 1)
                   );

DEV_DECLARE_STATIC(my_dev9, "mydev9", 0, mydrv9_drv,
                   DEV_STATIC_RES_MEM(0xd0200000, 0xd0200010),
                   DEV_STATIC_RES_DEV_ICU("/icu"),
                   DEV_STATIC_RES_IRQ(0, 0, DEV_IRQ_SENSE_HIGH_LEVEL, 0, 1)
                   );

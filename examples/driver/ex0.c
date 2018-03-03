/*
   Topics covered by this example:
     - Device driver minimal declarations
     - Device driver registering

   This example implements a simple driver which initializes but does
   nothing else: It does not control hardware and implements no device
   class API.

   A device object can be instantiated statically which is bound to
   this driver by putting this declaration in the application or board
   declaration source code:

   DEV_DECLARE_STATIC(my_dev1, "mydev1", 0, mydrv1_drv, );
*/

#include <hexo/types.h>
#include <hexo/endian.h>

#include <mutek/mem_alloc.h>
#include <mutek/printk.h>

#include <device/device.h>
#include <device/resources.h>
#include <device/driver.h>

/* Driver private data, could be allocated for each initialized device
   instance by the mydrv_init function. */
DRIVER_PV(struct mydrv_context_s
{
  /* no specific device state to store in this simple driver. */
});

/* The DEV_INIT macro yields the required prototype for the driver
   initialization function. Only the function name and the static
   attribute need to be provided. */
static DEV_INIT(mydrv_init)
{
  printk("initialization of device %p\n", dev);
  return 0;
}

static DEV_CLEANUP(mydrv_cleanup)
{
  printk("cleanup of device %p\n", dev);
  return 0;
}

/* Do not implement this function, rely on the default implementation
   provided by libdevice. */
#define mydrv_use dev_use_generic

/* Declare an instance of the driver_s struct. */
DRIVER_DECLARE(
               /* driver global variable identifier */
               mydrv0_drv,

               /* flags and description */
               0, "Simple driver example",

               /* prefix of *_init, *_clenup and *_use function names */
               mydrv,

               /* implement no device class */
               NULL
               );

/* Statically register the driver in the list of available drivers.
   This is actually only needed for dynamic device enumeration and
   shell listing of drivers. */
DRIVER_REGISTER(mydrv0_drv);

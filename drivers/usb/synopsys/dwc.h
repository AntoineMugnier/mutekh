#include <string.h>
#include <stdio.h>

#include <hexo/types.h>
#include <hexo/iospace.h>
#include <hexo/endian.h>
#include <hexo/bit.h>

#include <device/device.h>
#include <device/resources.h>
#include <device/driver.h>
#include <device/class/iomux.h>
#include <device/clock.h>

#include <mutek/printk.h>
#include <mutek/mem_alloc.h>
#include <mutek/kroutine.h>

#include <device/class/usbdev.h>
#include <device/usb/usb.h>

#define SYNOPSYS_USB_CTRL_RAM_SIZE 512 /* Size in 32 bit word */
#define SYNOPSYS_USBDEV_EP_MSK ((1 << (CONFIG_DRIVER_USB_SYNOPSYS_EP_COUNT + 1)) - 1)

enum synopsys_usbdev_disable_state_e
{
  SYNOPSYS_USBDEV_WAIT_SNAK,
  SYNOPSYS_USBDEV_WAIT_DIS,
};

struct synopsys_usbdev_private_s
{
  struct dev_usbdev_context_s usbdev_ctx;
  /* usb address */
  uintptr_t addr;
  /* Endpoint */
  struct usbdev_endpoint_s epi[CONFIG_DRIVER_USB_SYNOPSYS_EP_COUNT];
  struct usbdev_endpoint_s epo[CONFIG_DRIVER_USB_SYNOPSYS_EP_COUNT];
  /* On-going transfer */
  struct dev_usbdev_config_s * cfg;
  struct dev_usbdev_request_s *tri[CONFIG_DRIVER_USB_SYNOPSYS_EP_COUNT + 1];
  struct dev_usbdev_request_s *tro[CONFIG_DRIVER_USB_SYNOPSYS_EP_COUNT + 1];
  /* Disabling mask */
  uint8_t imask, omask;
  uint8_t event;
  /* Control endpoint map */
  uint8_t ctrl;
  /* For endpoint disable */
  enum synopsys_usbdev_disable_state_e dstate:1;
  bool_t connected:1;
  /* Pending event */
  bool_t  pevent:1;
};

FIRST_FIELD_ASSERT(synopsys_usbdev_private_s, usbdev_ctx)

struct usbdev_endpoint_s * synopsys_usbdev_endpoint(struct synopsys_usbdev_private_s *pv, 
                                                    enum usb_endpoint_dir_e dir,
                                                    uint8_t address);

error_t synopsys_usbdev_config(struct device_s *dev,
                               struct synopsys_usbdev_private_s *pv,
                               struct dev_usbdev_config_s * cfg);

error_t synopsys_usbdev_transfer(struct synopsys_usbdev_private_s *pv,
                                 struct dev_usbdev_request_s * tr);

bool_t synopsys_usb_irq(struct synopsys_usbdev_private_s *pv);

void synopsys_usb_reset_device(struct synopsys_usbdev_private_s *pv);

void synopsys_usbdev_start(struct synopsys_usbdev_private_s *pv);

DEV_USBDEV_ALLOC(synopsys_usbdev_alloc);

DEV_USBDEV_FREE(synopsys_usbdev_free);

void synopsys_usbdev_event(struct synopsys_usbdev_private_s *pv,
                           uint8_t event);

/* Send event to stack */
void synopsys_usbdev_stack_event(struct synopsys_usbdev_private_s *pv);

/* Abort a transfer on endpoint 0 after a event has occured */
void synopsys_usbdev_abort_ep0(struct synopsys_usbdev_private_s *pv);


enum usb_dfu_state_e
{
  USB_DFU_STATE_APP_IDLE            = 0x00,
  USB_DFU_STATE_APP_DETACH          = 0x01,
  USB_DFU_STATE_IDLE                = 0x02,
  USB_DFU_STATE_DOWNLOAD_SYNC       = 0x03,
  USB_DFU_STATE_DOWNLOAD_BUSY       = 0x04,
  USB_DFU_STATE_DOWNLOAD_IDLE       = 0x05,
  USB_DFU_STATE_MANIFEST_SYNC       = 0x06,
  USB_DFU_STATE_MANIFEST            = 0x07,
  USB_DFU_STATE_MANIFEST_WAIT_RESET = 0x08,
  USB_DFU_STATE_UPLOAD_IDLE         = 0x09,
  USB_DFU_STATE_ERROR               = 0x0a,
};

enum usb_dfu_status_e
{
  USB_DFU_STATUS_OK                 = 0x00,
  USB_DFU_STATUS_ERROR_TARGET       = 0x01,
  USB_DFU_STATUS_ERROR_FILE         = 0x02,
  USB_DFU_STATUS_ERROR_WRITE        = 0x03,
  USB_DFU_STATUS_ERROR_ERASE        = 0x04,
  USB_DFU_STATUS_ERROR_CHECK_ERASED = 0x05,
  USB_DFU_STATUS_ERROR_PROG         = 0x06,
  USB_DFU_STATUS_ERROR_VERIFY       = 0x07,
  USB_DFU_STATUS_ERROR_ADDRESS      = 0x08,
  USB_DFU_STATUS_ERROR_NOTDONE      = 0x09,
  USB_DFU_STATUS_ERROR_FIRMWARE     = 0x0a,
  USB_DFU_STATUS_ERROR_VENDOR       = 0x0b,
  USB_DFU_STATUS_ERROR_USBR         = 0x0c,
  USB_DFU_STATUS_ERROR_POR          = 0x0d,
  USB_DFU_STATUS_ERROR_UNKNOWN      = 0x0e,
  USB_DFU_STATUS_ERROR_STALLEDPKT   = 0x0f,
};

enum usb_dfu_request_e
{
  USB_DFU_REQUEST_DETACH    = 0,
  USB_DFU_REQUEST_DNLOAD    = 1,
  USB_DFU_REQUEST_UPLOAD    = 2,
  USB_DFU_REQUEST_GETSTATUS = 3,
  USB_DFU_REQUEST_CLRSTATUS = 4,
  USB_DFU_REQUEST_GETSTATE  = 5,
  USB_DFU_REQUEST_ABORT     = 6,
};

struct usb_dfu_status_s
{
  enum usb_dfu_status_e bStatus:8;
  uint16_t bwPollTimeoutLow;
  uint8_t bwPollTimeoutHigh;
  enum usb_dfu_state_e bState:8;
  uint8_t iString;
}__attribute__((packed));

struct usb_dfu_functional_descriptor_s
{
  struct usb_descriptor_header_s head;
  uint8_t  bmAttributes;
  uint16_t  wDetachTimeOut;
  uint16_t  wTransferSize;
  uint16_t bcdDFUVersion;
}__attribute__((packed));

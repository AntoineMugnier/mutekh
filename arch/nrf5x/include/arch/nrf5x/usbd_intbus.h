/***************************************
* Auto generated by BFGen, do not edit *
***************************************/

/*
   bfgen -o cdefs cdefs_use_reg_mask=1 cdefs_use_field_setval=1                \
     cdefs_use_field_shift=1 cdefs_use_field_shifted_mask=1                    \
     cdefs_sfx_field_shifter=_SHIFT_VAL
*/

#ifndef _USBD_INTBUS_BFGEN_DEFS_
#define _USBD_INTBUS_BFGEN_DEFS_

#define USBD_INTBUS_IN_IRQ_ADDR                      0x000007a9
#define USBD_INTBUS_IN_IRQ_MASK                      0x000000ff
  #define USBD_INTBUS_IN_IRQ_INIR_COUNT            8
  #define USBD_INTBUS_IN_IRQ_INIR(fidx)            (0x01 << ((fidx)))
  #define USBD_INTBUS_IN_IRQ_INIR_SHIFT(fidx)      ((fidx) + 0)

#define USBD_INTBUS_OUT_IRQ_ADDR                     0x000007aa
#define USBD_INTBUS_OUT_IRQ_MASK                     0x000000ff
  #define USBD_INTBUS_OUT_IRQ_OUTIR_COUNT          8
  #define USBD_INTBUS_OUT_IRQ_OUTIR(fidx)          (0x01 << ((fidx)))
  #define USBD_INTBUS_OUT_IRQ_OUTIR_SHIFT(fidx)    ((fidx) + 0)

#define USBD_INTBUS_USB_IRQ_ADDR                     0x000007ab
#define USBD_INTBUS_USB_IRQ_MASK                     0x00000000

#define USBD_INTBUS_IN_IEN_ADDR                      0x000007ac
#define USBD_INTBUS_IN_IEN_MASK                      0x000000ff
  #define USBD_INTBUS_IN_IEN_INIEN_COUNT           8
  #define USBD_INTBUS_IN_IEN_INIEN(fidx)           (0x01 << ((fidx)))
  #define USBD_INTBUS_IN_IEN_INIEN_SHIFT(fidx)     ((fidx) + 0)

#define USBD_INTBUS_OUT_IEN_ADDR                     0x000007ad
#define USBD_INTBUS_OUT_IEN_MASK                     0x000000ff
  #define USBD_INTBUS_OUT_IEN_OUTIEN_COUNT         8
  #define USBD_INTBUS_OUT_IEN_OUTIEN(fidx)         (0x01 << ((fidx)))
  #define USBD_INTBUS_OUT_IEN_OUTIEN_SHIFT(fidx)   ((fidx) + 0)

#define USBD_INTBUS_USB_IEN_ADDR                     0x000007ae
#define USBD_INTBUS_USB_IEN_MASK                     0x0000003f
  #define USBD_INTBUS_USB_IEN_SUDAV                0x01
  #define USBD_INTBUS_USB_IEN_SUDAV_SHIFT          0
  #define USBD_INTBUS_USB_IEN_SOF                  0x02
  #define USBD_INTBUS_USB_IEN_SOF_SHIFT            1
  #define USBD_INTBUS_USB_IEN_SUTOK                0x04
  #define USBD_INTBUS_USB_IEN_SUTOK_SHIFT          2
  #define USBD_INTBUS_USB_IEN_SUSP                 0x08
  #define USBD_INTBUS_USB_IEN_SUSP_SHIFT           3
  #define USBD_INTBUS_USB_IEN_URES                 0x10
  #define USBD_INTBUS_USB_IEN_URES_SHIFT           4
  #define USBD_INTBUS_USB_IEN_IBN                  0x20
  #define USBD_INTBUS_USB_IEN_IBN_SHIFT            5

/** Should be reset to 64 upon startup @multiple */
#define USBD_INTBUS_ISO_SIZE_ADDR                    0x000007e3
#define USBD_INTBUS_ISO_SIZE_MASK                    0x00000000

#endif

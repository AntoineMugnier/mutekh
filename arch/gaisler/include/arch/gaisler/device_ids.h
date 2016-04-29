
#ifndef _GAISLER_DEVICE_IDS_
#define _GAISLER_DEVICE_IDS_

#include <hexo/enum.h>

enum gaisler_vendor_ids_e
{
  GAISLER_VENDOR_GAISLER    = 0x01,
  GAISLER_VENDOR_PENDER     = 0x02,
  GAISLER_VENDOR_ESA        = 0x04,
  GAISLER_VENDOR_ASTRIUM    = 0x06,
  GAISLER_VENDOR_OPENCHIP   = 0x07,
  GAISLER_VENDOR_OPENCORES  = 0x08,
  GAISLER_VENDOR_CONTRIB    = 0x09,
  GAISLER_VENDOR_EONIC      = 0x0b,
  GAISLER_VENDOR_RADIONOR   = 0x0f,
  GAISLER_VENDOR_GLEICHMANN = 0x10,
  GAISLER_VENDOR_MENTA      = 0x11,
  GAISLER_VENDOR_SUN        = 0x13,
  GAISLER_VENDOR_EMBEDDIT   = 0xea,
  GAISLER_VENDOR_CAL        = 0xca,

  GAISLER_VENDOR_count      = 0xeb,
};

#ifdef CONFIG_GAISLER_DEVICE_IDS
extern const char *const gaisler_vendors_longnames[GAISLER_VENDOR_count];
#endif

ENUM_DESCRIPTOR(gaisler_device_ids_e, explicit);

enum gaisler_device_ids_e
{
  GAISLER_DEVICE_LEON2DSU  = 0x02,
  GAISLER_DEVICE_LEON3     = 0x03, /* name:cpu */
  GAISLER_DEVICE_LEON3DSU  = 0x04, /* name:dsu */
  GAISLER_DEVICE_ETHAHB    = 0x05,
  GAISLER_DEVICE_APBMST    = 0x06, /* name:apb */
  GAISLER_DEVICE_AHBUART   = 0x07, /* name:debug */
  GAISLER_DEVICE_SRCTRL    = 0x08, /* name:mem */
  GAISLER_DEVICE_SDCTRL    = 0x09, /* name:mem */
  GAISLER_DEVICE_SSRCTRL   = 0x0A, /* name:mem */
  GAISLER_DEVICE_I2C2AHB   = 0x0B, /* name:debug */
  GAISLER_DEVICE_APBUART   = 0x0C, /* name:uart */
  GAISLER_DEVICE_IRQMP     = 0x0D, /* name:icu */
  GAISLER_DEVICE_AHBRAM    = 0x0E, /* name:mem */
  GAISLER_DEVICE_AHBDPRAM  = 0x0F, /* name:mem */
  GAISLER_DEVICE_GRIOMMU2  = 0x10,
  GAISLER_DEVICE_GPTIMER   = 0x11, /* name:timer */
  GAISLER_DEVICE_PCITRG    = 0x12,
  GAISLER_DEVICE_PCISBRG   = 0x13,
  GAISLER_DEVICE_PCIFBRG   = 0x14,
  GAISLER_DEVICE_PCITRACE  = 0x15,
  GAISLER_DEVICE_DMACTRL   = 0x16,
  GAISLER_DEVICE_AHBTRACE  = 0x17, /* name:trace */
  GAISLER_DEVICE_DSUCTRL   = 0x18, /* name:dsu */
  GAISLER_DEVICE_CANAHB    = 0x19,
  GAISLER_DEVICE_GPIO      = 0x1A, /* name:gpio */
  GAISLER_DEVICE_AHBROM    = 0x1B, /* name:mem */
  GAISLER_DEVICE_AHBJTAG   = 0x1C, /* name:debug */
  GAISLER_DEVICE_ETHMAC    = 0x1D, /* name:eth */
  GAISLER_DEVICE_SWNODE    = 0x1E,
  GAISLER_DEVICE_SPW       = 0x1F,
  GAISLER_DEVICE_AHB2AHB   = 0x20, /* name:ahb */
  GAISLER_DEVICE_USBDC     = 0x21, /* name:usbdev */
  GAISLER_DEVICE_USB_DCL   = 0x22, /* name:debug */
  GAISLER_DEVICE_DDRMP     = 0x23,
  GAISLER_DEVICE_ATACTRL   = 0x24,
  GAISLER_DEVICE_DDRSP     = 0x25, /* name:mem */
  GAISLER_DEVICE_EHCI      = 0x26, /* name:usb */
  GAISLER_DEVICE_UHCI      = 0x27, /* name:usb */
  GAISLER_DEVICE_I2CMST    = 0x28, /* name:i2c */
  GAISLER_DEVICE_SPW2      = 0x29,
  GAISLER_DEVICE_AHBDMA    = 0x2A,
  GAISLER_DEVICE_NUHOSP3   = 0x2B,
  GAISLER_DEVICE_CLKGATE   = 0x2C,
  GAISLER_DEVICE_SPICTRL   = 0x2D, /* name:spi */
  GAISLER_DEVICE_DDR2SP    = 0x2E, /* name:mem */
  GAISLER_DEVICE_SLINK     = 0x2F,
  GAISLER_DEVICE_GRTM      = 0x30,
  GAISLER_DEVICE_GRTC      = 0x31,
  GAISLER_DEVICE_GRPW      = 0x32,
  GAISLER_DEVICE_GRCTM     = 0x33,
  GAISLER_DEVICE_GRHCAN    = 0x34,
  GAISLER_DEVICE_GRFIFO    = 0x35,
  GAISLER_DEVICE_GRADCDAC  = 0x36,
  GAISLER_DEVICE_GRPULSE   = 0x37,
  GAISLER_DEVICE_GRTIMER   = 0x38, /* name:timer */
  GAISLER_DEVICE_AHB2PP    = 0x39,
  GAISLER_DEVICE_GRVERSION = 0x3A,
  GAISLER_DEVICE_APB2PW    = 0x3B,
  GAISLER_DEVICE_PW2APB    = 0x3C,
  GAISLER_DEVICE_GRCAN     = 0x3D,
  GAISLER_DEVICE_I2CSLV    = 0x3E, /* name:si2c */
  GAISLER_DEVICE_U16550    = 0x3F,
  GAISLER_DEVICE_AHBMST_EM = 0x40,
  GAISLER_DEVICE_AHBSLV_EM = 0x41,
  GAISLER_DEVICE_GRTESTMOD = 0x42,
  GAISLER_DEVICE_ASCS      = 0x43,
  GAISLER_DEVICE_IPMVBCTRL = 0x44,
  GAISLER_DEVICE_SPIMCTRL  = 0x45, /* name:mem */
  GAISLER_DEVICE_L4STAT    = 0x47,
  GAISLER_DEVICE_LEON4     = 0x48, /* name:cpu */
  GAISLER_DEVICE_LEON4DSU  = 0x49, /* name:dsu */
  GAISLER_DEVICE_PWM       = 0x4A, /* name:pwm */
  GAISLER_DEVICE_L2CACHE   = 0x4B, /* name:cache */
  GAISLER_DEVICE_SDCTRL64  = 0x4C, /* name:mem */
  GAISLER_DEVICE_GR1553B   = 0x4D,
  GAISLER_DEVICE_1553TST   = 0x4E,
  GAISLER_DEVICE_GRIOMMU   = 0x4F, /* name:iommu */
  GAISLER_DEVICE_FTAHBRAM  = 0x50, /* name:mem */
  GAISLER_DEVICE_FTSRCTRL  = 0x51, /* name:mem */
  GAISLER_DEVICE_AHBSTAT   = 0x52, /* name:mem */
  GAISLER_DEVICE_LEON3FT   = 0x53,
  GAISLER_DEVICE_FTMCTRL   = 0x54, /* name:mem */
  GAISLER_DEVICE_FTSDCTRL  = 0x55, /* name:mem */
  GAISLER_DEVICE_FTSRCTRL8 = 0x56, /* name:mem */
  GAISLER_DEVICE_MEMSCRUB  = 0x57, /* name:mem */
  GAISLER_DEVICE_FTSDCTRL64= 0x58, /* name:mem */
  GAISLER_DEVICE_NANDFCTRL = 0x59, /* name:mem */
  GAISLER_DEVICE_N2DLLCTRL = 0x5A,
  GAISLER_DEVICE_N2PLLCTRL = 0x5B,
  GAISLER_DEVICE_SPI2AHB   = 0x5C, /* name:debug */
  GAISLER_DEVICE_DDRSDMUX  = 0x5D,
  GAISLER_DEVICE_AHBFROM   = 0x5E,
  GAISLER_DEVICE_PCIEXP    = 0x5F,
  GAISLER_DEVICE_APBPS2    = 0x60, /* name:input */
  GAISLER_DEVICE_VGACTRL   = 0x61, /* name:tty */
  GAISLER_DEVICE_LOGAN     = 0x62, /* name:debug */
  GAISLER_DEVICE_SVGACTRL  = 0x63, /* name:fb */
  GAISLER_DEVICE_T1AHB     = 0x64,
  GAISLER_DEVICE_MP7WRAP   = 0x65,
  GAISLER_DEVICE_GRSYSMON  = 0x66,
  GAISLER_DEVICE_GRACECTRL = 0x67,
  GAISLER_DEVICE_ATAHBSLV  = 0x68,
  GAISLER_DEVICE_ATAHBMST  = 0x69,
  GAISLER_DEVICE_ATAPBSLV  = 0x6A,
  GAISLER_DEVICE_MIGDDR2   = 0x6B,
  GAISLER_DEVICE_LCDCTRL   = 0x6C,
  GAISLER_DEVICE_SWITCHOVER= 0x6D,
  GAISLER_DEVICE_FIFOUART  = 0x6E,
  GAISLER_DEVICE_MUXCTRL   = 0x6F,
  GAISLER_DEVICE_B1553BC   = 0x70,
  GAISLER_DEVICE_B1553RT   = 0x71,
  GAISLER_DEVICE_B1553BRM  = 0x72,
  GAISLER_DEVICE_AES       = 0x73, /* name:aes */
  GAISLER_DEVICE_ECC       = 0x74, /* name:crypto */
  GAISLER_DEVICE_PCIF      = 0x75,
  GAISLER_DEVICE_CLKMOD    = 0x76,
  GAISLER_DEVICE_HAPSTRAK  = 0x77,
  GAISLER_DEVICE_TEST_1X2  = 0x78,
  GAISLER_DEVICE_WILD2AHB  = 0x79,
  GAISLER_DEVICE_BIO1      = 0x7A,
  GAISLER_DEVICE_AESDMA    = 0x7B, /* name:aes */
  GAISLER_DEVICE_GRPCI2    = 0x7C, /* name:pci */
  GAISLER_DEVICE_GRPCI2_DMA= 0x7D,
  GAISLER_DEVICE_GRPCI2_TB = 0x7E,
  GAISLER_DEVICE_MMA       = 0x7F,
  GAISLER_DEVICE_SATCAN    = 0x80,
  GAISLER_DEVICE_CANMUX    = 0x81,
  GAISLER_DEVICE_GRTMRX    = 0x82,
  GAISLER_DEVICE_GRTCTX    = 0x83,
  GAISLER_DEVICE_GRTMDESC  = 0x84,
  GAISLER_DEVICE_GRTMVC    = 0x85,
  GAISLER_DEVICE_GEFFE     = 0x86,
  GAISLER_DEVICE_GPREG     = 0x87,
  GAISLER_DEVICE_GRTMPAHB  = 0x88,
  GAISLER_DEVICE_SPWCUC    = 0x89,
  GAISLER_DEVICE_SPW2_DMA  = 0x8A,
  GAISLER_DEVICE_SPWROUTER = 0x8B,
  GAISLER_DEVICE_EDCLMST   = 0x8C,
  GAISLER_DEVICE_GRPWTX    = 0x8D,
  GAISLER_DEVICE_GRPWRX    = 0x8E,
  GAISLER_DEVICE_GPREGBANK = 0x8F,
  GAISLER_DEVICE_MIG_7SERIES = 0x90,
  GAISLER_DEVICE_GRSPW2_SIST = 0x91,
  GAISLER_DEVICE_SGMII     = 0x92,
  GAISLER_DEVICE_RGMII     = 0x93, /* name:mii */
  GAISLER_DEVICE_IRQGEN    = 0x94,
  GAISLER_DEVICE_GRDMAC    = 0x95,
  GAISLER_DEVICE_AHB2AVLA  = 0x96, /* name:avalon */
  GAISLER_DEVICE_SPWTDP    = 0x97,
  GAISLER_DEVICE_L3STAT    = 0x98,
  GAISLER_DEVICE_GR740THS  = 0x99,
  GAISLER_DEVICE_GRRM      = 0x9A,
  GAISLER_DEVICE_CMAP      = 0x9B,
  GAISLER_DEVICE_CPGEN     = 0x9C,
  GAISLER_DEVICE_AMBAPROT  = 0x9D,
  GAISLER_DEVICE_IGLOO2_BRIDGE = 0x9E,
  GAISLER_DEVICE_AHB2AXI   = 0x9F,
  GAISLER_DEVICE_AXI2AHB   = 0xA0,
  GAISLER_DEVICE_FDIR_RSTCTRL = 0xA1,
  GAISLER_DEVICE_APB3MST   = 0xA2,
  GAISLER_DEVICE_LRAM      = 0xA3,
  GAISLER_DEVICE_BOOTSEQ   = 0xA4,

  GAISLER_DEVICE_count     = 0xA5
};

extern const char *const gaisler_devices_names[GAISLER_DEVICE_count];

#ifdef CONFIG_GAISLER_DEVICE_IDS
extern const char *const gaisler_devices_longnames[GAISLER_DEVICE_count];
#endif

#endif


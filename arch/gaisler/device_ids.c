
#include <arch/device_ids.h>

const char *const gaisler_devices_names[GAISLER_DEVICE_count] =
  {
    [GAISLER_DEVICE_LEON3      ] = "cpu",
    [GAISLER_DEVICE_ETHAHB     ] = "eth",
    [GAISLER_DEVICE_APBMST     ] = "apb",
    [GAISLER_DEVICE_AHBRAM     ] = "memory",
    [GAISLER_DEVICE_AHBUART    ] = "uart",
    [GAISLER_DEVICE_SRCTRL     ] = "memory",
    [GAISLER_DEVICE_SDCTRL     ] = "memory",
    [GAISLER_DEVICE_SSRCTRL    ] = "memory",
    [GAISLER_DEVICE_APBUART    ] = "uart",
    [GAISLER_DEVICE_IRQMP      ] = "icu",
    [GAISLER_DEVICE_GPTIMER    ] = "timer",
    [GAISLER_DEVICE_PCITRG     ] = "pci",
    [GAISLER_DEVICE_PCISBRG    ] = "pci",
    [GAISLER_DEVICE_PCIFBRG    ] = "pci",
    [GAISLER_DEVICE_DMACTRL    ] = "dma",
    [GAISLER_DEVICE_GRPULSE    ] = "gpio",
    [GAISLER_DEVICE_GRTIMER    ] = "timer",
    [GAISLER_DEVICE_GPIO       ] = "gpio",
    [GAISLER_DEVICE_AHBROM     ] = "memory",
    [GAISLER_DEVICE_FTAHBRAM   ] = "memory",
    [GAISLER_DEVICE_FTSRCTRL   ] = "memory",
    [GAISLER_DEVICE_LEON3FT    ] = "cpu",
    [GAISLER_DEVICE_ETHMAC     ] = "eth",
    [GAISLER_DEVICE_VGACTRL    ] = "fb",
    [GAISLER_DEVICE_APBPS2     ] = "input",
    [GAISLER_DEVICE_SVGACTRL   ] = "fb",
    [GAISLER_DEVICE_AES        ] = "crypto",
    [GAISLER_DEVICE_ECC        ] = "crypto",
    [GAISLER_DEVICE_USBCTRL    ] = "usb",
    [GAISLER_DEVICE_DDRMP      ] = "memory",
    [GAISLER_DEVICE_ATACTRL    ] = "block",
    [GAISLER_DEVICE_DDRSP      ] = "memory",
    [GAISLER_DEVICE_LEON4      ] = "cpu",
    [GAISLER_DEVICE_EHCI       ] = "usb",
    [GAISLER_DEVICE_UHCI       ] = "usb",
  };

#ifdef CONFIG_GAISLER_DEVICE_IDS

const char *const gaisler_vendors_longnames[GAISLER_VENDOR_count] =
  {
    [GAISLER_VENDOR_GAISLER    ] = "Gaisler Research",
    [GAISLER_VENDOR_ESA        ] = "European Space Agency",
    [GAISLER_VENDOR_OPENCHIP   ] = "OpenChip",
    [GAISLER_VENDOR_OPENCORES  ] = "OpenCores",
    [GAISLER_VENDOR_CONTRIB    ] = "Various contributions",
    [GAISLER_VENDOR_EONIC      ] = "Eonic BV",
    [GAISLER_VENDOR_GLEICHMANN ] = "Gleichmann Electronics",
    [GAISLER_VENDOR_MENTA      ] = "Menta",
    [GAISLER_VENDOR_EMBEDDIT   ] = "Embedd.it",
    [GAISLER_VENDOR_SUN        ] = "Sun Microsystems",
    [GAISLER_VENDOR_RADIONOR   ] = "Radionor Communications",
  };

const char *const gaisler_devices_longnames[GAISLER_DEVICE_count] =
  {
    [GAISLER_DEVICE_LEON2DSU   ] = "Leon2 Debug Support Unit",
    [GAISLER_DEVICE_LEON3      ] = "Leon3 SPARC V8 Processor",
    [GAISLER_DEVICE_LEON3DSU   ] = "Leon3 Debug Support Unit",
    [GAISLER_DEVICE_ETHAHB     ] = "OC ethernet AHB interface",
    [GAISLER_DEVICE_AHBRAM     ] = "Generic AHB SRAM module",
    [GAISLER_DEVICE_APBMST     ] = "AHB/APB Bridge",
    [GAISLER_DEVICE_AHBUART    ] = "AHB Debug UART",
    [GAISLER_DEVICE_SRCTRL     ] = "Simple SRAM Controller",
    [GAISLER_DEVICE_SDCTRL     ] = "PC133 SDRAM Controller",
    [GAISLER_DEVICE_SSRCTRL    ] = "Synchronous SRAM Controller",
    [GAISLER_DEVICE_APBUART    ] = "Generic UART",
    [GAISLER_DEVICE_IRQMP      ] = "Multi-processor Interrupt Ctrl",
    [GAISLER_DEVICE_GPTIMER    ] = "Modular Timer Unit",
    [GAISLER_DEVICE_PCITRG     ] = "Simple 32-bit PCI Target",
    [GAISLER_DEVICE_PCISBRG    ] = "Simple 32-bit PCI Bridge",
    [GAISLER_DEVICE_PCIFBRG    ] = "Fast 32-bit PCI Bridge",
    [GAISLER_DEVICE_PCITRACE   ] = "32-bit PCI Trace Buffer",
    [GAISLER_DEVICE_DMACTRL    ] = "AMBA DMA controller",
    [GAISLER_DEVICE_AHBTRACE   ] = "AMBA Trace Buffer",
    [GAISLER_DEVICE_DSUCTRL    ] = "DSU/ETH controller",
    [GAISLER_DEVICE_GRTM       ] = "CCSDS Telemetry Encoder",
    [GAISLER_DEVICE_GRTC       ] = "CCSDS Telecommand Decoder",
    [GAISLER_DEVICE_GRPW       ] = "PacketWire to AMBA AHB I/F",
    [GAISLER_DEVICE_GRCTM      ] = "CCSDS Time Manager",
    [GAISLER_DEVICE_GRHCAN     ] = "ESA HurriCANe CAN with DMA",
    [GAISLER_DEVICE_GRFIFO     ] = "FIFO Controller",
    [GAISLER_DEVICE_GRADCDAC   ] = "ADC / DAC Interface",
    [GAISLER_DEVICE_GRPULSE    ] = "General Purpose I/O with Pulses",
    [GAISLER_DEVICE_GRTIMER    ] = "Timer Unit with Latches",
    [GAISLER_DEVICE_AHB2PP     ] = "AMBA AHB to Packet Parallel I/F",
    [GAISLER_DEVICE_GRVERSION  ] = "Version and Revision Register",
    [GAISLER_DEVICE_APB2PW     ] = "PacketWire Transmit Interface",
    [GAISLER_DEVICE_PW2APB     ] = "PacketWire Receive Interface",
    [GAISLER_DEVICE_AHBMST_EM  ] = "AMBA Master Emulator",
    [GAISLER_DEVICE_AHBSLV_EM  ] = "AMBA Slave Emulator",
    [GAISLER_DEVICE_CANAHB     ] = "OC CAN AHB interface",
    [GAISLER_DEVICE_GPIO       ] = "General Purpose I/O port",
    [GAISLER_DEVICE_AHBROM     ] = "Generic AHB ROM",
    [GAISLER_DEVICE_AHB2AHB    ] = "AHB-to-AHB Bridge",
    [GAISLER_DEVICE_NUHOSP3    ] = "Nuhorizons Spartan3 IO I/F",
    [GAISLER_DEVICE_FTAHBRAM   ] = "Generic FT AHB SRAM module",
    [GAISLER_DEVICE_FTSRCTRL   ] = "Simple FT SRAM Controller",
    [GAISLER_DEVICE_LEON3FT    ] = "Leon3-FT SPARC V8 Processor",
    [GAISLER_DEVICE_FTMCTRL    ] = "Memory controller with EDAC",
    [GAISLER_DEVICE_AHBSTAT    ] = "AHB Status Register",
    [GAISLER_DEVICE_AHBJTAG    ] = "JTAG Debug Link",
    [GAISLER_DEVICE_ETHMAC     ] = "GR Ethernet MAC",
    [GAISLER_DEVICE_SWNODE     ] = "SpaceWire Node Interface",
    [GAISLER_DEVICE_SPW        ] = "SpaceWire Serial Link",
    [GAISLER_DEVICE_VGACTRL    ] = "VGA controller",
    [GAISLER_DEVICE_APBPS2     ] = "PS2 interface",
    [GAISLER_DEVICE_LOGAN      ] = "On chip Logic Analyzer",
    [GAISLER_DEVICE_SVGACTRL   ] = "SVGA frame buffer",
    [GAISLER_DEVICE_B1553BC    ] = "AMBA Wrapper for Core1553BBC",
    [GAISLER_DEVICE_B1553RT    ] = "AMBA Wrapper for Core1553BRT",
    [GAISLER_DEVICE_B1553BRM   ] = "AMBA Wrapper for Core1553BRM",
    [GAISLER_DEVICE_AES        ] = "Advanced Encryption Standard",
    [GAISLER_DEVICE_ECC        ] = "Elliptic Curve Cryptography",
    [GAISLER_DEVICE_PCIF       ] = "AMBA Wrapper for CorePCIF",
    [GAISLER_DEVICE_USBCTRL    ] = "USB 2.0 Controller",
    [GAISLER_DEVICE_USBDCL     ] = "USB Debug Communication Link",
    [GAISLER_DEVICE_DDRMP      ] = "Multi-port DDR controller",
    [GAISLER_DEVICE_ATACTRL    ] = "ATA controller",
    [GAISLER_DEVICE_DDRSP      ] = "Single-port DDR266 controller",
    [GAISLER_DEVICE_GRTESTMOD  ] = "Test report module",
    [GAISLER_DEVICE_LEON4      ] = "Leon4 SPARC V8 Processor",
    [GAISLER_DEVICE_EHCI       ] = "USB EHCI controller",
    [GAISLER_DEVICE_UHCI       ] = "USB UHCI controller",
  };

#endif

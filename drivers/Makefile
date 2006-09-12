
ifeq ($(ARCH), ibmpc)
  include drivers/fb-vga/Makefile
  include drivers/icu-8259/Makefile
  include drivers/timer-8253/Makefile
  include drivers/tty-vga/Makefile
  include drivers/uart-8250/Makefile
  include drivers/enum-pci/Makefile
  include drivers/net-ns8390/Makefile
  include drivers/net-3c900/Makefile
endif

ifeq ($(ARCH), soclib)
  include drivers/tty-soclib/Makefile
  include drivers/icu-soclib/Makefile
  include drivers/timer-soclib/Makefile
endif



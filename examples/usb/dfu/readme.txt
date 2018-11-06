DFU Example
===========

This example is meant to be run on EFM32's STK3600.

Overview
--------

This example is split in two parts:
- A DFU bootloader that sits at start of flash,
- A target functional firmware.

Bootloader will stay in DFU mode if any of the following conditions
happen:
- Target firmware looks bad (too many 0s or ffs at the start of
  image),
- PB1 (the board button, not the EFM pin) is pressed on power-on.

Functional firmware simply exposes a CDC-ACM device with MutekH's
shell on it.

How to generate binaries
------------------------

Generate the bootloader image::

  $ make CONF=examples/usb/dfu/functional/config BUILD=efm32-stk3600:binary

Generate the target firmware and the matching `DFU` image::

  $ make CONF=examples/usb/dfu/bootloader/config BUILD=efm32-stk3600:binary
  $ python3 examples/usb/dfu/dfu_gen.py \
      dfu_functional-efm32-leopard.bin \
      dfu_functional-efm32-leopard.dfu \
      --vid 4242 --pid dead

Load bootloader on target
-------------------------

The usual way, with you programmer software of choice, load
`dfu-efm32-leopard.bin` on the target.

When bootloader is running, LED0 should be on, and device should
enumerate a DFU class service::

  $ dfu-util -l
  dfu-util 0.9
  
  Found DFU: [4242:dead] ver=0200, devnum=55, cfg=1, intf=0, path="1-5.1.1.1", alt=0, name="Firmware Upgrade", serial="DFU"

Load functional firmware
------------------------

`dfu-util` will happily use `.dfu` file and load it to the target.

::

  $ dfu-util -D dfu_functional-efm32-leopard.dfu

When functional firmware is running, LED1 should be on and device
should enumerate a CDC-ACM service.

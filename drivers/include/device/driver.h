/*
    This file is part of MutekH.

    MutekH is free software; you can redistribute it and/or modify it
    under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    MutekH is distributed in the hope that it will be useful, but
    WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with MutekH; if not, write to the Free Software Foundation,
    Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA

    Copyright Alexandre Becoulet <alexandre.becoulet@lip6.fr> (c) 2006

*/

#ifndef __DRIVER_H__
#define __DRIVER_H__

#include <hexo/types.h>
#include <hexo/error.h>
#include <hexo/device.h>

#define DEVENUM_TYPE_PCI 0x01
#define DEVENUM_TYPE_ISA 0x02
#define DEVENUM_TYPE_ATA 0x03

/** device structure identification informations. wildcard values are
    enum driver dependent */
struct devenum_ident_s
{
	uint8_t type;
	union {
		struct {
			uint16_t vendor;
			uint16_t device;
			uint32_t class;
		} pci;
		struct {
			uint16_t vendor;
		} isa;
		struct {
			const char *str;
		} ata;
	};
};


/**
   Shortcut for creating a PCI entry in a static devenum_ident_s
   array.

   @param _vendor the vendor id to match, -1 for wildcard
   @param _device the device id to match, -1 for wildcard
   @param _class the class to match, -1 for wildcard
 */
#define DEVENUM_PCI_ENTRY(_vendor, _device, _class)		\
	{ .type = DEVENUM_TYPE_PCI, { .pci = {				\
				.vendor = _vendor, .device = _device,	\
				.class = _class } } }

/**
   Shortcut for creating an ISA entry in a static devenum_ident_s
   array.

   @param _vendor the vendor id to match
 */
#define DEVENUM_ISA_ENTRY(_vendor)						\
	{ .type = DEVENUM_TYPE_PCI, { .isa = {				\
				.vendor = _vendor } } }

/**
   Shortcut for creating an ATA entry in a static devenum_ident_s
   array.

   @param _str the string to match from the device
 */
#define DEVENUM_ATA_ENTRY(_str)							\
	{ .type = DEVENUM_TYPE_ATA, { .ata = {				\
				.str = _str } } }

/** device driver object structure */

#define DRV_MAX_FUNC_COUNT	6

struct driver_s
{
  /* device class */
  enum device_class_e		class;

  /* device identifier table for detection (optional) */
  const struct devenum_ident_s	*id_table;

  dev_create_t			*f_create;
  dev_init_t			*f_init;
  dev_cleanup_t			*f_cleanup;
  dev_irq_t			*f_irq;

  union {
    void			*ptrs[DRV_MAX_FUNC_COUNT];

#ifdef __DEVICE_CHAR_H__
    struct dev_class_char_s	chr;
#endif

#ifdef __DEVICE_ICU_H__
    /** interrupt controller devices */
    struct dev_class_icu_s	icu;
#endif

#ifdef __DEVICE_FB_H__
    /** frame buffer devices */
    struct dev_class_fb_s	fb;
#endif

#ifdef __DEVICE_TIMER_H__
    struct dev_class_timer_s	timer;
#endif

#ifdef __DEVICE_INPUT_H__
    struct dev_class_input_s	input;
#endif

#ifdef __DEVICE_ENUM_H__
    /** device enumerator class */
    struct dev_class_enum_s	denum;
#endif

#ifdef __DEVICE_NET_H__
    struct dev_class_net_s	net;
#endif

#ifdef __DEVICE_SOUND_H__
    struct dev_class_sound_s	sound;
#endif

#ifdef __DEVICE_BLOCK_H__
    struct dev_class_block_s	blk;
#endif

#ifdef __DEVICE_SPI_H__
    struct dev_class_spi_s	spi;
#endif

#ifdef __DEVICE_LCD_H__
    struct dev_class_lcd_s	lcd;
#endif

#ifdef __DEVICE_GPIO_H__
    struct dev_class_gpio_s	gpio;
#endif

#ifdef __DEVICE_I2C_H__
    struct dev_class_i2c_s	i2c;
#endif
  } f;
};

/**
   Registers a driver (struct driver_s) in the global_driver_registry
   table.
 */
#define REGISTER_DRIVER(name) \
	const __attribute__((section (".drivers"))) \
	const struct driver_s *name##_drv_ptr = &name


/**
   Try to get a driver registered with these characteristics

   @param vendor Vendor of PCI device
   @param vendor Device of PCI device
   @param vendor Class of PCI device
   @return A driver if found, NULL otherwise
 */
struct driver_s *driver_get_matching_pci(
	uint16_t vendor,
	uint16_t device,
	uint32_t class);

/**
   Try to get a driver registered with these characteristics

   @param vendor Vendor of ISA device
   @return A driver if found, NULL otherwise
 */
struct driver_s *driver_get_matching_isa(
	uint16_t vendor);

/**
   Try to get a driver registered with these characteristics

   @param name Name of ata device
   @return A driver if found, NULL otherwise
 */
struct driver_s *driver_get_matching_ata(
	const char *name);

#endif

/*
    This file is part of MutekH.
    
    MutekH is free software; you can redistribute it and/or modify it
    under the terms of the GNU Lesser General Public License as
    published by the Free Software Foundation; version 2.1 of the
    License.
    
    MutekH is distributed in the hope that it will be useful, but
    WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.
    
    You should have received a copy of the GNU Lesser General Public
    License along with MutekH; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
    02110-1301 USA.

    Copyright (c) Eric Guthmuller 2010

*/

#ifndef DRIVER_SPI_OC_SPI_CONTROLLER_H_
#define DRIVER_SPI_OC_SPI_CONTROLLER_H_

#include <device/class/spi.h>
#include <device/device.h>

struct spi_oc_spi_controller_param_s
{
	uint_fast8_t lun_count;
};

DEV_INIT(spi_oc_spi_controller_init);
DEV_CLEANUP(spi_oc_spi_controller_cleanup);
DEV_IRQ(spi_oc_spi_controller_irq);
DEV_SPI_SET_BAUDRATE(spi_oc_spi_controller_set_baudrate);
DEV_SPI_SET_DATA_FORMAT(spi_oc_spi_controller_set_data_format);
DEV_SPI_REQUEST(spi_oc_spi_controller_request);

#endif


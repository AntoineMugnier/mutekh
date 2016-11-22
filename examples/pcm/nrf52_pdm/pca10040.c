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
    License along with this program.  If not, see
    <http://www.gnu.org/licenses/>.

    Copyright Nicolas Pouillon <nipo@ssji.net> (c) 2016
*/

#include <hexo/bit.h>
#include <arch/nrf5x/ids.h>

#ifdef CONFIG_DEVICE
# include <device/class/pcm.h>
# include <device/class/iomux.h>
# include <device/irq.h>
# include <device/resources.h>
# include <device/irq.h>
# include <arch/nrf5x/ids.h>
#endif

#if defined(CONFIG_DRIVER_NRF52_PDM)
DEV_DECLARE_STATIC(pcm_dev, "pcm0", 0, nrf52_pdm_drv,
                   NRF_STATIC_RES_PERIPHERAL_MEM(NRF5X_PDM),
                   DEV_STATIC_RES_DEV_ICU("/cpu"),
                   DEV_STATIC_RES_IRQ(0, NRF5X_PDM, DEV_IRQ_SENSE_HIGH_LEVEL, 0, 1),
                   DEV_STATIC_RES_DEV_IOMUX("/gpio"),
                   DEV_STATIC_RES_IOMUX("clk", 0, 3, 0, 0),
                   DEV_STATIC_RES_IOMUX("din", 0, 4, 0, 0)
                   );
#endif

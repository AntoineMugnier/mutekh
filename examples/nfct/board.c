#include <arch/nrf5x/ids.h>
#include <device/resources.h>
#include <device/class/nfc.h>
#include <device/class/icu.h>

DEV_DECLARE_STATIC(nfct_dev, "nfc0", 0, nrf52_nfct_drv,
                   NRF_STATIC_RES_PERIPHERAL_MEM(NRF5X_NFCT),
                   DEV_STATIC_RES_DEV_ICU("/cpu"),
                   DEV_STATIC_RES_IRQ(0, NRF5X_NFCT, DEV_IRQ_SENSE_HIGH_LEVEL, 0, 1),
                   );

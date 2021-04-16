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
*/

/* This example is intended to demo the static and extern rfpacket
   configuration possibilites on a s2lp hooked to an efm32 demo board
*/

// Standard lib
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
// Mutekh lib
#include <hexo/iospace.h>
#include <mutek/printk.h>
#include <mutek/startup.h>
#include <mutek/thread.h>
#include <mutek/kroutine.h>
// Devices lib
#include <device/class/gpio.h>
#include <device/class/timer.h>
#include <device/class/rfpacket.h>
// Other
#include <arch/efm32/pin.h>

// *** Definitions ***
// --- Private Constants ---
#define RFPACKET_WAIT_MS 500 // General purpose waiting time
#define RFPACKET_TEST_STEP_TIME_MS 25 // Waiting time between test steps
#define RFPACKET_RX_TIME_MS 100 // Maximum waiting time for slave test response and time between test packets
#define RFPACKET_RX_BUF_SIZE 256 // Size of the rx buffer

// --- Private Types ---

typedef struct _rfpacket_info {
    uint8_t rx_buf[RFPACKET_RX_BUF_SIZE];
    uint8_t gpio_data[4];
    uint32_t frequency;
    uint32_t size;
    dev_timer_delay_t msec;
    struct dev_rfpacket_rq_s rq_cont;
    struct dev_rfpacket_rq_s rq_struct;
    struct dev_rfpacket_rx_s rx_struct;
    struct dev_timer_rq_s trq_struct;
    struct dev_gpio_rq_s grq_struct;
    struct device_rfpacket_s rf_dev;
    struct device_timer_s timer_dev;
    struct device_gpio_s gpio_dev;
} rfpacket_info_t;

// --- Private Function Prototypes ---
static void rfpacket_wait(uint32_t wait_time);
static void rfpacket_button(void);
static void rfpacket_baserq(struct dev_rfpacket_rq_s *rq);
static struct dev_rfpacket_rx_s *rfpacket_rx_alloc(struct dev_rfpacket_rq_s *rq, size_t size);
static void rfpacket_rxc(void);
static bool rfpacket_send(const uint8_t *pBuf, uint16_t buf_size);
static void rfpacket_process(void);

// --- Private Variables ---

/* All the config (regular, static and extern) are the same */
/* Do not use regular config with CONFIG_DEVICE_RFPACKET_STATIC_RF_CONFIG
   and CONFIG_DEVICE_RFPACKET_STATIC_PKT_CONFIG as this deactivates the modem configuratoe
*/

// Regular config
static struct dev_rfpacket_rf_cfg_fsk_s reg_rfcfg = {
    .base = {
        .mod = DEV_RFPACKET_GFSK,
        .cache = {
            .id = 0,
            .dirty = 0
        },
    },
    .common = {
        .drate = 38400,
        .jam_rssi = (-90) << 3,
        .frequency = 865046875 - 3000,
        .chan_spacing = 93750,
        .rx_bw = 0,
        .freq_err = 868 * 20 /* ppm */,
    },
    .fairtx = {
        .mode = DEV_RFPACKET_LBT,
        .lbt.rssi = (-90) << 3,
        .lbt.duration = 5000, /** us */
    },
    .deviation = 19200,
    .symbols = 2,
};

static struct dev_rfpacket_pk_cfg_basic_s reg_pkcfg = {
    .base = {
        .format = DEV_RFPACKET_FMT_SLPC,
        .cache = {
            .id = 0,
            .dirty = 0
        },
    },
    .encoding = DEV_RFPACKET_CLEAR,
    .crc = 0x8005,
    .crc_seed = 0xffff,
    .sw_value = 0x1234,
    .sw_len = 15,
    .pb_pattern = 0x2,
    .pb_pattern_len = 1,
    .tx_pb_len = 64,
    .rx_pb_len = 64,
};

// Static rf config (see dev_nucleo_s2lp.c)
static struct dev_rfpacket_rf_cfg_static_s static_rfcfg = {
    .base = {
        .mod = DEV_RFPACKET_MOD_STATIC,
        .cache = {
            .id = 0,
            .dirty = 0
        },
    },
    .cfg_name = "static_rf",
};

// Extern rf config
static const uint8_t ext_rfcfg_array[] = {
  0x00, 0x96, 0x00, 0x00, 0x42, 0x42, 0x19, 0x03, 0x00, 0x18, 0x38, 0x0a,
  0x00, 0x0c, 0x3d, 0x00, 0x92, 0xa7, 0xa7, 0x03, 0x93, 0x33, 0x06, 0x00,
  0x62, 0x07, 0x01, 0x8a, 0xd0, 0x06, 0x00, 0x05, 0x02, 0x29, 0xa0, 0xca,
};

static struct dev_rfpacket_rf_cfg_extern_s extern_rfcfg = {
    .base = {
        .mod = DEV_RFPACKET_MOD_EXTERN,
        .cache = {
            .id = 0,
            .dirty = 0
        },
    },
    .p_cfg = (void *)ext_rfcfg_array,
};

// Static pk config (see dev_nucleo_s2lp.c)
static struct dev_rfpacket_pk_cfg_static_s static_pkcfg = {
    .base = {
        .format = DEV_RFPACKET_FMT_STATIC,
        .cache = {
            .id = 0,
            .dirty = 0
        },
    },
    .cfg_name = "static_pk",
};

// Extern pk config
static const uint8_t ext_pkcfg_array[] = {
  0x03, 0x40, 0x19, 0x08, 0x00, 0x2b, 0x40, 0x20, 0x00, 0x01, 0x01, 0x40,
  0x06, 0x00, 0x33, 0x00, 0x00, 0x34, 0x12, 0x05, 0x00, 0x39, 0x40, 0x03,
  0x08, 0x06, 0x00, 0x46, 0x01, 0x00, 0x01, 0x00,
};

static struct dev_rfpacket_pk_cfg_extern_s extern_pkcfg = {
    .base = {
        .format = DEV_RFPACKET_FMT_EXTERN,
        .cache = {
            .id = 0,
            .dirty = 0
        },
    },
    .p_cfg = (void *)ext_pkcfg_array,
};

static rfpacket_info_t pv;

// *** End Definitions ***

// *** Private Functions ***

static KROUTINE_EXEC(rfpacket_rx_pckt_cb) {
    struct dev_rfpacket_rx_s *rx = dev_rfpacket_rx_s_from_kr(kr);

    if (rx->error == 0) {
        pv.size = rx->size;
        pv.frequency = rx->frequency;
        uint8_t *pBuff = (uint8_t *)rx->buf;
        printk("Received on freq %d, chan %d - %P\n", rx->frequency,
              rx->channel, pBuff, rx->size);
    } else {
        printk("Rx packet error: %d\n", rx->error);
    }
}

static KROUTINE_EXEC(rfpacket_rx_cb) {
    struct dev_rfpacket_rq_s *rq = dev_rfpacket_rq_from_kr(kr);

    if (rq->error == -ENOTSUP) {
        printk("Bad RX configuration\n");
        abort();
    } else if (rq->error ==  -EBUSY) {
        printk("Jamming.\n");
    } else if (rq->error) {
        printk("Error during rxc: %d\n", rq->error);
    }
    rfpacket_rxc();
}

static KROUTINE_EXEC(rfpacket_tx_cb) {
    struct dev_rfpacket_rq_s *rq = dev_rfpacket_rq_from_kr(kr);

    if (rq->error == -ENOTSUP) {
        printk("Bad TX configuration\n");
    } else if (rq->error == -ETIMEDOUT) {
        printk("TX timeout\n");
    } else if (rq->error) {
        printk("Error during tx: %d\n", rq->error);
    } else {
        rfpacket_button();
    }
}

static KROUTINE_EXEC(rfpacket_button_cb) {
    printk(" *** Button pressed - Restarting test ***\n");
    rfpacket_wait(RFPACKET_WAIT_MS);
}

static KROUTINE_EXEC(rfpacket_wait_cb) {
    rfpacket_process();

}
static void rfpacket_wait(uint32_t wait_time) {
    struct dev_timer_rq_s *trq = &pv.trq_struct;
    trq->delay = wait_time * pv.msec;
    trq->rev = 0;
    dev_timer_rq_init(trq, rfpacket_wait_cb);
    error_t err = DEVICE_OP(&pv.timer_dev, request, trq);

    if (err == -ETIMEDOUT) {
        printk("Warning: Timer timeout\n");
        kroutine_exec(&trq->base.kr);
    } else if (err) {
        printk("Error: Timer failed: %d\n", err);
    }
}

static void rfpacket_button(void) {
    struct dev_gpio_rq_s *grq = &pv.grq_struct;
    grq->io_first = EFM32_PB10;
    grq->io_last = EFM32_PB10;
    grq->type = DEV_GPIO_UNTIL;
    grq->until.mask = dev_gpio_mask1;
    pv.gpio_data[0] = 0x1;
    grq->until.data = pv.gpio_data;
    dev_gpio_rq_init(grq, rfpacket_button_cb);
    DEVICE_OP(&pv.gpio_dev, request, grq);
}

static void rfpacket_baserq(struct dev_rfpacket_rq_s *rq) {
    rq->err_group = 0;
    rq->anchor = DEV_RFPACKET_TIMESTAMP_END;

    // Set the config you want to use here
    //rq->rf_cfg = &reg_rfcfg.base;
    //rq->pk_cfg = &reg_pkcfg.base;

    //rq->rf_cfg = &static_rfcfg.base;
    //rq->pk_cfg = &static_pkcfg.base;

    rq->rf_cfg = &extern_rfcfg.base;
    rq->pk_cfg = &extern_pkcfg.base;

    rq->channel = 0;
    rq->deadline = 0;
    rq->lifetime = 0;
}

static struct dev_rfpacket_rx_s *rfpacket_rx_alloc(struct dev_rfpacket_rq_s *rq,
                                                   size_t size) {
    if (size > RFPACKET_RX_BUF_SIZE) {
        return NULL;
    }
    struct dev_rfpacket_rx_s *rx = &pv.rx_struct;
    rx->buf = pv.rx_buf;
    kroutine_init_deferred(&rx->kr, &rfpacket_rx_pckt_cb);
    rx->size = size;
    return rx;
}

static void rfpacket_rxc(void) {
    struct dev_rfpacket_rq_s *rq = &pv.rq_cont;
    rq->rx_alloc = &rfpacket_rx_alloc;
    rq->type = DEV_RFPACKET_RQ_RX_CONT;
    dev_rfpacket_rq_init(rq, &rfpacket_rx_cb);
    DEVICE_OP(&pv.rf_dev, request, rq, NULL);
}

static bool rfpacket_send(const uint8_t *pBuf, uint16_t buf_size) {
    // Send packet
    struct dev_rfpacket_rq_s *rq = &pv.rq_struct;
    rq->type = DEV_RFPACKET_RQ_TX;
    rq->tx_pwr = 64; // in 0.125dbm
    rq->deadline = 0;
    rq->lifetime = 0;
    // Set data and size
    rq->tx_size = buf_size;
    rq->tx_buf = pBuf;
    dev_rfpacket_rq_init(rq, &rfpacket_tx_cb);
    DEVICE_OP(&pv.rf_dev, request, rq, NULL);
    return true;
}

static void rfpacket_process(void) {
    static char cmd[] ="abcd";
    rfpacket_send((uint8_t *)cmd, sizeof(cmd));
}

// *** Public Functions ***

void app_start(void) {
    printk("*** START OF TEST ***\n");
    // Retrieve devices
    ensure(!device_get_accessor_by_path(&pv.rf_dev.base, NULL, "rfpacket*", DRIVER_CLASS_RFPACKET));
    ensure(!device_get_accessor_by_path(&pv.timer_dev.base,  NULL, "rfpacket*", DRIVER_CLASS_TIMER));
    ensure(!device_get_accessor_by_path(&pv.gpio_dev.base,  NULL, "gpio", DRIVER_CLASS_GPIO));

    // Set button gpio
    DEVICE_OP(&pv.gpio_dev, set_mode, EFM32_PB10, EFM32_PB10, dev_gpio_mask1, DEV_PIN_INPUT_PULLUP);

    // Set timer reference
    dev_timer_init_sec(&pv.timer_dev, &pv.msec, 0, 1, 1000);
    // Init module
    rfpacket_baserq(&pv.rq_struct);
    rfpacket_baserq(&pv.rq_cont);
    // Start test
    rfpacket_rxc();
    rfpacket_wait(RFPACKET_WAIT_MS);
}
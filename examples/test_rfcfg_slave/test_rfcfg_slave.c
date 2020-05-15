/*
  This test application will send commands to a slave application to see if the slave rfpacket driver can change its config
  Things that are tested:
    -modulation (ASK/FSK)
    -datarate
    -frequency
    -chan spacing
    -chan hoping
*/

// *** Libraries include **
// Standard lib
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
// Mutekh lib
#include <mutek/printk.h>
#include <mutek/startup.h>
#include <mutek/thread.h>
#include <mutek/kroutine.h>
// Devices lib
#include <device/class/timer.h>
#include <device/class/rfpacket.h>

// *** Definitions ***
// --- Private Constants ---
#define TEST_RFCFG_WAIT_MS 250 // General purpose waiting time
#define TEST_RFCFG_RX_BUF_SIZE 40 // Size of the rx buffer

#define TEST_RFCFG_BASIC_PCK_STRUCT_SIZE 28
#define TEST_RFCFG_FSK_STRUCT_SIZE 36
#define TEST_RFCFG_ASK_STRUCT_SIZE 32
#define TEST_RFCFG_LORA_STRUCT_SIZE 28

// Commands
#define TEST_RFCFG_ACK_CMD 'o'
#define TEST_RFCFG_CHAN_CMD 'c'
#define TEST_RFCFG_ACK_RSP 'k'
#define TEST_RFCFG_RF_PAYLOAD 'r'
#define TEST_RFCFG_PK_PAYLOAD 'p'


// --- Private Types ---
typedef struct _test_rfcfg_info {
    uint8_t channel;
    bool send_ack;
    uint8_t tx_buf[4];
    uint8_t rx_buf[TEST_RFCFG_RX_BUF_SIZE];
    uint8_t curr_pkcfg[28];
    uint8_t curr_rfcfg[36];
    dev_timer_delay_t msec;
    struct dev_rfpacket_rq_s rq_cont;
    struct dev_rfpacket_rq_s rq_struct;
    struct dev_rfpacket_rx_s rx_struct;
    struct dev_timer_rq_s trq_struct;
    struct device_rfpacket_s rf_dev;
    struct device_timer_s timer_dev;
} test_rfcfg_info_t;

// --- Private Function Prototypes ---
static void test_rfcfg_rxc(void);
static void test_rfcfg_cancel_rxc(void);
static struct dev_rfpacket_rx_s *test_rfcfg_rx_alloc(struct dev_rfpacket_rq_s *rq, size_t size);
static void test_rfcfg_send(uint8_t *pBuf, uint32_t buf_size);
static void test_rfcfg_send_ack(void);
static void test_rfcfg_process_rx(uint8_t *pBuff, uint16_t buff_size);

// --- Private Variables ---
static test_rfcfg_info_t pv = {
    .curr_pkcfg = {0x00, 0x00, 0x00, 0x00, 0x05, 0x80, 0x00, 0x00, 0xff, 0xff, 0x00, 0x00, 0xba, 0xab, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x2f, 0x00, 0x40, 0x00, 0x10, 0x00, 0x00, 0x00},
    .curr_rfcfg = {0x01, 0x00, 0x30, 0xfd, 0x00, 0x96, 0x00, 0x00, 0xcb, 0xa8, 0x8f, 0x33, 0x36, 0x6e, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0xd0, 0x43, 0x00, 0x00, 0x88, 0x13, 0x00, 0x00, 0x30, 0xfd, 0x01, 0x00, 0x00, 0x4b, 0x00, 0x02},
};

// *** End Definitions ***

// *** Private Functions ***

static KROUTINE_EXEC(test_rfcfg_rx_pckt_cb) {
    struct dev_rfpacket_rx_s *rx = dev_rfpacket_rx_s_from_kr(kr);

    if (rx->error == 0) {
        uint8_t *pBuff = (uint8_t *)rx->buf;
        //printk("Received on chan %d - %P\n", rx->channel, pBuff, rx->size);
        test_rfcfg_process_rx(pBuff, rx->size);
    } else {
        printk("Rx packet error: %d\n", rx->error);
    }
}

static KROUTINE_EXEC(test_rfcfg_rx_cb) {
    struct dev_rfpacket_rq_s *rq = dev_rfpacket_rq_from_kr(kr);

    if (rq->error == -ENOTSUP) {
        printk("Bad RX configuration\n");
    } else if (rq->error ==  -EBUSY) {
        printk("Jamming.\n");
    } else if (rq->error) {
        printk("Error during rxc: %d\n", rq->error);
    }
    test_rfcfg_rxc();
}

static KROUTINE_EXEC(test_rfcfg_tx_cb) {
    struct dev_rfpacket_rq_s *rq = dev_rfpacket_rq_from_kr(kr);

    if (rq->error == -ENOTSUP) {
        printk("Bad TX configuration\n");
    } else if (rq->error == -ETIMEDOUT) {
        printk("TX timeout\n");
    } else if (rq->error) {
        printk("Error during tx: %d\n", rq->error);
    }
}

static void test_rfcfg_rxc(void) {
    struct dev_rfpacket_rq_s *rq = &pv.rq_cont;
    rq->anchor = DEV_RFPACKET_TIMESTAMP_END;
    rq->pk_cfg = (struct dev_rfpacket_pk_cfg_s *)pv.curr_pkcfg;
    rq->rf_cfg = (struct dev_rfpacket_rf_cfg_s *)pv.curr_rfcfg;
    rq->channel = pv.channel;
    rq->deadline = 0;
    rq->lifetime = 0;
    rq->err_group = 0;
    rq->rx_alloc = &test_rfcfg_rx_alloc;
    rq->type = DEV_RFPACKET_RQ_RX_CONT;
    dev_rfpacket_rq_init(rq, &test_rfcfg_rx_cb);
    DEVICE_OP(&pv.rf_dev, request, rq, NULL);
}

static void test_rfcfg_cancel_rxc(void) {
    struct dev_rfpacket_rq_s *rq = &pv.rq_cont;
    //printk("Canceling rxc\n");
    DEVICE_OP(&pv.rf_dev, cancel, rq);
}

static struct dev_rfpacket_rx_s *test_rfcfg_rx_alloc(struct dev_rfpacket_rq_s *rq, size_t size) {
    if (size > TEST_RFCFG_RX_BUF_SIZE) {
        return NULL;
    }
    struct dev_rfpacket_rx_s *rx = &pv.rx_struct;
    rx->buf = pv.rx_buf;
    kroutine_init_deferred(&rx->kr, &test_rfcfg_rx_pckt_cb);
    rx->size = size;
    return rx;
}

static void test_rfcfg_send(uint8_t *pBuf, uint32_t buf_size) {
    struct dev_rfpacket_rq_s *rq = &pv.rq_struct;
    rq->anchor = DEV_RFPACKET_TIMESTAMP_END;
    rq->pk_cfg = (struct dev_rfpacket_pk_cfg_s *)pv.curr_pkcfg;
    rq->rf_cfg = (struct dev_rfpacket_rf_cfg_s *)pv.curr_rfcfg;
    rq->channel = pv.channel;
    rq->deadline = 0;
    rq->lifetime = 0;
    rq->err_group = 0;
    rq->tx_pwr = 64;
    rq->type = DEV_RFPACKET_RQ_TX;
    // Set data and size
    rq->tx_size = buf_size;
    rq->tx_buf = pBuf;
    dev_rfpacket_rq_init(rq, &test_rfcfg_tx_cb);
    DEVICE_OP(&pv.rf_dev, request, rq, NULL);
}

static void test_rfcfg_send_ack(void) {
    pv.tx_buf[0] = TEST_RFCFG_ACK_RSP;
    test_rfcfg_send((uint8_t *)&pv.tx_buf, sizeof(pv.tx_buf));
}

static void test_rfcfg_process_rx(uint8_t *pBuff, uint16_t buff_size) {

    if (((char)pBuff[0] == TEST_RFCFG_ACK_CMD) && (buff_size == 4)) {
        //printk("Received ack command\n");
        test_rfcfg_send_ack();
    } else if (((char)pBuff[0] == TEST_RFCFG_CHAN_CMD)  && (buff_size == 4)) {
        pv.channel = pBuff[1];
        // Update rxc with new channel
        test_rfcfg_cancel_rxc();
        //printk("Received channel command %d\n", pv.channel);
    } else if ((char)pBuff[0] == TEST_RFCFG_RF_PAYLOAD) {
        buff_size--;
        // Get config
        switch (pBuff[1]) {
            case DEV_RFPACKET_FSK:
            case DEV_RFPACKET_GFSK:
                if (buff_size != TEST_RFCFG_FSK_STRUCT_SIZE) {
                    printk("Received bad FSK config\n");
                    return;
                }
                //printk("Received FSK config\n");
            break;

            case DEV_RFPACKET_ASK:
                if (buff_size != TEST_RFCFG_ASK_STRUCT_SIZE) {
                    printk("Received bad ASK config\n");
                    return;
                }
                //printk("Received ASK config\n");
            break;

            case DEV_RFPACKET_LORA:
                if (buff_size != TEST_RFCFG_LORA_STRUCT_SIZE) {
                    printk("Received bad LORA config\n");
                    return;
                }
                //printk("Received LORA config\n");
            break;
        }
        memcpy(pv.curr_rfcfg, &pBuff[1], buff_size);
        struct dev_rfpacket_rf_cfg_s *rfcfg = (struct dev_rfpacket_rf_cfg_s *)pv.curr_rfcfg;
        rfcfg->cache.dirty = 1;
        //printk("Received drate: %d\n", rfcfg->drate);
        test_rfcfg_send_ack();
    } else if ((char)pBuff[0] == TEST_RFCFG_PK_PAYLOAD) {
        buff_size--;
        //printk("Received packet configuration\n");
        // Get config 
    } else {
        printk("Unknown message received: %P\n", pBuff, buff_size);
    }
}

// *** Public Functions ***

void app_start(void) {
    printk("Init started.\n");
    // Retrieve devices
    ensure(!device_get_accessor_by_path(&pv.rf_dev.base, NULL, "rfpacket0", DRIVER_CLASS_RFPACKET));
    ensure(!device_get_accessor_by_path(&pv.timer_dev.base,  NULL, "rfpacket0", DRIVER_CLASS_TIMER));
    // Set timer reference
    dev_timer_init_sec(&pv.timer_dev, &pv.msec, 0, 1, 1000);
    // Start test
    memset(&pv.tx_buf, 0xFF, sizeof(pv.tx_buf));
    test_rfcfg_rxc();
}
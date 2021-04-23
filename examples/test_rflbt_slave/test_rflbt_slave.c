/*
  This test application will respond to a master application to see if it respects the ENT300220 Clause 9 LBT specifications
  Things that are tested:
    - Ack time
    - Lbt on clear channel
    - Lbt on busy channel with fixed and random disturb packet lengths
    - Time dead (time between end of listen and start of transmission - slave reported)
  Things that are not tested:
    - Tx off (Minimum off time after a transmission, which can be a full sequence, must be >100 ms)
    - Tx on (Maximum on time - <1s for a single packet, <4s for a sequence, <100s per 200kHz per hour)
  How is it tested:
    - Measure the time between the end of the request and the start of the response
    - Compute the mean value of this response time for TEST_LOOP_NB iterations on each test
    - Eventually correct the value to take into account latencies
    - Compute the mean difference of this response time to check for randomness/variability
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
#define TX_CHANNEL 0
#define RX_CHANNEL 0
#define MAX_PACKET_SIZE 255
#define WAIT_PERIOD_MS 250
#define RX_TIME_MS 100
#define MASTER_MS_TICK_NB 32

// Commands
#define NEXT_CMD 'n'
#define LBT_CMD 'l'
#define LBT_BSY_CMD 'b'
#define GIVE_TD_CMD 'd'
#define ACK_RSP 'k'

#define SYNC_TIME (((pkcfg.tx_pb_len + pkcfg.sw_len + 1) * pv.msec * 1000) / rfcfg.common.drate)

// --- Private Types ---
enum _test_rflbt_state
{
    TEST_IDLE,
    TEST_ACK,
};

typedef struct _test_rflbt_info
{
    uint8_t state;
    bool calc_td;
    uint8_t tx_buf[4];
    uint8_t rx_buf[MAX_PACKET_SIZE];
    uint32_t mean_count;
    struct dev_timer_rq_s trq_struct;
    struct dev_rfpacket_rq_s rq_cont;
    struct dev_rfpacket_rq_s rq_struct;
    struct dev_rfpacket_rx_s rx_struct;
    struct device_rfpacket_s rf_dev;
    struct device_timer_s timer_dev;
    dev_timer_delay_t msec;
    dev_timer_value_t td_val;
    dev_timer_value_t mean_td;
} test_rflbt_info_t;

// --- Private Function Prototypes ---
static void test_rflbt_process_rx(uint8_t *pBuf);
//static void test_rflbt_wait(void);
static void test_rflbt_baserq(struct dev_rfpacket_rq_s *rq);
static void test_rflbt_rx_cont(void);
// static void test_rflbt_listen(uint32_t dl, uint32_t ttl);
static void test_rflbt_send_ack(bool islbt);
static void test_rflbt_send_td(void);

// --- Private Variables ---
static struct dev_rfpacket_pk_cfg_basic_s pkcfg =
{
    .base =
    {
        .format = DEV_RFPACKET_FMT_SLPC,
        .cache =
        {
            .id = 0,
            .dirty = 0
        },
    },
    .encoding = DEV_RFPACKET_CLEAR,
    .crc = 0x8005,
    .crc_seed = 0xffff,
    .sw_value = 0xabba,
    .sw_len = 15,
    .pb_pattern = 0x2,
    .pb_pattern_len = 1,
    .tx_pb_len = 64,
    .rx_pb_len = 16,
};

static struct dev_rfpacket_rf_cfg_fsk_s rfcfg =
{
    .base =
    {
        .mod = DEV_RFPACKET_GFSK,
        .cache =
        {
            .id = 0,
            .dirty = 0
        },
    },
    .common =
    {
        .drate = 38400,
        .jam_rssi = (-90) << 3,
        .frequency = 865056875,
        .chan_spacing = 93750,
        .rx_bw = 0,
        .freq_err = 868 * 20 /* ppm */,
    },
    .fairtx =
    {
        .mode = DEV_RFPACKET_LBT,
        .lbt.rssi = (-95) << 3,
        .lbt.duration = 5000, /** us */
    },
    .deviation = 19200,
    .symbols = 2,
};

static test_rflbt_info_t pv;

// *** End Definitions ***

// *** Private Functions ***

static KROUTINE_EXEC(test_rflbt_rx_cont_pckt_cb)
{
    struct dev_rfpacket_rx_s *rx = dev_rfpacket_rx_s_from_kr(kr);

    if (!rx->error)
    {
        uint8_t *pBuff = (uint8_t *)rx->buf;
        //printk("Received on chan %d - %P\n", rx->channel, pBuff, rx->size);
        test_rflbt_process_rx(pBuff);
    }
    else
        printk("Rx packet failed with error %d\n", rx->error);

}

static struct dev_rfpacket_rx_s *test_rflbt_rx_cont_alloc(struct dev_rfpacket_rq_s *rq, size_t size)
{
    if (size > MAX_PACKET_SIZE)
        return NULL;

    struct dev_rfpacket_rx_s *rx = &pv.rx_struct;
    rx->buf = pv.rx_buf;
    kroutine_init_deferred(&rx->kr, &test_rflbt_rx_cont_pckt_cb);
    rx->size = size;
    return rx;
}

static KROUTINE_EXEC(test_rflbt_rx_cont_cb)
{
    struct dev_rfpacket_rq_s *rq = dev_rfpacket_rq_from_kr(kr);

    if (rq->error == -ENOTSUP)
        printk("Bad RX configuration\n");

    else if (rq->error ==  -EBUSY)
        printk("Jamming.\n");

    test_rflbt_rx_cont();
}

static KROUTINE_EXEC(test_rflbt_tx_cb)
{
    struct dev_rfpacket_rq_s *rq = dev_rfpacket_rq_from_kr(kr);

    if (rq->error == -ENOTSUP)
        printk("Bad TX configuration\n");

    else if ((rq->error == -ETIMEDOUT) || (rq->error == -EAGAIN) || (rq->error == -EBUSY))
    {
        printk("Channel was busy (err %d). Retrying\n", rq->error);
        test_rflbt_send_ack(true);
    }
    else if (!rq->error)
    {
        if (pv.calc_td)
        {
            // Time dead calculation
            pv.td_val = rq->tx_lbt_td;
            if (pv.td_val > SYNC_TIME)
            {
                // Correction for preamble + sync word
                pv.td_val -= SYNC_TIME;
            }
            pv.mean_td += pv.td_val;
            pv.mean_count++;
            pv.calc_td = false;
        }
    }
    else
        printk("Error during tx: %d\n", rq->error);

}

static void test_rflbt_baserq(struct dev_rfpacket_rq_s *rq)
{
    rq->err_group = 0;
    rq->anchor = DEV_RFPACKET_TIMESTAMP_START;
    rq->pk_cfg = &pkcfg.base;
    rq->rf_cfg = &rfcfg.base;
}

static void test_rflbt_rx_cont(void)
{
    struct dev_rfpacket_rq_s *rq = &pv.rq_cont;
    rq->type = DEV_RFPACKET_RQ_RX_CONT;
    rq->deadline = 0;
    rq->lifetime = 0;
    rq->rx_alloc = &test_rflbt_rx_cont_alloc;
    dev_rfpacket_rq_init(rq, &test_rflbt_rx_cont_cb);
    DEVICE_OP(&pv.rf_dev, request, rq, NULL);
}

static void test_rflbt_cancel_rx_cont(void)
{
    struct dev_rfpacket_rq_s *rq = &pv.rq_cont;
    DEVICE_OP(&pv.rf_dev, cancel, rq);
}

static void test_rflbt_process_rx(uint8_t *pBuff)
{
    char command = (char)pBuff[0];

    // dev_timer_value_t timeval;
    // DEVICE_OP(&pv.timer_dev, get_value, &timeval, 0);
    // printk("[_app] [%d] Received cmd: %c\n", (uint32_t)timeval, command);

    switch (command)
    {
        case NEXT_CMD:
            //printk("Received next command.\n");
            test_rflbt_send_ack(false);
        break;

        case LBT_CMD:
            //printk("Received lbt command.\n");
            test_rflbt_send_ack(true);
        break;

        case LBT_BSY_CMD:
            //printk("Received lbt busy command.\n");
            pv.calc_td = true;
            //test_rflbt_cancel_rx_cont(); // debug to test no_rx tx_lbt case
            test_rflbt_send_ack(true);
        break;

        case GIVE_TD_CMD:
            test_rflbt_send_td();
            pv.mean_td = 0;
            pv.mean_count = 0;
        break;

        default:
        break;
    }
}

static void test_rflbt_send_ack(bool islbt)
{
    struct dev_rfpacket_rq_s *rq = &pv.rq_struct;
    rq->channel = TX_CHANNEL;
    rq->type = (islbt) ? DEV_RFPACKET_RQ_TX_FAIR : DEV_RFPACKET_RQ_TX;
    rq->deadline = 0;
    rq->lifetime = (islbt) ? RX_TIME_MS * pv.msec : 0;
    // Set data and size
    memset(&pv.tx_buf, 0xFF, sizeof(pv.tx_buf));
    pv.tx_buf[0] = ACK_RSP;
    rq->tx_size = sizeof(pv.tx_buf);
    rq->tx_buf = pv.tx_buf;
    //printk("Sending ack\n");
    rq->tx_pwr = 64;
    dev_rfpacket_rq_init(rq, &test_rflbt_tx_cb);
    DEVICE_OP(&pv.rf_dev, request, rq, NULL);
}

static void test_rflbt_send_td(void)
{
    struct dev_rfpacket_rq_s *rq = &pv.rq_struct;
    rq->channel = TX_CHANNEL;
    rq->type = DEV_RFPACKET_RQ_TX;
    rq->deadline = 0;
    rq->lifetime = 0;
    // Set data and size
    uint32_t mean_val = (pv.mean_td/pv.mean_count) * MASTER_MS_TICK_NB / pv.msec;
    //printk("TD: %d ms", mean_val);
    for (uint8_t idx = 0; idx < sizeof(mean_val); idx++)
        pv.tx_buf[idx] = (uint8_t)(mean_val >> (8*idx));

    rq->tx_size = sizeof(pv.tx_buf);
    rq->tx_buf = pv.tx_buf;
    rq->tx_pwr = 64;
    dev_rfpacket_rq_init(rq, &test_rflbt_tx_cb);
    DEVICE_OP(&pv.rf_dev, request, rq, NULL);
}

// *** Public Functions ***

void app_start(void)
{
    printk("Init started.\n");
    // Retrieve devices
    ensure(!device_get_accessor_by_path(&pv.rf_dev.base, NULL, "rfpacket*", DRIVER_CLASS_RFPACKET));
    ensure(!device_get_accessor_by_path(&pv.timer_dev.base,  NULL, "rfpacket*", DRIVER_CLASS_TIMER));
    // Set timer reference
    dev_timer_init_sec(&pv.timer_dev, &pv.msec, 0, 1, 1000);
    // Init module
    test_rflbt_baserq(&pv.rq_cont);
    test_rflbt_baserq(&pv.rq_struct);
    pv.state = TEST_IDLE;
    memset(&pv.tx_buf, 0xFF, sizeof(pv.tx_buf));
    test_rflbt_rx_cont();
}
/*
  This test application will send commands to a slave application to see if the slave rfpacket driver can change its config
  Things that are tested:
    -datarate
    -frequency
    -chan spacing
    -chan hoping
    -modulation (ASK/FSK)
*/

// *** Libraries include **
#include <arch/efm32/pin.h>
#include "test_rfcfg_config.h"

// *** Definitions ***
// --- Private Constants ---
#define TEST_RFCFG_SLAVE_EXCHANGE // This token activate the test of a slave board
#define TEST_RFCFG_WAIT_MS 250 // General purpose waiting time
#define TEST_RFCFG_TEST_STEP_TIME_MS 25 // Waiting time between test steps
#define TEST_RFCFG_RX_TIME_MS 100 // Maximum waiting time for slave test response and time between test packets
#define TEST_RFCFG_RX_BUF_SIZE 4 // Size of the rx buffer
#define TEST_RFCFG_MIN_CHAN 0
#define TEST_RFCFG_MAX_CHAN 10
// Commands
#define TEST_RFCFG_ACK_CMD 'o'
#define TEST_RFCFG_CHAN_CMD 'c'
#define TEST_RFCFG_ACK_RSP 'k'

// --- Private Types ---
enum _test_rfcfg_state
{
    TEST_RFCFG_STATE_START,
    TEST_RFCFG_WAIT_TX_CFG,
    TEST_RFCFG_TX,
    TEST_RFCFG_WAIT_ACK,
    TEST_RFCFG_CHAN_HOP_SEND_CHAN,
    TEST_RFCFG_CHAN_HOP_ASK_ACK,
    TEST_RFCFG_CHAN_HOP_WAIT_ACK,
    TEST_RFCFG_STATE_END,
    TEST_RFCFG_WAIT_BUTTON,
};

typedef struct _test_rfcfg_info
{
    uint8_t state;
    uint8_t channel;
    bool packet_ok;
    uint8_t rx_buf[TEST_RFCFG_RX_BUF_SIZE];
    uint8_t cmd_buf[4];
    uint8_t gpio_data[4];
    uint32_t test_idx;
    uint32_t test_counter;
    uint32_t chan_hop_counter;
    dev_timer_delay_t msec;
    struct dev_rfpacket_rq_s rq_struct;
    struct dev_rfpacket_rx_s rx_struct;
    struct dev_timer_rq_s trq_struct;
    struct dev_gpio_rq_s grq_struct;
    struct device_rfpacket_s rf_dev;
    struct device_timer_s timer_dev;
    struct device_gpio_s gpio_dev;
} test_rfcfg_info_t;

// --- Private Function Prototypes ---
static void test_rfcfg_wait(uint32_t wait_time);
static void test_rfcfg_baserq(struct dev_rfpacket_rq_s *rq);
static void test_rfcfg_button(void);
static struct dev_rfpacket_rx_s *test_rfcfg_rx_alloc(struct dev_rfpacket_rq_s *rq, size_t size);
static void test_rfcfg_send(uint8_t *pBuf, uint32_t buf_size);
static void test_rfcfg_send_ack_cmd(void);
static void test_rfcfg_init_process(void);
static void test_rfcfg_process(void);

#ifdef TEST_RFCFG_SLAVE_EXCHANGE
static void test_rfcfg_receive(uint32_t ttl);
static void test_rfcfg_send_chan(void);
static uint8_t test_rfcfg_new_chan(struct device_timer_s *timer, uint8_t old_chan);
#endif
// --- Private Variables ---
static test_rfcfg_info_t pv;

// *** End Definitions ***

// *** Private Functions ***

static KROUTINE_EXEC(test_rfcfg_rx_pckt_cb)
{
    struct dev_rfpacket_rx_s *rx = dev_rfpacket_rx_s_from_kr(kr);

    if (rx->error == 0)
    {
        uint8_t *pBuff = (uint8_t *)rx->buf;
        if (*(char *)pBuff == TEST_RFCFG_ACK_RSP)
        {
            pv.packet_ok = true;
            //printk("Received slave ack\n");
        }
        else
            printk("Received bad packet\n");

    }
    else
        printk("Rx packet error: %d\n", rx->error);

}

static KROUTINE_EXEC(test_rfcfg_tx_cb)
{
    struct dev_rfpacket_rq_s *rq = dev_rfpacket_rq_from_kr(kr);

    if (rq->error == -ENOTSUP)
        printk("Bad TX configuration\n");

    if (rq->error == -ETIMEDOUT)
        printk("TX timeout\n");

    test_rfcfg_process();
}

static KROUTINE_EXEC(test_rfcfg_wait_cb)
{
    test_rfcfg_process();
}

static KROUTINE_EXEC(test_rfcfg_button_cb)
{
    printk(" *** Button pressed - Restarting test ***\n");
    test_rfcfg_init_process();
    test_rfcfg_wait(TEST_RFCFG_WAIT_MS);
}

static void test_rfcfg_wait(uint32_t wait_time)
{
    struct dev_timer_rq_s *trq = &pv.trq_struct;
    trq->delay = wait_time * pv.msec;
    trq->rev = 0;
    dev_timer_rq_init(trq, test_rfcfg_wait_cb);
    error_t err = DEVICE_OP(&pv.timer_dev, request, trq);

    if (err == -ETIMEDOUT)
    {
        printk("Warning: Timer timeout\n");
        kroutine_exec(&trq->base.kr);
    }
    else if (err)
        printk("Error: Timer failed: %d\n", err);

}

static void test_rfcfg_button(void)
{
    struct dev_gpio_rq_s *grq = &pv.grq_struct;
    //grq->io_first = EFM32_PF6;
    //grq->io_last = EFM32_PF6;
    grq->io_first = EFM32_PB10;
    grq->io_last = EFM32_PB10;
    grq->type = DEV_GPIO_UNTIL;
    grq->until.mask = dev_gpio_mask1;
    pv.gpio_data[0] = 0x1;
    grq->until.data = pv.gpio_data;
    dev_gpio_rq_init(grq, test_rfcfg_button_cb);
    DEVICE_OP(&pv.gpio_dev, request, grq);
}

static void test_rfcfg_baserq(struct dev_rfpacket_rq_s *rq)
{
    rq->err_group = 0;
    rq->anchor = DEV_RFPACKET_TIMESTAMP_END;
    rq->pk_cfg = def_pkcfg;
    rq->rf_cfg = def_rfcfg;
    rq->channel = 0;
    rq->deadline = 0;
    rq->lifetime = 0;
}

static struct dev_rfpacket_rx_s *test_rfcfg_rx_alloc(struct dev_rfpacket_rq_s *rq, size_t size)
{
    if (size > TEST_RFCFG_RX_BUF_SIZE)
        return NULL;

    struct dev_rfpacket_rx_s *rx = &pv.rx_struct;
    rx->buf = pv.rx_buf;
    kroutine_init_deferred(&rx->kr, &test_rfcfg_rx_pckt_cb);
    rx->size = size;
    return rx;
}

static void test_rfcfg_send(uint8_t *pBuf, uint32_t buf_size)
{
    struct dev_rfpacket_rq_s *rq = &pv.rq_struct;
    rq->type = DEV_RFPACKET_RQ_TX;
    rq->tx_pwr = 64; // in 0.125dbm
    // Set data and size
    rq->tx_size = buf_size;
    rq->tx_buf = pBuf;
    dev_rfpacket_rq_init(rq, &test_rfcfg_tx_cb);
    DEVICE_OP(&pv.rf_dev, request, rq, NULL);
}

static void test_rfcfg_send_ack_cmd(void)
{
    pv.cmd_buf[0] = TEST_RFCFG_ACK_CMD;
    test_rfcfg_send((uint8_t *)&pv.cmd_buf, sizeof(pv.cmd_buf));
}

static void test_rfcfg_init_process(void)
{
    pv.state = TEST_RFCFG_STATE_START;
    pv.test_idx = 0;
    pv.test_counter = p_test_rfcfg_array[pv.test_idx].count - 1;
}

#ifdef TEST_RFCFG_SLAVE_EXCHANGE
static KROUTINE_EXEC(test_rfcfg_rx_cb)
{
    struct dev_rfpacket_rq_s *rq = dev_rfpacket_rq_from_kr(kr);

    if (rq->error == -ENOTSUP)
        printk("Bad RX configuration\n");

    else if (rq->error)
        printk("RX error: %d\n", rq->error);

    if(!pv.packet_ok)
    {
        printk("Error: slave didn't respond !\n");
        //pv.state = TEST_RFCFG_WAIT_BUTTON;
    }
    pv.packet_ok = false;
    test_rfcfg_process();
}

static void test_rfcfg_receive(uint32_t ttl)
{
    struct dev_rfpacket_rq_s *rq = &pv.rq_struct;
    rq->type = DEV_RFPACKET_RQ_RX;
    rq->lifetime = ttl * pv.msec;
    rq->rx_alloc = &test_rfcfg_rx_alloc;
    dev_rfpacket_rq_init(rq, &test_rfcfg_rx_cb);
    DEVICE_OP(&pv.rf_dev, request, rq, NULL);
}

static void test_rfcfg_send_chan(void)
{
    pv.cmd_buf[0] = TEST_RFCFG_CHAN_CMD;
    pv.cmd_buf[1] = pv.channel;
    test_rfcfg_send((uint8_t *)&pv.cmd_buf, sizeof(pv.cmd_buf));
}

static uint8_t test_rfcfg_new_chan(struct device_timer_s *timer, uint8_t old_chan)
{
    dev_timer_value_t t;
    uint8_t new_chan;
    DEVICE_OP(timer, get_value, &t, 0);
    do
    {
        new_chan = (uint8_t)rand_64_range_r(&t, TEST_RFCFG_MIN_CHAN, TEST_RFCFG_MAX_CHAN);
    } while (old_chan == new_chan);

    return new_chan;
}
#endif

static void test_rfcfg_process(void)
{
    switch (pv.state)
    {
        default:
        case TEST_RFCFG_STATE_START:
            // Start current test
            p_test_rfcfg_array[pv.test_idx].chg_cfg(&pv.rq_struct);
        #ifdef TEST_RFCFG_SLAVE_EXCHANGE

       {   // Send config to slave
            //printk("Sending configuration to slave\n");
            uint8_t *slave_cfg;
            uint8_t slave_cfg_size;
            test_rfcfg_get_slave_config(&slave_cfg, &slave_cfg_size);
            test_rfcfg_send(slave_cfg, slave_cfg_size);
            pv.state = TEST_RFCFG_WAIT_TX_CFG;
        }
        #else
            pv.state = TEST_RFCFG_TX;
            test_rfcfg_wait(TEST_RFCFG_TEST_STEP_TIME_MS);
        #endif
        break;

        #ifdef TEST_RFCFG_SLAVE_EXCHANGE
        case TEST_RFCFG_WAIT_TX_CFG:
            // Wait in rx for ack
            //printk("Waiting slave configuration ack\n");
            pv.state = TEST_RFCFG_TX;
            test_rfcfg_wait(TEST_RFCFG_TEST_STEP_TIME_MS);
        break;
        #endif

        case TEST_RFCFG_TX:
            // Send packet with new config
            //printk("Sending ack command to slave\n");
            test_rfcfg_update_rq_config(&pv.rq_struct);
            test_rfcfg_send_ack_cmd();
        #ifdef TEST_RFCFG_SLAVE_EXCHANGE
            pv.state = TEST_RFCFG_WAIT_ACK;
        #else
            pv.state = TEST_RFCFG_STATE_END;
        #endif
        break;

        #ifdef TEST_RFCFG_SLAVE_EXCHANGE
        case TEST_RFCFG_WAIT_ACK:
            // Wait in rx for ack
            //printk("Waiting slave ack\n");
            test_rfcfg_tx_end();
            test_rfcfg_receive(TEST_RFCFG_RX_TIME_MS);
            pv.chan_hop_counter = p_test_rfcfg_array[pv.test_idx].chan_hop_count;
            if (pv.chan_hop_counter == 0)
                pv.state = TEST_RFCFG_STATE_END;

            else
            {
                pv.chan_hop_counter--;
                pv.state = TEST_RFCFG_CHAN_HOP_SEND_CHAN;
            }
        break;

        case TEST_RFCFG_CHAN_HOP_SEND_CHAN:
            // Get new channel
            pv.channel = test_rfcfg_new_chan(&pv.timer_dev, pv.rq_struct.channel);
            //printk("New channel: %d\n", pv.channel);
            // Send channel to slave
            test_rfcfg_send_chan();
            pv.state = TEST_RFCFG_CHAN_HOP_ASK_ACK;
        break;

        case TEST_RFCFG_CHAN_HOP_ASK_ACK:
            // Reset buffer
            memset(pv.cmd_buf, 0, sizeof(pv.cmd_buf));
            // Update channel
            pv.rq_struct.channel = pv.channel;
            // Send ack cmd
            test_rfcfg_send_ack_cmd();
            pv.state = TEST_RFCFG_CHAN_HOP_WAIT_ACK;
        break;

        case TEST_RFCFG_CHAN_HOP_WAIT_ACK:
            //printk("Waiting slave channel ack\n");
            // Wait in rx for ack
            test_rfcfg_receive(TEST_RFCFG_RX_TIME_MS);
            if (pv.chan_hop_counter > 0)
            {
                pv.chan_hop_counter--;
                pv.state = TEST_RFCFG_CHAN_HOP_SEND_CHAN;
            }
            else
                pv.state = TEST_RFCFG_STATE_END;

        break;
        #endif

        case TEST_RFCFG_STATE_END:
            #ifndef TEST_RFCFG_SLAVE_EXCHANGE
                test_rfcfg_tx_end();
            #endif
            if (pv.test_counter > 0)
            {
                pv.test_counter--;
                pv.state = TEST_RFCFG_STATE_START;
                test_rfcfg_wait(TEST_RFCFG_TEST_STEP_TIME_MS);
            }
            else
            {
                test_rfcfg_test_end(pv.test_idx);
                pv.test_idx++;
                if (pv.test_idx < test_rfcfg_array_size)
                {
                    pv.test_counter = p_test_rfcfg_array[pv.test_idx].count - 1;
                    pv.state = TEST_RFCFG_STATE_START;
                    test_rfcfg_wait(TEST_RFCFG_WAIT_MS);
                }
                else
                {
                    pv.state = TEST_RFCFG_WAIT_BUTTON;
                    test_rfcfg_wait(TEST_RFCFG_WAIT_MS);
                }
            }
        break;

        case TEST_RFCFG_WAIT_BUTTON:
            printk("*** END OF TEST ***\n");
            test_rfcfg_button();
        break;
    }
}

// *** Public Functions ***

void app_start(void)
{
    printk("*** START OF TEST ***\n");
    // Retrieve devices
    ensure(!device_get_accessor_by_path(&pv.rf_dev.base, NULL, "rfpacket0", DRIVER_CLASS_RFPACKET));
    ensure(!device_get_accessor_by_path(&pv.timer_dev.base,  NULL, "rfpacket0", DRIVER_CLASS_TIMER));
    ensure(!device_get_accessor_by_path(&pv.gpio_dev.base,  NULL, "gpio", DRIVER_CLASS_GPIO));
    // Set timer reference
    dev_timer_init_sec(&pv.timer_dev, &pv.msec, 0, 1, 1000);
    // Init restart button
    //DEVICE_OP(&pv.gpio_dev, set_mode, EFM32_PF6, EFM32_PF6, dev_gpio_mask1, DEV_PIN_INPUT_PULLUP);
    DEVICE_OP(&pv.gpio_dev, set_mode, EFM32_PB10, EFM32_PB10, dev_gpio_mask1, DEV_PIN_INPUT_PULLUP);
    // Init module
    test_rfcfg_baserq(&pv.rq_struct);
    test_rfcfg_config_init(&pv.timer_dev);
    test_rfcfg_init_process();
    // Start test
    test_rfcfg_wait(TEST_RFCFG_WAIT_MS);
}
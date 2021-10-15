/*
  This test application will send commands to a slave application to see if the slave rfpacket driver respects the ENT300220 Clause 9 LBT specifications
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
#include <device/class/gpio.h>
#include <device/class/timer.h>
#include <device/class/rfpacket.h>
// Board
#include <arch/efm32/pin.h>

// *** Definitions ***
// --- Private Constants ---
#define TEST_LOOP_NB 20 // Number of iterations on each test
#define TEST_INCR_NB 5 // Number of fixed size disturb test (with size incrementation between each)
#define MAX_RETRY_NB 5 // Number of retry to detect the slave
#define TX_CHANNEL 0
#define RX_CHANNEL 0
#define BASE_PACKET_SIZE 128 // Base size for the fixed size disturb test
#define MAX_PACKET_SIZE 255 // Maximum packet size
#define WAIT_PERIOD_MS 250 // Waiting time between tests
#define SYNC_TIME_MS 250 // Waiting time between slave detection packet
#define RX_TIME_MS 25 // Maximum waiting time for slave test response and time between test packets
#define DISTURB_DEADLINE_MS 0 // Waiting time before sending disturb packet
#define TD_MS_TICK_NB 32 // Value to divide td to get ms
#define TX_POWER 0

// Commands
#define NEXT_CMD 'n'
#define LBT_CMD 'l'
#define LBT_BSY_CMD 'b'
#define GIVE_TD_CMD 'd'
#define ACK_RSP 'k'

// Acceptance values
#define ACK_TEST_ACC_MAX_VAL_MS 5
#define LBT_CLR_TEST_ACC_MIN_VAL_MS 4
#define LBT_CLR_TEST_ACC_MAX_VAL_MS 6
#define LBT_BSY_TEST_ACC_MIN_VAL_MS 4
#define LBT_BSY_TEST_ACC_MAX_VAL_MS 11
#define LBT_BSY_MEAN_DIFF_MIN_VAL_TCKS 24
#define LBT_BSY_MEAN_DIFF_MAX_VAL_MS 3
#define TD_ACC_MAX_VAL_MS 5

// PCKT SIZE
#define CRC_SIZE 2
#define PAYLOAD_LENGTH_SIZE 1
// Assuming the slave use the same tx preamble length as the master
#define PCKT_SIZE(size) (pkcfg.tx_pb_len + pkcfg.sw_len + 1 + 8 * ((size) + CRC_SIZE + PAYLOAD_LENGTH_SIZE))
#define PCKT_TIME(size) (((uint64_t)PCKT_SIZE(size) * 1000 * pv.msec) / rfcfg.common.drate)

// --- Private Types ---
enum _test_rflbt_state
{
    STATE_SYNC,
    STATE_WAIT_SYNC,
    STATE_TEST_ACK,
    STATE_TEST_WAIT_ACK,
    STATE_TEST_LBT_CLR,
    STATE_TEST_WAIT_LBT_CLR,
    STATE_TEST_LBT_BSY,
    STATE_TEST_WAIT_LBT_BSY,
    STATE_TEST_LBT_BSY_RND,
    STATE_TEST_WAIT_LBT_BSY_RND,
    STATE_TEST_REQ_TIME_DEAD,
    STATE_TEST_WAIT_TIME_DEAD,
    STATE_TEST_PRINT_RESULT,
    STATE_TEST_END,
};

typedef struct _test_rflbt_result
{
    uint32_t ack;
    uint32_t lbt_clr;
    uint32_t lbt_busy_delay_a[TEST_INCR_NB];
    uint32_t lbt_busy_md_a[TEST_INCR_NB];
    uint32_t lbt_busy_min_a[TEST_INCR_NB];
    uint32_t lbt_busy_max_a[TEST_INCR_NB];
    uint32_t lbt_busy_rand_delay;
    uint32_t lbt_busy_rand_md;
    uint32_t lbt_busy_rand_min;
    uint32_t lbt_busy_rand_max;
} test_rflbt_result_t;

typedef struct _test_rflbt_info
{
    uint8_t state;
    bool packet_ok;
    uint8_t rx_buf[4];
    uint8_t cmd_buf[4];
    uint8_t gpio_data[4];
    uint8_t dist_buf[MAX_PACKET_SIZE];
    uint16_t rand_size_array[TEST_LOOP_NB];
    uint32_t results_array[TEST_LOOP_NB];
    uint32_t packet_size;
    uint32_t val_counter;
    uint32_t test_counter;
    uint32_t incr_counter;
    uint32_t base_resp_time;
    uint32_t time_dead;
    uint32_t min_time;
    uint32_t max_time;
    dev_timer_delay_t msec;
    dev_timer_delay_t mean_result;
    dev_timer_value_t reply_time;
    struct dev_rfpacket_rq_s rq_struct;
    struct dev_rfpacket_rq_s rq_disturb;
    struct dev_rfpacket_rx_s rx_struct;
    struct dev_timer_rq_s trq_struct;
    struct dev_gpio_rq_s grq_struct;
    struct device_rfpacket_s rf_dev;
    struct device_timer_s timer_dev;
    struct device_gpio_s gpio_dev;
    test_rflbt_result_t result_struct;
} test_rflbt_info_t;



// --- Private Function Prototypes ---
static void test_rflbt_mean_add(dev_timer_value_t newVal);
static void test_rflbt_set_min_max(uint32_t val);
static void test_rflbt_result_clear(void);
static uint32_t test_rflbt_mean_calc(void);
static uint32_t test_rflbt_mean_diff_calc(uint32_t mean);
static void test_rflbt_wait(uint32_t wait_time);
static void test_rflbt_baserq(struct dev_rfpacket_rq_s *rq);
static void test_rflbt_disturb(uint32_t dl, uint16_t size);
static void test_rflbt_send(uint8_t *pBuf, uint32_t buf_size);
static void test_rflbt_send_char(char c);
static void test_rflbt_receive(uint32_t ttl);
static void test_rflbt_end(void);
static void test_rflbt_process(void);

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
        .frequency = 865046875,
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
static void test_rflbt_mean_add(dev_timer_value_t newVal)
{
    pv.mean_result += newVal;
    pv.val_counter++;
}

static void test_rflbt_set_min_max(uint32_t val)
{
    pv.min_time = __MIN(pv.min_time, val);
    pv.max_time = __MAX(pv.max_time, val);
}

static void test_rflbt_result_clear(void)
{
    pv.mean_result = 0;
    pv.val_counter = 0;
    pv.min_time = ~0;
    pv.max_time = 0;
}

static uint32_t test_rflbt_mean_calc(void)
{
    return (pv.val_counter > 0) ? pv.mean_result/pv.val_counter : 0;
}

static uint32_t test_rflbt_mean_diff_calc(uint32_t mean)
{
    uint32_t acc_diff = 0;
    uint32_t *val_array = pv.results_array;
    uint32_t val_count = pv.val_counter;

    for (uint32_t idx = 0; idx < val_count; idx++)
    {
        uint32_t val = val_array[idx];
        acc_diff += (val > mean) ? val - mean : mean - val;
    }
    //printk("acc_diff: %d\n", acc_diff);
    return (val_count > 0) ? acc_diff/val_count : 0;
}

static KROUTINE_EXEC(test_rflbt_rx_pckt_cb)
{
    struct dev_rfpacket_rx_s *rx = dev_rfpacket_rx_s_from_kr(kr);

    if (rx->error == 0)
    {
        uint8_t *pBuff = (uint8_t *)rx->buf;
        // Check data
        if (*(char *)pBuff == ACK_RSP)
        {
            pv.reply_time = rx->timestamp - pv.reply_time - (PCKT_TIME(rx->size));
            //printk("Time: %lld ticks\n", pv.reply_time);

            if ((pv.state == STATE_TEST_LBT_BSY) || (pv.state == STATE_TEST_LBT_BSY_RND))
            {
                pv.results_array[pv.val_counter] = (uint32_t)(pv.reply_time);
                //printk("Reply time: %d %d\n", pv.val_counter, pv.resultks[pv.val_counter]);
            }
            test_rflbt_set_min_max(pv.reply_time);
            test_rflbt_mean_add(pv.reply_time);
            pv.packet_ok = true;

            if (pv.state == STATE_SYNC)
            {
                pv.state = STATE_TEST_ACK;
                pv.test_counter = TEST_LOOP_NB;
            }
            //printk("Received ack.\n");
        }
        else if (pv.state == STATE_TEST_PRINT_RESULT)
        {
            pv.packet_ok = true;
            pv.time_dead = 0;

            for (uint8_t idx = 0; idx < rx->size; idx++)
                pv.time_dead += pBuff[idx] << (8*idx);

            // Normalize value to tick number
            pv.time_dead = pv.time_dead * pv.msec / TD_MS_TICK_NB;
        }
    }
}

static struct dev_rfpacket_rx_s *test_rflbt_rx_alloc(struct dev_rfpacket_rq_s *rq, size_t size)
{
    if (size > MAX_PACKET_SIZE)
        return NULL;

    struct dev_rfpacket_rx_s *rx = &pv.rx_struct;
    rx->buf = pv.rx_buf;
    kroutine_init_deferred(&rx->kr, &test_rflbt_rx_pckt_cb);
    rx->size = size;
    return rx;
}

static KROUTINE_EXEC(test_rflbt_rx_cb)
{
    struct dev_rfpacket_rq_s *rq = dev_rfpacket_rq_from_kr(kr);

    if (rq->error == -ENOTSUP)
        printk("Bad RX configuration\n");

    else if (rq->error)
        printk("RX error: %d\n", rq->error);

    if(!pv.packet_ok && (pv.state != STATE_SYNC))
    {
        printk("Error: slave didn't respond !\n");
        test_rflbt_end();
    }
    pv.packet_ok = false;
    test_rflbt_process();
}

static KROUTINE_EXEC(test_rflbt_tx_cb)
{
    struct dev_rfpacket_rq_s *rq = dev_rfpacket_rq_from_kr(kr);

    if (rq->error == -ENOTSUP)
        printk("Bad TX configuration\n");

    if (rq->error == -ETIMEDOUT)
        printk("TX timeout\n");

    pv.reply_time = rq->tx_timestamp;
    // dev_timer_value_t timeval;
    // DEVICE_OP(&pv.timer_dev, get_value, &timeval, 0);
    test_rflbt_process();
}

static KROUTINE_EXEC(test_rflbt_dist_cb)
{
    struct dev_rfpacket_rq_s *rq = dev_rfpacket_rq_from_kr(kr);

    if (rq->error)
        printk("Tx error: %d\n", rq->error);

    else
    {
        if(!pv.packet_ok)
        {
            pv.reply_time = rq->tx_timestamp;
            //printk("Disturb ok\n");
        }
        else
            printk("Packet received before disturb\n");
    }
}

static KROUTINE_EXEC(test_rflbt_wait_cb)
{
    test_rflbt_process();
}

static KROUTINE_EXEC(test_rflbt_button_cb)
{
    printk(" *** Button pressed - Restarting test ***\n");
    pv.test_counter = MAX_RETRY_NB;
    pv.state = STATE_SYNC;
    test_rflbt_result_clear();

    test_rflbt_wait(WAIT_PERIOD_MS);
}

static void test_rflbt_wait(uint32_t wait_time)
{
    struct dev_timer_rq_s *trq = &pv.trq_struct;
    trq->delay = wait_time * pv.msec;
    trq->rev = 0;
    //printk("Waiting initiated !\n");
    dev_timer_rq_init(trq, test_rflbt_wait_cb);
    error_t err = DEVICE_OP(&pv.timer_dev, request, trq);

    if (err == -ETIMEDOUT)
    {
        printk("Warning: Timer timeout\n");
        kroutine_exec(&trq->base.kr);
    }
    else if (err)
        printk("Error: Timer failed: %d\n", err);
}

static void test_rflbt_button(void)
{
    struct dev_gpio_rq_s *grq = &pv.grq_struct;
    grq->io_first = EFM32_PF6;
    grq->io_last = EFM32_PF6;
    // grq->io_first = EFM32_PB10;
    // grq->io_last = EFM32_PB10;
    grq->type = DEV_GPIO_UNTIL;
    grq->until.mask = dev_gpio_mask1;
    pv.gpio_data[0] = 0x1;
    grq->until.data = pv.gpio_data;
    dev_gpio_rq_init(grq, test_rflbt_button_cb);
    DEVICE_OP(&pv.gpio_dev, request, grq);
}

static void test_rflbt_baserq(struct dev_rfpacket_rq_s *rq)
{
    rq->err_group = 0;
    rq->anchor = DEV_RFPACKET_TIMESTAMP_END;
    rq->pk_cfg = &pkcfg.base;
    rq->rf_cfg = &rfcfg.base;
}

static void test_rflbt_disturb(uint32_t dl, uint16_t size)
{
    struct dev_rfpacket_rq_s *rq = &pv.rq_disturb;
    // Set request struct
    rq->channel = TX_CHANNEL;
    rq->type = DEV_RFPACKET_RQ_TX;
    if (dl > 0)
    {
        DEVICE_OP(&pv.timer_dev, get_value, &rq->deadline, 0);
        rq->deadline += dl * pv.msec;
    }
    else
        rq->deadline = 0;

    rq->lifetime = 0;
    // Set data and size
    rq->tx_size = size;
    rq->tx_buf = pv.dist_buf;
    rq->tx_pwr = TX_POWER;
    dev_rfpacket_rq_init(rq, &test_rflbt_dist_cb);
    DEVICE_OP(&pv.rf_dev, request, rq, NULL);
}

static void test_rflbt_send(uint8_t *pBuf, uint32_t buf_size)
{
    struct dev_rfpacket_rq_s *rq = &pv.rq_struct;
    rq->channel = TX_CHANNEL;
    rq->type = DEV_RFPACKET_RQ_TX;
    rq->deadline = 0;
    rq->lifetime = 0;
    // Set data and size
    rq->tx_size = buf_size;
    rq->tx_buf = pBuf;
    //printk("Sending next command\n");
    rq->tx_pwr = TX_POWER;
    dev_rfpacket_rq_init(rq, &test_rflbt_tx_cb);
    DEVICE_OP(&pv.rf_dev, request, rq, NULL);
}

static void test_rflbt_send_char(char c)
{
    memset(pv.cmd_buf, 0, sizeof(pv.cmd_buf));
    pv.cmd_buf[0] = c;
    test_rflbt_send((uint8_t *)&pv.cmd_buf, sizeof(uint32_t));
}

static void test_rflbt_receive(uint32_t ttl)
{
    struct dev_rfpacket_rq_s *rq = &pv.rq_struct;
    rq->channel = RX_CHANNEL;
    rq->type = DEV_RFPACKET_RQ_RX;
    rq->deadline = 0;
    rq->lifetime = ttl * pv.msec;
    rq->rx_alloc = &test_rflbt_rx_alloc;
    dev_rfpacket_rq_init(rq, &test_rflbt_rx_cb);
    DEVICE_OP(&pv.rf_dev, request, rq, NULL);
}

static void test_rflbt_end(void)
{
    pv.state = STATE_TEST_END;
    pv.test_counter = TEST_LOOP_NB;
    test_rflbt_result_clear();
    printk("*** END OF TEST ***\n");
}

static void test_rflbt_print_result(void)
{
    uint32_t result, corr_result, corr_min, corr_max;
    // ACK
    result = pv.result_struct.ack;
    printk("ACK test mean result: %d ticks / %d ms\n", result, result/pv.msec);

    // Acceptance test
    if (result > ACK_TEST_ACC_MAX_VAL_MS * pv.msec)
        printk("Test failed, max accepted value is: %d ms\n", ACK_TEST_ACC_MAX_VAL_MS);

    else
        printk("Test passed\n");

    // LBT CLR
    result = pv.result_struct.lbt_clr;
    corr_result = result;

    if (result > (pv.base_resp_time + pv.time_dead))
        corr_result = result - pv.base_resp_time - pv.time_dead;

    printk("* LBT clear chan test mean result: (%d) %d ticks / %d ms\n", result, corr_result, corr_result/pv.msec);

    // Acceptance test
    if ((corr_result < LBT_CLR_TEST_ACC_MIN_VAL_MS * pv.msec) || (corr_result > LBT_CLR_TEST_ACC_MAX_VAL_MS * pv.msec))
        printk("Test failed, value should be between: %d ms and %d ms\n", LBT_CLR_TEST_ACC_MIN_VAL_MS, LBT_CLR_TEST_ACC_MAX_VAL_MS);

    else
        printk("Test passed\n");

    // LBT BSY INCR
    for (uint32_t idx = 0; idx < TEST_INCR_NB; idx++)
    {
        uint32_t packet_size = BASE_PACKET_SIZE + idx;
        result = pv.result_struct.lbt_busy_delay_a[idx];
        corr_result = result;
        if (result > (pv.time_dead))
            corr_result = result - pv.time_dead;

        corr_min = pv.result_struct.lbt_busy_min_a[idx];
        if (corr_min > (pv.time_dead))
            corr_min -= pv.time_dead;

        corr_max = pv.result_struct.lbt_busy_max_a[idx];
        if (corr_max > (pv.time_dead))
            corr_max -= pv.time_dead;

        printk("* LBT busy chan #%d test. Packet_size: %d, mean result: (%d) %d ticks / %d ms, mean difference: %d ticks / %d ms\n", idx+1,
            packet_size, result, corr_result, corr_result/pv.msec, pv.result_struct.lbt_busy_md_a[idx], pv.result_struct.lbt_busy_md_a[idx]/pv.msec);
        printk("Min time: %d ticks, max time: %d ticks\n", corr_min, corr_max);
        // Acceptance test
        if ((corr_result < LBT_BSY_TEST_ACC_MIN_VAL_MS * pv.msec) || (corr_result > LBT_BSY_TEST_ACC_MAX_VAL_MS * pv.msec))
            printk("Test failed, value should be between: %d ms and %d ms\n", LBT_BSY_TEST_ACC_MIN_VAL_MS, LBT_BSY_TEST_ACC_MAX_VAL_MS);

        else
            printk("Test passed\n");

        if (pv.result_struct.lbt_busy_md_a[idx] < LBT_BSY_MEAN_DIFF_MIN_VAL_TCKS)
            printk("Warning: mean difference is low, waiting time might not be random.\n");

        else if (pv.result_struct.lbt_busy_md_a[idx] > LBT_BSY_MEAN_DIFF_MAX_VAL_MS * pv.msec)
            printk("Warning: mean difference is too high, unstable environment ?\n");

    }
    // LBT BSY RAND
    result = pv.result_struct.lbt_busy_rand_delay;
    corr_result = result;

    if (result > (pv.time_dead))
        corr_result = result - pv.time_dead;

    corr_min = pv.result_struct.lbt_busy_rand_min;

    if (corr_min > (pv.time_dead))
        corr_min -= pv.time_dead;

    corr_max = pv.result_struct.lbt_busy_rand_max;

    if (corr_max > (pv.time_dead))
        corr_max -= pv.time_dead;

    printk("* LBT busy chan random size test mean result: (%d) %d ticks / %d ms, mean difference: %d ticks / %d ms\n",
        result, corr_result, corr_result/pv.msec, pv.result_struct.lbt_busy_rand_md, pv.result_struct.lbt_busy_rand_md/pv.msec);
    printk("Min time: %d ticks, max time: %d ticks\n", corr_min, corr_max);

    // Acceptance test
    if ((corr_result < LBT_BSY_TEST_ACC_MIN_VAL_MS * pv.msec) || (corr_result > LBT_BSY_TEST_ACC_MAX_VAL_MS * pv.msec))
        printk("Test failed, value should be between: %d ms and %d ms\n", LBT_BSY_TEST_ACC_MIN_VAL_MS, LBT_BSY_TEST_ACC_MAX_VAL_MS);

    else
        printk("Test passed\n");

    if (pv.result_struct.lbt_busy_rand_md < LBT_BSY_MEAN_DIFF_MIN_VAL_TCKS)
        printk("Warning: mean difference is low, waiting time might not be random.\n");

    else if (pv.result_struct.lbt_busy_rand_md > LBT_BSY_MEAN_DIFF_MAX_VAL_MS * pv.msec)
        printk("Warning: mean difference is too high, unstable environment ?\n");

    printk("Disturb sizes: ");

    for (uint32_t idx = 0; idx < TEST_LOOP_NB; idx++)
        printk("%d, ", pv.rand_size_array[idx]);

    printk("\n");
    // TD
    printk("* Time dead result: %d ticks / %d ms\n", pv.time_dead, pv.time_dead/pv.msec);

    if (pv.time_dead > TD_ACC_MAX_VAL_MS * pv.msec)
        printk("Test failed, max accepted value is: %d ms\n", TD_ACC_MAX_VAL_MS);

    else
        printk("Test passed\n");
}

static void test_rflbt_process(void)
{
    switch (pv.state)
    {
        case STATE_SYNC:
            if (pv.test_counter > 0)
            {
                pv.state = STATE_WAIT_SYNC;
                pv.test_counter--;
                test_rflbt_send_char(NEXT_CMD);
            }
            else
            {
                printk("Error: Couldn't contact slave !\n");
                test_rflbt_end();
                test_rflbt_button();
                //test_rflbt_wait(WAIT_PERIOD_MS);
            }
        break;

        case STATE_WAIT_SYNC:
            pv.state = STATE_SYNC;
            test_rflbt_receive(SYNC_TIME_MS);
        break;

        case STATE_TEST_ACK:
            if (pv.test_counter > 0)
            {
                pv.test_counter--;
                pv.state = STATE_TEST_WAIT_ACK;
                test_rflbt_send_char(NEXT_CMD);
            }
            else
            {
                pv.result_struct.ack = test_rflbt_mean_calc();
                pv.base_resp_time = pv.result_struct.ack;
                pv.state = STATE_TEST_LBT_CLR;
                pv.test_counter = TEST_LOOP_NB;
                test_rflbt_result_clear();
                test_rflbt_wait(WAIT_PERIOD_MS);
            }
        break;

        case STATE_TEST_WAIT_ACK:
            pv.state = STATE_TEST_ACK;
            test_rflbt_receive(RX_TIME_MS);
        break;

        case STATE_TEST_LBT_CLR:
            if (pv.test_counter > 0)
            {
                pv.test_counter--;
                pv.state = STATE_TEST_WAIT_LBT_CLR;
                test_rflbt_send_char(LBT_CMD);
            }
            else
            {
                pv.result_struct.lbt_clr = test_rflbt_mean_calc();
                pv.state = STATE_TEST_LBT_BSY;
                pv.test_counter = TEST_LOOP_NB;
                test_rflbt_result_clear();
                test_rflbt_wait(WAIT_PERIOD_MS);
                pv.packet_size = BASE_PACKET_SIZE;
                pv.incr_counter = 0;
            }
        break;

        case STATE_TEST_WAIT_LBT_CLR:
            pv.state = STATE_TEST_LBT_CLR;
            test_rflbt_receive(RX_TIME_MS);
        break;

        case STATE_TEST_LBT_BSY:
            if (pv.test_counter > 0)
            {
                pv.test_counter--;
                pv.state = STATE_TEST_WAIT_LBT_BSY;
                test_rflbt_send_char(LBT_BSY_CMD);
                test_rflbt_disturb(DISTURB_DEADLINE_MS, pv.packet_size);
            }
            else
            {
                // Note results
                pv.result_struct.lbt_busy_delay_a[pv.incr_counter] = test_rflbt_mean_calc();
                pv.result_struct.lbt_busy_md_a[pv.incr_counter] = test_rflbt_mean_diff_calc(pv.result_struct.lbt_busy_delay_a[pv.incr_counter]);
                pv.result_struct.lbt_busy_min_a[pv.incr_counter] = pv.min_time;
                pv.result_struct.lbt_busy_max_a[pv.incr_counter] = pv.max_time;
                pv.test_counter = TEST_LOOP_NB;
                // Set next test
                if (pv.incr_counter < (TEST_INCR_NB - 1))
                {
                    pv.packet_size++;
                    pv.incr_counter++;
                }
                else
                    pv.state = STATE_TEST_LBT_BSY_RND;

                // Reset vars and go wait till next test
                test_rflbt_result_clear();
                test_rflbt_wait(WAIT_PERIOD_MS);
            }
        break;

        case STATE_TEST_WAIT_LBT_BSY:
            pv.state = STATE_TEST_LBT_BSY;
            test_rflbt_receive(RX_TIME_MS);
        break;

        case STATE_TEST_LBT_BSY_RND:
            if (pv.test_counter > 0)
            {
                pv.test_counter--;
                pv.state = STATE_TEST_WAIT_LBT_BSY_RND;
                test_rflbt_send_char(LBT_BSY_CMD);
                uint16_t rand_size = 64 + rand() % sizeof(pv.dist_buf)/2;
                pv.rand_size_array[pv.val_counter] = rand_size;
                test_rflbt_disturb(DISTURB_DEADLINE_MS, rand_size);
            }
            else
            {
                pv.result_struct.lbt_busy_rand_delay = test_rflbt_mean_calc();
                pv.result_struct.lbt_busy_rand_md = test_rflbt_mean_diff_calc(pv.result_struct.lbt_busy_rand_delay);
                pv.result_struct.lbt_busy_rand_min = pv.min_time;
                pv.result_struct.lbt_busy_rand_max = pv.max_time;
                pv.state = STATE_TEST_REQ_TIME_DEAD;
                pv.test_counter = TEST_LOOP_NB;
                test_rflbt_result_clear();
                test_rflbt_wait(WAIT_PERIOD_MS);
            }
        break;

        case STATE_TEST_WAIT_LBT_BSY_RND:
            pv.state = STATE_TEST_LBT_BSY_RND;
            test_rflbt_receive(RX_TIME_MS);
        break;

        case STATE_TEST_REQ_TIME_DEAD:
            test_rflbt_send_char(GIVE_TD_CMD);
            pv.state = STATE_TEST_WAIT_TIME_DEAD;
        break;

        case STATE_TEST_WAIT_TIME_DEAD:
            test_rflbt_receive(RX_TIME_MS);
            pv.state = STATE_TEST_PRINT_RESULT;
        break;

        case STATE_TEST_PRINT_RESULT:
            test_rflbt_print_result();
            test_rflbt_end();
            test_rflbt_wait(WAIT_PERIOD_MS);
        break;

        case STATE_TEST_END:
            test_rflbt_button();
            //test_rflbt_wait(2 * WAIT_PERIOD_MS);
        break;

        default:
            pv.state = STATE_TEST_END;
        break;
    }
}

// *** Public Functions ***

void app_start(void)
{
    printk("*** START OF TEST ***\n");
    // Retrieve devices
    ensure(!device_get_accessor_by_path(&pv.rf_dev.base, NULL, "rfpacket*", DRIVER_CLASS_RFPACKET));
    ensure(!device_get_accessor_by_path(&pv.timer_dev.base,  NULL, "rfpacket*", DRIVER_CLASS_TIMER));
    ensure(!device_get_accessor_by_path(&pv.gpio_dev.base,  NULL, "gpio", DRIVER_CLASS_GPIO));
    // // Set timer reference
    dev_timer_init_sec(&pv.timer_dev, &pv.msec, 0, 1, 1000);
    printk("Msec: %d\n", pv.msec);

    // Init buffer
    for (uint8_t idx = 1; idx < sizeof(pv.dist_buf)/10; idx++)
        pv.dist_buf[idx*10] = 0x55;

    // // Init button
    DEVICE_OP(&pv.gpio_dev, set_mode, EFM32_PF6, EFM32_PF6, dev_gpio_mask1, DEV_PIN_INPUT_PULLUP);
    // DEVICE_OP(&pv.gpio_dev, set_mode, EFM32_PB10, EFM32_PB10, dev_gpio_mask1, DEV_PIN_INPUT_PULLUP);
    // Init module
    test_rflbt_baserq(&pv.rq_struct);
    test_rflbt_baserq(&pv.rq_disturb);
    pv.test_counter = MAX_RETRY_NB;
    pv.state = STATE_SYNC;
    test_rflbt_result_clear();
    test_rflbt_wait(WAIT_PERIOD_MS);
}
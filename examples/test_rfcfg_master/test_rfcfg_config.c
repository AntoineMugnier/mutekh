// *** Libraries include **
#include "test_rfcfg_config.h"

// --- Public variables ---

// Basic packet (28B)
static struct dev_rfpacket_pk_cfg_basic_s basic_pkcfg = {
    .base = {
        .format = DEV_RFPACKET_FMT_SLPC,
        .encoding = DEV_RFPACKET_CLEAR,
        .cache = {
            .id = 0,
            .dirty = 0
        },
    },
    .crc = 0x8005,
    .crc_seed = 0xffff,
    .sw_value = 0xabba,
    .sw_len = 15,
    .pb_pattern = 0x2,
    .pb_pattern_len = 1,
    .tx_pb_len = 64,
    .rx_pb_len = 16,
};

// Fsk (36B)
static struct dev_rfpacket_rf_cfg_fsk_s fsk_rfcfg = {
    .base = {
        .mod = DEV_RFPACKET_GFSK,
        .cache = {
            .id = 0,
            .dirty = 0
        },
        .drate = 38400,
        .jam_rssi = (-90) << 3,
        .frequency = 865056875 - 20000,
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

// Ask (32B)
static struct dev_rfpacket_rf_cfg_ask_s ask_rfcfg = {
    .base = {
        .mod = DEV_RFPACKET_ASK,
        .cache = {
            .id = 0,
            .dirty = 0
        },
        .drate = 38400,
        .jam_rssi = (-90) << 3,
        .frequency = 865056875 - 20000,
        .chan_spacing = 93750,
        .rx_bw = 0,
        .freq_err = 868 * 20 /* ppm */,
    },
    .fairtx = {
        .mode = DEV_RFPACKET_LBT,
        .lbt.rssi = (-90) << 3,
        .lbt.duration = 5000, /** us */
    },
    .symbols = 2,
};

// Lora (28B)

// Set default config
struct dev_rfpacket_pk_cfg_s *def_pkcfg = &basic_pkcfg.base;
struct dev_rfpacket_rf_cfg_s *def_rfcfg = &fsk_rfcfg.base;

// --- Private Constants ---
#define TEST_RFCFG_RF_PAYLOAD 'r'
#define TEST_RFCFG_PK_PAYLOAD 'p'

#define TEST_RFCFG_DRATE_MIN 100
#define TEST_RFCFG_DRATE_MAX 60000

#define TEST_RFCFG_CHSPACE_MIN 50000
#define TEST_RFCFG_CHSPACE_MAX 100000

#define TEST_RFCFG_FREQ400_MIN 420000000
#define TEST_RFCFG_FREQ400_MAX 460000000

#define TEST_RFCFG_FREQ800_MIN 840000000
#define TEST_RFCFG_FREQ800_MAX 920000000

#define TEST_RFCFG_FSK_FDEV_MIN 5000
#define TEST_RFCFG_FSK_FDEV_MAX 25000

// --- Private Types ---
typedef struct _test_rfcfg_config_info {
    uint8_t cfg_buffer[37];
    uint8_t slave_cfg_size;
    uint8_t *slave_cfg;
    uint32_t time_counter;
    dev_timer_delay_t test_time;
    dev_timer_value_t start_time;
    struct dev_rfpacket_rf_cfg_s *new_rfcfg;
    struct dev_rfpacket_pk_cfg_s *new_pkcfg;
    struct device_timer_s *timer;
} test_rfcfg_config_info_t;

// --- Private Function Prototypes ---
static void print_slave_rfcfg(const struct dev_rfpacket_rf_cfg_s *rfcfg);
//static void print_slave_pkcfg(const struct dev_rfpacket_pk_cfg_s *pkcfg);

// --- Private Variables ---
static test_rfcfg_config_info_t pv;

// --- Private functions ---
static void print_slave_rfcfg(const struct dev_rfpacket_rf_cfg_s *rfcfg) {
    if (rfcfg == &fsk_rfcfg.base) {
        memcpy(&pv.cfg_buffer[1], &fsk_rfcfg, sizeof(struct dev_rfpacket_rf_cfg_fsk_s));
        pv.cfg_buffer[0] = (uint8_t)TEST_RFCFG_RF_PAYLOAD;
        pv.slave_cfg = pv.cfg_buffer;
        pv.slave_cfg_size = sizeof(struct dev_rfpacket_rf_cfg_fsk_s) + 1;
        //printk("Rfconfig: %P\n", &pv.cfg_buffer[1], sizeof(fsk_rfcfg));
    } else if (rfcfg == &ask_rfcfg.base) {
        memcpy(&pv.cfg_buffer[1], &ask_rfcfg, sizeof(struct dev_rfpacket_rf_cfg_ask_s));
        pv.cfg_buffer[0] = (uint8_t)TEST_RFCFG_RF_PAYLOAD;
        pv.slave_cfg = pv.cfg_buffer;
        pv.slave_cfg_size = sizeof(struct dev_rfpacket_rf_cfg_ask_s) + 1;
        //printk("Rfconfig: %P\n", &pv.cfg_buffer[1], sizeof(ask_rfcfg));        
    } else { // TODO LORA
        printk("Unknown config struct\n");
        pv.slave_cfg = NULL;
    }
}

// static void print_slave_pkcfg(const struct dev_rfpacket_pk_cfg_s *pkcfg) {
//     if (pkcfg == &basic_pkcfg.base) {
//         memcpy(&pv.cfg_buffer[1], &basic_pkcfg, sizeof(struct dev_rfpacket_pk_cfg_basic_s));
//         pv.cfg_buffer[0] = (uint8_t)TEST_RFCFG_PK_PAYLOAD;
//         pv.slave_cfg = pv.cfg_buffer;
//         pv.slave_cfg_size = sizeof(struct dev_rfpacket_pk_cfg_basic_s) + 1;
//         //printk("Pkconfig: %P\n", &pv.cfg_buffer[1], sizeof(basic_pkcfg));
//     } else {
//         printk("Unknown packet struct\n");
//         pv.slave_cfg = NULL;
//     }
// }

// --- Public functions ---

// Init this module and main module test array
void test_rfcfg_config_init(struct device_timer_s *timer) {
    pv.timer = timer;
}

void test_rfcfg_get_slave_config(uint8_t **p_buf, uint8_t *p_buf_size) {
    *p_buf = pv.slave_cfg;
    *p_buf_size = pv.slave_cfg_size;
}   

void test_rfcfg_update_rq_config(struct dev_rfpacket_rq_s *rq) {
    // Start time measurement
    DEVICE_OP(pv.timer, get_value, &pv.start_time, 0);    
    // Update request config
    if (pv.new_pkcfg != NULL) {
        rq->pk_cfg = pv.new_pkcfg;
        pv.new_pkcfg->cache.dirty = 1;
    }
    if (pv.new_rfcfg != NULL) {
        rq->rf_cfg = pv.new_rfcfg;
        pv.new_rfcfg->cache.dirty = 1;
    }
}

void test_rfcfg_tx_end(void) {
    // End time measurement
    dev_timer_value_t end_time;
    DEVICE_OP(pv.timer, get_value, &end_time, 0);
    if (pv.time_counter == 0) {
        pv.test_time = end_time - pv.start_time;
    } else {
        pv.test_time = __MIN(end_time - pv.start_time, pv.test_time);
    }
    pv.time_counter++;
}

void test_rfcfg_test_end(uint32_t idx) {
    printk("Test %d min test time: %d,\n", idx, pv.test_time);
    pv.test_time = 0;
    pv.time_counter = 0;
}






// --- Test functions ---

static void def_set_cfg(struct dev_rfpacket_rq_s *rq) {
    // Set new parameters
    //printk("Default config test\n");
    // Indicate that there is no change
    pv.new_rfcfg = NULL;
    pv.new_pkcfg = NULL;
    print_slave_rfcfg(rq->rf_cfg);
    // Debug
    //print_slave_pkcfg(rq->pk_cfg);
}

static void drate_set_cfg(struct dev_rfpacket_rq_s *rq) {
    //printk("Data rate config test\n");
    // Set new drate value
    dev_timer_value_t t;
    DEVICE_OP(pv.timer, get_value, &t, 0);
    struct dev_rfpacket_rf_cfg_s *rfcfg = (struct dev_rfpacket_rf_cfg_s *)rq->rf_cfg;
    rfcfg->drate = rand_64_range_r(&t, TEST_RFCFG_DRATE_MIN, TEST_RFCFG_DRATE_MAX);
    //printk("Drate value: %d\n", rq->rf_cfg->drate);
    // Indicate that there is rfcfg change
    pv.new_rfcfg = rfcfg;
    pv.new_pkcfg = NULL;
    // Set slave buffer
    print_slave_rfcfg(rfcfg);
}

static void chan_space_set_cfg(struct dev_rfpacket_rq_s *rq) {
    // Set new freq value
    dev_timer_value_t t;
    DEVICE_OP(pv.timer, get_value, &t, 0);
    struct dev_rfpacket_rf_cfg_s *rfcfg = (struct dev_rfpacket_rf_cfg_s *)rq->rf_cfg;
    rfcfg->chan_spacing = rand_64_range_r(&t, TEST_RFCFG_CHSPACE_MIN/10, TEST_RFCFG_CHSPACE_MAX/10) * 10; 
    //printk("channel space value: %d\n", rfcfg->chan_spacing);
    // Indicate that there is rfcfg change
    pv.new_rfcfg = rfcfg;
    pv.new_pkcfg = NULL;
    // Set slave buffer
    print_slave_rfcfg(rfcfg);
}

static void freq400_set_cfg(struct dev_rfpacket_rq_s *rq) {
    // Set new freq value
    dev_timer_value_t t;
    DEVICE_OP(pv.timer, get_value, &t, 0);
    struct dev_rfpacket_rf_cfg_s *rfcfg = (struct dev_rfpacket_rf_cfg_s *)rq->rf_cfg;
    rfcfg->frequency = rand_64_range_r(&t, TEST_RFCFG_FREQ400_MIN, TEST_RFCFG_FREQ400_MAX); 
    //printk("frequency400 value: %d\n", rfcfg->frequency);
    // Indicate that there is rfcfg change
    pv.new_rfcfg = rfcfg;
    pv.new_pkcfg = NULL;
    // Set slave buffer
    print_slave_rfcfg(rfcfg);
}

static void freq800_set_cfg(struct dev_rfpacket_rq_s *rq) {
    // Set new freq value
    dev_timer_value_t t;
    DEVICE_OP(pv.timer, get_value, &t, 0);
    struct dev_rfpacket_rf_cfg_s *rfcfg = (struct dev_rfpacket_rf_cfg_s *)rq->rf_cfg;
    rfcfg->frequency = rand_64_range_r(&t, TEST_RFCFG_FREQ800_MIN, TEST_RFCFG_FREQ800_MAX); 
    //printk("frequency800 value: %d\n", rfcfg->frequency);
    // Indicate that there is rfcfg change
    pv.new_rfcfg = rfcfg;
    pv.new_pkcfg = NULL;
    // Set slave buffer
    print_slave_rfcfg(rfcfg);
}

static void fix_freq_set_cfg(struct dev_rfpacket_rq_s *rq) {
    struct dev_rfpacket_rf_cfg_s *rfcfg = (struct dev_rfpacket_rf_cfg_s *)rq->rf_cfg;
    rfcfg->frequency = 865056875;
    pv.new_rfcfg = rfcfg;
    pv.new_pkcfg = NULL;
    // Set slave buffer
    print_slave_rfcfg(rfcfg);
}

static void fsk_fdev_set_cfg(struct dev_rfpacket_rq_s *rq) {
    // Set new freq dev value
    dev_timer_value_t t;
    DEVICE_OP(pv.timer, get_value, &t, 0);
    struct dev_rfpacket_rf_cfg_fsk_s *rfcfg = &fsk_rfcfg;
    rfcfg->deviation = rand_64_range_r(&t, TEST_RFCFG_FSK_FDEV_MIN/10, TEST_RFCFG_FSK_FDEV_MAX/10) * 10; 
    //printk("fsk freq deviation value: %d\n", rfcfg->deviation);
    // Indicate that there is rfcfg change
    pv.new_rfcfg = &rfcfg->base;
    pv.new_pkcfg = NULL;
    // Set slave buffer
    print_slave_rfcfg(pv.new_rfcfg);
}

static void fsk_4gfsk_set_cfg(struct dev_rfpacket_rq_s *rq) {
    struct dev_rfpacket_rf_cfg_fsk_s *rfcfg = &fsk_rfcfg;
    rfcfg->symbols = 4; 
    //printk("4-gfsk test\n");
    // Indicate that there is rfcfg change
    pv.new_rfcfg = &rfcfg->base;
    pv.new_pkcfg = NULL;
    // Set slave buffer
    print_slave_rfcfg(pv.new_rfcfg);
}

static void ook_set_cfg(struct dev_rfpacket_rq_s *rq) {
    struct dev_rfpacket_rf_cfg_ask_s *rfcfg = &ask_rfcfg;
    rfcfg->symbols = 2; 
    //printk("ook test\n");
    // Indicate that there is rfcfg change
    pv.new_rfcfg = &rfcfg->base;
    pv.new_pkcfg = NULL;
    // Set slave buffer
    print_slave_rfcfg(pv.new_rfcfg);
}

// --- Test variables ---

// Test block array
static const test_rfcfg_test_block_t test_rfcfg_array[] = {
// #tests, #chan hops, func pointer
    {5, 0, def_set_cfg},
    {5, 0, drate_set_cfg},
    //{3, 3, chan_space_set_cfg},
    {1, 0, freq400_set_cfg},
    {1, 0, freq800_set_cfg},
    {1, 0, fix_freq_set_cfg},
    {1, 0, fsk_fdev_set_cfg},
    {1, 0, fsk_4gfsk_set_cfg},
    {1, 0, ook_set_cfg},
};

// External variables
const test_rfcfg_test_block_t *p_test_rfcfg_array = test_rfcfg_array;
const uint32_t test_rfcfg_array_size = ARRAY_SIZE(test_rfcfg_array);
#ifndef __TEST_RFCFG_CONFIG_H__
#define __TEST_RFCFG_CONFIG_H__

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

// --- Public Types ---
typedef void (*test_rfcfg_change_config)(struct dev_rfpacket_rq_s *rq);

typedef struct _test_rfcfg_test_block {
    uint32_t count; // Number of time the test will be executed
    uint32_t chan_hop_count; // Number of channel hop to do after a test
    test_rfcfg_change_config chg_cfg;
} test_rfcfg_test_block_t;

// --- Public constants ---
// --- Public variables ---
// Default configuration
struct dev_rfpacket_pk_cfg_s *def_pkcfg;
struct dev_rfpacket_rf_cfg_s *def_rfcfg;
// Test array
const test_rfcfg_test_block_t *p_test_rfcfg_array;
const uint32_t test_rfcfg_array_size;


// --- Public Functions ---
void test_rfcfg_config_init(struct device_timer_s *timer);
void test_rfcfg_get_slave_config(uint8_t **p_buf, uint8_t *p_buf_size);
void test_rfcfg_update_rq_config(struct dev_rfpacket_rq_s *rq);
void test_rfcfg_tx_end(void);
void test_rfcfg_test_end(uint32_t idx);

#endif
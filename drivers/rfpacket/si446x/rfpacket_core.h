#ifndef RFPACKET_CORE_H_
#define RFPACKET_CORE_H_

#include <device/class/rfpacket.h>

enum dev_rfpacket_state_s {
	DEV_RFPACKET_STATE_INITIALISING,
	DEV_RFPACKET_STATE_ENTER_SLEEP,
	DEV_RFPACKET_STATE_SLEEP,
	DEV_RFPACKET_STATE_AWAKING,
	DEV_RFPACKET_STATE_READY,
	DEV_RFPACKET_STATE_CONFIG,
	DEV_RFPACKET_STATE_CONFIG_RXC,
	DEV_RFPACKET_STATE_CONFIG_RXC_PENDING_STOP,
	DEV_RFPACKET_STATE_RX,
	DEV_RFPACKET_STATE_RXC,
	DEV_RFPACKET_STATE_STOPPING_RXC,
	DEV_RFPACKET_STATE_PAUSE_RXC,
	DEV_RFPACKET_STATE_TX,
	DEV_RFPACKET_STATE_TX_LBT,
	DEV_RFPACKET_STATE_TX_LBT_STOPPING_RXC,
};

enum dev_rfpacket_status_s {
	DEV_RFPACKET_STATUS_RX_DONE = 0,
	DEV_RFPACKET_STATUS_TX_DONE,
	DEV_RFPACKET_STATUS_RX_TIMEOUT,
	DEV_RFPACKET_STATUS_TX_TIMEOUT,
	DEV_RFPACKET_STATUS_CRC_ERR,
	DEV_RFPACKET_STATUS_JAMMING_ERR,
	DEV_RFPACKET_STATUS_OTHER_ERR,
	DEV_RFPACKET_STATUS_MISC,
};

// Cross-reference forward declaration
struct dev_rfpacket_ctx_s;

// Driver interface prototypes
typedef error_t (*dev_rfpacket_driver_check_config)(struct dev_rfpacket_ctx_s *gpv, struct dev_rfpacket_rq_s *rq);
typedef void (*dev_rfpacket_driver_rx)(struct dev_rfpacket_ctx_s *gpv, struct dev_rfpacket_rq_s *rq, bool_t isRetry);
typedef void (*dev_rfpacket_driver_tx)(struct dev_rfpacket_ctx_s *gpv, struct dev_rfpacket_rq_s *rq, bool_t isRetry);
typedef void (*dev_rfpacket_driver_cancel_rxc)(struct dev_rfpacket_ctx_s *gpv);
typedef bool_t (*dev_rfpacket_driver_wakeup)(struct dev_rfpacket_ctx_s *gpv);
typedef bool_t (*dev_rfpacket_driver_sleep)(struct dev_rfpacket_ctx_s *gpv);
typedef void (*dev_rfpacket_driver_idle)(struct dev_rfpacket_ctx_s *gpv);

struct dev_rfpacket_driver_interface_s {
	dev_rfpacket_driver_check_config check_config;
	dev_rfpacket_driver_rx rx;
	dev_rfpacket_driver_tx tx;
	dev_rfpacket_driver_cancel_rxc cancel_rxc;
	dev_rfpacket_driver_wakeup wakeup;
	dev_rfpacket_driver_sleep sleep;
	dev_rfpacket_driver_idle idle;
};

struct dev_rfpacket_ctx_s {
	// Time values
	dev_timer_value_t timeout;
	dev_timer_value_t deadline;
	dev_timer_value_t timestamp; // Filled by driver
	dev_timer_value_t rxc_timeout;
  	dev_timer_value_t lbt_timestamp; // Filled by driver
  	dev_timer_delay_t time_byte; // Filled by driver
	// Request for received packets
	struct dev_rfpacket_rx_s *rxrq;
	struct dev_rfpacket_rq_s *rq;
	// Current working size and buffer
	uint8_t *buffer;
	uint16_t size;
	// State
	enum dev_rfpacket_state_s state;
	// Status
	enum dev_rfpacket_status_s status; // Filled by driver
	// Device struct
	struct device_timer_s *timer; // Filled by driver
	// Queues
	dev_request_queue_root_t rx_cont_queue;
	dev_request_queue_root_t queue;
 	// Driver function interface
 	const struct dev_rfpacket_driver_interface_s *drv; // Filled by driver
	// Driver private data
 	void *pvdata; // Filled by driver
 	// Stats 
#ifdef CONFIG_DEVICE_RFPACKET_STATISTICS
  	struct dev_rfpacket_stats_s stats;
#endif
};

// Rfpacket event functions
bool_t dev_rfpacket_init_done(struct dev_rfpacket_ctx_s *pv);
void dev_rfpacket_config_notsup(struct dev_rfpacket_ctx_s *pv, struct dev_rfpacket_rq_s *rq);
bool_t dev_rfpacket_config_state_check(struct dev_rfpacket_ctx_s *pv, struct dev_rfpacket_rq_s *rq);
bool_t dev_rfpacket_can_rxtx(struct dev_rfpacket_ctx_s *pv, struct dev_rfpacket_rq_s *rq);
void dev_rfpacket_request(struct dev_rfpacket_ctx_s *pv, struct dev_rfpacket_rq_s *rq);
error_t dev_rfpacket_cancel(struct dev_rfpacket_ctx_s *pv, struct dev_rfpacket_rq_s *rq);
uintptr_t dev_rfpacket_alloc(struct dev_rfpacket_ctx_s *pv);
void dev_rfpacket_req_done(struct device_s *dev, struct dev_rfpacket_ctx_s *pv);
error_t dev_rfpacket_use(void *param, enum dev_use_op_e op, struct dev_rfpacket_ctx_s *pv);
void dev_rfpacket_init(struct dev_rfpacket_ctx_s *pv);
error_t dev_rfpacket_clean_check(struct dev_rfpacket_ctx_s *pv);
#endif
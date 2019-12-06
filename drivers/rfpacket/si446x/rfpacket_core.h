#ifndef RFPACKET_CORE_H_
#define RFPACKET_CORE_H_

#include <device/class/rfpacket.h>

#define RFPACKET_MAX_PACKET_SIZE    256

enum rfpacket_state_s {
	RFPACKET_STATE_INITIALISING,
	RFPACKET_STATE_ENTER_SLEEP,
	RFPACKET_STATE_SLEEP,
	RFPACKET_STATE_AWAKING,
	RFPACKET_STATE_READY,
	RFPACKET_STATE_CONFIG,
	RFPACKET_STATE_CONFIG_RXC,
	RFPACKET_STATE_CONFIG_RXC_PENDING_STOP,
	RFPACKET_STATE_RX,
	RFPACKET_STATE_RXC,
	RFPACKET_STATE_STOPPING_RXC,
	RFPACKET_STATE_PAUSE_RXC,
	RFPACKET_STATE_TX,
	RFPACKET_STATE_TX_LBT,
	RFPACKET_STATE_TX_LBT_STOPPING_RXC,
};

enum rfpacket_status_s {
	RFPACKET_STATUS_RX_DONE = 0,
	RFPACKET_STATUS_TX_DONE,
	RFPACKET_STATUS_RX_TIMEOUT,
	RFPACKET_STATUS_TX_TIMEOUT,
	RFPACKET_STATUS_CRC_ERR,
	RFPACKET_STATUS_JAMMING_ERR,
	RFPACKET_STATUS_OTHER_ERR,
	RFPACKET_STATUS_MISC,
};

// Cross-reference forward declaration
struct rfpacket_ctx_s;

// Driver interface prototypes
typedef error_t (*rfpacket_driver_check_config)(struct rfpacket_ctx_s *gpv, struct dev_rfpacket_rq_s *rq);
typedef void (*rfpacket_driver_rx)(struct rfpacket_ctx_s *gpv, struct dev_rfpacket_rq_s *rq, bool_t isRetry);
typedef void (*rfpacket_driver_tx)(struct rfpacket_ctx_s *gpv, struct dev_rfpacket_rq_s *rq, bool_t isRetry);
typedef void (*rfpacket_driver_cancel_rxc)(struct rfpacket_ctx_s *gpv);
typedef bool_t (*rfpacket_driver_wakeup)(struct rfpacket_ctx_s *gpv);
typedef bool_t (*rfpacket_driver_sleep)(struct rfpacket_ctx_s *gpv);
typedef void (*rfpacket_driver_idle)(struct rfpacket_ctx_s *gpv);

struct rfpacket_driver_interface_s {
	rfpacket_driver_check_config check_config;
	rfpacket_driver_rx rx;
	rfpacket_driver_tx tx;
	rfpacket_driver_cancel_rxc cancel_rxc;
	rfpacket_driver_wakeup wakeup;
	rfpacket_driver_sleep sleep;
	rfpacket_driver_idle idle;
};

struct rfpacket_ctx_s {
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
	enum rfpacket_state_s state;
	// Status
	enum rfpacket_status_s status; // Filled by driver
	// Device struct
	struct device_timer_s *timer; // Filled by driver
	// Queues
	dev_request_queue_root_t rx_cont_queue;
	dev_request_queue_root_t queue;
 	// Driver function interface
 	const struct rfpacket_driver_interface_s *drv; // Filled by driver
	// Driver private data
 	void *pvdata; // Filled by driver
 	// Stats 
#ifdef CONFIG_DEVICE_RFPACKET_STATISTICS
  	struct dev_rfpacket_stats_s stats;
#endif
};

// Rfpacket event functions
bool_t rfpacket_init_done(struct rfpacket_ctx_s *pv);
void rfpacket_config_notsup(struct rfpacket_ctx_s *pv, struct dev_rfpacket_rq_s *rq);
bool_t rfpacket_config_state_check(struct rfpacket_ctx_s *pv, struct dev_rfpacket_rq_s *rq);
bool_t rfpacket_can_rxtx(struct rfpacket_ctx_s *pv, struct dev_rfpacket_rq_s *rq);
void rfpacket_request(struct rfpacket_ctx_s *pv, struct dev_rfpacket_rq_s *rq);
error_t rfpacket_cancel(struct rfpacket_ctx_s *pv, struct dev_rfpacket_rq_s *rq);
uintptr_t rfpacket_alloc(struct rfpacket_ctx_s *pv);
void rfpacket_req_done(struct device_s *dev, struct rfpacket_ctx_s *pv);
error_t rfpacket_use(void *param, enum dev_use_op_e op, struct rfpacket_ctx_s *pv);
void rfpacket_init(struct rfpacket_ctx_s *pv);
error_t rfpacket_clean_check(struct rfpacket_ctx_s *pv);
#endif
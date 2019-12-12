#ifndef RFPACKET_CORE_H_
#define RFPACKET_CORE_H_

#include <device/class/rfpacket.h>

/** @This specifies the internal state used in the rfpacket fsm */
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

/** @This specifies the possible request status for the rfpacket fsm */
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

/** @This are the function prototypes in @ref dev_rfpacket_driver_interface_s */
typedef error_t (*dev_rfpacket_driver_check_config)(struct dev_rfpacket_ctx_s *gpv, struct dev_rfpacket_rq_s *rq);
typedef void (*dev_rfpacket_driver_rx)(struct dev_rfpacket_ctx_s *gpv, struct dev_rfpacket_rq_s *rq, bool_t isRetry);
typedef void (*dev_rfpacket_driver_tx)(struct dev_rfpacket_ctx_s *gpv, struct dev_rfpacket_rq_s *rq, bool_t isRetry);
typedef void (*dev_rfpacket_driver_cancel_rxc)(struct dev_rfpacket_ctx_s *gpv);
typedef bool_t (*dev_rfpacket_driver_wakeup)(struct dev_rfpacket_ctx_s *gpv);
typedef bool_t (*dev_rfpacket_driver_sleep)(struct dev_rfpacket_ctx_s *gpv);
typedef void (*dev_rfpacket_driver_idle)(struct dev_rfpacket_ctx_s *gpv);

/** @This structure contains the driver callback used in the rfpacket fsm, @csee dev_rfpacket_ctx_s */
struct dev_rfpacket_driver_interface_s {
	dev_rfpacket_driver_check_config check_config;
	dev_rfpacket_driver_rx rx;
	dev_rfpacket_driver_tx tx;
	dev_rfpacket_driver_cancel_rxc cancel_rxc;
	dev_rfpacket_driver_wakeup wakeup;
	dev_rfpacket_driver_sleep sleep;
	dev_rfpacket_driver_idle idle;
};

/** @This struct contains all the data to run a rfpacket fsm instance */
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

/** @This function is called by a rfpacket driver to check if the rfpacket
	fsm is not in an init state. Returns @tt TRUE if it isn't in init. */
config_depend(CONFIG_DEVICE_RFPACKET)
bool_t dev_rfpacket_init_done(struct dev_rfpacket_ctx_s *pv);

/** @This function is called by a rfpacket driver to check if the rfpacket
	fsm is in a configuration state. Returns @tt TRUE if it is in configuration. */
config_depend(CONFIG_DEVICE_RFPACKET)
bool_t dev_rfpacket_config_state_check(struct dev_rfpacket_ctx_s *pv, struct dev_rfpacket_rq_s *rq);

/** @This function is called by a rfpacket driver to check if it can clean the
	driver instance. Returns @tt 0 if allowed, @tt -EBUSY otherwise*/
config_depend(CONFIG_DEVICE_RFPACKET)
error_t dev_rfpacket_clean_check(struct dev_rfpacket_ctx_s *pv);

/** @This function is called by a rfpacket driver to check if it can rx during a tx.
	This will only happen if the driver receives a @tt DEV_RFPACKET_RQ_TX_FAIR request
	while it has an active @tt DEV_RFPACKET_RQ_RX_CONT or @tt DEV_RFPACKET_RQ_RX_TIMEOUT
	request. Returns @tt TRUE only if the tx and rx configuration are the same structure. */
config_depend(CONFIG_DEVICE_RFPACKET)
bool_t dev_rfpacket_can_rxtx(struct dev_rfpacket_ctx_s *pv, struct dev_rfpacket_rq_s *rq);

/** @This function is called by a rfpacket driver to indicate that it encountered
	an unsupported configuration. */
config_depend(CONFIG_DEVICE_RFPACKET)
void dev_rfpacket_config_notsup(struct dev_rfpacket_ctx_s *pv, struct dev_rfpacket_rq_s *rq);

/** @This function is called by a rfpacket driver to process an incoming request
	through the rfpacket fsm. */
config_depend(CONFIG_DEVICE_RFPACKET)
void dev_rfpacket_request(struct dev_rfpacket_ctx_s *pv, struct dev_rfpacket_rq_s *rq);

/** @This function is called by a rfpacket driver to cancel a request through 
	the rfpacket fsm. */
config_depend(CONFIG_DEVICE_RFPACKET)
error_t dev_rfpacket_cancel(struct dev_rfpacket_ctx_s *pv, struct dev_rfpacket_rq_s *rq);

/** @This function is called by a rfpacket driver to allocate a rx buffer through
	the rfpacket fsm. */
config_depend(CONFIG_DEVICE_RFPACKET)
uintptr_t dev_rfpacket_alloc(struct dev_rfpacket_ctx_s *pv);

/** @This function is called by a rfpacket driver to process a request end through
	the rfpacket fsm. */
config_depend(CONFIG_DEVICE_RFPACKET)
void dev_rfpacket_req_done(struct device_s *dev, struct dev_rfpacket_ctx_s *pv);

/** @This function is called by a rfpacket driver to process a @ref DEV_USE call through
	the rfpacket fsm. */
config_depend(CONFIG_DEVICE_RFPACKET)
error_t dev_rfpacket_use(void *param, enum dev_use_op_e op, struct dev_rfpacket_ctx_s *pv);

/** @This function is called by a rfpacket driver to initialize	the rfpacket fsm. */
config_depend(CONFIG_DEVICE_RFPACKET)
void dev_rfpacket_init(struct dev_rfpacket_ctx_s *pv);

#endif
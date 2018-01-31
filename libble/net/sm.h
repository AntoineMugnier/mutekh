#include <mutek/buffer_pool.h>
#include <mutek/bytecode.h>
#include <device/class/crypto.h>

#include <net/layer.h>
#include <net/task.h>
#include <net/scheduler.h>

#include <ble/net/sm.h>
#include <ble/net/generic.h>
#include <ble/crypto.h>
#include <ble/protocol/sm.h>
#include <ble/protocol/l2cap.h>

#include <ble/peer.h>
#include <device/class/crypto.h>

struct ble_sm_handler_s;
struct dev_rng_s;

enum sm_state_e
{
  SM_IDLE,
  SM_WAIT_USER,
  SM_WAIT_REQUEST,
  SM_WAIT_RESPONSE,
  SM_VM_IDLE,
  SM_VM_WAIT_RAND,
  SM_VM_WAIT_CONF,
  SM_VM_WAIT_STK,
  SM_VM_WAIT_TX,
  SM_VM_WAIT_RX,
  SM_VM_WAIT_ENCRYPTION,
  SM_FAIL,
};

/**
 BLE Security manager layer.

 Handles device pairing and bonding.
 */
struct ble_sm_s
{
  struct net_layer_s layer;

  struct dev_rng_s *rng;

  struct dev_crypto_rq_s aes_rq;
  struct device_crypto_s aes;
  struct dev_crypto_context_s aes_ctx;
  uint8_t aes_io[32];

  struct bc_context_s vm;
  enum sm_state_e state;
  
  struct ble_peer_s *peer;

  bool_t security_requested;

  struct ble_addr_s local_addr;

  uint8_t mrand[16];
  uint8_t srand[16];
  uint8_t pconf[16];
  uint8_t mconf[16];
  uint8_t sconf[16];
  uint8_t stk[16];
  uint8_t preq[7];
  uint8_t pres[7];
  uint8_t tk[16];

  bool_t has_oob;
  uint8_t io_cap;

  struct kroutine_sequence_s seq;
  struct buffer_s *pkt;
};

STRUCT_COMPOSE(ble_sm_s, layer);

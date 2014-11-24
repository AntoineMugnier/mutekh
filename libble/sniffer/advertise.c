#include <hexo/types.h>
#include <mutek/printk.h>
#include <mutek/buffer_pool.h>

#include <ble/protocol/data.h>
#include <ble/sniffer/radio.h>

#include <ble/protocol/gap.h>
#include <ble/protocol/address.h>
#include <ble/protocol/advertise.h>
#include <ble/protocol/company.h>
#include <ble/sniffer/advertise.h>
#include <ble/protocol/radio.h>

struct ble_advertise_observer
{
    struct ble_sniffer *radio;
    void *pvdata;
    const struct ble_advertise_handler *handler;
    struct ble_sniffer_request rq[3];
};

static KROUTINE_EXEC(adv_done)
{
    struct ble_sniffer_request *rq = KROUTINE_CONTAINER(kr, *rq, base.kr);
    uint8_t idx = rq->white_iv - 37;
    struct ble_advertise_observer *obs = KROUTINE_CONTAINER(rq, *obs, rq[idx]);
    const struct ble_advertise_handler *handler = obs->handler;
    struct buffer_s *packet;

#if 0
    printk("%lld Adv req chan %d done: %d\n", rq->started_on, rq->white_iv, rq->status);
#endif

    while ((packet = buffer_queue_pop(&rq->queue))) {
      handler->adv(obs->pvdata, rq->white_iv, packet);
      buffer_refdec(packet);
    }

    if (idx == 2) {
        handler->done(obs->pvdata);
        free(obs);
    }
}

static void advertise_observer_setup(struct ble_advertise_observer *obs)
{
    for (uint8_t i = 0; i < 3; ++i) {
        struct ble_sniffer_request *rq = &obs->rq[i];

        kroutine_init(&rq->base.kr, adv_done, KROUTINE_INTERRUPTIBLE);
        rq->start = 0;
        rq->initial_timeout = i == 0 ? 0 : BLE_T_ADV_UNIT;
        rq->window = BUFFER_TIME(37) + BLE_T_IFS + BUFFER_TIME(6) + BLE_T_IFS + BUFFER_TIME(0);
        rq->access = BLE_ADVERTISE_AA;
        rq->crc_init = BLE_ADVERTISE_CRCINIT;
        rq->frequency = ble_channel_freq_mhz(37 + i);
        rq->white_iv = 37 + i;
        rq->rx_packet_max = 3;
        rq->chain_mode = 0;//i == 0 ? 0 : (BLE_SNIFFER_CHAIN_DONE | BLE_SNIFFER_CHAIN_IFS_TIMEOUT | BLE_SNIFFER_CHAIN_END_TIMEOUT);
        rq->filter = 0;//BLE_SNIFFER_FILTER_ADVCONSTANT | BLE_SNIFFER_FILTER_RSSI;
        rq->rssi_min = 70;
        rq->t_ifs_max = 150;
    }

    ble_sniffer_request(obs->radio, &obs->rq[0], &obs->rq[1], &obs->rq[2], NULL);
}

void ble_advertise_oberve(
    void *pvdata,
    const struct ble_advertise_handler *handler,
    struct ble_sniffer *radio)
{
    struct ble_advertise_observer *obs = malloc(sizeof(*obs));

    obs->radio = radio;
    obs->pvdata = pvdata;
    obs->handler = handler;

    advertise_observer_setup(obs);
}

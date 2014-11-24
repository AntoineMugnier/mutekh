#ifndef BLE_SNIFFER_ADVERTISE_H_
#define BLE_SNIFFER_ADVERTISE_H_

struct buffer;
struct ble_sniffer;
struct ble_address;

struct ble_advertise_handler
{
    void (*adv)(void *pvdata, uint8_t channel, struct buffer_s *packet);
    void (*done)(void *pvdata);
};

void ble_advertise_oberve(
    void *pvdata,
    const struct ble_advertise_handler *handler,
    struct ble_sniffer *radio);

#endif

#include <mutek/printk.h>

#include <arch/nrf5x/ids.h>
#include <arch/nrf5x/gpio.h>

#include <hexo/power.h>
#include <hexo/interrupt.h>

#include "poweroff.h"

void poweroff(void)
{
    printk("poweroff\n");

    cpu_interrupt_disable();

    for (uint8_t i = 0; i < 31; ++i)
        nrf_reg_set(NRF5X_GPIO_ADDR, NRF_GPIO_PIN_CNF(i), 0
                    | NRF_GPIO_PIN_CNF_DIR_INPUT
                    | NRF_GPIO_PIN_CNF_INPUT_DISCONNECT
                    | NRF_GPIO_PIN_CNF_PULL_DISABLED
                    | NRF_GPIO_PIN_CNF_SENSE_DISABLED);

    nrf_reg_set(NRF5X_GPIO_ADDR, NRF_GPIO_PIN_CNF(16), 0
                | NRF_GPIO_PIN_CNF_DIR_INPUT
                | NRF_GPIO_PIN_CNF_INPUT_CONNECT
                | NRF_GPIO_PIN_CNF_PULL_DISABLED
                | NRF_GPIO_PIN_CNF_SENSE_DISABLED);

    nrf_reg_set(NRF5X_GPIO_ADDR, NRF_GPIO_PIN_CNF(25), 0
                | NRF_GPIO_PIN_CNF_DIR_INPUT
                | NRF_GPIO_PIN_CNF_INPUT_CONNECT
                | NRF_GPIO_PIN_CNF_PULL_DISABLED
                | NRF_GPIO_PIN_CNF_SENSE_DISABLED);

    printk("gpio done\n");

    for (uint16_t i = 0; i < 10; ++i)
        asm volatile("");

    // Sense button press
    nrf_reg_set(NRF5X_GPIO_ADDR, NRF_GPIO_PIN_CNF(16), 0
                | NRF_GPIO_PIN_CNF_DIR_INPUT
                | NRF_GPIO_PIN_CNF_INPUT_CONNECT
                | NRF_GPIO_PIN_CNF_PULL_UP
                | NRF_GPIO_PIN_CNF_SENSE_LOW);

    nrf_reg_set(NRF5X_GPIO_ADDR, NRF_GPIO_PIN_CNF(25), 0
                | NRF_GPIO_PIN_CNF_DIR_INPUT
                | NRF_GPIO_PIN_CNF_INPUT_CONNECT
                | NRF_GPIO_PIN_CNF_PULL_UP
                | NRF_GPIO_PIN_CNF_SENSE_LOW);

    // Ensure leds are disabled
    nrf_reg_set(NRF5X_GPIO_ADDR, NRF_GPIO_PIN_CNF(17), 0
                | NRF_GPIO_PIN_CNF_DIR_INPUT
                | NRF_GPIO_PIN_CNF_INPUT_DISCONNECT
                | NRF_GPIO_PIN_CNF_PULL_UP
                | NRF_GPIO_PIN_CNF_SENSE_DISABLED);

    nrf_reg_set(NRF5X_GPIO_ADDR, NRF_GPIO_PIN_CNF(18), 0
                | NRF_GPIO_PIN_CNF_DIR_INPUT
                | NRF_GPIO_PIN_CNF_INPUT_DISCONNECT
                | NRF_GPIO_PIN_CNF_PULL_UP
                | NRF_GPIO_PIN_CNF_SENSE_DISABLED);

    power_shutdown();
}

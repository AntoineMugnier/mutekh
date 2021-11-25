/*
    This file is part of MutekH.

    MutekH is free software; you can redistribute it and/or modify it
    under the terms of the GNU Lesser General Public License as
    published by the Free Software Foundation; version 2.1 of the
    License.

    MutekH is distributed in the hope that it will be useful, but
    WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this program.  If not, see
    <http://www.gnu.org/licenses/>.

    Copyright (c) Nicolas Pouillon <nipo@ssji.net>, 2016
*/

#include <hexo/types.h>
#include <device/irq.h>
#include <device/class/i2c.h>
#include <device/request.h>

enum sx1509_state_e {
  STATE_INIT,
  STATE_IDLE,
  STATE_READING,
  STATE_DISABLING,
  STATE_DISABLED,
  STATE_IRQ_PENDING,
};

struct sx1509_kbd_context_s
{
  struct device_i2c_ctrl_s i2c;
  struct device_timer_s *timer;
  struct dev_irq_src_s irq_ep;
  struct dev_i2c_ctrl_bytecode_rq_s i2c_rq;
  struct dev_timer_rq_s timer_rq;

  enum sx1509_state_e state;

  dev_request_queue_root_t queue;

  uint32_t row_mask, col_mask;
  uint16_t last_rowcol;
  uint8_t rows, cols;

  uint64_t value_cur;
  uint64_t value_next;
  uint64_t value_last;
};

#define SX1509_INPUT_DISABLE     0x00 // Input buffer disable
#define SX1509_LONG_SLEW         0x02 // Output buffer long slew
#define SX1509_LOW_DRIVE         0x04 // Output buffer low drive
#define SX1509_PULL_UP           0x06 // Pull-up
#define SX1509_PULL_DOWN         0x08 // Pull-down
#define SX1509_OPEN_DRAIN        0x0A // Open drain
#define SX1509_INVERT            0x0C // Polarity inversion
#define SX1509_DIR_INPUT         0x0E // Pin Direction
#define SX1509_DATA              0x10 // Data
#define SX1509_IRQ_DISABLE       0x12 // Interrupt mask, per line
#define SX1509_SENSE_            0x14 // Sense for I/O
#define SX1509_INTERRUPT_SOURCE  0x18 // Interrupt source
#define SX1509_EVENT_STATUS      0x1A // Event status
#define SX1509_LEVEL_SHIFTER_1   0x1C // Level shifter
#define SX1509_LEVEL_SHIFTER_2   0x1D // Level shifter
#define SX1509_CLOCK             0x1E // Clock management
#define SX1509_MISC              0x1F // Miscellaneous device settings
#define SX1509_LED_DRIVER_ENABLE 0x20 // LED driver enable
#define SX1509_DEBOUNCE_CONFIG   0x22 // Debounce configuration
#define SX1509_DEBOUNCE_ENABLE   0x23 // Debounce enable
#define SX1509_KEY_CONFIG        0x25 // Key scan configuration
#define SX1509_KEY_DATA          0x27 // Key value
#define SX1509_HIGH_INPUT        0x69 // High input enable
#define SX1509_RESET             0x7D // Software reset
#define SX1509_TEST_1            0x7E // Test
#define SX1509_TEST_2            0x7F // Test

#define SX1509_CLOCK_FOSC_MASK     0x60
#define SX1509_CLOCK_FOSC_OFF      0x00
#define SX1509_CLOCK_FOSC_OSCIN    0x20
#define SX1509_CLOCK_FOSC_2MHZ     0x40
#define SX1509_CLOCK_OSCIO_MASK    0x10
#define SX1509_CLOCK_OSCIO_OUT     0x10
#define SX1509_CLOCK_OSCIO_IN      0x00
#define SX1509_CLOCK_OSCOUT_DIV(n) ((n) & 0xf)

#define SX1509_MISC_LEDB_FADE_MASK      0x80
#define SX1509_MISC_LEDB_FADE_LINEAR    0x00
#define SX1509_MISC_LEDB_FADE_LOG       0x80
#define SX1509_MISC_LED_DIV(n)          (((n) & 0x7) << 4)
#define SX1509_MISC_LEDA_FADE_MASK      0x08
#define SX1509_MISC_LEDA_FADE_LINEAR    0x00
#define SX1509_MISC_LEDA_FADE_LOG       0x08
#define SX1509_MISC_NRESET_MASK         0x04
#define SX1509_MISC_NRESET_POR          0x00
#define SX1509_MISC_NRESET_BLINK        0x04
#define SX1509_MISC_I2C_ADDR_MASK       0x02
#define SX1509_MISC_I2C_ADDR_AUTOINC    0x00
#define SX1509_MISC_I2C_ADDR_FIXED      0x02
#define SX1509_MISC_NINT_CLEAR_MASK     0x01
#define SX1509_MISC_NINT_CLEAR_AUTO     0x00
#define SX1509_MISC_NINT_CLEAR_EXPLICIT 0x01

#define SX1509_SENSE(x)         (SX1509_SENSE_ + 3 - (((x) >> 2) & 0x3))
#define SX1509_SENSE_NONE(x)    (0 << (((x) & 3) * 2))
#define SX1509_SENSE_RISING(x)  (1 << (((x) & 3) * 2))
#define SX1509_SENSE_FALLING(x) (2 << (((x) & 3) * 2))
#define SX1509_SENSE_BOTH(x)    (3 << (((x) & 3) * 2))

#define SX1509_DEBOUNCE_CONFIG_TIME(x) ((x) & 0x7)

#define SX1509_KEY_CONFIG1_AUTO_SLEEP(x) (((x) & 0x7) << 4)
#define SX1509_KEY_CONFIG1_ROW_TIME(x)   ((x) & 0x7)
#define SX1509_KEY_CONFIG2_ROW_COUNT(x)  (((x) & 0x7) << 3)
#define SX1509_KEY_CONFIG2_COL_COUNT(x)  ((x) & 0x7)

#define SX1509_RESET_MAGIC1 0x12
#define SX1509_RESET_MAGIC2 0x34


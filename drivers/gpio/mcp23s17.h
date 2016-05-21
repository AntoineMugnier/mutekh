#ifndef _MCP23S17_H_
#define _MCP23S17_H_

#include <device/class/spi.h>

# define MCP23S17_PIN_NB      16

# define MCP23S17_PIN_0         0
# define MCP23S17_PIN_1         1
# define MCP23S17_PIN_2         2
# define MCP23S17_PIN_3         3
# define MCP23S17_PIN_4         4
# define MCP23S17_PIN_5         5
# define MCP23S17_PIN_6         6
# define MCP23S17_PIN_7         7
# define MCP23S17_PIN_8         8
# define MCP23S17_PIN_9         9
# define MCP23S17_PIN_10       10
# define MCP23S17_PIN_11       11
# define MCP23S17_PIN_12       12
# define MCP23S17_PIN_13       13
# define MCP23S17_PIN_14       14
# define MCP23S17_PIN_15       15

/* MCP23S17 registers */
# define MCP23S17_REG_IODIR   0x00
# define MCP23S17_REG_IPOL    0x02
# define MCP23S17_REG_GPINTEN 0x04
# define MCP23S17_REG_DEFVAL  0x06
# define MCP23S17_REG_INTCON  0x08
# define MCP23S17_REG_IOCON   0x0A
# define MCP23S17_REG_GPPU    0x0C
# define MCP23S17_REG_INTF    0x0E
# define MCP23S17_REG_INTCAP  0x10
# define MCP23S17_REG_GPIO    0x12
# define MCP23S17_REG_OLAT    0x14


/*
bit 7 BANK: Controls how the registers are addressed
1 = The registers associated with each port are separated into different banks
0 = The registers are in the same bank (addresses are sequential)
*/
# define MCP23S17_REG_IOCON_BANK    0x80
/*
bit 6 MIRROR: INT Pins Mirror bit
1 = The INT pins are internally connected
0 = The INT pins are not connected.
*/
# define MCP23S17_REG_IOCON_MIRROR  0x40
/*
bit 5 SEQOP: Sequential Operation mode bit.
1 = Sequential operation disabled, address pointer does not increment.
0 = Sequential operation enabled, address pointer increments.
*/
# define MCP23S17_REG_IOCON_SEQOP   0x20
/*
bit 4 DISSLW: Slew Rate control bit for SDA output.
1 = Slew rate disabled.
0 = Slew rate enabled.
*/
# define MCP23S17_REG_IOCON_DISSLW  0x10
/*
bit 3 HAEN: Hardware Address Enable bit (MCP23S17 only).
Address pins are always enabled on MCP23017.
1 = Enables the MCP23S17 address pins.
0 = Disables the MCP23S17 address pins.
*/
# define MCP23S17_REG_IOCON_HAEN    0x08
/*
bit 2 ODR: This bit configures the INT pin as an open-drain output.
1 = Open-drain output (overrides the INTPOL bit).
0 = Active driver output (INTPOL bit sets the polarity).
*/
# define MCP23S17_REG_IOCON_ODR     0x04
/*
bit 1 INTPOL: This bit sets the polarity of the INT output pin.
1 = Active-high.
0 = Active-low.
*/
# define MCP23S17_REG_IOCON_INTPOL  0x02
/*
bit 0 Unimplemented: Read as ‘0’
*/

/* VM registers */
# define BC_PV          0
# define BC_SPI_BUFF    1
# define BC_SPI_TX_0    2
# define BC_SPI_TX_1    3
# define BC_SPI_TX_2    4
# define BC_SPI_TX_3    5
# define BC_SPI_RX_0    6
# define BC_SPI_RX_1    7
# define BC_SPI_RX_2    8
# define BC_SPI_RX_3    9
# define BC_REG_0      10
# define BC_REG_1      11
# define BC_REG_2      12
# define BC_REG_3      13
# define BC_REG_4      14
# define BC_REG_5      15


extern const struct bc_descriptor_s mcp23s17_bytecode;

extern void mcp23s17_bc_sync_cache(void);
extern void mcp23s17_bc_write_output_mode(void);
extern void mcp23s17_bc_write_input_mode(void);
extern void mcp23s17_bc_write_output(void);
extern void mcp23s17_bc_read_input(void);
extern void mcp23s17_bc_read_irq(void);
extern void mcp23s17_bc_setup_irq(void);

# define MCP23S17_INIT_OP           0x1
# define MCP23S17_REQUEST_OP        0x2
#ifdef CONFIG_DRIVER_MCP23S17_ICU
#  define MCP23S17_IRQ_PROCESS_OP   0x4
#  define MCP23S17_IRQ_SETUP_OP     0x8
#endif

DRIVER_PV(struct mcp23s17_private_s
{
  /* SPI */
  struct device_spi_ctrl_s  spi;
  struct dev_spi_ctrl_bytecode_rq_s  spi_req;

  /* Queue */
  dev_request_queue_root_t  rq_pending;

#ifdef CONFIG_DRIVER_MCP23S17_ICU
  struct dev_irq_sink_s     sinks_ep[CONFIG_DRIVER_MCP23S17_IRQ_COUNT];
  struct dev_irq_src_s      src_ep;
  uint8_t                   sinks_map[MCP23S17_PIN_NB];

  /* Interrupts setup */
  uint16_t    int_update_mask;  /* Interrupts to update mask */
  uint16_t    int_update_en;    /* Interrupts to update enable/disable */
#endif

  /* Registers cache */
  uint16_t    iodir_cache;    /* Direction register cache */
  uint16_t    gppu_cache;     /* Pull-up register cache */
  uint16_t    olat_cache;     /* Output register cache */
  uint16_t    gpio_cache;     /* Input register cache */
#ifdef CONFIG_DRIVER_MCP23S17_ICU
  uint16_t    gpinten_cache;  /* Interrupts enable cache */
  uint16_t    intcon_cache;   /* Interrupts config cache */
  uint16_t    defval_cache;   /* Interrupts default value cache */
  uint16_t    intf_cache;     /* Interrupts flag cache */
  uint16_t    edge_sensitive; /* Any edge sensitivity pin mask */
#endif
  uint8_t     icon_cache;     /* Config register cache */

  uint8_t     device_opcode;  /* Device address */

  uint8_t     pending_op;     /* Pending operations mask */
  uint8_t     current_op;     /* Current operation mask */
});

#endif /* _MCP23S17_H_ */

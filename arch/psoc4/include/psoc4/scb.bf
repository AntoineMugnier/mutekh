name: scb
longname: Cypress PSoC4 Serial Communication Block
spec_title: PRoC BLE Architecture TRM, Document No. 001-93191 Rev A
spec_version: Dec 26, 2014

bound: 32

%ctrl: 32
	longname: Generic control register
	address: 0x000

	%%ovs: 0-3
        longname: Oversampling rate.
        doc: Oversampling lasts (AVS + 1) SCB clock.

	%%ec_am_mode: 8-8
        longname: Address matching clock source (SPI and I2C)
        0: Internal
        1: External

    %%ec_op_mode: 9-9
        longname: Clock source (SPI and I2C)
        0: Internal
        1: External

    %%ez_mode: 10-10
        longname: EZ-mode (SPI and I2C)

    %%byte_mode: 11-11
        longname: FIFO Byte-mode
        0: 8-bit elements
        1: 16-bit elements

    %%addr_accept: 16-16
        doc: Determines whether a received matching address is accepted in the RX FIFO

    %%block: 17-17
        doc: Order EZ accesses

    %%mode: 24-25
        doc: Mode of operation
        0: I2C
        1: SPI
        2: UART

    %%enabled: 31-31
        
%status: 32
	longname: Generic status register
	address: 0x004
    direction: r

	%%ec_busy: 0-0
		doc: Inidicates whether the externally clocked logic is potentially accessing the EZ memory

%spi_ctrl: 32
	longname: SPI Control register
	address: 0x020

	%%continuous: 0-0
		doc: Continuous or separated SPI data transfers

	%%select_precede: 1-1
		doc: In Ti mode, whether strobe precedes or is concurrent with first byte

	%%cpha: 2-2
		doc: Clock phase

	%%cpol: 3-3
        doc: Clock polarity

	%%last_miso_sample: 4-4
		doc: Whether to invert MISO sampling wrt default of current mode.

	%%sclk_continuous: 5-5
		doc: Whether to generate clock cycles even if no slave is selected.

	%%ssel_polarity0: 8-8
		doc: Polarity of SSEL0

	%%ssel_polarity1: 9-9
		doc: Polarity of SSEL1

	%%ssel_polarity2: 10-10
		doc: Polarity of SSEL2

	%%ssel_polarity3: 11-11
		doc: Polarity of SSEL3

	%%loopback: 16-16
		doc: Whether to loopback MISO and MOSI lines.

	%%mode: 24-25
		longname: Mode of operation
        0: MOTOROLA
        1: TI
        2: NS

	%%slave_select: 26-27
		doc: For master mode, select relevant SSEL line, SCB block must be disabled while changes are made to this register.

	%%master_mode: 31-31
		doc: Whether we are SPI master
        
%spi_status: 32
	longname: Generic status register
	address: 0x024
    direction: r

	%%bus_busy: 0-0
		doc: SPI bus is busy

	%%ec_busy: 1-1
		doc: Inidicates whether the externally clocked logic is potentially accessing the EZ memory

    %%curr_ez_busy: 8-15
        doc: SPI current EZ address

    %%base_ez_address: 16-23
        doc: SPI base EZ address

%uart_ctrl: 32
	longname: UART Control register
	address: 0x040

	%%loopback: 16-16
		doc: Whether to loopback RX and TX lines.

	%%mode: 24-25
		longname: Mode of operation
        0: STD
        1: SMARTCARD
        2: IRDA

%uart_tx_ctrl: 32
	longname: UART Transmit Control register
	address: 0x044

	%%stop_bits: 0-2
		doc: Count of stop bits - 1

	%%parity: 4-4
		doc: Parity mode
        0: even
        1: odd

	%%parity_enabled: 5-5
		doc: Whether to include a parity bit

	%%retry_on_nack: 8-8
		doc: In smartcard mode, retry byte if a NACK is received

%uart_rx_ctrl: 32
	longname: UART Receier Control register
	address: 0x048

	%%stop_bits: 0-2
		doc: Count of stop bits - 1

	%%parity: 4-4
		doc: Parity mode
        0: even
        1: odd

	%%parity_enabled: 5-5
		doc: Whether to expect a parity bit

	%%polarity: 6-6
		doc: Whether to invert line

	%%drop_on_parity_error: 8-8
		doc: Whether to discard incoming data when parity check fails

	%%drop_on_frame_error: 9-9
		doc: Whether to discard incoming data when handshaking fails

	%%mp_mode: 10-10
		doc: Multiprocessor mode, should be enabled with a 9-bit frame size

    %%lin_mode: 12-12
        doc: perform break detection and baud rate detection on the incoming data

    %%skip_start: 13-13
        doc: Receiver skips start bit detection for the first received data frame

    %%break_width: 16-19
        doc: Count of clock period - 1 data line must be 0 to deteck a Break condition

%uart_rx_status: 32
	longname: UART receive status register
	address: 0x04c
    direction: r

	%%br_counter: 0-11
		doc: UART baudrate detection counter. Bit clock is br_counter / 8.

%uart_flow_ctrl: 32
	longname: UART Flow control register
	address: 0x050

	%%trigger_level: 0-3
		doc: Assert RTS when FIFO contains less entries than this field

	%%rts_polarity: 16-16
		0: active low
        1: active high

	%%cts_polarity: 24-24
		0: active low
        1: active high

	%%cts_enabled: 25-25
        doc: Whether to stop sending when CTS line is not active

# TODO : i2c regs

%tx_ctrl: 32
    longname: Transmitter control register
    address: 0x200

    %%data_width: 0-3
        doc: Data frame bit count - 1. Should be set to 1 for I2C

    %%msb_first: 8-8
        doc: Whether to send MSB first. Should be set to 1 for I2C

%tx_fifo_ctrl: 32
    longname: Transmitter FIFO control register
    address: 0x204

    %%trigger_level: 0-3
        doc: Generate a "transmitter trigger" event when transmitter FIFO fillness count is less than this register

    %%clear: 16-16
        doc: Whether to clear the TX FIFO. This register must be reset to 0 from software.

    %%freeze: 17-17
        doc: Hardware reads from FIFO dont update read pointer

%tx_fifo_status: 32
    longname: Transmitter FIFO status register
    address: 0x208
    direction: r

    %%used: 0-4
        doc: Count of entries in the TX FIFO

    %%sr_valid: 15-15
        doc: Whether shift register currently holds a valid data frame

    %%rd_ptr: 16-19
        doc: FIFO read pointer

    %%wr_ptr: 24-27
        doc: FIFO write pointer

%tx_fifo_wr: 32
    longname: Transmitter FIFO write register
    address: 0x240

    %%data: 0-15
        direction: w

%rx_ctrl: 32
    longname: Receiver control register
    address: 0x300

    %%data_width: 0-3
        doc: Data frame bit count - 1. Should be set to 1 for I2C

    %%msb_first: 8-8
        doc: Whether to send MSB first. Should be set to 1 for I2C

    %%median: 9-9
        doc: Whether to sample input bits 3 times and do a majority vote

%rx_fifo_ctrl: 32
    longname: Receiver FIFO control register
    address: 0x304

    %%trigger_level: 0-3
        doc: Generate a "receiver trigger" event when receiver FIFO fillness count is more than this register

    %%clear: 16-16
        doc: Whether to clear the RX FIFO. This register must be reset to 0 from software.

    %%freeze: 17-17
        doc: Hardware writes to FIFO dont update read pointer

%rx_fifo_status: 32
    longname: Receiver FIFO status register
    address: 0x308
    direction: r

    %%used: 0-4
        doc: Count of entries in the RX FIFO

    %%sr_valid: 15-15
        doc: Whether shift register currently holds a valid data frame

    %%rd_ptr: 16-19
        doc: FIFO read pointer

    %%wr_ptr: 24-27
        doc: FIFO write pointer

%rx_match: 32
    longname: Slave address and mask register
    address: 0x310

    %%addr_: 0-7
        doc: Slave device address

    %%mask_: 16-23
        doc: Mask for device address

%rx_fifo_rd: 32
    longname: Receiver FIFO read register
    address: 0x340
    direction: r

    %%data: 0-15

%rx_fifo_rd_silent: 32
    longname: Receiver FIFO read register without FIFO pop
    address: 0x344
    doc: A read operation to this register retrieves the first item of the RX fifo without updating FIFO pointers
    direction: r

    %%data: 0-15

# TODO: EZ registers

%intr_cause: 32
    longname: Active clocked interrupt signal register
    address: 0xe00
    direction: r

    %%m: 0-0
        longname: Master interrupt

    %%s: 1-1
        longname: Slave interrupt

    %%tx: 2-2
        longname: Transmitter interrupt

    %%rx: 3-3
        longname: Receiver interrupt

    %%i2c_ec: 4-4
        longname: Externally clock I2C interrupt

    %%spi_ec: 5-5
        longname: Externally clock SPI interrupt

# TODO: I2C cause registers
# TODO: SPI cause registers
# TODO: SPI cause registers
# TODO: Master cause registers
# TODO: Slave cause registers

%intr_tx: 32
    address: 0xf80
    longname: Transmitter interrupt request register

    %%trigger: 0-0
        doc: Less entries in the TX FIFO than the value specified by TX_FIFO_CTRL

    %%not_full: 1-1
        doc: TX FIFO is not full

    %%empty: 4-4
        doc: TX FIFO empty

    %%overflow: 5-5
        doc: Attempt to write to a full TX FIFO

    %%underflow: 6-6
        doc: Attempt to read from an empty TX FIFO

    %%blocked: 7-7
        doc: SW cannot get access to the EZ memory

    %%uart_nack: 8-8
        doc: UART transmitter received a negative acknowledgement in SmartCard mode

    %%uart_done: 9-9
        doc: UART transmitter done event

    %%uart_arb_lost: 10-10
        doc: UART lost arbitration

%intr_tx_set: 32
    address: 0xf84
    longname: Transmitter interrupt setter register
    direction: w

    %%trigger: 0-0
        doc: Less entries in the TX FIFO than the value specified by TX_FIFO_CTRL

    %%not_full: 1-1
        doc: TX FIFO is not full

    %%empty: 4-4
        doc: TX FIFO empty

    %%overflow: 5-5
        doc: Attempt to write to a full TX FIFO

    %%underflow: 6-6
        doc: Attempt to read from an empty TX FIFO

    %%blocked: 7-7
        doc: SW cannot get access to the EZ memory

    %%uart_nack: 8-8
        doc: UART transmitter received a negative acknowledgement in SmartCard mode

    %%uart_done: 9-9
        doc: UART transmitter done event

    %%uart_arb_lost: 10-10
        doc: UART lost arbitration

%intr_tx_mask: 32
    address: 0xf88
    longname: Transmitter interrupt mask register

    %%trigger: 0-0
        doc: Less entries in the TX FIFO than the value specified by TX_FIFO_CTRL

    %%not_full: 1-1
        doc: TX FIFO is not full

    %%empty: 4-4
        doc: TX FIFO empty

    %%overflow: 5-5
        doc: Attempt to write to a full TX FIFO

    %%underflow: 6-6
        doc: Attempt to read from an empty TX FIFO

    %%blocked: 7-7
        doc: SW cannot get access to the EZ memory

    %%uart_nack: 8-8
        doc: UART transmitter received a negative acknowledgement in SmartCard mode

    %%uart_done: 9-9
        doc: UART transmitter done event

    %%uart_arb_lost: 10-10
        doc: UART lost arbitration

%intr_tx_masked: 32
    address: 0xf8c
    longname: Transmitter interrupt masked register

    %%trigger: 0-0
        doc: Less entries in the TX FIFO than the value specified by TX_FIFO_CTRL

    %%not_full: 1-1
        doc: TX FIFO is not full

    %%empty: 4-4
        doc: TX FIFO empty

    %%overflow: 5-5
        doc: Attempt to write to a full TX FIFO

    %%underflow: 6-6
        doc: Attempt to read from an empty TX FIFO

    %%blocked: 7-7
        doc: SW cannot get access to the EZ memory

    %%uart_nack: 8-8
        doc: UART transmitter received a negative acknowledgement in SmartCard mode

    %%uart_done: 9-9
        doc: UART transmitter done event

    %%uart_arb_lost: 10-10
        doc: UART lost arbitration


%intr_rx: 32
    address: 0xfc0
    longname: Receiver interrupt request register

    %%trigger: 0-0
        doc: More entries in the RX FIFO than the value specified by RX_FIFO_CTRL

    %%not_empty: 2-2
        doc: RX FIFO is not empty

    %%full: 3-3
        doc: RX FIFO full

    %%overflow: 5-5
        doc: Attempt to write to a full RX FIFO

    %%underflow: 6-6
        doc: Attempt to read from an empty RX FIFO

    %%blocked: 7-7
        doc: SW cannot get access to the EZ memory

    %%frame_error: 8-8
        doc: Frame error in received data frame

    %%parity_error: 9-9
        doc: Parity error in received data frame

    %%baud_detect: 10-10
        doc: LIN baudrate detection completed

    %%break_detect: 11-11
        doc: Break detection is successful

%intr_rx_set: 32
    address: 0xfc4
    longname: Receiver interrupt setter register
    direction: w

    %%trigger: 0-0
        doc: Less entries in the RX FIFO than the value specified by RX_FIFO_CTRL

    %%not_empty: 2-2
        doc: RX FIFO is not empty

    %%full: 3-3
        doc: RX FIFO full

    %%overflow: 5-5
        doc: Attempt to write to a full RX FIFO

    %%underflow: 6-6
        doc: Attempt to read from an empty RX FIFO

    %%blocked: 7-7
        doc: SW cannot get access to the EZ memory

    %%frame_error: 8-8
        doc: Frame error in received data frame

    %%parity_error: 9-9
        doc: Parity error in received data frame

    %%baud_detect: 10-10
        doc: LIN baudrate detection completed

    %%break_detect: 11-11
        doc: Break detection is successful

%intr_rx_mask: 32
    address: 0xfc8
    longname: Receiver interrupt mask register

    %%trigger: 0-0
        doc: Less entries in the RX FIFO than the value specified by RX_FIFO_CTRL

    %%not_empty: 2-2
        doc: RX FIFO is not empty

    %%full: 3-3
        doc: RX FIFO full

    %%overflow: 5-5
        doc: Attempt to write to a full RX FIFO

    %%underflow: 6-6
        doc: Attempt to read from an empty RX FIFO

    %%blocked: 7-7
        doc: SW cannot get access to the EZ memory

    %%frame_error: 8-8
        doc: Frame error in received data frame

    %%parity_error: 9-9
        doc: Parity error in received data frame

    %%baud_detect: 10-10
        doc: LIN baudrate detection completed

    %%break_detect: 11-11
        doc: Break detection is successful

%intr_rx_masked: 32
    address: 0xfcc
    longname: Receiver interrupt masked register

    %%trigger: 0-0
        doc: Less entries in the RX FIFO than the value specified by RX_FIFO_CTRL

    %%not_empty: 2-2
        doc: RX FIFO is not empty

    %%full: 3-3
        doc: RX FIFO full

    %%overflow: 5-5
        doc: Attempt to write to a full RX FIFO

    %%underflow: 6-6
        doc: Attempt to read from an empty RX FIFO

    %%blocked: 7-7
        doc: SW cannot get access to the EZ memory

    %%frame_error: 8-8
        doc: Frame error in received data frame

    %%parity_error: 9-9
        doc: Parity error in received data frame

    %%baud_detect: 10-10
        doc: LIN baudrate detection completed

    %%break_detect: 11-11
        doc: Break detection is successful

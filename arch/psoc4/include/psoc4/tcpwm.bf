name: tcpwm
longname: Cypress Timer, Counter, PWM Counter (CNT)

bound: 32
align: 32

%cnt_ctrl: 32
    address: 0x40200100
    count: 4
    stride: 0x40
    longname: Counter control register

    %%mode: 24-26
        doc: Counter mode
        0: Timer
        2: Capture
        3: Quad: Quadrature encoding
        4: pwm: Pulse width modulation
        5: pwm_dt: PWM with deadtime insertion
        6: pwm_pr: Pseudo random pulse width modulation

    %%quadrature_mode: 20-21
        doc: In QUAD mode selects quadrature encoding mode
        0x0: X1: X1 encoding (QUAD mode)
        0x1: X2: X2 encoding (QUAD mode)
#        0x1: INV_OUT: When bit 0 is '1', QUADRATURE_MODE[0] inverts "dt_line_out" (PWM/PWM_DT modes)
        0x2: X4: X4 encoding (QUAD mode)
#        0x2: INV_COMPL_OUT: When bit 1 is '1', QUADRATURE_MODE[1] inverts "dt_line_compl_out" (PWM/PWM_DT modes)

    %%one_shot: 18-18
        doc: Whether to turn counter off after termination

    %%up_down_mode: 16-17
        doc: Determines counter direction
        0x0: count_up
        0x1: count_down
        0x2: count_updn1
        0x3: count_updn2

    %%generic: 8-15
        doc: Divide by 2^generic

    %%pwm_stop_on_kill: 3-3
        doc: Whether the counter stops on a kill events

    %%pwm_sync_kill: 2-2
        doc: Whether kiss is synchronous

    %%auto_reload_period: 1-1
        doc: Whether to switch PERIOD on a terminal count event with and actively pending siwtch event

    %%auto_reload_cc: 0-0
        doc: Whether to switch CC on a terminal count event with an actively pending switch event

%cnt_status: 32
    address: 0x40200104
    count: 4
    stride: 0x40
    longname: Counter status register

%cnt_counter: 32
    address: 0x40200108
    count: 4
    stride: 0x40
    longname: Counter count register

%cnt_cc: 32
    address: 0x4020010c
    count: 4
    stride: 0x40
    longname: Counter compare/capture register

%cnt_cc_buff: 32
    address: 0x40200110
    count: 4
    stride: 0x40
    longname: Counter buffered compare/capture register

%cnt_period: 32
    address: 0x40200114
    count: 4
    stride: 0x40
    longname: Counter period register

%cnt_period_buff: 32
    address: 0x40200118
    count: 4
    stride: 0x40
    longname: Counter buffered period register

%cnt_tr_ctrl0: 32
    address: 0x40200120
    count: 4
    stride: 0x40
    longname: Counter trigger control register 0

%cnt_tr_ctrl1: 32
    address: 0x40200124
    count: 4
    stride: 0x40
    longname: Counter trigger control register 1

%cnt_tr_ctrl2: 32
    address: 0x40200128
    count: 4
    stride: 0x40
    longname: Counter trigger control register 2

%cnt_intr: 32
    address: 0x40200130
    count: 4
    stride: 0x40
    longname: Interrupt request register

%cnt_intr_set: 32
    address: 0x40200134
    count: 4
    stride: 0x40
    longname: Interrupt set request register

%cnt_intr_mask: 32
    address: 0x40200138
    count: 4
    stride: 0x40
    longname: Interrupt mask register

%cnt_intr_masked: 32
    address: 0x4020013c
    count: 4
    stride: 0x40
    longname: Interrupt masked request register

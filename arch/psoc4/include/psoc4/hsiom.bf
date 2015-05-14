name: hsiom
longname: Cypress PSoC4 High-Speed IO Matrix

bound: 32
align: 32

%amux_split_ctl: 32
    address: 0x40022100
    count: 3
    stride: 0x4
    doc: AMUX splitter cell control, retained

    %%switch_aa_sl: 0-0
        doc: T-switch control for Left AMUXBUSA switch
        0: open
        1: closed

    %%switch_aa_sr: 1-1
        doc: T-switch control for Right AMUXBUSA switch
        0: open
        1: closed

    %%switch_aa_s0: 2-2
        doc: T-switch control for VSSA/Gnd AMUXBUSA switch
        0: open
        1: closed

    %%switch_bb_sl: 4-4
        doc: T-switch control for Left AMUXBUSA switch
        0: open
        1: closed

    %%switch_bb_sr: 5-5
        doc: T-switch control for Right AMUXBUSA switch
        0: open
        1: closed

    %%switch_bb_s0: 6-6
        doc: T-switch control for VSSA/Gnd AMUXBUSA switch
        0: open
        1: closed


%port_sel: 32
    address: 0x40020100
    count: 6
    stride: 0x100
    doc: Port selection register, retained

    %%io_sel: 0-3
        stride: 4
        count: 8
        doc: IO pad routing selection
        0x0: GPIO: SW controlled GPIO
        0x4: CSD sense: CSD sense connection (analog mode)
        0x5: CSD shield: CSD shield connection (analog mode)
        0x6: AMUXA: AMUXBUS A connection
        0x7: AMUXB: AMUXBUS B connection. This mode is also used for CSD GPIO charging
        0x8: ACT_0: Chip specific Active source 0 connection
        0x9: ACT_1: Chip specific Active source 1 connection
        0xa: ACT_2: Chip specific Active source 2 connection
        0xb: ACT_3: Chip specific Active source 3 connection
        0xc: LCD_COM_DS_0: LCD common connection. Chip specific DeepSleep source 0 connection
        0xd: LCD_SEG_DS_1: LCD segment connection. Chip specific DeepSleep source 1 connection
        0xe: DS_2: Chip specific DeepSleep source 2 connection
        0xf: DS_3: Chip specific DeepSleep source 3 connection

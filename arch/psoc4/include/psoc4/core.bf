name: core
longname: Cypress System Resource Sub System

bound: 32
align: 32

%pwr_control: 32
    address: 0x400B0000
    longname: Power Mode Control

    %%hibernate: 31-31
        doc: Selects between HIBERNATE/DEEPSLEEP modes when Cortex-M0 enters low power mode
        0: DEEP_SLEEP: Enter DeepSleep mode when CPU asserts SLEEPDEEP signal
        1: HIBERNATE: Enter Hibernate mode when CPU asserts SLEEPDEEP signal

    %%lfclk_short: 29-29
        doc: Short Vcclfclk and Vccdpslp power rails in DeepSleep power mode

    %%hibernate_disable: 28-28
        doc: Disable interpretation of hibernate field. This bit is a write-once bit until the next reset

    %%fimo_disable: 27-27
        doc: Force IMO to operate at 12MHz, ignore its frequency and trim settings and operate independent on its external references

    %%hvmon_reload: 25-25
        doc: Firmware writes 1 to reload HV State in hibernate shadow copy. Hardware clears this bit after reload was successful. Wait at least 9 cycles after writing/recalling NVL before reloading the HVMON.

    %%hvmon_enable: 24-24
        longname: State Monitoring enable

    %%ext_vccd: 23-23
        doc: Should be set by firmware if Vccd is provided externally (on Vccd pin). Setting this bit turns off the active regulator and will lead to system reset (PBOD) unless both Vddd and Vccd pins are supplied externally

    %%lpm_ready: 5-5
        direction: r
        doc: Indicates whether the low power mode regulators are ready to enter DEEPSLEEP or HIBERNATE mode
        0: auto: If DEEPSLEEP or HIBERNATE mode is requested, device will enter SLEEP mode. When low power regulators are ready, device will automatically enter the originally requested mode
        1: normal: Normal operation. DEEPSLEEP and HIBERNATE work as described

    %%debug_session: 4-4
        direction: r
        doc: Indicates whether a debug session is active

    %%power_mode: 0-3
        direction: r
        doc: Current power mode of the device. Note that this field cannot be read in all power modes on actual silicon
        0: reset
        1: active
        2: sleep
        3: deep_sleep
        4: hibernate

%pwr_intr: 32
    address: 0x400B0004
    longname: Power System Interrupt
    direction: r

    %%lvd: 1-1
       doc: Indicates an Low Voltage Detect interrupt

%pwr_intr_mask: 32
    address: 0x400B0008
    longname: Power System Interrupt Mask

    %%lvd: 1-1
       doc: Propagate Low Voltage Detect interrupt

%pwr_key_delay: 32
    address: 0x400B000C
    longname: Power System Key

    %%wakeup_holdoff: 0-9
        doc: Delay (in 12MHz IMO clock cycles) to wait for references to settle on wakeup from hibernate/deepsleep

%pwr_bg_config: 32
    address: 0x400B0014
    longname: Bandgap Trim and Configuration

    %%bg_dft_en: 0-0
        doc: Enables DFT capability for Bandgap

    %%bg_dft_vref_sel: 1-4
        doc: ADFT mux select for Reference System characterization. Select a voltage reference to output on adft_bg_ref
        0: vgnd
        1: vref_fast[0]
        2: vref_fast[1]
        3: vref_fast[2]
        4: vref_fast[3]
        5: vref_fast[4]
        6: vref_fast[5]
        7: vref_fast[6]
        8: vref_fast[7]
        9: vref[0]
        10: vref[1]
        11: vref[2]
        12: vctat
        13: iref_dft (see BG_DFT_ICORE_SEL)
        14: imo_iref (current)
        15: inl_imoref (voltage) 

    %%bg_dft_core_sel: 5-5
        doc: ADFT mux select for Bandgap characterization. Selects which BG core signal to output on adft_bg_core
        0: vcore: BG core voltage selected by BG_DFT_VCORE_SEL (mux2out)
        1: icore: BG core current selected by BG_DFT_ICORE_SEL (mux1out)

    %%bg_dft_icore_sel: 6-7
        doc: ADFT mux select for Bandgap characterization. Selects a BG core current to output on mux1out
        0: iptat
        1: ictat
        2: inl_cross_over detect
        3: iref9p6u_dft

    %%bg_dft_vcore_sel: 8-8
        doc: ADFT mux select for Bandgap characterization. Selects a BG core voltage to output on mux2out
        0: vout
        1: vgnd

    %%vref_en: 16-18
        doc: Reference voltage enable. Each bit enables a reference voltage used by a peripheral. vref[0] enables SRSS.VREF[0] to 1.024V. vref[1] enables SRSS.VREF[1] to 1.024V. vref[2] enables SRSS.VREF[2] to 1.2V


%pwr_vmon_config: 32
    address: 0x400B0018
    longname: Voltage Monitoring Trim and Configuration

    %%lvd_en: 0-0
        doc: Enable Low Voltage Detect circuit

    %%lvd_sel: 1-4
        doc: Threshold selection for Low Voltage Detect circuit
        0: 1.75 V
        1: 1.80 V
        2: 1.90 V
        3: 2.00 V
        4: 2.10 V
        5: 2.20 V
        6: 2.30 V
        7: 2.40 V
        8: 2.50 V
        9: 2.60 V
        10: 2.70 V
        11: 2.80 V
        12: 2.90 V
        13: 3.00 V
        14: 3.20 V
        15: 4.50 V         

    %%vmon_ddft_sel: 5-7
        doc: DDFT mux select for HVPOR, PBOD, and LVD circuits
        0: 0
        1: pbod_out
        2: Pulse Strecher output of pbod Monitor
        3: hvpbod_out
        4: Pulse Strecher output of hvpbod Monitor
        5: lvi_out
        6: Pulse Strecher output of lvi Monitor
        7: 0

    %%vmon_adft_sel: 8-9
        doc: ADFT mux select for HVPOR, PBOD, and LVD circuits
        0: Hi-Z
        1: Comparator Input of pbod Monitor
        2: Comparator Input of hvpbod Monitor
        3: Comparator Input of lvi Monitor 

%pwr_ddft_select: 32
    address: 0x400B0020
    longname: Digital DFT Select

    %%ddft1_sel: 0-3
        doc: Signal select for ddft1 output
        0: wakeup_a
        1: ipor_reset
        2: hbod_reset_raw_n
        3: lpcomp_dis
        4: power_up_delayed
        5: awake
        6: hvmon_out_of_sync
        7: pbod_reset
        8: hvbod_reset
        9: lpm_ready
        10: io_disable_req_lv
        11: bootref_en

    %%ddft2_sel: 4-7
        doc: Signal select for ddft2 output
        0: act_power_en_a
        1: power_up_raw
        2: act_power_good_a
        3: fastrefs_valid
        4: vmon
        5: bootref_outen
        6: bootref_refsw
        7: active_inrush_dis
        8: awake
        9: hvpor_reset_n
        10: lpcomp_dis
        11: wakeup_a
        12: vmon_valid
        13: block_rst_awake
        14: slpholdreq_n
        15: io_disable_delayed 

%pwr_dft_key: 32
    address: 0x400B0024
    longname: DFT Safety Override

    %%vmon_pd: 20-20
        doc: Disables the VMON block, which includes PBOD, HVBOD, and LVD circuits. Set BODS_OFF=1 before to prevent an unintended reset

    %%io_disable_bypass: 19-19
        doc: Bypasses the IO disable logic for testing the delay-line that is part of the glitch-free IO reset circuitry

    %%dft_mode: 18-18
        doc: Enable DfT modes other than the above

    %%bods_off: 17-17
        doc: Forces all outputs of BOD detectors to be ignored, effectively disabling all brown-out detection

    %%hbod_off_awake: 16-16
        doc: Forces the output of the HBOD to be blocked (ignored) while in Active or Sleep mode

    %%key16: 0-15
        doc: This field must be set to 0xe4c5 to unlock other fields in this
        0xe4c5: key

%pwr_bod_key: 32
    address: 0x400B0028
    longname: BOD Detection Key

    %%key16: 0-16
        0x3a71: Brown-out

%pwr_stop: 32
    address: 0x400B002C
    longname: STOP Mode

    %%stop: 31-31
        doc: Enter STOP mode on set. Both UNLOCK and FREEZE must have been set before

    %%freeze: 17-17
        doc: Freeze the configuration when set. Must be written twice

    %%polarity: 16-16
        doc: Polarity for WAKEUP signal. WAKEUP must match this bit to wakeup the chip

    %%unlock: 8-15
        doc: Locking value for FREEZE and STOP
        0x3a: key

    %%token: 0-7
        doc: Firmware-specific token that can be read/written to distinguish between wakeup and reset.  0 on reset.

%clk_select: 32
    address: 0x400B0100
    longname: Clock Select

    %%sysclk_div: 19-21
        longname: SYSCLK Pre-Scaler Value
        doc: SYSCLK = HFCLK / 2^(sysclk_div + 1)

    %%half_en: 18-18
        longname: FLASH Wait-state count
        doc: CPUSSv1 only. Must be set when HFCLK > 24 MHz

    %%hfclk_sel: 16-17
        longname: HFCLK Source
        0: direct_sel: Selected by direct_sel
        1: dbl: Output of doubler
        2: pll: Output of PLL

    %%wdt_lock: 14-15
        doc: Prohibits writing to WDT_* registers and CLK_ILO register when not 0. Requires at least two different writes to unlock
        0x0: no_chg: No effect
        0x1: clr0: Clears bit 0
        0x2: clr1: Clears bit 1
        0x3: set01: Sets both bits 0 and 1 

    %%dpllref_sel: 12-13
    %%dpllin_sel: 9-11
    %%pll_sel: 6-8
    %%dbl_sel: 3-5 

    %%direct_sel: 0-2
        doc: Source for HFCLK when HFCLK_SEL=DIRECT_SEL
        0x0: IMO: Internal R/C Oscillator
        0x1: EXTCLK: External Clock Pin
        0x2: ECO: Internal Crystal Oscillator

%clk_ilo_config: 32
    address: 0x400B0104
    longname: ILO Configuration

    %%enable: 31-31
        doc: Master enable for ILO oscillator

    %%satbias: 2-2
        longname: PFET bias
        0x0: SATURATED: Enable saturated PFET bias
        0x1: SUBTHRESHOLD: Enable subthreshold PFET bias

    %%turbo: 1-1
        longname: Turbo mode
        doc: For faster startup from coma power down

    %%pd_mode: 0-0
        longname: Power down mode
        0: sleep
        1: coma

%clk_imo_config: 32
    address: 0x400B0108
    longname: IMO Configuration

    %%enable: 31-31
        doc: Master enable for IMO oscillator

    %%en_clk2x: 30-30
        doc: Enables main oscillator doubler circuit that can be used for TSS Charge Pumps

    %%en_clk36: 29-29
        doc: Enables 36MHz secondary oscillator that can be used for Pump or Flash Pump

    %%test_usb_mode: 28-28
        doc: Forces IMO into USB mode

    %%pump_sel: 25-27
        doc: Selects operating source for Pump clock
        0x0: GND: No clock, connect to gnd
        0x1: IMO: Use main IMO output
        0x2: DBL: Use doubler output
        0x3: CLK36: Use 36MHz oscillator
        0x4: FF1: Use divided clock FF1

    %%test_fastbias: 24-24
        doc: Forces the IMO into FIMO mode

    %%en_fastbias: 23-23
        doc: Forces the FIMO's fast bias circuits to remain powered

    %%flashpump_sel: 22-22
        doc: Selects operating source for SPCIF Timer/Flash Pump clock
        0x0: GND: No clock, connect to gnd
        0x1: CLK36: Use 36MHz oscillator

%clk_imo_spread: 32
    address: 0x400B010C
    longname: IMO Spread Spectrum Configuration

    %%ss_mode: 30-31
        doc: Spread Spectrum Mode
        0x0: OFF: Off, do not change SS_VALUE
        0x1: TRIANGLE: Modulate using triangle wave (see SS_MAX)
        0x2: LFSR: Modulate using pseudo random sequence (using LFSR)

    %%ss_range: 28-29
        doc: Spread spectrum range
        0x0: 0-1%
        0x1: 0-2%
        0x2: 0-4%

    %%ss_max: 8-12
        doc: Maximum counter value for spread spectrum

    %%ss_value: 0-4
        doc: Current offset value for spread spectrum modulation

%clk_dft_select: 32
    address: 0x400B0110
    longname: Clock DFT Mode Selection

    %%dft_div2: 12-13
        doc: DFT Output Divide Down for DFT output #2
        0x0: NO_DIV: Direct Output
        0x1: DIV_BY_2: Divide by 2
        0x2: DIV_BY_4: Divide by 4
        0x3: DIV_BY_8: Divide by 8

    %%dft_sel2: 8-11
        doc: Select signal for DFT output #2
        0x0: NC: Disabled - output is not connected
        0x1: ILO: ILO output
        0x2: WCO: WCO output
        0x3: IMO: IMO primary output
        0x4: ECO: ECO output
        0x5: PLL: PLL output
        0x6: DPLL_OUT: DPLL output
        0x7: DPLL_REF: DPLL reference input
        0x8: DBL: DBL output
        0x9: IMO2X: IMO 2x Clock Output
        0xa: IMO36: IMO 36MHz Clock Output
        0xb: HFCLK: HFCLK
        0xc: LFCLK: LFCLK
        0xd: SYSCLK: SYSCLK
        0xe: EXTCLK: EXTCLK
        0xf: HALFSYSCLK: 0 - removed

    %%dft_div1: 4-5
        doc: DFT Output Divide Down for DFT output #1
        0x0: NO_DIV: Direct Output
        0x1: DIV_BY_2: Divide by 2
        0x2: DIV_BY_4: Divide by 4
        0x3: DIV_BY_8: Divide by 8

    %%dft_sel1: 0-3
        doc: Select signal for DFT output #1
        0x0: NC: Disabled - output is not connected
        0x1: ILO: ILO output
        0x2: WCO: WCO output
        0x3: IMO: IMO primary output
        0x4: ECO: ECO output
        0x5: PLL: PLL output
        0x6: DPLL_OUT: DPLL output
        0x7: DPLL_REF: DPLL reference input
        0x8: DBL: DBL output
        0x9: IMO2X: IMO 2x Clock Output
        0xa: IMO36: IMO 36MHz Clock Output
        0xb: HFCLK: HFCLK
        0xc: LFCLK: LFCLK
        0xd: SYSCLK: SYSCLK
        0xe: EXTCLK: EXTCLK
        0xf: HALFSYSCLK: 0 - removed

%wdt_ctrlow: 32
    address: 0x400B0200
    longname: Watchdog Counters 0/1
    direction: r

    %%wdt_ctr1: 16-31
        doc: Current value of WDT Counter 1

    %%wdt_ctr0: 0-15
        doc: Current value of WDT Counter 0

%wdt_ctrhigh: 32
    address: 0x400B0204
    longname: Watchdog Counter 2
    direction: r

    %%wdt_ctr2: 0-31
        doc: Current value of WDT Counter 2

%wdt_match: 32
    address: 0x400B0208
    longname: Watchdog counter match values

    %%wdt_match1: 16-31
        doc: Match value of WDT Counter 1

    %%wdt_match0: 0-15
        doc: Match value of WDT Counter 0

%wdt_config: 32
    address: 0x400B020C
    longname: Watchdog Counters Configuration

    %%lfclk_sel: 30-31
        doc: Select source for LFCLK:
        0: ILO: Internal R/C Oscillator
        1: WCO: Internal Crystal Oscillator

    %%wdt_bits2: 24-28
        doc: Bit to observe for WDT_INT2, assert WDT_INT2 when bit N toggles

    %%wdt_mode2: 16-16
        doc: Watchdog Counter 2 Mode
        0x0: NOTHING: Free running counter with no interrupt requests
        0x1: INT: Free running counter with interrupt request when a specified bit in CTR2 toggles (see WDT_BITS2)

    %%wdt_cascade1_2: 11-11
        doc: Cascade Watchdog Counters 1 and 2. Counter 2 increments the cycle after WDT_CTR1=WDT_MATCH1

    %%wdt_clear1: 10-10
        doc: Clear Watchdog Counter when WDT_CTR1=WDT_MATCH1

    %%wdt_mode1: 8-9
        doc: Watchdog Counter Action on Match (WDT_CTR1=WDT_MATCH1)
        0x0: NOTHING: Do nothing
        0x1: INT: Assert WDT_INTx
        0x2: RESET: Assert WDT Reset
        0x3: INT_THEN_RESET: Assert WDT_INTx, assert WDT Reset after 3rd unhandled interrupt

    %%wdt_cascade0_1: 3-3
        doc: Cascade Watchdog Counters 0 and 1

    %%wdt_clear0: 2-2
        doc: Clear Watchdog Counter when WDT_CTR0=WDT_MATCH0

    %%wdt_mode0: 0-1
        doc: Watchdog Counter Action on Match (WDT_CTR0=WDT_MATCH0)
        0x0: NOTHING: Do nothing
        0x1: INT: Assert WDT_INTx
        0x2: RESET: Assert WDT Reset
        0x3: INT_THEN_RESET: Assert WDT_INTx, assert WDT Reset after 3rd unhandled interrupt
        
%wdt_control: 32
    address: 0x400B0210
    longname: Watchdog Counters Control

    %%wdt_reset: 3-3
        doc: Reset counter back to 0
        count: 3
        stride: 8

    %%wdt_int: 2-2
        doc: WDT Interrupt Request
        count: 3
        stride: 8

    %%wdt_enabled: 1-1
        doc: Indicates actual state of counter
        count: 3
        stride: 8

    %%wdt_enable: 0-0
        doc: Enable Counter
        count: 3
        stride: 8

%res_cause: 32
    address: 0x400B0300
    longname: Reset Cause Observation

    %%reset_soft: 4-4
        doc: CPU requested a system reset through it's SYSRESETREQ
    %%reset_prot_fault: 3-3
        doc: A protection violation occurred that requires a RESET
    %%reset_wdt: 0-0
        doc: A WatchDog Timer reset has occurred since last power cycle

%pwr_bg_trim3: 32
    address: 0x400BFF18
    longname: Bandgap Trim 3

    %%inl_cross_imo: 3-6
       doc: IMO Irefgen INL cross-over point control for centering curve at 30C
    %%inl_trim_imo: 0-2
       doc: IMO Irefgen nonlinear current trim for curvature correction

%pwr_bg_trim4: 32
    address: 0x400BFF1C
    longname: Bandgap Trim 4

    %%abs_trim_imo: 0-5 
        doc: IMO-irefgen output current magnitude trim

%pwr_bg_trim5: 32
    address: 0x400BFF20
    longname: Bandgap Trim 5

    %%tmpco_trim_imo: 0-5 
        doc: IMO-irefgen output current temperature coefficient trim

%clk_ilo_trim: 32
    address: 0x400BFF24
    longname: ILO Trim

    %%coarse_trim: 4-7
        doc: Adjusts the ILO bias

    %%trim: 0-3
        doc: Trim bits to control frequency

%clk_imo_trim1: 32
    address: 0x400BFF28
    longname: IMO Trim 1

    %%offset: 0-7
        doc: Frequency trim bits

%clk_imo_trim2: 32
    address: 0x400BFF2C
    longname: IMO Trim 2

    %%freq: 0-5
        doc: Frequency to be selected [3-12] => [3MHz-12MHz], [14-25] => [13MHz-24MHz], [27-35] => [25MHz-33MHz], [37-43] => [34MHz-40MHz], [46-53] => [41MHz-48MHz] 

%clk_imo_trim4: 32
    address: 0x400BFF34
    longname: IMO Trim 4

    %%fsoffset: 5-7
        doc: Full-speed USB offset

    %%gain: 0-4
        doc: Gain for IMO

%pwr_rsvd_trim: 32
    address: 0x400BFF38
    longname: Reserved, unused registers

    %%rsvd_trim: 0-3
        doc: Reserved, unused

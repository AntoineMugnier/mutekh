/***************************************
* Auto generated by BFGen, do not edit *
***************************************/

/*
   bfgen -I arm/dwt.bf                                                         \
     -O /home/nipo/projects/mutekh/cpu/arm32m/include/cpu/arm32m/dwt.h -o cdefs \
     cdefs_use_field_shift=1
*/

#ifndef _DWT_BFGEN_DEFS_
#define _DWT_BFGEN_DEFS_

/** Control Register @multiple */
#define DWT_CTRL_ADDR                                0xe0001000
/** CYCCNTENA bit. Enables CYCCNT. Unsupported if NOCYCCNT @multiple */
  #define DWT_CTRL_CYCCNTENA                       0x00000001
  #define DWT_CTRL_CYCCNTENA_SHIFT                 0
/** Reload value for the POSTCNT counter. Unsupported if NOCYCCNT @multiple */
  #define DWT_CTRL_POSTPRESET_SHIFT                1
  #define DWT_CTRL_POSTPRESET(v)                   ((v) << 1)
  #define DWT_CTRL_POSTPRESET_SET(x, v)            do { (x) = (((x) & ~0x1e) | ((v) << 1)); } while(0)
  #define DWT_CTRL_POSTPRESET_GET(x)               (((x) >> 1) & 0xf)
/** Initial value for the POSTCNT counter. Unsupported if NOCYCCNT @multiple */
  #define DWT_CTRL_POSTINIT_SHIFT                  5
  #define DWT_CTRL_POSTINIT(v)                     ((v) << 5)
  #define DWT_CTRL_POSTINIT_SET(x, v)              do { (x) = (((x) & ~0x1e0) | ((v) << 5)); } while(0)
  #define DWT_CTRL_POSTINIT_GET(x)                 (((x) >> 5) & 0xf)
/** Selects the position of the POSTCNT tap on the CYCCNT counter. Unsupported if
   NOCYCCNT @multiple */
  #define DWT_CTRL_CYCTAP_SHIFT                    9
  #define DWT_CTRL_CYCTAP(v)                       ((DWT_CTRL_CYCTAP_##v) << 9)
  #define DWT_CTRL_CYCTAP_SET(x, v)                do { (x) = (((x) & ~0x200) | ((DWT_CTRL_CYCTAP_##v) << 9)); } while(0)
  #define DWT_CTRL_CYCTAP_GET(x)                   (((x) >> 9) & 0x1)
/** POSTCNT tap at CYCCNT[6], every 64 cycles */
    #define DWT_CTRL_CYCTAP_EVERY_64                 0x00000000
/** POSTCNT tap at CYCCNT[10], every 1024 cycles */
    #define DWT_CTRL_CYCTAP_EVERY_1024               0x00000001
/** Selects the position of the synchronization packet counter tap on the CYCCNT
   counter. This determines the Synchronization packet rate @multiple */
  #define DWT_CTRL_SYNCTAP_SHIFT                   10
  #define DWT_CTRL_SYNCTAP(v)                      ((DWT_CTRL_SYNCTAP_##v) << 10)
  #define DWT_CTRL_SYNCTAP_SET(x, v)               do { (x) = (((x) & ~0xc00) | ((DWT_CTRL_SYNCTAP_##v) << 10)); } while(0)
  #define DWT_CTRL_SYNCTAP_GET(x)                  (((x) >> 10) & 0x3)
/** Disabled. No Synchronization packets */
    #define DWT_CTRL_SYNCTAP_NONE                    0x00000000
/** Synchronization counter tap at CYCCNT[24], triggers every 16M cycles */
    #define DWT_CTRL_SYNCTAP_EVERY_16M               0x00000001
/** Synchronization counter tap at CYCCNT[26], triggers every 64M cycles */
    #define DWT_CTRL_SYNCTAP_EVERY_64M               0x00000002
/** Synchronization counter tap at CYCCNT[28], triggers every 256M cycles */
    #define DWT_CTRL_SYNCTAP_EVERY_256M              0x00000003
/** Enables use of POSTCNT counter as a timer for Periodic PC sample packet
   generation. Unsupported if NOTRCPKT || NOCYCCNT @multiple */
  #define DWT_CTRL_PCSAMPLENA                      0x00001000
  #define DWT_CTRL_PCSAMPLENA_SHIFT                12
/** Enables generation of exception trace. Unsupported if NOPRFCNT @multiple */
  #define DWT_CTRL_EXCTRCENA                       0x00010000
  #define DWT_CTRL_EXCTRCENA_SHIFT                 16
/** Enables generation of the CPI counter overflow event. Unsupported if NOPRFCNT
   @multiple */
  #define DWT_CTRL_CPIEVTENA                       0x00020000
  #define DWT_CTRL_CPIEVTENA_SHIFT                 17
/** Enables generation of the Exception overhead counter overflow event.
   Unsupported if NOPRFCNT @multiple */
  #define DWT_CTRL_EXCEVTENA                       0x00040000
  #define DWT_CTRL_EXCEVTENA_SHIFT                 18
/** Enables generation of the Sleep counter overflow event. Unsupported if
   NOPRFCNT @multiple */
  #define DWT_CTRL_SLEEPEVTENA                     0x00080000
  #define DWT_CTRL_SLEEPEVTENA_SHIFT               19
/** Enables generation of the LSU counter overflow event. Unsupported if NOPRFCNT
   @multiple */
  #define DWT_CTRL_LSUEVTENA                       0x00100000
  #define DWT_CTRL_LSUEVTENA_SHIFT                 20
/** Enables generation of the Folded-instruction counter overflow event.
   Unsupported if NOPRFCNT @multiple */
  #define DWT_CTRL_FOLDEVTENA                      0x00200000
  #define DWT_CTRL_FOLDEVTENA_SHIFT                21
/** Enables POSTCNT underflow Event counter packets generation. Unsupported if
   NOTRCPKT || NOCYCCNT @multiple */
  #define DWT_CTRL_CYCEVTENA                       0x00400000
  #define DWT_CTRL_CYCEVTENA_SHIFT                 22
/** Shows whether the implementation supports the profiling counters @multiple */
  #define DWT_CTRL_NOPFRCNT                        0x01000000
  #define DWT_CTRL_NOPFRCNT_SHIFT                  24
/** Shows whether the implementation supports a cycle counter @multiple */
  #define DWT_CTRL_NOCYCCNT                        0x02000000
  #define DWT_CTRL_NOCYCCNT_SHIFT                  25
/** Shows whether the implementation includes external match signals, CMPMATCH[N]
   @multiple */
  #define DWT_CTRL_NOEXTTRIG                       0x04000000
  #define DWT_CTRL_NOEXTTRIG_SHIFT                 26
/** Shows whether the implementation supports trace sampling and exception
   tracing. TRCPKT => CYCCNT @multiple */
  #define DWT_CTRL_NOTRCPKT                        0x08000000
  #define DWT_CTRL_NOTRCPKT_SHIFT                  27
/** Number of comparators implemented. A value of zero indicates no comparator
   support @multiple */
  #define DWT_CTRL_NUMCOMP_SHIFT                   28
  #define DWT_CTRL_NUMCOMP(v)                      ((v) << 28)
  #define DWT_CTRL_NUMCOMP_SET(x, v)               do { (x) = (((x) & ~0xf0000000) | ((v) << 28)); } while(0)
  #define DWT_CTRL_NUMCOMP_GET(x)                  (((x) >> 28) & 0xf)

/** Cycle Count Register @multiple */
#define DWT_CYCCNT_ADDR                              0xe0001004
/** Incrementing cycle counter value. When enabled, CYCCNT increments on each
   processor clock cycle. On overflow, CYCCNT wraps to zero @multiple */
  #define DWT_CYCCNT_CYCCNT_SHIFT                  0
  #define DWT_CYCCNT_CYCCNT(v)                     ((v) << 0)
  #define DWT_CYCCNT_CYCCNT_SET(x, v)              do { (x) = (((x) & ~0xffffffff) | ((v) << 0)); } while(0)
  #define DWT_CYCCNT_CYCCNT_GET(x)                 (((x) >> 0) & 0xffffffff)

/** CPI Count Register @multiple */
#define DWT_CPICNT_ADDR                              0xe0001008
/** The base CPI counter. Counts additional cycles required to execute
   multi-cycle instructions, except those recorded by DWT_LSUCNT, and counts any
   instruction fetch stalls @multiple */
  #define DWT_CPICNT_CPICNT_SHIFT                  0
  #define DWT_CPICNT_CPICNT(v)                     ((v) << 0)
  #define DWT_CPICNT_CPICNT_SET(x, v)              do { (x) = (((x) & ~0xff) | ((v) << 0)); } while(0)
  #define DWT_CPICNT_CPICNT_GET(x)                 (((x) >> 0) & 0xff)

/** Exception Overhead Count Register @multiple */
#define DWT_EXCCNT_ADDR                              0xe000100c
/** The exception overhead counter. Counts the total cycles spent in exception
   processing @multiple */
  #define DWT_EXCCNT_EXCCNT_SHIFT                  0
  #define DWT_EXCCNT_EXCCNT(v)                     ((v) << 0)
  #define DWT_EXCCNT_EXCCNT_SET(x, v)              do { (x) = (((x) & ~0xff) | ((v) << 0)); } while(0)
  #define DWT_EXCCNT_EXCCNT_GET(x)                 (((x) >> 0) & 0xff)

/** Sleep Count Register @multiple */
#define DWT_SLEEPCNT_ADDR                            0xe0001010
/** Sleep counter. Counts the total number of cycles that the processor is
   sleeping @multiple */
  #define DWT_SLEEPCNT_SLEEPCNT_SHIFT              0
  #define DWT_SLEEPCNT_SLEEPCNT(v)                 ((v) << 0)
  #define DWT_SLEEPCNT_SLEEPCNT_SET(x, v)          do { (x) = (((x) & ~0xff) | ((v) << 0)); } while(0)
  #define DWT_SLEEPCNT_SLEEPCNT_GET(x)             (((x) >> 0) & 0xff)

/** LSU Count Register @multiple */
#define DWT_LSUCNT_ADDR                              0xe0001014
/** Load-store counter. Increments on any additional cycles required to execute
   load or store instructions @multiple */
  #define DWT_LSUCNT_LSUCNT_SHIFT                  0
  #define DWT_LSUCNT_LSUCNT(v)                     ((v) << 0)
  #define DWT_LSUCNT_LSUCNT_SET(x, v)              do { (x) = (((x) & ~0xff) | ((v) << 0)); } while(0)
  #define DWT_LSUCNT_LSUCNT_GET(x)                 (((x) >> 0) & 0xff)

/** Folded-instruction Count Register @multiple */
#define DWT_FOLDCNT_ADDR                             0xe0001018
/** Folded-instruction counter. Increments on each instruction that takes 0
   cycles @multiple */
  #define DWT_FOLDCNT_FOLDCNT_SHIFT                0
  #define DWT_FOLDCNT_FOLDCNT(v)                   ((v) << 0)
  #define DWT_FOLDCNT_FOLDCNT_SET(x, v)            do { (x) = (((x) & ~0xff) | ((v) << 0)); } while(0)
  #define DWT_FOLDCNT_FOLDCNT_GET(x)               (((x) >> 0) & 0xff)

/** Program Counter Sample Register @multiple */
#define DWT_PCSR_ADDR                                0xe000101c
/** Executed Instruction Address sample value @multiple */
  #define DWT_PCSR_EIASAMPLE_SHIFT                 0
  #define DWT_PCSR_EIASAMPLE(v)                    ((v) << 0)
  #define DWT_PCSR_EIASAMPLE_SET(x, v)             do { (x) = (((x) & ~0xffffffff) | ((v) << 0)); } while(0)
  #define DWT_PCSR_EIASAMPLE_GET(x)                (((x) >> 0) & 0xffffffff)

/** Value that triggers watchpoint events @multiple */
#define DWT_COMP_ADDR(ridx)                          (0xe0001020 + (ridx) * 16)
#define DWT_COMP_COUNT                               4

/** Applies a mask when matching against COMP @multiple */
#define DWT_MASK_ADDR(ridx)                          (0xe0001024 + (ridx) * 16)
#define DWT_MASK_COUNT                               4
/** Count of bits to ignore from LSB of COMP @multiple */
  #define DWT_MASK_SIZE_SHIFT                      0
  #define DWT_MASK_SIZE(v)                         ((v) << 0)
  #define DWT_MASK_SIZE_SET(x, v)                  do { (x) = (((x) & ~0xf) | ((v) << 0)); } while(0)
  #define DWT_MASK_SIZE_GET(x)                     (((x) >> 0) & 0xf)

/** Control operation of the comparator @multiple */
#define DWT_FUNCTION_ADDR(ridx)                      (0xe0001028 + (ridx) * 16)
#define DWT_FUNCTION_COUNT                           4
  #define DWT_FUNCTION_FUNCTION_SHIFT              0
  #define DWT_FUNCTION_FUNCTION(v)                 ((DWT_FUNCTION_FUNCTION_##v) << 0)
  #define DWT_FUNCTION_FUNCTION_SET(x, v)          do { (x) = (((x) & ~0xf) | ((DWT_FUNCTION_FUNCTION_##v) << 0)); } while(0)
  #define DWT_FUNCTION_FUNCTION_GET(x)             (((x) >> 0) & 0xf)
    #define DWT_FUNCTION_FUNCTION_DISABLED           0x00000000
/** Sample and emit PC (EMITRANGE = 0) or Address Offset (EMITRANGE = 1) through ITM */
    #define DWT_FUNCTION_FUNCTION_PC                 0x00000001
/** Emit Data (EMITRANGE = 0) or data and Address Offset (EMITRANGE = 1) through ITM on read or write */
    #define DWT_FUNCTION_FUNCTION_DATA               0x00000002
/** Emit PC and Data Value (EMITRANGE = 0) or PC and Address Offset (EMITRANGE = 1) through ITM on read or write */
    #define DWT_FUNCTION_FUNCTION_PC_DATA            0x00000003
/** Watchpoint on PC match */
    #define DWT_FUNCTION_FUNCTION_WATCHPOINT_PC      0x00000004
/** Watchpoint on read */
    #define DWT_FUNCTION_FUNCTION_WATCHPOINT_READ    0x00000005
/** Watchpoint on write */
    #define DWT_FUNCTION_FUNCTION_WATCHPOINT_WRITE   0x00000006
/** Watchpoint on read or write */
    #define DWT_FUNCTION_FUNCTION_WATCHPOINT_READ_WRITE 0x00000007
/** ETM trigger on PC match */
    #define DWT_FUNCTION_FUNCTION_ETM_PC             0x00000008
/** ETM trigger on read */
    #define DWT_FUNCTION_FUNCTION_ETM_READ           0x00000009
/** ETM trigger on write */
    #define DWT_FUNCTION_FUNCTION_ETM_WRITE          0x0000000a
/** ETM trigger on read or write */
    #define DWT_FUNCTION_FUNCTION_ETM_READ_WRITE     0x0000000b
/** Depends on FUNCTION = (1, 2, 3) @multiple */
  #define DWT_FUNCTION_EMITRANGE                   0x00000020
  #define DWT_FUNCTION_EMITRANGE_SHIFT             5
/** Compare against clock cycle counter (only in comparator 0) @multiple */
  #define DWT_FUNCTION_CYCMATCH                    0x00000080
  #define DWT_FUNCTION_CYCMATCH_SHIFT              7
/** Data value compare (only in comparator 1) @multiple */
  #define DWT_FUNCTION_DATAVMATCH                  0x00000100
  #define DWT_FUNCTION_DATAVMATCH_SHIFT            8
/** Whether DATAVADDR1 is supported @multiple */
  #define DWT_FUNCTION_LNK1ENA                     0x00000200
  #define DWT_FUNCTION_LNK1ENA_SHIFT               9
/** Data word size to match @multiple */
  #define DWT_FUNCTION_DATAVSIZE_SHIFT             10
  #define DWT_FUNCTION_DATAVSIZE(v)                ((DWT_FUNCTION_DATAVSIZE_##v) << 10)
  #define DWT_FUNCTION_DATAVSIZE_SET(x, v)         do { (x) = (((x) & ~0xc00) | ((DWT_FUNCTION_DATAVSIZE_##v) << 10)); } while(0)
  #define DWT_FUNCTION_DATAVSIZE_GET(x)            (((x) >> 10) & 0x3)
/** a byte */
    #define DWT_FUNCTION_DATAVSIZE_BYTE              0x00000000
/** a half word */
    #define DWT_FUNCTION_DATAVSIZE_HALFWORD          0x00000001
/** a word */
    #define DWT_FUNCTION_DATAVSIZE_WORD              0x00000002
/** Linked address comparator to use when DATAVMATCH = 1 @multiple */
  #define DWT_FUNCTION_DATAVADDR0_SHIFT            12
  #define DWT_FUNCTION_DATAVADDR0(v)               ((v) << 12)
  #define DWT_FUNCTION_DATAVADDR0_SET(x, v)        do { (x) = (((x) & ~0xf000) | ((v) << 12)); } while(0)
  #define DWT_FUNCTION_DATAVADDR0_GET(x)           (((x) >> 12) & 0xf)
/** Linked address comparator to use when DATAVMATCH = 1 and LNK1ENA = 1
   @multiple */
  #define DWT_FUNCTION_DATAVADDR1_SHIFT            16
  #define DWT_FUNCTION_DATAVADDR1(v)               ((v) << 16)
  #define DWT_FUNCTION_DATAVADDR1_SET(x, v)        do { (x) = (((x) & ~0xf0000) | ((v) << 16)); } while(0)
  #define DWT_FUNCTION_DATAVADDR1_GET(x)           (((x) >> 16) & 0xf)
/** Set when comparator matches @multiple */
  #define DWT_FUNCTION_MATCHED                     0x01000000
  #define DWT_FUNCTION_MATCHED_SHIFT               24

#define DWT_LSR_ADDR                                 0xe0001fb4

/** Peripheral Identification Register 4 @multiple */
#define DWT_PID4_ADDR                                0xe0001fd0
/** JEP106 continuation code @multiple */
  #define DWT_PID4_JEP106_SHIFT                    0
  #define DWT_PID4_JEP106(v)                       ((v) << 0)
  #define DWT_PID4_JEP106_SET(x, v)                do { (x) = (((x) & ~0xf) | ((v) << 0)); } while(0)
  #define DWT_PID4_JEP106_GET(x)                   (((x) >> 0) & 0xf)
/** 4KB Count @multiple */
  #define DWT_PID4_C4KB_SHIFT                      4
  #define DWT_PID4_C4KB(v)                         ((v) << 4)
  #define DWT_PID4_C4KB_SET(x, v)                  do { (x) = (((x) & ~0xf0) | ((v) << 4)); } while(0)
  #define DWT_PID4_C4KB_GET(x)                     (((x) >> 4) & 0xf)

/** Peripheral Identification Register 5 @multiple */
#define DWT_PID5_ADDR                                0xe0001fd4

/** Peripheral Identification Register 6 @multiple */
#define DWT_PID6_ADDR                                0xe0001fd8

/** Peripheral Identification Register 7 @multiple */
#define DWT_PID7_ADDR                                0xe0001fdc

/** Peripheral Identification Register 0 @multiple */
#define DWT_PID0_ADDR                                0xe0001fe0
/** Part Number [7:0] @multiple */
  #define DWT_PID0_PARTNUMBER_SHIFT                0
  #define DWT_PID0_PARTNUMBER(v)                   ((v) << 0)
  #define DWT_PID0_PARTNUMBER_SET(x, v)            do { (x) = (((x) & ~0xff) | ((v) << 0)); } while(0)
  #define DWT_PID0_PARTNUMBER_GET(x)               (((x) >> 0) & 0xff)

/** Peripheral Identification Register 1 @multiple */
#define DWT_PID1_ADDR                                0xe0001fe4
/** Part Number [11:8] @multiple */
  #define DWT_PID1_PARTNUMBER_SHIFT                0
  #define DWT_PID1_PARTNUMBER(v)                   ((v) << 0)
  #define DWT_PID1_PARTNUMBER_SET(x, v)            do { (x) = (((x) & ~0xf) | ((v) << 0)); } while(0)
  #define DWT_PID1_PARTNUMBER_GET(x)               (((x) >> 0) & 0xf)
/** JEP106 identity code [3:0] @multiple */
  #define DWT_PID1_JEP106_IDENTITY_CODE_SHIFT      4
  #define DWT_PID1_JEP106_IDENTITY_CODE(v)         ((v) << 4)
  #define DWT_PID1_JEP106_IDENTITY_CODE_SET(x, v)  do { (x) = (((x) & ~0xf0) | ((v) << 4)); } while(0)
  #define DWT_PID1_JEP106_IDENTITY_CODE_GET(x)     (((x) >> 4) & 0xf)

/** Peripheral Identification Register 2 @multiple */
#define DWT_PID2_ADDR                                0xe0001fe8
/** JEP106 identity code [6:4] @multiple */
  #define DWT_PID2_JEP106_IDENTITY_CODE_SHIFT      0
  #define DWT_PID2_JEP106_IDENTITY_CODE(v)         ((v) << 0)
  #define DWT_PID2_JEP106_IDENTITY_CODE_SET(x, v)  do { (x) = (((x) & ~0x7) | ((v) << 0)); } while(0)
  #define DWT_PID2_JEP106_IDENTITY_CODE_GET(x)     (((x) >> 0) & 0x7)
/** Revision @multiple */
  #define DWT_PID2_REVISION_SHIFT                  4
  #define DWT_PID2_REVISION(v)                     ((v) << 4)
  #define DWT_PID2_REVISION_SET(x, v)              do { (x) = (((x) & ~0xf0) | ((v) << 4)); } while(0)
  #define DWT_PID2_REVISION_GET(x)                 (((x) >> 4) & 0xf)

/** Peripheral Identification Register 3 @multiple */
#define DWT_PID3_ADDR                                0xe0001fec
/** Customer Modified @multiple */
  #define DWT_PID3_CUSTOMERMODIFIED_SHIFT          0
  #define DWT_PID3_CUSTOMERMODIFIED(v)             ((v) << 0)
  #define DWT_PID3_CUSTOMERMODIFIED_SET(x, v)      do { (x) = (((x) & ~0xf) | ((v) << 0)); } while(0)
  #define DWT_PID3_CUSTOMERMODIFIED_GET(x)         (((x) >> 0) & 0xf)
/** RevAnd @multiple */
  #define DWT_PID3_REVAND_SHIFT                    4
  #define DWT_PID3_REVAND(v)                       ((v) << 4)
  #define DWT_PID3_REVAND_SET(x, v)                do { (x) = (((x) & ~0xf0) | ((v) << 4)); } while(0)
  #define DWT_PID3_REVAND_GET(x)                   (((x) >> 4) & 0xf)

/** Component Identification Register 0 @multiple */
#define DWT_CID0_ADDR                                0xe0001ff0
/** Preamble @multiple */
  #define DWT_CID0_PREAMBLE_SHIFT                  0
  #define DWT_CID0_PREAMBLE(v)                     ((v) << 0)
  #define DWT_CID0_PREAMBLE_SET(x, v)              do { (x) = (((x) & ~0xff) | ((v) << 0)); } while(0)
  #define DWT_CID0_PREAMBLE_GET(x)                 (((x) >> 0) & 0xff)

/** Component Identification Register 1 @multiple */
#define DWT_CID1_ADDR                                0xe0001ff4
/** Preamble @multiple */
  #define DWT_CID1_PREAMBLE_SHIFT                  0
  #define DWT_CID1_PREAMBLE(v)                     ((v) << 0)
  #define DWT_CID1_PREAMBLE_SET(x, v)              do { (x) = (((x) & ~0xf) | ((v) << 0)); } while(0)
  #define DWT_CID1_PREAMBLE_GET(x)                 (((x) >> 0) & 0xf)
/** Component class @multiple */
  #define DWT_CID1_COMPONENTCLASS_SHIFT            4
  #define DWT_CID1_COMPONENTCLASS(v)               ((DWT_CID1_COMPONENTCLASS_##v) << 4)
  #define DWT_CID1_COMPONENTCLASS_SET(x, v)        do { (x) = (((x) & ~0xf0) | ((DWT_CID1_COMPONENTCLASS_##v) << 4)); } while(0)
  #define DWT_CID1_COMPONENTCLASS_GET(x)           (((x) >> 4) & 0xf)
    #define DWT_CID1_COMPONENTCLASS_ROM_TABLE        0x00000001
    #define DWT_CID1_COMPONENTCLASS_CORESIGHT_COMPONENT 0x00000009
    #define DWT_CID1_COMPONENTCLASS_PRIMECELL_OF_SYSTEM_COMPONENT_WITH_NO_STANDARDIZED_REGISTER_LAYOUT_FOR_BACKWARD_COMPATIBILITY 0x0000000f

/** Component Identification Register 2 @multiple */
#define DWT_CID2_ADDR                                0xe0001ff8
/** Preamble @multiple */
  #define DWT_CID2_PREAMBLE_SHIFT                  0
  #define DWT_CID2_PREAMBLE(v)                     ((v) << 0)
  #define DWT_CID2_PREAMBLE_SET(x, v)              do { (x) = (((x) & ~0xff) | ((v) << 0)); } while(0)
  #define DWT_CID2_PREAMBLE_GET(x)                 (((x) >> 0) & 0xff)

/** Component Identification Register 3 @multiple */
#define DWT_CID3_ADDR                                0xe0001ffc
/** Preamble @multiple */
  #define DWT_CID3_PREAMBLE_SHIFT                  0
  #define DWT_CID3_PREAMBLE(v)                     ((v) << 0)
  #define DWT_CID3_PREAMBLE_SET(x, v)              do { (x) = (((x) & ~0xff) | ((v) << 0)); } while(0)
  #define DWT_CID3_PREAMBLE_GET(x)                 (((x) >> 0) & 0xff)

#endif

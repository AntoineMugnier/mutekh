/***************************************
* Auto generated by BFGen, do not edit *
***************************************/

/*
   bfgen -I /opt/bfgen/defs/arm/arm_v7m.bf -o cdefs cdefs_use_field_shift=1    \
     -O cpu/arm/include/cpu/arm32m/v7m.h
*/

#ifndef _ARMV7M_BFGEN_DEFS_
#define _ARMV7M_BFGEN_DEFS_

#define ARMV7M_MCR_ADDR                              0xe000e000

#define ARMV7M_ICTR_ADDR                             0xe000e004
/** Number of groups of 32 irq lines minus one @multiple */
  #define ARMV7M_ICTR_INTLINESNUM_SHIFT            0
  #define ARMV7M_ICTR_INTLINESNUM(v)               ((v) << 0)
  #define ARMV7M_ICTR_INTLINESNUM_SET(x, v)        do { (x) = (((x) & ~0xf) | ((v) << 0)); } while(0)
  #define ARMV7M_ICTR_INTLINESNUM_GET(x)           (((x) >> 0) & 0xf)

#define ARMV7M_ACTLR_ADDR                            0xe000e008

#define ARMV7M_SYST_CSR_ADDR                         0xe000e010
/** Set to enable counter @multiple */
  #define ARMV7M_SYST_CSR_ENABLE                   0x00000001
  #define ARMV7M_SYST_CSR_ENABLE_SHIFT             0
/** Set to enable SysTick irq @multiple */
  #define ARMV7M_SYST_CSR_TICKINT                  0x00000002
  #define ARMV7M_SYST_CSR_TICKINT_SHIFT            1
  #define ARMV7M_SYST_CSR_CLKSOURCE_SHIFT          2
  #define ARMV7M_SYST_CSR_CLKSOURCE(v)             ((ARMV7M_SYST_CSR_CLKSOURCE_##v) << 2)
  #define ARMV7M_SYST_CSR_CLKSOURCE_SET(x, v)      do { (x) = (((x) & ~0x4) | ((ARMV7M_SYST_CSR_CLKSOURCE_##v) << 2)); } while(0)
  #define ARMV7M_SYST_CSR_CLKSOURCE_GET(x)         (((x) >> 2) & 0x1)
    #define ARMV7M_SYST_CSR_CLKSOURCE_EXTERNAL       0x00000000
    #define ARMV7M_SYST_CSR_CLKSOURCE_CPU            0x00000001
  #define ARMV7M_SYST_CSR_COUNTFLAG                0x00010000
  #define ARMV7M_SYST_CSR_COUNTFLAG_SHIFT          16

#define ARMV7M_SYST_RVR_ADDR                         0xe000e014
  #define ARMV7M_SYST_RVR_RELOAD_SHIFT             0
  #define ARMV7M_SYST_RVR_RELOAD(v)                ((v) << 0)
  #define ARMV7M_SYST_RVR_RELOAD_SET(x, v)         do { (x) = (((x) & ~0xffffff) | ((v) << 0)); } while(0)
  #define ARMV7M_SYST_RVR_RELOAD_GET(x)            (((x) >> 0) & 0xffffff)

#define ARMV7M_SYST_CVR_ADDR                         0xe000e018
  #define ARMV7M_SYST_CVR_CURRENT_SHIFT            0
  #define ARMV7M_SYST_CVR_CURRENT(v)               ((v) << 0)
  #define ARMV7M_SYST_CVR_CURRENT_SET(x, v)        do { (x) = (((x) & ~0xffffffff) | ((v) << 0)); } while(0)
  #define ARMV7M_SYST_CVR_CURRENT_GET(x)           (((x) >> 0) & 0xffffffff)

#define ARMV7M_SYST_CALIB_ADDR                       0xe000e01c
  #define ARMV7M_SYST_CALIB_TENMS_SHIFT            0
  #define ARMV7M_SYST_CALIB_TENMS(v)               ((v) << 0)
  #define ARMV7M_SYST_CALIB_TENMS_SET(x, v)        do { (x) = (((x) & ~0xffffff) | ((v) << 0)); } while(0)
  #define ARMV7M_SYST_CALIB_TENMS_GET(x)           (((x) >> 0) & 0xffffff)
  #define ARMV7M_SYST_CALIB_SKEW                   0x40000000
  #define ARMV7M_SYST_CALIB_SKEW_SHIFT             30
  #define ARMV7M_SYST_CALIB_NOREF                  0x80000000
  #define ARMV7M_SYST_CALIB_NOREF_SHIFT            31

#define ARMV7M_NVIC_ISER_ADDR(ridx)                  (0xe000e100 + (ridx) * 4)
#define ARMV7M_NVIC_ISER_COUNT                       16
/** Enables, or reads the enabled state of one or more interrupts. @multiple */
  #define ARMV7M_NVIC_ISER_SETENA_COUNT            32
  #define ARMV7M_NVIC_ISER_SETENA(fidx)            (0x00000001 << ((fidx)))
  #define ARMV7M_NVIC_ISER_SETENA_SHIFT(fidx)      ((fidx) + 0)

#define ARMV7M_NVIC_ICER_ADDR(ridx)                  (0xe000e180 + (ridx) * 4)
#define ARMV7M_NVIC_ICER_COUNT                       16
/** Disables, or reads the enabled state of one or more interrupts. @multiple */
  #define ARMV7M_NVIC_ICER_CLRENA_COUNT            32
  #define ARMV7M_NVIC_ICER_CLRENA(fidx)            (0x00000001 << ((fidx)))
  #define ARMV7M_NVIC_ICER_CLRENA_SHIFT(fidx)      ((fidx) + 0)

#define ARMV7M_NVIC_ISPR_ADDR(ridx)                  (0xe000e200 + (ridx) * 4)
#define ARMV7M_NVIC_ISPR_COUNT                       16
/** On writes, sets the status of one or more interrupts to pending. On reads,
   shows the pending status of the interrupts. @multiple */
  #define ARMV7M_NVIC_ISPR_SETPEND_COUNT           32
  #define ARMV7M_NVIC_ISPR_SETPEND(fidx)           (0x00000001 << ((fidx)))
  #define ARMV7M_NVIC_ISPR_SETPEND_SHIFT(fidx)     ((fidx) + 0)

#define ARMV7M_NVIC_ICPR_ADDR(ridx)                  (0xe000e280 + (ridx) * 4)
#define ARMV7M_NVIC_ICPR_COUNT                       16
/** On writes, clears the status of one or more interrupts to pending. On reads,
   shows the pending status of the interrupts. @multiple */
  #define ARMV7M_NVIC_ICPR_CLRPEND_COUNT           32
  #define ARMV7M_NVIC_ICPR_CLRPEND(fidx)           (0x00000001 << ((fidx)))
  #define ARMV7M_NVIC_ICPR_CLRPEND_SHIFT(fidx)     ((fidx) + 0)

#define ARMV7M_NVIC_IABR_ADDR(ridx)                  (0xe000e300 + (ridx) * 4)
#define ARMV7M_NVIC_IABR_COUNT                       16
/** Current active state for the associated interrupt @multiple */
  #define ARMV7M_NVIC_IABR_ACTIVE_COUNT            32
  #define ARMV7M_NVIC_IABR_ACTIVE(fidx)            (0x00000001 << ((fidx)))
  #define ARMV7M_NVIC_IABR_ACTIVE_SHIFT(fidx)      ((fidx) + 0)

#define ARMV7M_NVIC_IPR_ADDR(ridx)                   (0xe000e400 + (ridx) * 4)
#define ARMV7M_NVIC_IPR_COUNT                        124
/** If an interrupt is not implemented, the corresponding PRI_Nx field can be
   RAZ/WI. @multiple */
  #define ARMV7M_NVIC_IPR_PRI_COUNT                4
  #define ARMV7M_NVIC_IPR_PRI_SHIFT(fidx)          ((fidx) * 8 + 0)
  #define ARMV7M_NVIC_IPR_PRI(fidx, v)             ((v) << ((fidx) * 8 + 0))
  #define ARMV7M_NVIC_IPR_PRI_SET(fidx, x, v)      do { (x) = (((x) & ~(0xff << ((fidx) * 8))) | ((v) << ((fidx) * 8 + 0))); } while(0)
  #define ARMV7M_NVIC_IPR_PRI_GET(fidx, x)         (((x) >> ((fidx) * 8 + 0)) & 0xff)

#define ARMV7M_CPUID_ADDR                            0xe000ed00
  #define ARMV7M_CPUID_REVISION_SHIFT              0
  #define ARMV7M_CPUID_REVISION(v)                 ((v) << 0)
  #define ARMV7M_CPUID_REVISION_SET(x, v)          do { (x) = (((x) & ~0xf) | ((v) << 0)); } while(0)
  #define ARMV7M_CPUID_REVISION_GET(x)             (((x) >> 0) & 0xf)
  #define ARMV7M_CPUID_PARTNO_SHIFT                4
  #define ARMV7M_CPUID_PARTNO(v)                   ((v) << 4)
  #define ARMV7M_CPUID_PARTNO_SET(x, v)            do { (x) = (((x) & ~0xfff0) | ((v) << 4)); } while(0)
  #define ARMV7M_CPUID_PARTNO_GET(x)               (((x) >> 4) & 0xfff)
  #define ARMV7M_CPUID_VARIANT_SHIFT               20
  #define ARMV7M_CPUID_VARIANT(v)                  ((v) << 20)
  #define ARMV7M_CPUID_VARIANT_SET(x, v)           do { (x) = (((x) & ~0xf00000) | ((v) << 20)); } while(0)
  #define ARMV7M_CPUID_VARIANT_GET(x)              (((x) >> 20) & 0xf)
  #define ARMV7M_CPUID_IMPLEM_SHIFT                24
  #define ARMV7M_CPUID_IMPLEM(v)                   ((v) << 24)
  #define ARMV7M_CPUID_IMPLEM_SET(x, v)            do { (x) = (((x) & ~0xff000000) | ((v) << 24)); } while(0)
  #define ARMV7M_CPUID_IMPLEM_GET(x)               (((x) >> 24) & 0xff)

#define ARMV7M_ICSR_ADDR                             0xe000ed04
  #define ARMV7M_ICSR_VECTACTIVE_SHIFT             0
  #define ARMV7M_ICSR_VECTACTIVE(v)                ((v) << 0)
  #define ARMV7M_ICSR_VECTACTIVE_SET(x, v)         do { (x) = (((x) & ~0x1ff) | ((v) << 0)); } while(0)
  #define ARMV7M_ICSR_VECTACTIVE_GET(x)            (((x) >> 0) & 0x1ff)
  #define ARMV7M_ICSR_RETTOBASE                    0x00000800
  #define ARMV7M_ICSR_RETTOBASE_SHIFT              11
  #define ARMV7M_ICSR_VECTPENDING_SHIFT            12
  #define ARMV7M_ICSR_VECTPENDING(v)               ((v) << 12)
  #define ARMV7M_ICSR_VECTPENDING_SET(x, v)        do { (x) = (((x) & ~0x1ff000) | ((v) << 12)); } while(0)
  #define ARMV7M_ICSR_VECTPENDING_GET(x)           (((x) >> 12) & 0x1ff)
  #define ARMV7M_ICSR_ISRPENDING                   0x00400000
  #define ARMV7M_ICSR_ISRPENDING_SHIFT             22
  #define ARMV7M_ICSR_ISRPREEMPT                   0x00800000
  #define ARMV7M_ICSR_ISRPREEMPT_SHIFT             23
  #define ARMV7M_ICSR_PENDSTCLR                    0x02000000
  #define ARMV7M_ICSR_PENDSTCLR_SHIFT              25
  #define ARMV7M_ICSR_PENDSTSET                    0x04000000
  #define ARMV7M_ICSR_PENDSTSET_SHIFT              26
  #define ARMV7M_ICSR_PENDSVCLR                    0x08000000
  #define ARMV7M_ICSR_PENDSVCLR_SHIFT              27
  #define ARMV7M_ICSR_PENDSVSET                    0x10000000
  #define ARMV7M_ICSR_PENDSVSET_SHIFT              28
  #define ARMV7M_ICSR_NMIPENDSET                   0x80000000
  #define ARMV7M_ICSR_NMIPENDSET_SHIFT             31

#define ARMV7M_VTOR_ADDR                             0xe000ed08
  #define ARMV7M_VTOR_TBLOFF_SHIFT                 7
  #define ARMV7M_VTOR_TBLOFF(v)                    ((v) << 7)
  #define ARMV7M_VTOR_TBLOFF_SET(x, v)             do { (x) = (((x) & ~0xffffff80) | ((v) << 7)); } while(0)
  #define ARMV7M_VTOR_TBLOFF_GET(x)                (((x) >> 7) & 0x1ffffff)

#define ARMV7M_AIRCR_ADDR                            0xe000ed0c

#define ARMV7M_SCR_ADDR                              0xe000ed10
  #define ARMV7M_SCR_SLEEPONNEXIT                  0x00000002
  #define ARMV7M_SCR_SLEEPONNEXIT_SHIFT            1
  #define ARMV7M_SCR_SLEEPDEEP                     0x00000004
  #define ARMV7M_SCR_SLEEPDEEP_SHIFT               2
  #define ARMV7M_SCR_SEVONPEND                     0x00000010
  #define ARMV7M_SCR_SEVONPEND_SHIFT               4

#define ARMV7M_CCR_ADDR                              0xe000ed14

#define ARMV7M_SHPR_ADDR(ridx)                       (0xe000ed18 + (ridx) * 4)
#define ARMV7M_SHPR_COUNT                            3

#define ARMV7M_SHCSR_ADDR                            0xe000ed24
  #define ARMV7M_SHCSR_MEMFAULTACT                 0x00000001
  #define ARMV7M_SHCSR_MEMFAULTACT_SHIFT           0
  #define ARMV7M_SHCSR_BUSFAULTACT                 0x00000002
  #define ARMV7M_SHCSR_BUSFAULTACT_SHIFT           1
  #define ARMV7M_SHCSR_USGFAULTACT                 0x00000008
  #define ARMV7M_SHCSR_USGFAULTACT_SHIFT           3
  #define ARMV7M_SHCSR_SVCALLACT                   0x00000080
  #define ARMV7M_SHCSR_SVCALLACT_SHIFT             7
  #define ARMV7M_SHCSR_MONITORACT                  0x00000100
  #define ARMV7M_SHCSR_MONITORACT_SHIFT            8
  #define ARMV7M_SHCSR_PENDSVACT                   0x00000400
  #define ARMV7M_SHCSR_PENDSVACT_SHIFT             10
  #define ARMV7M_SHCSR_SYSTICKACT                  0x00000800
  #define ARMV7M_SHCSR_SYSTICKACT_SHIFT            11
  #define ARMV7M_SHCSR_USGFAULTPENDED              0x00001000
  #define ARMV7M_SHCSR_USGFAULTPENDED_SHIFT        12
  #define ARMV7M_SHCSR_MEMFAULTPENDED              0x00002000
  #define ARMV7M_SHCSR_MEMFAULTPENDED_SHIFT        13
  #define ARMV7M_SHCSR_BUSFAULTPENDED              0x00004000
  #define ARMV7M_SHCSR_BUSFAULTPENDED_SHIFT        14
  #define ARMV7M_SHCSR_SVCALLPENDED                0x00008000
  #define ARMV7M_SHCSR_SVCALLPENDED_SHIFT          15
  #define ARMV7M_SHCSR_MEMFAULTENA                 0x00010000
  #define ARMV7M_SHCSR_MEMFAULTENA_SHIFT           16
  #define ARMV7M_SHCSR_BUSFAULTENA                 0x00020000
  #define ARMV7M_SHCSR_BUSFAULTENA_SHIFT           17
  #define ARMV7M_SHCSR_USGFAULTENA                 0x00040000
  #define ARMV7M_SHCSR_USGFAULTENA_SHIFT           18

#define ARMV7M_CFSR_ADDR                             0xe000ed28
  #define ARMV7M_CFSR_MM_IACCVIOL                  0x00000001
  #define ARMV7M_CFSR_MM_IACCVIOL_SHIFT            0
  #define ARMV7M_CFSR_MM_DACCVIOL                  0x00000002
  #define ARMV7M_CFSR_MM_DACCVIOL_SHIFT            1
  #define ARMV7M_CFSR_MM_MUNSTKER                  0x00000008
  #define ARMV7M_CFSR_MM_MUNSTKER_SHIFT            3
  #define ARMV7M_CFSR_MM_MSTKER                    0x00000010
  #define ARMV7M_CFSR_MM_MSTKER_SHIFT              4
  #define ARMV7M_CFSR_MM_MLSPERR                   0x00000020
  #define ARMV7M_CFSR_MM_MLSPERR_SHIFT             5
  #define ARMV7M_CFSR_MM_ARVALID                   0x00000040
  #define ARMV7M_CFSR_MM_ARVALID_SHIFT             6
  #define ARMV7M_CFSR_BUS_IBUSERR                  0x00000100
  #define ARMV7M_CFSR_BUS_IBUSERR_SHIFT            8
  #define ARMV7M_CFSR_BUS_PRECISERR                0x00000200
  #define ARMV7M_CFSR_BUS_PRECISERR_SHIFT          9
  #define ARMV7M_CFSR_BUS_IMPRECISERR              0x00000400
  #define ARMV7M_CFSR_BUS_IMPRECISERR_SHIFT        10
  #define ARMV7M_CFSR_BUS_UNSTKERR                 0x00000800
  #define ARMV7M_CFSR_BUS_UNSTKERR_SHIFT           11
  #define ARMV7M_CFSR_BUS_STKERR                   0x00001000
  #define ARMV7M_CFSR_BUS_STKERR_SHIFT             12
  #define ARMV7M_CFSR_BUS_LSPERR                   0x00002000
  #define ARMV7M_CFSR_BUS_LSPERR_SHIFT             13
  #define ARMV7M_CFSR_BUS_ARVALID                  0x00008000
  #define ARMV7M_CFSR_BUS_ARVALID_SHIFT            15
  #define ARMV7M_CFSR_USG_UNDEFINSTR               0x00010000
  #define ARMV7M_CFSR_USG_UNDEFINSTR_SHIFT         16
  #define ARMV7M_CFSR_USG_INVSTATE                 0x00020000
  #define ARMV7M_CFSR_USG_INVSTATE_SHIFT           17
  #define ARMV7M_CFSR_USG_INVPC                    0x00040000
  #define ARMV7M_CFSR_USG_INVPC_SHIFT              18
  #define ARMV7M_CFSR_USG_NOCP                     0x00080000
  #define ARMV7M_CFSR_USG_NOCP_SHIFT               19
  #define ARMV7M_CFSR_USG_UNALIGNED                0x01000000
  #define ARMV7M_CFSR_USG_UNALIGNED_SHIFT          24
  #define ARMV7M_CFSR_USG_DIVBYZERO                0x02000000
  #define ARMV7M_CFSR_USG_DIVBYZERO_SHIFT          25

#define ARMV7M_HFSR_ADDR                             0xe000ed2c

#define ARMV7M_DFSR_ADDR                             0xe000ed30

#define ARMV7M_MMFAR_ADDR                            0xe000ed34

#define ARMV7M_BFAR_ADDR                             0xe000ed38

#define ARMV7M_AFSR_ADDR                             0xe000ed3c

#define ARMV7M_CPACR_ADDR                            0xe000ed88

#define ARMV7M_MPU_TYPE_ADDR                         0xe000ed90
  #define ARMV7M_MPU_TYPE_SEPARATE                 0x00000001
  #define ARMV7M_MPU_TYPE_SEPARATE_SHIFT           0
  #define ARMV7M_MPU_TYPE_DREGION_SHIFT            8
  #define ARMV7M_MPU_TYPE_DREGION(v)               ((v) << 8)
  #define ARMV7M_MPU_TYPE_DREGION_SET(x, v)        do { (x) = (((x) & ~0xff00) | ((v) << 8)); } while(0)
  #define ARMV7M_MPU_TYPE_DREGION_GET(x)           (((x) >> 8) & 0xff)
  #define ARMV7M_MPU_TYPE_IREGION_SHIFT            16
  #define ARMV7M_MPU_TYPE_IREGION(v)               ((v) << 16)
  #define ARMV7M_MPU_TYPE_IREGION_SET(x, v)        do { (x) = (((x) & ~0xff0000) | ((v) << 16)); } while(0)
  #define ARMV7M_MPU_TYPE_IREGION_GET(x)           (((x) >> 16) & 0xff)

#define ARMV7M_MPU_CTRL_ADDR                         0xe000ed94
/** Enable memory protection @multiple */
  #define ARMV7M_MPU_CTRL_ENABLE                   0x00000001
  #define ARMV7M_MPU_CTRL_ENABLE_SHIFT             0
/** MPU enabled during Fault handlers @multiple */
  #define ARMV7M_MPU_CTRL_HFNMIENA                 0x00000002
  #define ARMV7M_MPU_CTRL_HFNMIENA_SHIFT           1
/** Enable default background memory region @multiple */
  #define ARMV7M_MPU_CTRL_PRIVDEFENA               0x00000004
  #define ARMV7M_MPU_CTRL_PRIVDEFENA_SHIFT         2

#define ARMV7M_MPU_RNR_ADDR                          0xe000ed98
/** select memory region for RBAR and RASR @multiple */
  #define ARMV7M_MPU_RNR_REGION_SHIFT              0
  #define ARMV7M_MPU_RNR_REGION(v)                 ((v) << 0)
  #define ARMV7M_MPU_RNR_REGION_SET(x, v)          do { (x) = (((x) & ~0xff) | ((v) << 0)); } while(0)
  #define ARMV7M_MPU_RNR_REGION_GET(x)             (((x) >> 0) & 0xff)

#define ARMV7M_MPU_RBAR_ADDR                         0xe000ed9c
  #define ARMV7M_MPU_RBAR_REGION_SHIFT             0
  #define ARMV7M_MPU_RBAR_REGION(v)                ((v) << 0)
  #define ARMV7M_MPU_RBAR_REGION_SET(x, v)         do { (x) = (((x) & ~0xf) | ((v) << 0)); } while(0)
  #define ARMV7M_MPU_RBAR_REGION_GET(x)            (((x) >> 0) & 0xf)
  #define ARMV7M_MPU_RBAR_VALID                    0x00000010
  #define ARMV7M_MPU_RBAR_VALID_SHIFT              4
  #define ARMV7M_MPU_RBAR_ADDRESS_SHIFT            5
  #define ARMV7M_MPU_RBAR_ADDRESS(v)               ((v) << 5)
  #define ARMV7M_MPU_RBAR_ADDRESS_SET(x, v)        do { (x) = (((x) & ~0xffffffe0) | ((v) << 5)); } while(0)
  #define ARMV7M_MPU_RBAR_ADDRESS_GET(x)           (((x) >> 5) & 0x7ffffff)

#define ARMV7M_MPU_RASR_ADDR                         0xe000eda0
  #define ARMV7M_MPU_RASR_ENABLE                   0x00000001
  #define ARMV7M_MPU_RASR_ENABLE_SHIFT             0
  #define ARMV7M_MPU_RASR_SIZE_SHIFT               1
  #define ARMV7M_MPU_RASR_SIZE(v)                  ((v) << 1)
  #define ARMV7M_MPU_RASR_SIZE_SET(x, v)           do { (x) = (((x) & ~0x3e) | ((v) << 1)); } while(0)
  #define ARMV7M_MPU_RASR_SIZE_GET(x)              (((x) >> 1) & 0x1f)
  #define ARMV7M_MPU_RASR_SRD_SHIFT                8
  #define ARMV7M_MPU_RASR_SRD(v)                   ((v) << 8)
  #define ARMV7M_MPU_RASR_SRD_SET(x, v)            do { (x) = (((x) & ~0xff00) | ((v) << 8)); } while(0)
  #define ARMV7M_MPU_RASR_SRD_GET(x)               (((x) >> 8) & 0xff)
  #define ARMV7M_MPU_RASR_B                        0x00010000
  #define ARMV7M_MPU_RASR_B_SHIFT                  16
  #define ARMV7M_MPU_RASR_C                        0x00020000
  #define ARMV7M_MPU_RASR_C_SHIFT                  17
  #define ARMV7M_MPU_RASR_S                        0x00040000
  #define ARMV7M_MPU_RASR_S_SHIFT                  18
  #define ARMV7M_MPU_RASR_TEX_SHIFT                19
  #define ARMV7M_MPU_RASR_TEX(v)                   ((v) << 19)
  #define ARMV7M_MPU_RASR_TEX_SET(x, v)            do { (x) = (((x) & ~0x380000) | ((v) << 19)); } while(0)
  #define ARMV7M_MPU_RASR_TEX_GET(x)               (((x) >> 19) & 0x7)
  #define ARMV7M_MPU_RASR_AP_SHIFT                 24
  #define ARMV7M_MPU_RASR_AP(v)                    ((v) << 24)
  #define ARMV7M_MPU_RASR_AP_SET(x, v)             do { (x) = (((x) & ~0x7000000) | ((v) << 24)); } while(0)
  #define ARMV7M_MPU_RASR_AP_GET(x)                (((x) >> 24) & 0x7)
  #define ARMV7M_MPU_RASR_XN                       0x10000000
  #define ARMV7M_MPU_RASR_XN_SHIFT                 28

#define ARMV7M_MPU_RBAR_A_ADDR(ridx)                 (0xe000eda4 + (ridx) * 8)
#define ARMV7M_MPU_RBAR_A_COUNT                      3

#define ARMV7M_MPU_RASR_A_ADDR(ridx)                 (0xe000eda8 + (ridx) * 8)
#define ARMV7M_MPU_RASR_A_COUNT                      3

#define ARMV7M_STIR_ADDR                             0xe000ef00

#define ARMV7M_FPCCR_ADDR                            0xe000ef34

#define ARMV7M_FPCAR_ADDR                            0xe000ef38

#define ARMV7M_FPDCSR_ADDR                           0xe000ef3c

#define ARMV7M_MVFR0_ADDR                            0xe000ef40

#define ARMV7M_MVFR1_ADDR                            0xe000ef44

/** Processor Identification @multiple */
#define ARMV7M_PID4_ADDR                             0xe000efd0

/** Processor Identification @multiple */
#define ARMV7M_PID5_ADDR                             0xe000efd4

/** Processor Identification @multiple */
#define ARMV7M_PID6_ADDR                             0xe000efd8

/** Processor Identification @multiple */
#define ARMV7M_PID7_ADDR                             0xe000efdc

/** Processor Identification @multiple */
#define ARMV7M_PID0_ADDR                             0xe000efe0

/** Processor Identification @multiple */
#define ARMV7M_PID1_ADDR                             0xe000efe4

/** Processor Identification @multiple */
#define ARMV7M_PID2_ADDR                             0xe000efe8

/** Processor Identification @multiple */
#define ARMV7M_PID3_ADDR                             0xe000efec

/** Component Identification @multiple */
#define ARMV7M_CID0_ADDR                             0xe000eff0

/** Component Identification @multiple */
#define ARMV7M_CID1_ADDR                             0xe000eff4

/** Component Identification @multiple */
#define ARMV7M_CID2_ADDR                             0xe000eff8

/** Component Identification @multiple */
#define ARMV7M_CID3_ADDR                             0xe000effc

#endif


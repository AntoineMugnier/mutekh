/***************************************
* Auto generated by BFGen, do not edit *
***************************************/

/*
   bfgen -I pl390icu_gicd.bitfield -o cdefs cdefs_sfx_shifted_field_mask= cdefs_sfx_field_shifted= cdefs_use_reg_comment=5 -O pl390icu_gicd_regs.h
*/

#ifndef _PL390_GICD_BFGEN_DEFS_
#define _PL390_GICD_BFGEN_DEFS_

/** Defines the address of the @em{distributor control} 32 bits register. @multiple */
#define PL390_GICD_CTRL_ADDR                         0x00000000
  #define PL390_GICD_CTRL_ENABLE                     0x00000001

/** Defines the address of the @em{interrupt controller type} 32 bits register. @multiple */
#define PL390_GICD_TYPER_ADDR                        0x00000004
  #define PL390_GICD_TYPER_ITLINES(x)                ((x) << 0)
  #define PL390_GICD_TYPER_ITLINES_GET(x)            (((x) >> 0) & 0x1f)
  #define PL390_GICD_TYPER_CPUNUMBER(x)              ((x) << 5)
  #define PL390_GICD_TYPER_CPUNUMBER_GET(x)          (((x) >> 5) & 0x7)
  #define PL390_GICD_TYPER_SECUREXT                  0x00000400
  #define PL390_GICD_TYPER_LSPI(x)                   ((x) << 11)
  #define PL390_GICD_TYPER_LSPI_GET(x)               (((x) >> 11) & 0x1f)

/** Defines the address of the @em{distributor implementer indentification} 32 bits register. @multiple */
#define PL390_GICD_IIDR_ADDR                         0x00000008
  #define PL390_GICD_IIDR_IMP(x)                     ((x) << 0)
  #define PL390_GICD_IIDR_IMP_GET(x)                 (((x) >> 0) & 0xfff)
  #define PL390_GICD_IIDR_REV(x)                     ((x) << 12)
  #define PL390_GICD_IIDR_REV_GET(x)                 (((x) >> 12) & 0xf)
  #define PL390_GICD_IIDR_VAR(x)                     ((x) << 16)
  #define PL390_GICD_IIDR_VAR_GET(x)                 (((x) >> 16) & 0xf)
  #define PL390_GICD_IIDR_PID(x)                     ((x) << 24)
  #define PL390_GICD_IIDR_PID_GET(x)                 (((x) >> 24) & 0xff)

/** Defines the address of the @em{interrupt group} 32 bits register. @multiple */
#define PL390_GICD_IGGROUP_ADDR                      0x00000080
  #define PL390_GICD_IGGROUP_GROUP_COUNT             32
  #define PL390_GICD_IGGROUP_GROUP(fidx)             (0x00000001 << (fidx))
    #define PL390_GICD_IGGROUP_GROUP_GROUP_1     0x1
    #define PL390_GICD_IGGROUP_GROUP_GROUP_0     0x0

/** Defines the address of the @em{interrupt set-enable} 32 bits register. @multiple */
#define PL390_GICD_ISENABLER_ADDR(ridx)              (0x00000100 + (ridx) * 4)
#define PL390_GICD_ISENABLER_COUNT                   32
  #define PL390_GICD_ISENABLER_IRQ_COUNT             32
  #define PL390_GICD_ISENABLER_IRQ(fidx)             (0x00000001 << (fidx))
    #define PL390_GICD_ISENABLER_IRQ_ENABLE      0x1

/** Defines the address of the @em{interrupt clear-enable} 32 bits register. @multiple */
#define PL390_GICD_ICENABLER_ADDR(ridx)              (0x00000180 + (ridx) * 4)
#define PL390_GICD_ICENABLER_COUNT                   32
  #define PL390_GICD_ICENABLER_IRQ_COUNT             32
  #define PL390_GICD_ICENABLER_IRQ(fidx)             (0x00000001 << (fidx))
    #define PL390_GICD_ICENABLER_IRQ_DISABLE     0x1

/** Defines the address of the @em{interrupt set-pending} 32 bits register. @multiple */
#define PL390_GICD_ISPENDR_ADDR(ridx)                (0x00000200 + (ridx) * 4)
#define PL390_GICD_ISPENDR_COUNT                     32
  #define PL390_GICD_ISPENDR_IRQ_COUNT               32
  #define PL390_GICD_ISPENDR_IRQ(fidx)               (0x00000001 << (fidx))

/** Defines the address of the @em{interrupt clear-pending} 32 bits register. @multiple */
#define PL390_GICD_ICPENDR_ADDR(ridx)                (0x00000280 + (ridx) * 4)
#define PL390_GICD_ICPENDR_COUNT                     32
  #define PL390_GICD_ICPENDR_IRQ_COUNT               32
  #define PL390_GICD_ICPENDR_IRQ(fidx)               (0x00000001 << (fidx))

/** Defines the address of the @em{interrupt set-active} 32 bits register. @multiple */
#define PL390_GICD_ISACTIVER_ADDR(ridx)              (0x00000300 + (ridx) * 4)
#define PL390_GICD_ISACTIVER_COUNT                   32
  #define PL390_GICD_ISACTIVER_IRQ_COUNT             32
  #define PL390_GICD_ISACTIVER_IRQ(fidx)             (0x00000001 << (fidx))

/** Defines the address of the @em{interrupt clear-active} 32 bits register. @multiple */
#define PL390_GICD_ICACTIVER_ADDR(ridx)              (0x00000380 + (ridx) * 4)
#define PL390_GICD_ICACTIVER_COUNT                   32
  #define PL390_GICD_ICACTIVER_IRQ_COUNT             32
  #define PL390_GICD_ICACTIVER_IRQ(fidx)             (0x00000001 << (fidx))

/** Defines the address of the @em{interrupt priority} 32 bits register. @multiple */
#define PL390_GICD_IPRIORITY_ADDR(ridx)              (0x00000400 + (ridx) * 4)
#define PL390_GICD_IPRIORITY_COUNT                   256
  #define PL390_GICD_IPRIORITY_IRQ_COUNT             4
  #define PL390_GICD_IPRIORITY_IRQ(fidx, x)          ((x) << ((fidx) + 0))
  #define PL390_GICD_IPRIORITY_IRQ_GET(fidx, x)      (((x) >> ((fidx) + 0)) & 0xff)

/** Defines the address of the @em{interrupt processor targets} 32 bits register. @multiple */
#define PL390_GICD_ITARGETSR_ADDR(ridx)              (0x00000800 + (ridx) * 4)
#define PL390_GICD_ITARGETSR_COUNT                   256
  #define PL390_GICD_ITARGETSR_IRQ_COUNT             4
  #define PL390_GICD_ITARGETSR_IRQ(fidx, x)          ((x) << ((fidx) + 0))
  #define PL390_GICD_ITARGETSR_IRQ_GET(fidx, x)      (((x) >> ((fidx) + 0)) & 0xff)

/** Defines the address of the @em{interrupt configuration} 32 bits register. @multiple */
#define PL390_GICD_ICFGR_ADDR(ridx)                  (0x00000c00 + (ridx) * 4)
#define PL390_GICD_ICFGR_COUNT                       64
  #define PL390_GICD_ICFGR_IRQ_COUNT                 16
  #define PL390_GICD_ICFGR_IRQ(fidx, x)              ((x) << ((fidx) + 0))
  #define PL390_GICD_ICFGR_IRQ_GET(fidx, x)          (((x) >> ((fidx) + 0)) & 0x3)

/** Defines the address of the @em{interrupt pin status} 32 bits register. @multiple */
#define PL390_GICD_ISTATUS_ADDR(ridx)                (0x00000d00 + (ridx) * 4)
#define PL390_GICD_ISTATUS_COUNT                     32
  #define PL390_GICD_ISTATUS_IRQ_COUNT               32
  #define PL390_GICD_ISTATUS_IRQ(fidx)               (0x00000001 << (fidx))

/** Defines the address of the @em{non-secur access control} 32 bits register. @multiple */
#define PL390_GICD_NSACR_ADDR(ridx)                  (0x00000e00 + (ridx) * 4)
#define PL390_GICD_NSACR_COUNT                       64
  #define PL390_GICD_NSACR_IRQ_COUNT                 16
  #define PL390_GICD_NSACR_IRQ(fidx, x)              ((x) << ((fidx) + 0))
  #define PL390_GICD_NSACR_IRQ_GET(fidx, x)          (((x) >> ((fidx) + 0)) & 0x3)

/** Defines the address of the @em{software generated interrupt} 32 bits register. @multiple */
#define PL390_GICD_SGIR_ADDR                         0x00000f00
  #define PL390_GICD_SGIR_SGIINTID(x)                ((x) << 0)
  #define PL390_GICD_SGIR_SGIINTID_GET(x)            (((x) >> 0) & 0xf)
  #define PL390_GICD_SGIR_NSATT                      0x00008000
  #define PL390_GICD_SGIR_TGTLIST(x)                 ((x) << 16)
  #define PL390_GICD_SGIR_TGTLIST_GET(x)             (((x) >> 16) & 0xff)
  #define PL390_GICD_SGIR_TGTFILT(x)                 ((x) << 24)
  #define PL390_GICD_SGIR_TGTFILT_GET(x)             (((x) >> 24) & 0x3)

/** Defines the address of the @em{sgi clear-pending} 32 bits register. @multiple */
#define PL390_GICD_CPENDSGIR_ADDR(ridx)              (0x00000f10 + (ridx) * 4)
#define PL390_GICD_CPENDSGIR_COUNT                   4
  #define PL390_GICD_CPENDSGIR_SGI_COUNT             4
  #define PL390_GICD_CPENDSGIR_SGI(fidx, x)          ((x) << ((fidx) + 0))
  #define PL390_GICD_CPENDSGIR_SGI_GET(fidx, x)      (((x) >> ((fidx) + 0)) & 0xff)

/** Defines the address of the @em{sgi set-pending} 32 bits register. @multiple */
#define PL390_GICD_SPENDSGIR_ADDR(ridx)              (0x00000f20 + (ridx) * 4)
#define PL390_GICD_SPENDSGIR_COUNT                   4
  #define PL390_GICD_SPENDSGIR_SGI_COUNT             4
  #define PL390_GICD_SPENDSGIR_SGI(fidx, x)          ((x) << ((fidx) + 0))
  #define PL390_GICD_SPENDSGIR_SGI_GET(fidx, x)      (((x) >> ((fidx) + 0)) & 0xff)

#endif


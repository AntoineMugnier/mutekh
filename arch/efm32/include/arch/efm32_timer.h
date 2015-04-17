/***************************************
* Auto generated by BFGen, do not edit *
***************************************/

/*
   bfgen -o cdefs -O arch/efm32/include/arch/efm32_timer.h cdefs_use_reg_mask=1 \
     cdefs_use_field_setval=1
*/

#ifndef _EFM32_TIMER_BFGEN_DEFS_
#define _EFM32_TIMER_BFGEN_DEFS_

#define EFM32_TIMER_CTRL_ADDR                        0x00000000
#define EFM32_TIMER_CTRL_MASK                        0x3f032ffb
/** These bit set the counting mode for the Timer. Note, when Quadrature Decoder
   Mode is selected (MODE = 'b11), the CLKSEL is don't care. The Timer is clocked
   by the Decoder Mode clock output. @multiple */
  #define EFM32_TIMER_CTRL_MODE(v)                   ((EFM32_TIMER_CTRL_MODE_##v) << 0)
  #define EFM32_TIMER_CTRL_MODE_SET(x, v)            do { (x) = (((x) & ~0x3) | ((EFM32_TIMER_CTRL_MODE_##v) << 0)); } while(0)
  #define EFM32_TIMER_CTRL_MODE_SETVAL(x, v)         do { (x) = (((x) & ~0x3) | ((v) << 0)); } while(0)
  #define EFM32_TIMER_CTRL_MODE_GET(x)               (((x) >> 0) & 0x3)
    #define EFM32_TIMER_CTRL_MODE_UP                   0x00000000
    #define EFM32_TIMER_CTRL_MODE_DOWN                 0x00000001
    #define EFM32_TIMER_CTRL_MODE_UPDOWN               0x00000002
    #define EFM32_TIMER_CTRL_MODE_QDEC                 0x00000003
/** When this bit is set, the Timer is started/stopped/reloaded by
   start/stop/reload commands in the other timers. @multiple */
  #define EFM32_TIMER_CTRL_SYNC(v)                   ((EFM32_TIMER_CTRL_SYNC_##v) << 3)
  #define EFM32_TIMER_CTRL_SYNC_SET(x, v)            do { (x) = (((x) & ~0x8) | ((EFM32_TIMER_CTRL_SYNC_##v) << 3)); } while(0)
  #define EFM32_TIMER_CTRL_SYNC_SETVAL(x, v)         do { (x) = (((x) & ~0x8) | ((v) << 3)); } while(0)
  #define EFM32_TIMER_CTRL_SYNC_GET(x)               (((x) >> 3) & 0x1)
    #define EFM32_TIMER_CTRL_SYNC_NONE                 0x00000000
    #define EFM32_TIMER_CTRL_SYNC_SYNC                 0x00000001
/** Thisble/disable one shot mode. @multiple */
  #define EFM32_TIMER_CTRL_OSMEN                     0x00000010
/** This bit sets the mode for the quadrature decoder. @multiple */
  #define EFM32_TIMER_CTRL_QDM(v)                    ((EFM32_TIMER_CTRL_QDM_##v) << 5)
  #define EFM32_TIMER_CTRL_QDM_SET(x, v)             do { (x) = (((x) & ~0x20) | ((EFM32_TIMER_CTRL_QDM_##v) << 5)); } while(0)
  #define EFM32_TIMER_CTRL_QDM_SETVAL(x, v)          do { (x) = (((x) & ~0x20) | ((v) << 5)); } while(0)
  #define EFM32_TIMER_CTRL_QDM_GET(x)                (((x) >> 5) & 0x1)
    #define EFM32_TIMER_CTRL_QDM_X2                    0x00000000
    #define EFM32_TIMER_CTRL_QDM_X4                    0x00000001
/** Set this bit to enable timer to run in debug mode. @multiple */
  #define EFM32_TIMER_CTRL_DEBUGRUN(v)               ((EFM32_TIMER_CTRL_DEBUGRUN_##v) << 6)
  #define EFM32_TIMER_CTRL_DEBUGRUN_SET(x, v)        do { (x) = (((x) & ~0x40) | ((EFM32_TIMER_CTRL_DEBUGRUN_##v) << 6)); } while(0)
  #define EFM32_TIMER_CTRL_DEBUGRUN_SETVAL(x, v)     do { (x) = (((x) & ~0x40) | ((v) << 6)); } while(0)
  #define EFM32_TIMER_CTRL_DEBUGRUN_GET(x)           (((x) >> 6) & 0x1)
    #define EFM32_TIMER_CTRL_DEBUGRUN_FROZEN           0x00000000
    #define EFM32_TIMER_CTRL_DEBUGRUN_DEBUG            0x00000001
/** When this bit is set, the DMA requests are cleared when the corresponding DMA
   channel is active. This enables the timer DMA requests to be cleared without
   accessing the timer. @multiple */
  #define EFM32_TIMER_CTRL_DMACLRACT                 0x00000080
/** These bits select the action taken in the counter when a rising edge occurs
   on the input. @multiple */
  #define EFM32_TIMER_CTRL_RISEA(v)                  ((EFM32_TIMER_CTRL_RISEA_##v) << 8)
  #define EFM32_TIMER_CTRL_RISEA_SET(x, v)           do { (x) = (((x) & ~0x300) | ((EFM32_TIMER_CTRL_RISEA_##v) << 8)); } while(0)
  #define EFM32_TIMER_CTRL_RISEA_SETVAL(x, v)        do { (x) = (((x) & ~0x300) | ((v) << 8)); } while(0)
  #define EFM32_TIMER_CTRL_RISEA_GET(x)              (((x) >> 8) & 0x3)
    #define EFM32_TIMER_CTRL_RISEA_NONE                0x00000000
    #define EFM32_TIMER_CTRL_RISEA_START               0x00000001
    #define EFM32_TIMER_CTRL_RISEA_STOP                0x00000002
    #define EFM32_TIMER_CTRL_RISEA_RELOADSTART         0x00000003
/** These bits select the action taken in the counter when a falling edge occurs
   on the input. @multiple */
  #define EFM32_TIMER_CTRL_FALLA(v)                  ((EFM32_TIMER_CTRL_FALLA_##v) << 10)
  #define EFM32_TIMER_CTRL_FALLA_SET(x, v)           do { (x) = (((x) & ~0xc00) | ((EFM32_TIMER_CTRL_FALLA_##v) << 10)); } while(0)
  #define EFM32_TIMER_CTRL_FALLA_SETVAL(x, v)        do { (x) = (((x) & ~0xc00) | ((v) << 10)); } while(0)
  #define EFM32_TIMER_CTRL_FALLA_GET(x)              (((x) >> 10) & 0x3)
    #define EFM32_TIMER_CTRL_FALLA_NONE                0x00000000
    #define EFM32_TIMER_CTRL_FALLA_START               0x00000001
    #define EFM32_TIMER_CTRL_FALLA_STOP                0x00000002
    #define EFM32_TIMER_CTRL_FALLA_RELOADSTART         0x00000003
/** Enable 2x count mode. @multiple */
  #define EFM32_TIMER_CTRL_X2CNT                     0x00002000
/** These bits select the clock source for the timer. @multiple */
  #define EFM32_TIMER_CTRL_CLKSEL(v)                 ((EFM32_TIMER_CTRL_CLKSEL_##v) << 16)
  #define EFM32_TIMER_CTRL_CLKSEL_SET(x, v)          do { (x) = (((x) & ~0x30000) | ((EFM32_TIMER_CTRL_CLKSEL_##v) << 16)); } while(0)
  #define EFM32_TIMER_CTRL_CLKSEL_SETVAL(x, v)       do { (x) = (((x) & ~0x30000) | ((v) << 16)); } while(0)
  #define EFM32_TIMER_CTRL_CLKSEL_GET(x)             (((x) >> 16) & 0x3)
    #define EFM32_TIMER_CTRL_CLKSEL_PRESCHFPERCLK      0x00000000
    #define EFM32_TIMER_CTRL_CLKSEL_CC1                0x00000001
    #define EFM32_TIMER_CTRL_CLKSEL_TIMEROUF           0x00000002
/** These bits select the prescaling factor. @multiple */
  #define EFM32_TIMER_CTRL_PRESC(v)                  ((EFM32_TIMER_CTRL_PRESC_##v) << 24)
  #define EFM32_TIMER_CTRL_PRESC_SET(x, v)           do { (x) = (((x) & ~0xf000000) | ((EFM32_TIMER_CTRL_PRESC_##v) << 24)); } while(0)
  #define EFM32_TIMER_CTRL_PRESC_SETVAL(x, v)        do { (x) = (((x) & ~0xf000000) | ((v) << 24)); } while(0)
  #define EFM32_TIMER_CTRL_PRESC_GET(x)              (((x) >> 24) & 0xf)
    #define EFM32_TIMER_CTRL_PRESC_DIV1                0x00000000
    #define EFM32_TIMER_CTRL_PRESC_DIV2                0x00000001
    #define EFM32_TIMER_CTRL_PRESC_DIV4                0x00000002
    #define EFM32_TIMER_CTRL_PRESC_DIV8                0x00000003
    #define EFM32_TIMER_CTRL_PRESC_DIV16               0x00000004
    #define EFM32_TIMER_CTRL_PRESC_DIV32               0x00000005
    #define EFM32_TIMER_CTRL_PRESC_DIV64               0x00000006
    #define EFM32_TIMER_CTRL_PRESC_DIV128              0x00000007
    #define EFM32_TIMER_CTRL_PRESC_DIV256              0x00000008
    #define EFM32_TIMER_CTRL_PRESC_DIV512              0x00000009
    #define EFM32_TIMER_CTRL_PRESC_DIV1024             0x0000000a
/** Enable ATI makes CCPOL always track the polarity of the inputs. @multiple */
  #define EFM32_TIMER_CTRL_ATI                       0x10000000
/** When enabled, compare output is set to COIST value at Reload-Start event.
   @multiple */
  #define EFM32_TIMER_CTRL_RSSCOIST                  0x20000000

#define EFM32_TIMER_CMD_ADDR                         0x00000004
#define EFM32_TIMER_CMD_MASK                         0x00000003
/** Write a 1 to this bit to start timer. @multiple */
  #define EFM32_TIMER_CMD_START                      0x00000001
/** Write a 1 to this bit to stop timer. @multiple */
  #define EFM32_TIMER_CMD_STOP                       0x00000002

#define EFM32_TIMER_STATUS_ADDR                      0x00000008
#define EFM32_TIMER_STATUS_MASK                      0x07070707
/** Indicates if timer is running or not. @multiple */
  #define EFM32_TIMER_STATUS_RUNNING                 0x00000001
/** Indicates count direction. @multiple */
  #define EFM32_TIMER_STATUS_DIR(v)                  ((EFM32_TIMER_STATUS_DIR_##v) << 1)
  #define EFM32_TIMER_STATUS_DIR_SET(x, v)           do { (x) = (((x) & ~0x2) | ((EFM32_TIMER_STATUS_DIR_##v) << 1)); } while(0)
  #define EFM32_TIMER_STATUS_DIR_SETVAL(x, v)        do { (x) = (((x) & ~0x2) | ((v) << 1)); } while(0)
  #define EFM32_TIMER_STATUS_DIR_GET(x)              (((x) >> 1) & 0x1)
    #define EFM32_TIMER_STATUS_DIR_UP                  0x00000000
    #define EFM32_TIMER_STATUS_DIR_DOWN                0x00000001
/** This indicates that TIMERn_TOPB contains valid data that has not been written
   to TIMERn_TOP. This bit is also cleared when TIMERn_TOP is written. @multiple
   */
  #define EFM32_TIMER_STATUS_TOPBV                   0x00000004
/** This field indicates that the TIMERn_CCn_CCVB registers contain data which
   have not been written to TIMERn_CCn_CCV. These bits are only used in output
   compare/pwm mode and are cleared when CCMODE is written to 0b00 (Off).
   @multiple */
  #define EFM32_TIMER_STATUS_CCVBV_COUNT             3
  #define EFM32_TIMER_STATUS_CCVBV(fidx, v)          ((EFM32_TIMER_STATUS_CCVBV_##v) << ((fidx) + 8))
  #define EFM32_TIMER_STATUS_CCVBV_SET(fidx, x, v)   do { (x) = (((x) & ~(0x100 << ((fidx)))) | ((EFM32_TIMER_STATUS_CCVBV_##v) << ((fidx) + 8))); } while(0)
  #define EFM32_TIMER_STATUS_CCVBV_SETVAL(fidx, x, v) do { (x) = (((x) & ~(0x100 << ((fidx)))) | ((v) << ((fidx) + 8))); } while(0)
  #define EFM32_TIMER_STATUS_CCVBV_GET(fidx, x)      (((x) >> ((fidx) + 8)) & 0x1)
  #define EFM32_TIMER_STATUS_CCVBV_INVALID           0x00000000
  #define EFM32_TIMER_STATUS_CCVBV_VALID             0x00000001
/** This bit indicates that TIMERn_CCn_CCV contains a valid capture value. These
   bits are only used in input capture mode and are cleared when CCMODE is
   written to 0b00 (Off). @multiple */
  #define EFM32_TIMER_STATUS_ICV_COUNT               3
  #define EFM32_TIMER_STATUS_ICV(fidx, v)            ((EFM32_TIMER_STATUS_ICV_##v) << ((fidx) + 16))
  #define EFM32_TIMER_STATUS_ICV_SET(fidx, x, v)     do { (x) = (((x) & ~(0x10000 << ((fidx)))) | ((EFM32_TIMER_STATUS_ICV_##v) << ((fidx) + 16))); } while(0)
  #define EFM32_TIMER_STATUS_ICV_SETVAL(fidx, x, v)  do { (x) = (((x) & ~(0x10000 << ((fidx)))) | ((v) << ((fidx) + 16))); } while(0)
  #define EFM32_TIMER_STATUS_ICV_GET(fidx, x)        (((x) >> ((fidx) + 16)) & 0x1)
  #define EFM32_TIMER_STATUS_ICV_INVALID             0x00000000
  #define EFM32_TIMER_STATUS_ICV_VALID               0x00000001
/** In Input Capture mode, this bit indicates the polarity of the edge that
   triggered capture in TIMERn_CCn_CCV. In Compare/PWM mode, this bit indicates
   the polarity of the selected input to CC channel n. These bits are cleared
   when CCMODE is written to 0b00 (Off). @multiple */
  #define EFM32_TIMER_STATUS_CCPOL_COUNT             3
  #define EFM32_TIMER_STATUS_CCPOL(fidx, v)          ((EFM32_TIMER_STATUS_CCPOL_##v) << ((fidx) + 24))
  #define EFM32_TIMER_STATUS_CCPOL_SET(fidx, x, v)   do { (x) = (((x) & ~(0x1000000 << ((fidx)))) | ((EFM32_TIMER_STATUS_CCPOL_##v) << ((fidx) + 24))); } while(0)
  #define EFM32_TIMER_STATUS_CCPOL_SETVAL(fidx, x, v) do { (x) = (((x) & ~(0x1000000 << ((fidx)))) | ((v) << ((fidx) + 24))); } while(0)
  #define EFM32_TIMER_STATUS_CCPOL_GET(fidx, x)      (((x) >> ((fidx) + 24)) & 0x1)
  #define EFM32_TIMER_STATUS_CCPOL_INVALID           0x00000000
  #define EFM32_TIMER_STATUS_CCPOL_VALID             0x00000001

#define EFM32_TIMER_IEN_ADDR                         0x0000000c
#define EFM32_TIMER_IEN_MASK                         0x00000773
/** Enable/disable overflow interrupt. @multiple */
  #define EFM32_TIMER_IEN_OF                         0x00000001
/** Enable/disable underflow interrupt. @multiple */
  #define EFM32_TIMER_IEN_UF                         0x00000002
/** Enable/disable Compare/Capture ch n interrupt. @multiple */
  #define EFM32_TIMER_IEN_CC_COUNT                   3
  #define EFM32_TIMER_IEN_CC(fidx)                   (0x00000010 << ((fidx)))
/** Enable/disable Compare/Capture ch n input capture buffer overflow interrupt.
   @multiple */
  #define EFM32_TIMER_IEN_ICBOF_COUNT                3
  #define EFM32_TIMER_IEN_ICBOF(fidx)                (0x00000100 << ((fidx)))

#define EFM32_TIMER_IF_ADDR                          0x00000010
#define EFM32_TIMER_IF_MASK                          0x00000773
/** This bit indicates that there has been an overflow. @multiple */
  #define EFM32_TIMER_IF_OF                          0x00000001
/** This bit indicates that there has been an underflow. @multiple */
  #define EFM32_TIMER_IF_UF                          0x00000002
/** This bit indicates that there has been an interrupt event on Compare/Capture
   channel n. @multiple */
  #define EFM32_TIMER_IF_CC_COUNT                    3
  #define EFM32_TIMER_IF_CC(fidx)                    (0x00000010 << ((fidx)))
/** This bit indicates that a new capture value has pushed an unread value out of
   the TIMERn_CCn_CCV/TIMERn_CCn_CCVB register pair. @multiple */
  #define EFM32_TIMER_IF_ICBOF_COUNT                 3
  #define EFM32_TIMER_IF_ICBOF(fidx)                 (0x00000100 << ((fidx)))

#define EFM32_TIMER_IFS_ADDR                         0x00000014
#define EFM32_TIMER_IFS_MASK                         0x00000773
/** Writing a 1 to this bit will set the overflow interrupt flag. @multiple */
  #define EFM32_TIMER_IFS_OF                         0x00000001
/** Writing a 1 to this bit will set the underflow interrupt flag. @multiple */
  #define EFM32_TIMER_IFS_UF                         0x00000002
/** Writing a 1 to this bit will set Compare/Capture channel n interrupt flag.
   @multiple */
  #define EFM32_TIMER_IFS_CC_COUNT                   3
  #define EFM32_TIMER_IFS_CC(fidx)                   (0x00000010 << ((fidx)))
/** Writing a 1 to this bit will set Compare/Capture channel n input capture
   buffer overflow interrupt flag. @multiple */
  #define EFM32_TIMER_IFS_ICBOF_COUNT                3
  #define EFM32_TIMER_IFS_ICBOF(fidx)                (0x00000100 << ((fidx)))

#define EFM32_TIMER_IFC_ADDR                         0x00000018
#define EFM32_TIMER_IFC_MASK                         0x00000773
/** Writing a 1 to this bit will clear the overflow interrupt flag. @multiple */
  #define EFM32_TIMER_IFC_OF                         0x00000001
/** Writing a 1 to this bit will clear the underflow interrupt flag. @multiple */
  #define EFM32_TIMER_IFC_UF                         0x00000002
/** Writing a 1 to this bit will clear Compare/Capture channel n interrupt flag.
   @multiple */
  #define EFM32_TIMER_IFC_CC_COUNT                   3
  #define EFM32_TIMER_IFC_CC(fidx)                   (0x00000010 << ((fidx)))
/** Writing a 1 to this bit will clear Compare/Capture channel n input capture
   buffer overflow interrupt flag. @multiple */
  #define EFM32_TIMER_IFC_ICBOF_COUNT                3
  #define EFM32_TIMER_IFC_ICBOF(fidx)                (0x00000100 << ((fidx)))

#define EFM32_TIMER_TOP_ADDR                         0x0000001c
#define EFM32_TIMER_TOP_MASK                         0x0000ffff
/** These bits hold the TOP value. @multiple */
  #define EFM32_TIMER_TOP_TOPB(v)                    ((v) << 0)
  #define EFM32_TIMER_TOP_TOPB_SET(x, v)             do { (x) = (((x) & ~0xffff) | ((v) << 0)); } while(0)
  #define EFM32_TIMER_TOP_TOPB_GET(x)                (((x) >> 0) & 0xffff)

#define EFM32_TIMER_TOPB_ADDR                        0x00000020
#define EFM32_TIMER_TOPB_MASK                        0x0000ffff
/** These bits hold the TOP buffer value. @multiple */
  #define EFM32_TIMER_TOPB_TOPB(v)                   ((v) << 0)
  #define EFM32_TIMER_TOPB_TOPB_SET(x, v)            do { (x) = (((x) & ~0xffff) | ((v) << 0)); } while(0)
  #define EFM32_TIMER_TOPB_TOPB_GET(x)               (((x) >> 0) & 0xffff)

#define EFM32_TIMER_CNT_ADDR                         0x00000024
#define EFM32_TIMER_CNT_MASK                         0x0000ffff
/** These bits hold the counter value. @multiple */
  #define EFM32_TIMER_CNT_CNT(v)                     ((v) << 0)
  #define EFM32_TIMER_CNT_CNT_SET(x, v)              do { (x) = (((x) & ~0xffff) | ((v) << 0)); } while(0)
  #define EFM32_TIMER_CNT_CNT_GET(x)                 (((x) >> 0) & 0xffff)

#define EFM32_TIMER_ROUTE_ADDR                       0x00000028
#define EFM32_TIMER_ROUTE_MASK                       0x00070007
/** Enable/disable CC Channel n output/input connection to pin. @multiple */
  #define EFM32_TIMER_ROUTE_CCPEN_COUNT              3
  #define EFM32_TIMER_ROUTE_CCPEN(fidx)              (0x00000001 << ((fidx)))
/** Decides the location of the CC pins. @multiple */
  #define EFM32_TIMER_ROUTE_LOCATION(v)              ((EFM32_TIMER_ROUTE_LOCATION_##v) << 16)
  #define EFM32_TIMER_ROUTE_LOCATION_SET(x, v)       do { (x) = (((x) & ~0x70000) | ((EFM32_TIMER_ROUTE_LOCATION_##v) << 16)); } while(0)
  #define EFM32_TIMER_ROUTE_LOCATION_SETVAL(x, v)    do { (x) = (((x) & ~0x70000) | ((v) << 16)); } while(0)
  #define EFM32_TIMER_ROUTE_LOCATION_GET(x)          (((x) >> 16) & 0x7)
    #define EFM32_TIMER_ROUTE_LOCATION_L0C0            0x00000000
    #define EFM32_TIMER_ROUTE_LOCATION_L0C1            0x00000001
    #define EFM32_TIMER_ROUTE_LOCATION_L0C2            0x00000002
    #define EFM32_TIMER_ROUTE_LOCATION_L0C3            0x00000003
    #define EFM32_TIMER_ROUTE_LOCATION_L0C4            0x00000004
    #define EFM32_TIMER_ROUTE_LOCATION_L0C5            0x00000005

#define EFM32_TIMER_CC_CTRL_ADDR(ridx)               (0x00000030 + (ridx) * 4)
#define EFM32_TIMER_CC_CTRL_COUNT                    3
#define EFM32_TIMER_CC_CTRL_MASK                     0x1f333f17
/** These bits select the mode for Compare/Capture channel. @multiple */
  #define EFM32_TIMER_CC_CTRL_MODE(v)                ((EFM32_TIMER_CC_CTRL_MODE_##v) << 0)
  #define EFM32_TIMER_CC_CTRL_MODE_SET(x, v)         do { (x) = (((x) & ~0x3) | ((EFM32_TIMER_CC_CTRL_MODE_##v) << 0)); } while(0)
  #define EFM32_TIMER_CC_CTRL_MODE_SETVAL(x, v)      do { (x) = (((x) & ~0x3) | ((v) << 0)); } while(0)
  #define EFM32_TIMER_CC_CTRL_MODE_GET(x)            (((x) >> 0) & 0x3)
    #define EFM32_TIMER_CC_CTRL_MODE_OFF               0x00000000
    #define EFM32_TIMER_CC_CTRL_MODE_INPUTCAPTURE      0x00000001
    #define EFM32_TIMER_CC_CTRL_MODE_OUTPUTCOMPARE     0x00000002
    #define EFM32_TIMER_CC_CTRL_MODE_PWM               0x00000003
/** Setting this bit inverts the output from the CC channel (Output compare,PWM).
   @multiple */
  #define EFM32_TIMER_CC_CTRL_OUTINV                 0x00000004
/** This bit is only used in Output Compare and PWM mode. When this bit is set in
   compare mode,the output is set high when the counter is disabled. When
   counting resumes, this value will represent the initial value for the output.
   If the bit is cleared, the output will be cleared when the counter is
   disabled. In PWM mode, the output will always be low when disabled, regardless
   of this bit. However, this bit will represent the initial value of the output,
   once it is enabled. @multiple */
  #define EFM32_TIMER_CC_CTRL_COIST                  0x00000010
/** Select output action on compare match. @multiple */
  #define EFM32_TIMER_CC_CTRL_CMOA(v)                ((EFM32_TIMER_CC_CTRL_CMOA_##v) << 8)
  #define EFM32_TIMER_CC_CTRL_CMOA_SET(x, v)         do { (x) = (((x) & ~0x300) | ((EFM32_TIMER_CC_CTRL_CMOA_##v) << 8)); } while(0)
  #define EFM32_TIMER_CC_CTRL_CMOA_SETVAL(x, v)      do { (x) = (((x) & ~0x300) | ((v) << 8)); } while(0)
  #define EFM32_TIMER_CC_CTRL_CMOA_GET(x)            (((x) >> 8) & 0x3)
    #define EFM32_TIMER_CC_CTRL_CMOA_NONE              0x00000000
    #define EFM32_TIMER_CC_CTRL_CMOA_TOGGLE            0x00000001
    #define EFM32_TIMER_CC_CTRL_CMOA_CLEAR             0x00000002
    #define EFM32_TIMER_CC_CTRL_CMOA_UP                0x00000003
/** Select output action on counter overflow. @multiple */
  #define EFM32_TIMER_CC_CTRL_COFOA(v)               ((EFM32_TIMER_CC_CTRL_COFOA_##v) << 10)
  #define EFM32_TIMER_CC_CTRL_COFOA_SET(x, v)        do { (x) = (((x) & ~0xc00) | ((EFM32_TIMER_CC_CTRL_COFOA_##v) << 10)); } while(0)
  #define EFM32_TIMER_CC_CTRL_COFOA_SETVAL(x, v)     do { (x) = (((x) & ~0xc00) | ((v) << 10)); } while(0)
  #define EFM32_TIMER_CC_CTRL_COFOA_GET(x)           (((x) >> 10) & 0x3)
    #define EFM32_TIMER_CC_CTRL_COFOA_NONE             0x00000000
    #define EFM32_TIMER_CC_CTRL_COFOA_TOGGLE           0x00000001
    #define EFM32_TIMER_CC_CTRL_COFOA_CLEAR            0x00000002
    #define EFM32_TIMER_CC_CTRL_COFOA_UP               0x00000003
/** Select output action on counter underflow. @multiple */
  #define EFM32_TIMER_CC_CTRL_CUFOA(v)               ((EFM32_TIMER_CC_CTRL_CUFOA_##v) << 12)
  #define EFM32_TIMER_CC_CTRL_CUFOA_SET(x, v)        do { (x) = (((x) & ~0x3000) | ((EFM32_TIMER_CC_CTRL_CUFOA_##v) << 12)); } while(0)
  #define EFM32_TIMER_CC_CTRL_CUFOA_SETVAL(x, v)     do { (x) = (((x) & ~0x3000) | ((v) << 12)); } while(0)
  #define EFM32_TIMER_CC_CTRL_CUFOA_GET(x)           (((x) >> 12) & 0x3)
    #define EFM32_TIMER_CC_CTRL_CUFOA_NONE             0x00000000
    #define EFM32_TIMER_CC_CTRL_CUFOA_TOGGLE           0x00000001
    #define EFM32_TIMER_CC_CTRL_CUFOA_CLEAR            0x00000002
    #define EFM32_TIMER_CC_CTRL_CUFOA_UP               0x00000003
/** Select PRS input channel for Compare/Capture channel. @multiple */
  #define EFM32_TIMER_CC_CTRL_PRSSEL(v)              ((EFM32_TIMER_CC_CTRL_PRSSEL_##v) << 16)
  #define EFM32_TIMER_CC_CTRL_PRSSEL_SET(x, v)       do { (x) = (((x) & ~0x30000) | ((EFM32_TIMER_CC_CTRL_PRSSEL_##v) << 16)); } while(0)
  #define EFM32_TIMER_CC_CTRL_PRSSEL_SETVAL(x, v)    do { (x) = (((x) & ~0x30000) | ((v) << 16)); } while(0)
  #define EFM32_TIMER_CC_CTRL_PRSSEL_GET(x)          (((x) >> 16) & 0x3)
    #define EFM32_TIMER_CC_CTRL_PRSSEL_PRSCH0          0x00000000
    #define EFM32_TIMER_CC_CTRL_PRSSEL_PRSCH1          0x00000001
    #define EFM32_TIMER_CC_CTRL_PRSSEL_PRSCH2          0x00000002
    #define EFM32_TIMER_CC_CTRL_PRSSEL_PRSCH3          0x00000003
/** Select Compare/Capture channel input. @multiple */
  #define EFM32_TIMER_CC_CTRL_INSEL(v)               ((EFM32_TIMER_CC_CTRL_INSEL_##v) << 20)
  #define EFM32_TIMER_CC_CTRL_INSEL_SET(x, v)        do { (x) = (((x) & ~0x100000) | ((EFM32_TIMER_CC_CTRL_INSEL_##v) << 20)); } while(0)
  #define EFM32_TIMER_CC_CTRL_INSEL_SETVAL(x, v)     do { (x) = (((x) & ~0x100000) | ((v) << 20)); } while(0)
  #define EFM32_TIMER_CC_CTRL_INSEL_GET(x)           (((x) >> 20) & 0x1)
    #define EFM32_TIMER_CC_CTRL_INSEL_PIN              0x00000000
    #define EFM32_TIMER_CC_CTRL_INSEL_PRS              0x00000001
/** Enable digital filter. @multiple */
  #define EFM32_TIMER_CC_CTRL_FILT(v)                ((EFM32_TIMER_CC_CTRL_FILT_##v) << 21)
  #define EFM32_TIMER_CC_CTRL_FILT_SET(x, v)         do { (x) = (((x) & ~0x200000) | ((EFM32_TIMER_CC_CTRL_FILT_##v) << 21)); } while(0)
  #define EFM32_TIMER_CC_CTRL_FILT_SETVAL(x, v)      do { (x) = (((x) & ~0x200000) | ((v) << 21)); } while(0)
  #define EFM32_TIMER_CC_CTRL_FILT_GET(x)            (((x) >> 21) & 0x1)
    #define EFM32_TIMER_CC_CTRL_FILT_DISBALED          0x00000000
    #define EFM32_TIMER_CC_CTRL_FILT_ENABLED           0x00000001
/** These bits control which edges the edge detector triggers on. The output is
   used for input capture and external clock input. @multiple */
  #define EFM32_TIMER_CC_CTRL_ICEDGE(v)              ((EFM32_TIMER_CC_CTRL_ICEDGE_##v) << 24)
  #define EFM32_TIMER_CC_CTRL_ICEDGE_SET(x, v)       do { (x) = (((x) & ~0x3000000) | ((EFM32_TIMER_CC_CTRL_ICEDGE_##v) << 24)); } while(0)
  #define EFM32_TIMER_CC_CTRL_ICEDGE_SETVAL(x, v)    do { (x) = (((x) & ~0x3000000) | ((v) << 24)); } while(0)
  #define EFM32_TIMER_CC_CTRL_ICEDGE_GET(x)          (((x) >> 24) & 0x3)
    #define EFM32_TIMER_CC_CTRL_ICEDGE_RISING          0x00000000
    #define EFM32_TIMER_CC_CTRL_ICEDGE_FALLING         0x00000001
    #define EFM32_TIMER_CC_CTRL_ICEDGE_BOTH            0x00000002
    #define EFM32_TIMER_CC_CTRL_ICEDGE_NONE            0x00000003
/** These bits control when a Compare/Capture PRS output pulse, interrupt flag
   and DMA request is set. @multiple */
  #define EFM32_TIMER_CC_CTRL_ICEVCTRL(v)            ((EFM32_TIMER_CC_CTRL_ICEVCTRL_##v) << 26)
  #define EFM32_TIMER_CC_CTRL_ICEVCTRL_SET(x, v)     do { (x) = (((x) & ~0xc000000) | ((EFM32_TIMER_CC_CTRL_ICEVCTRL_##v) << 26)); } while(0)
  #define EFM32_TIMER_CC_CTRL_ICEVCTRL_SETVAL(x, v)  do { (x) = (((x) & ~0xc000000) | ((v) << 26)); } while(0)
  #define EFM32_TIMER_CC_CTRL_ICEVCTRL_GET(x)        (((x) >> 26) & 0x3)
    #define EFM32_TIMER_CC_CTRL_ICEVCTRL_EVERYEDGE     0x00000000
    #define EFM32_TIMER_CC_CTRL_ICEVCTRL_EVERYSECONDEDGE 0x00000001
    #define EFM32_TIMER_CC_CTRL_ICEVCTRL_RISING        0x00000002
    #define EFM32_TIMER_CC_CTRL_ICEVCTRL_FALLING       0x00000003
/** Select PRS pulse or level. @multiple */
  #define EFM32_TIMER_CC_CTRL_PRSCONF(v)             ((EFM32_TIMER_CC_CTRL_PRSCONF_##v) << 28)
  #define EFM32_TIMER_CC_CTRL_PRSCONF_SET(x, v)      do { (x) = (((x) & ~0x10000000) | ((EFM32_TIMER_CC_CTRL_PRSCONF_##v) << 28)); } while(0)
  #define EFM32_TIMER_CC_CTRL_PRSCONF_SETVAL(x, v)   do { (x) = (((x) & ~0x10000000) | ((v) << 28)); } while(0)
  #define EFM32_TIMER_CC_CTRL_PRSCONF_GET(x)         (((x) >> 28) & 0x1)
    #define EFM32_TIMER_CC_CTRL_PRSCONF_PULSE          0x00000000
    #define EFM32_TIMER_CC_CTRL_PRSCONF_LEVEL          0x00000001

#define EFM32_TIMER_CC_CCV_ADDR(ridx)                (0x00000034 + (ridx) * 16)
#define EFM32_TIMER_CC_CCV_COUNT                     3
#define EFM32_TIMER_CC_CCV_MASK                      0x0000ffff
/** In input capture mode, this field holds the first unread capture value. When
   reading this register in input capture mode, then contents of the
   TIMERn_CCx_CCVB register will be written to TIMERn_CCx_CCV in the next cycle.
   In compare mode, this fields holds the compare value. @multiple */
  #define EFM32_TIMER_CC_CCV_CCV(v)                  ((v) << 0)
  #define EFM32_TIMER_CC_CCV_CCV_SET(x, v)           do { (x) = (((x) & ~0xffff) | ((v) << 0)); } while(0)
  #define EFM32_TIMER_CC_CCV_CCV_GET(x)              (((x) >> 0) & 0xffff)

#define EFM32_TIMER_CC_CCVP_ADDR(ridx)               (0x00000038 + (ridx) * 16)
#define EFM32_TIMER_CC_CCVP_COUNT                    3
#define EFM32_TIMER_CC_CCVP_MASK                     0x0000ffff
/** This field is used to read the CC value without pulling data through the FIFO
   in capture mode. @multiple */
  #define EFM32_TIMER_CC_CCVP_CCVP(v)                ((v) << 0)
  #define EFM32_TIMER_CC_CCVP_CCVP_SET(x, v)         do { (x) = (((x) & ~0xffff) | ((v) << 0)); } while(0)
  #define EFM32_TIMER_CC_CCVP_CCVP_GET(x)            (((x) >> 0) & 0xffff)

#define EFM32_TIMER_CC_CCVB_ADDR(ridx)               (0x0000003c + (ridx) * 16)
#define EFM32_TIMER_CC_CCVB_COUNT                    3
#define EFM32_TIMER_CC_CCVB_MASK                     0x0000ffff
/** In Input Capture mode, this field holds the last capture value if the
   TIMERn_CCx_CCV register already contains an earlier unread capture value. In
   Output Compare or PWM mode, this field holds the CC buffer value which will be
   written to TIMERn_CCx_CCV on an update event if TIMERn_CCx_CCVB contains valid
   data. @multiple */
  #define EFM32_TIMER_CC_CCVB_CCVB(v)                ((v) << 0)
  #define EFM32_TIMER_CC_CCVB_CCVB_SET(x, v)         do { (x) = (((x) & ~0xffff) | ((v) << 0)); } while(0)
  #define EFM32_TIMER_CC_CCVB_CCVB_GET(x)            (((x) >> 0) & 0xffff)

#endif


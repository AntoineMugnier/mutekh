/***************************************
* Auto generated by BFGen, do not edit *
***************************************/

/*
   bfgen -o cdefs -O arch/efm32/include/arch/efm32/gpio.h cdefs_use_reg_mask=1 \
     cdefs_use_field_setval=1
*/

#ifndef _EFM32_GPIO_BFGEN_DEFS_
#define _EFM32_GPIO_BFGEN_DEFS_

#define EFM32_GPIO_CTRL_ADDR(ridx)                   (0x00000000 + (ridx) * 36)
#define EFM32_GPIO_CTRL_COUNT                        6
#define EFM32_GPIO_CTRL_MASK                         0x00000003
/** Select drive mode for all pins on port configured with alternate drive
   strength @multiple */
  #define EFM32_GPIO_CTRL_DRIVEMODE(v)               ((EFM32_GPIO_CTRL_DRIVEMODE_##v) << 0)
  #define EFM32_GPIO_CTRL_DRIVEMODE_SET(x, v)        do { (x) = (((x) & ~0x3) | ((EFM32_GPIO_CTRL_DRIVEMODE_##v) << 0)); } while(0)
  #define EFM32_GPIO_CTRL_DRIVEMODE_SETVAL(x, v)     do { (x) = (((x) & ~0x3) | ((v) << 0)); } while(0)
  #define EFM32_GPIO_CTRL_DRIVEMODE_GET(x)           (((x) >> 0) & 0x3)
    #define EFM32_GPIO_CTRL_DRIVEMODE_STANDART         0x00000000
    #define EFM32_GPIO_CTRL_DRIVEMODE_LOWEST           0x00000001
    #define EFM32_GPIO_CTRL_DRIVEMODE_HIGHT            0x00000002
    #define EFM32_GPIO_CTRL_DRIVEMODE_LOW              0x00000003

#define EFM32_GPIO_MODEL_ADDR(ridx)                  (0x00000004 + (ridx) * 36)
#define EFM32_GPIO_MODEL_COUNT                       6
#define EFM32_GPIO_MODEL_MASK                        0xffffffff
  #define EFM32_GPIO_MODEL_MODE_COUNT                8
  #define EFM32_GPIO_MODEL_MODE(fidx, v)             ((EFM32_GPIO_MODEL_MODE_##v) << ((fidx) * 4 + 0))
  #define EFM32_GPIO_MODEL_MODE_SET(fidx, x, v)      do { (x) = (((x) & ~(0xf << ((fidx) * 4))) | ((EFM32_GPIO_MODEL_MODE_##v) << ((fidx) * 4 + 0))); } while(0)
  #define EFM32_GPIO_MODEL_MODE_SETVAL(fidx, x, v)   do { (x) = (((x) & ~(0xf << ((fidx) * 4))) | ((v) << ((fidx) * 4 + 0))); } while(0)
  #define EFM32_GPIO_MODEL_MODE_GET(fidx, x)         (((x) >> ((fidx) * 4 + 0)) & 0xf)
  #define EFM32_GPIO_MODEL_MODE_DISABLED             0x00000000
  #define EFM32_GPIO_MODEL_MODE_INPUT                0x00000001
  #define EFM32_GPIO_MODEL_MODE_INPUTPULL            0x00000002
  #define EFM32_GPIO_MODEL_MODE_INPUTPULLFILTER      0x00000003
  #define EFM32_GPIO_MODEL_MODE_PUSHPULL             0x00000004
  #define EFM32_GPIO_MODEL_MODE_PUSHPULLDRIVE        0x00000005
  #define EFM32_GPIO_MODEL_MODE_WIREDOR              0x00000006
  #define EFM32_GPIO_MODEL_MODE_WIREDORPULLDOWN      0x00000007
  #define EFM32_GPIO_MODEL_MODE_WIREDAND             0x00000008
  #define EFM32_GPIO_MODEL_MODE_WIREDANDFILTER       0x00000009
  #define EFM32_GPIO_MODEL_MODE_WIREDANDPULLUP       0x0000000a
  #define EFM32_GPIO_MODEL_MODE_WIREDANDPULLUPFILTER 0x0000000b
  #define EFM32_GPIO_MODEL_MODE_WIREDANDDRIVE        0x0000000c
  #define EFM32_GPIO_MODEL_MODE_WIREDANDDRIVEFILTER  0x0000000d
  #define EFM32_GPIO_MODEL_MODE_WIREDANDDRIVEPULLUP  0x0000000e
  #define EFM32_GPIO_MODEL_MODE_WIREDANDDRIVEPULLUPFILTER 0x0000000f

#define EFM32_GPIO_MODEH_ADDR(ridx)                  (0x00000008 + (ridx) * 36)
#define EFM32_GPIO_MODEH_COUNT                       6
#define EFM32_GPIO_MODEH_MASK                        0xffffffff
  #define EFM32_GPIO_MODEH_MODE_COUNT                8
  #define EFM32_GPIO_MODEH_MODE(fidx, v)             ((EFM32_GPIO_MODEH_MODE_##v) << ((fidx) * 4 + 0))
  #define EFM32_GPIO_MODEH_MODE_SET(fidx, x, v)      do { (x) = (((x) & ~(0xf << ((fidx) * 4))) | ((EFM32_GPIO_MODEH_MODE_##v) << ((fidx) * 4 + 0))); } while(0)
  #define EFM32_GPIO_MODEH_MODE_SETVAL(fidx, x, v)   do { (x) = (((x) & ~(0xf << ((fidx) * 4))) | ((v) << ((fidx) * 4 + 0))); } while(0)
  #define EFM32_GPIO_MODEH_MODE_GET(fidx, x)         (((x) >> ((fidx) * 4 + 0)) & 0xf)
  #define EFM32_GPIO_MODEH_MODE_DISABLED             0x00000000
  #define EFM32_GPIO_MODEH_MODE_INPUT                0x00000001
  #define EFM32_GPIO_MODEH_MODE_INPUTPULL            0x00000002
  #define EFM32_GPIO_MODEH_MODE_INPUTPULLFILTER      0x00000003
  #define EFM32_GPIO_MODEH_MODE_PUSHPULL             0x00000004
  #define EFM32_GPIO_MODEH_MODE_PUSHPULLDRIVE        0x00000005
  #define EFM32_GPIO_MODEH_MODE_WIREDOR              0x00000006
  #define EFM32_GPIO_MODEH_MODE_WIREDORPULLDOWN      0x00000007
  #define EFM32_GPIO_MODEH_MODE_WIREDAND             0x00000008
  #define EFM32_GPIO_MODEH_MODE_WIREDANDFILTER       0x00000009
  #define EFM32_GPIO_MODEH_MODE_WIREDANDPULLUP       0x0000000a
  #define EFM32_GPIO_MODEH_MODE_WIREDANDPULLUPFILTER 0x0000000b
  #define EFM32_GPIO_MODEH_MODE_WIREDANDDRIVE        0x0000000c
  #define EFM32_GPIO_MODEH_MODE_WIREDANDDRIVEFILTER  0x0000000d
  #define EFM32_GPIO_MODEH_MODE_WIREDANDDRIVEPULLUP  0x0000000e
  #define EFM32_GPIO_MODEH_MODE_WIREDANDDRIVEPULLUPFILTER 0x0000000f

#define EFM32_GPIO_DOUT_ADDR(ridx)                   (0x0000000c + (ridx) * 36)
#define EFM32_GPIO_DOUT_COUNT                        6
#define EFM32_GPIO_DOUT_MASK                         0x0000ffff
/** Data out on port n. @multiple */
  #define EFM32_GPIO_DOUT_DOUT_COUNT                 16
  #define EFM32_GPIO_DOUT_DOUT(fidx)                 (0x00000001 << ((fidx)))

#define EFM32_GPIO_DOUTSET_ADDR(ridx)                (0x00000010 + (ridx) * 36)
#define EFM32_GPIO_DOUTSET_COUNT                     6
#define EFM32_GPIO_DOUTSET_MASK                      0x0000ffff
/** Write bits to 1 to set corresponding bits in GPIO_Px_DOUT. Bits written to 0
   will have no effect. @multiple */
  #define EFM32_GPIO_DOUTSET_DOUTSET_COUNT           16
  #define EFM32_GPIO_DOUTSET_DOUTSET(fidx)           (0x00000001 << ((fidx)))

#define EFM32_GPIO_DOUTCLR_ADDR(ridx)                (0x00000014 + (ridx) * 36)
#define EFM32_GPIO_DOUTCLR_COUNT                     6
#define EFM32_GPIO_DOUTCLR_MASK                      0x0000ffff
/** Write bits to 1 to clear corresponding bits in GPIO_Px_DOUT. Bits written to
   0 will have no effect. @multiple */
  #define EFM32_GPIO_DOUTCLR_DOUTCLR_COUNT           16
  #define EFM32_GPIO_DOUTCLR_DOUTCLR(fidx)           (0x00000001 << ((fidx)))

#define EFM32_GPIO_DOUTTGL_ADDR(ridx)                (0x00000018 + (ridx) * 36)
#define EFM32_GPIO_DOUTTGL_COUNT                     6
#define EFM32_GPIO_DOUTTGL_MASK                      0x0000ffff
/** Write bits to 1 to toggle corresponding bits in GPIO_Px_DOUT. Bits written to
   0 will have no effect @multiple */
  #define EFM32_GPIO_DOUTTGL_DOUTTGL_COUNT           16
  #define EFM32_GPIO_DOUTTGL_DOUTTGL(fidx)           (0x00000001 << ((fidx)))

#define EFM32_GPIO_DIN_ADDR(ridx)                    (0x0000001c + (ridx) * 36)
#define EFM32_GPIO_DIN_COUNT                         6
#define EFM32_GPIO_DIN_MASK                          0x0000ffff
/** Port Data Input @multiple */
  #define EFM32_GPIO_DIN_DIN_COUNT                   16
  #define EFM32_GPIO_DIN_DIN(fidx)                   (0x00000001 << ((fidx)))

#define EFM32_GPIO_PINLOCKN_ADDR(ridx)               (0x00000020 + (ridx) * 36)
#define EFM32_GPIO_PINLOCKN_COUNT                    6
#define EFM32_GPIO_PINLOCKN_MASK                     0x0000ffff
/** Shows unlocked pins in the port. To lock pin n, clear bit n. The pin is then
   locked until reset. @multiple */
  #define EFM32_GPIO_PINLOCKN_PINLOCKN_COUNT         16
  #define EFM32_GPIO_PINLOCKN_PINLOCKN(fidx)         (0x00000001 << ((fidx)))

#define EFM32_GPIO_EXTIPSELL_ADDR                    0x00000100
#define EFM32_GPIO_EXTIPSELL_MASK                    0x77777777
/** Select input port for external low interrupt (0-7). @multiple */
  #define EFM32_GPIO_EXTIPSELL_EXT_COUNT             8
  #define EFM32_GPIO_EXTIPSELL_EXT(fidx, v)          ((EFM32_GPIO_EXTIPSELL_EXT_##v) << ((fidx) * 4 + 0))
  #define EFM32_GPIO_EXTIPSELL_EXT_SET(fidx, x, v)   do { (x) = (((x) & ~(0x7 << ((fidx) * 4))) | ((EFM32_GPIO_EXTIPSELL_EXT_##v) << ((fidx) * 4 + 0))); } while(0)
  #define EFM32_GPIO_EXTIPSELL_EXT_SETVAL(fidx, x, v) do { (x) = (((x) & ~(0x7 << ((fidx) * 4))) | ((v) << ((fidx) * 4 + 0))); } while(0)
  #define EFM32_GPIO_EXTIPSELL_EXT_GET(fidx, x)      (((x) >> ((fidx) * 4 + 0)) & 0x7)
  #define EFM32_GPIO_EXTIPSELL_EXT_PORT_A            0x00000000
  #define EFM32_GPIO_EXTIPSELL_EXT_PORT_B            0x00000001
  #define EFM32_GPIO_EXTIPSELL_EXT_PORT_C            0x00000002
  #define EFM32_GPIO_EXTIPSELL_EXT_PORT_D            0x00000003
  #define EFM32_GPIO_EXTIPSELL_EXT_PORT_E            0x00000004
  #define EFM32_GPIO_EXTIPSELL_EXT_PORT_F            0x00000005

#define EFM32_GPIO_EXTIPSELH_ADDR                    0x00000104
#define EFM32_GPIO_EXTIPSELH_MASK                    0x77777777
/** Select input port for external high interrupt (8-15). @multiple */
  #define EFM32_GPIO_EXTIPSELH_EXT_COUNT             8
  #define EFM32_GPIO_EXTIPSELH_EXT(fidx, v)          ((EFM32_GPIO_EXTIPSELH_EXT_##v) << ((fidx) * 4 + 0))
  #define EFM32_GPIO_EXTIPSELH_EXT_SET(fidx, x, v)   do { (x) = (((x) & ~(0x7 << ((fidx) * 4))) | ((EFM32_GPIO_EXTIPSELH_EXT_##v) << ((fidx) * 4 + 0))); } while(0)
  #define EFM32_GPIO_EXTIPSELH_EXT_SETVAL(fidx, x, v) do { (x) = (((x) & ~(0x7 << ((fidx) * 4))) | ((v) << ((fidx) * 4 + 0))); } while(0)
  #define EFM32_GPIO_EXTIPSELH_EXT_GET(fidx, x)      (((x) >> ((fidx) * 4 + 0)) & 0x7)
  #define EFM32_GPIO_EXTIPSELH_EXT_PORT_A            0x00000000
  #define EFM32_GPIO_EXTIPSELH_EXT_PORT_B            0x00000001
  #define EFM32_GPIO_EXTIPSELH_EXT_PORT_C            0x00000002
  #define EFM32_GPIO_EXTIPSELH_EXT_PORT_D            0x00000003
  #define EFM32_GPIO_EXTIPSELH_EXT_PORT_E            0x00000004
  #define EFM32_GPIO_EXTIPSELH_EXT_PORT_F            0x00000005

#define EFM32_GPIO_EXTIRISE_ADDR                     0x00000108
#define EFM32_GPIO_EXTIRISE_MASK                     0x0000ffff
/** Set bit n to enable triggering of external interrupt n on rising edge
   @multiple */
  #define EFM32_GPIO_EXTIRISE_EXT_COUNT              16
  #define EFM32_GPIO_EXTIRISE_EXT(fidx)              (0x00000001 << ((fidx)))

#define EFM32_GPIO_EXTIFALL_ADDR                     0x0000010c
#define EFM32_GPIO_EXTIFALL_MASK                     0x0000ffff
/** Set bit n to enable triggering of external interrupt n on falling edge.
   @multiple */
  #define EFM32_GPIO_EXTIFALL_EXT_COUNT              16
  #define EFM32_GPIO_EXTIFALL_EXT(fidx)              (0x00000001 << ((fidx)))

#define EFM32_GPIO_IEN_ADDR                          0x00000110
#define EFM32_GPIO_IEN_MASK                          0x0000ffff
/** Set bit n to enable external interrupt from pin n. @multiple */
  #define EFM32_GPIO_IEN_EXT_COUNT                   16
  #define EFM32_GPIO_IEN_EXT(fidx)                   (0x00000001 << ((fidx)))

#define EFM32_GPIO_IF_ADDR                           0x00000114
#define EFM32_GPIO_IF_MASK                           0x0000ffff
/** Pin n external interrupt flag. @multiple */
  #define EFM32_GPIO_IF_EXT_COUNT                    16
  #define EFM32_GPIO_IF_EXT(fidx)                    (0x00000001 << ((fidx)))

#define EFM32_GPIO_IFS_ADDR                          0x00000118
#define EFM32_GPIO_IFS_MASK                          0x0000ffff
/** Write bit n to 1 to set interrupt flag n @multiple */
  #define EFM32_GPIO_IFS_EXT_COUNT                   16
  #define EFM32_GPIO_IFS_EXT(fidx)                   (0x00000001 << ((fidx)))

#define EFM32_GPIO_IFC_ADDR                          0x0000011c
#define EFM32_GPIO_IFC_MASK                          0x0000ffff
/** Write bit n to 1 to clear external interrupt flag n. @multiple */
  #define EFM32_GPIO_IFC_EXT_COUNT                   16
  #define EFM32_GPIO_IFC_EXT(fidx)                   (0x00000001 << ((fidx)))

#define EFM32_GPIO_ROUTE_ADDR                        0x00000120
#define EFM32_GPIO_ROUTE_MASK                        0x00000003
/** Enable Serial Wire Clock connection to pin. WARNING: When this pin is
   disabled, the device can no longer be accessed by a debugger. A reset will set
   the pin back to a default state as enabled. If you disable this pin, make sure
   you have at least a 3 second timeout at the start of you program code before
   you disable the pin. This way, the debugger will have time to halt the device
   after a reset before the pin is disabled. @multiple */
  #define EFM32_GPIO_ROUTE_SWCLKPEN                  0x00000001
/** Enable Serial Wire Data connection to pin. WARNING: When this pin is
   disabled, the device can no longer be accessed by a debugger. A reset will set
   the pin back to a default state as enabled. If you disable this pin, make sure
   you have at least a 3 second timeout at the start of you program code before
   you disable the pin. This way, the debugger will have time to halt the device
   after a reset before the pin is disabled. @multiple */
  #define EFM32_GPIO_ROUTE_SWDIOPEN                  0x00000002

#define EFM32_GPIO_INSENSE_ADDR                      0x00000124
#define EFM32_GPIO_INSENSE_MASK                      0x00000003
/** Set this bit to enable input sensing for interrupts. @multiple */
  #define EFM32_GPIO_INSENSE_INT                     0x00000001
/** Set this bit to enable input sensing for PRS. @multiple */
  #define EFM32_GPIO_INSENSE_PRS                     0x00000002

#define EFM32_GPIO_LOCK_ADDR                         0x00000128
#define EFM32_GPIO_LOCK_MASK                         0x0000ffff
/** Write any other value than the unlock code to lock MODEL, MODEH, CTRL,
   PINLOCKN, EPISELL, EIPSELH, INSENSE and SWDPROUTE from editing. Write the
   unlock code to unlock. When reading the register, bit 0 is set when the lock
   is enabled. @multiple */
  #define EFM32_GPIO_LOCK_LOCKKEY(v)                 ((EFM32_GPIO_LOCK_LOCKKEY_##v) << 0)
  #define EFM32_GPIO_LOCK_LOCKKEY_SET(x, v)          do { (x) = (((x) & ~0xffff) | ((EFM32_GPIO_LOCK_LOCKKEY_##v) << 0)); } while(0)
  #define EFM32_GPIO_LOCK_LOCKKEY_SETVAL(x, v)       do { (x) = (((x) & ~0xffff) | ((v) << 0)); } while(0)
  #define EFM32_GPIO_LOCK_LOCKKEY_GET(x)             (((x) >> 0) & 0xffff)
    #define EFM32_GPIO_LOCK_LOCKKEY_LOCK               0x00000000
    #define EFM32_GPIO_LOCK_LOCKKEY_UNLOCK             0x0000a534

#define EFM32_GPIO_PCTRL_ADDR                        0x0000012c
#define EFM32_GPIO_PCTRL_MASK                        0x00000001
/** Set to enable EM4 retention of output enable, output value and pull enable.
   @multiple */
  #define EFM32_GPIO_PCTRL_EM4RET                    0x00000001

#define EFM32_GPIO_CMD_ADDR                          0x00000130
#define EFM32_GPIO_CMD_MASK                          0x00000001
/** Write 1 to clear all wake-up requests. @multiple */
  #define EFM32_GPIO_CMD_EM4WUCLR                    0x00000001

#define EFM32_GPIO_EM4WUEN_ADDR                      0x00000134
#define EFM32_GPIO_EM4WUEN_MASK                      0x0000003f
/** Write 1 to enable wake-up request, write 0 to disable wake-up request.
   @multiple */
  #define EFM32_GPIO_EM4WUEN_EM4WUEN(v)              ((EFM32_GPIO_EM4WUEN_EM4WUEN_##v) << 0)
  #define EFM32_GPIO_EM4WUEN_EM4WUEN_SET(x, v)       do { (x) = (((x) & ~0x3f) | ((EFM32_GPIO_EM4WUEN_EM4WUEN_##v) << 0)); } while(0)
  #define EFM32_GPIO_EM4WUEN_EM4WUEN_SETVAL(x, v)    do { (x) = (((x) & ~0x3f) | ((v) << 0)); } while(0)
  #define EFM32_GPIO_EM4WUEN_EM4WUEN_GET(x)          (((x) >> 0) & 0x3f)
    #define EFM32_GPIO_EM4WUEN_EM4WUEN_A0              0x00000001
    #define EFM32_GPIO_EM4WUEN_EM4WUEN_C9              0x00000004
    #define EFM32_GPIO_EM4WUEN_EM4WUEN_F1              0x00000008
    #define EFM32_GPIO_EM4WUEN_EM4WUEN_F2              0x00000010
    #define EFM32_GPIO_EM4WUEN_EM4WUEN_E13             0x00000020

#define EFM32_GPIO_EM4WUPOL_ADDR                     0x00000138
#define EFM32_GPIO_EM4WUPOL_MASK                     0x0000003f
/** Write bit n to 1 for high wake-up request. Write bit n to 0 for low wake-up
   request @multiple */
  #define EFM32_GPIO_EM4WUPOL_EM4WUPOL(v)            ((EFM32_GPIO_EM4WUPOL_EM4WUPOL_##v) << 0)
  #define EFM32_GPIO_EM4WUPOL_EM4WUPOL_SET(x, v)     do { (x) = (((x) & ~0x3f) | ((EFM32_GPIO_EM4WUPOL_EM4WUPOL_##v) << 0)); } while(0)
  #define EFM32_GPIO_EM4WUPOL_EM4WUPOL_SETVAL(x, v)  do { (x) = (((x) & ~0x3f) | ((v) << 0)); } while(0)
  #define EFM32_GPIO_EM4WUPOL_EM4WUPOL_GET(x)        (((x) >> 0) & 0x3f)
    #define EFM32_GPIO_EM4WUPOL_EM4WUPOL_A0            0x00000001
    #define EFM32_GPIO_EM4WUPOL_EM4WUPOL_C9            0x00000004
    #define EFM32_GPIO_EM4WUPOL_EM4WUPOL_F1            0x00000008
    #define EFM32_GPIO_EM4WUPOL_EM4WUPOL_F2            0x00000010
    #define EFM32_GPIO_EM4WUPOL_EM4WUPOL_E13           0x00000020

#define EFM32_GPIO_EM4WUCAUSE_ADDR                   0x0000013c
#define EFM32_GPIO_EM4WUCAUSE_MASK                   0x0000003f
/** Bit n indicates which pin the wake-up request occurred. @multiple */
  #define EFM32_GPIO_EM4WUCAUSE_EM4WUCAUSE(v)        ((EFM32_GPIO_EM4WUCAUSE_EM4WUCAUSE_##v) << 0)
  #define EFM32_GPIO_EM4WUCAUSE_EM4WUCAUSE_SET(x, v) do { (x) = (((x) & ~0x3f) | ((EFM32_GPIO_EM4WUCAUSE_EM4WUCAUSE_##v) << 0)); } while(0)
  #define EFM32_GPIO_EM4WUCAUSE_EM4WUCAUSE_SETVAL(x, v) do { (x) = (((x) & ~0x3f) | ((v) << 0)); } while(0)
  #define EFM32_GPIO_EM4WUCAUSE_EM4WUCAUSE_GET(x)    (((x) >> 0) & 0x3f)
    #define EFM32_GPIO_EM4WUCAUSE_EM4WUCAUSE_A0        0x00000001
    #define EFM32_GPIO_EM4WUCAUSE_EM4WUCAUSE_C9        0x00000004
    #define EFM32_GPIO_EM4WUCAUSE_EM4WUCAUSE_F1        0x00000008
    #define EFM32_GPIO_EM4WUCAUSE_EM4WUCAUSE_F2        0x00000010
    #define EFM32_GPIO_EM4WUCAUSE_EM4WUCAUSE_E13       0x00000020

#endif


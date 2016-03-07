/***************************************
* Auto generated by BFGen, do not edit *
***************************************/

/*
   bfgen -o cdefs -I /opt/bfgen/defs/efm32/efm32_prs.bf cdefs_use_reg_mask=1
*/

#ifndef _EFM32_PRS_BFGEN_DEFS_
#define _EFM32_PRS_BFGEN_DEFS_

#define EFM32_PRS_SWPULSE_ADDR                       0x00000000
#define EFM32_PRS_SWPULSE_MASK                       0x000000ff
/** Write to 1 to generate one HFPERCLK cycle high pulse. This pulse is XOR'ed
   with the corresponding bit in the SWLEVEL register and the selected PRS input
   signal to generate the channel output. @multiple */
  #define EFM32_PRS_SWPULSE_CH_COUNT               8
  #define EFM32_PRS_SWPULSE_CH(fidx)               (0x00000001 << ((fidx)))

#define EFM32_PRS_SWLEVEL_ADDR                       0x00000004
#define EFM32_PRS_SWLEVEL_MASK                       0x000000ff
/** The value in this register is XOR'ed with the corresponding bit in the
   SWPULSE register and the selected PRS input signal to generate the channel
   output. @multiple */
  #define EFM32_PRS_SWLEVEL_CH_COUNT               8
  #define EFM32_PRS_SWLEVEL_CH(fidx)               (0x00000001 << ((fidx)))

#define EFM32_PRS_CTRL_ADDR(ridx)                    (0x00000010 + (ridx) * 4)
#define EFM32_PRS_CTRL_COUNT                         8
#define EFM32_PRS_CTRL_MASK                          0x033f0007
/** Select signal input to PRS channel. @multiple */
  #define EFM32_PRS_CTRL_SIGSEL(v)                 ((v) << 0)
  #define EFM32_PRS_CTRL_SIGSEL_SET(x, v)          do { (x) = (((x) & ~0x7) | ((v) << 0)); } while(0)
  #define EFM32_PRS_CTRL_SIGSEL_GET(x)             (((x) >> 0) & 0x7)
/** Select input source to PRS channel. @multiple */
  #define EFM32_PRS_CTRL_SOURCESEL(v)              ((EFM32_PRS_CTRL_SOURCESEL_##v) << 16)
  #define EFM32_PRS_CTRL_SOURCESEL_SET(x, v)       do { (x) = (((x) & ~0x3f0000) | ((EFM32_PRS_CTRL_SOURCESEL_##v) << 16)); } while(0)
  #define EFM32_PRS_CTRL_SOURCESEL_GET(x)          (((x) >> 16) & 0x3f)
    #define EFM32_PRS_CTRL_SOURCESEL_NONE            0x00000000
    #define EFM32_PRS_CTRL_SOURCESEL_VCMP            0x00000001
    #define EFM32_PRS_CTRL_SOURCESEL_ACMP0           0x00000002
    #define EFM32_PRS_CTRL_SOURCESEL_ACMP1           0x00000003
    #define EFM32_PRS_CTRL_SOURCESEL_DAC0            0x00000006
    #define EFM32_PRS_CTRL_SOURCESEL_ADC0            0x00000008
    #define EFM32_PRS_CTRL_SOURCESEL_USART0          0x00000010
    #define EFM32_PRS_CTRL_SOURCESEL_USART1          0x00000011
    #define EFM32_PRS_CTRL_SOURCESEL_USART2          0x00000012
    #define EFM32_PRS_CTRL_SOURCESEL_TIMER0          0x0000001c
    #define EFM32_PRS_CTRL_SOURCESEL_TIMER1          0x0000001d
    #define EFM32_PRS_CTRL_SOURCESEL_TIMER2          0x0000001e
    #define EFM32_PRS_CTRL_SOURCESEL_RTC             0x00000028
    #define EFM32_PRS_CTRL_SOURCESEL_UART0           0x00000029
    #define EFM32_PRS_CTRL_SOURCESEL_GPIOL           0x00000030
    #define EFM32_PRS_CTRL_SOURCESEL_GPIOH           0x00000031
/** Select edge detection. @multiple */
  #define EFM32_PRS_CTRL_EDSEL(v)                  ((EFM32_PRS_CTRL_EDSEL_##v) << 24)
  #define EFM32_PRS_CTRL_EDSEL_SET(x, v)           do { (x) = (((x) & ~0x3000000) | ((EFM32_PRS_CTRL_EDSEL_##v) << 24)); } while(0)
  #define EFM32_PRS_CTRL_EDSEL_GET(x)              (((x) >> 24) & 0x3)
    #define EFM32_PRS_CTRL_EDSEL_OFF                 0x00000000
    #define EFM32_PRS_CTRL_EDSEL_POSEDGE             0x00000001
    #define EFM32_PRS_CTRL_EDSEL_PNEGEDGE            0x00000002
    #define EFM32_PRS_CTRL_EDSEL_BOTHEDGES           0x00000003

#endif


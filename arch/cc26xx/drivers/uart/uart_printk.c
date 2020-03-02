#include <hexo/types.h>
#include <hexo/endian.h>
#include <hexo/iospace.h>

#include <mutek/printk.h>
#include <mutek/startup.h>

#include <arch/cc26xx/memory_map.h>
#include <arch/cc26xx/prcm.h>
#include <arch/cc26xx/ioc.h>
#include <arch/cc26xx/gpio.h>
#include <arch/cc26xx/uart.h>


static void printk_out_char(uint8_t c)
{
  while (cpu_mem_read_32(CC26XX_UART0_BASE + CC26XX_UART_FR_ADDR)
    & CC26XX_UART_FR_TXFF);

  cpu_mem_write_8(CC26XX_UART0_BASE + CC26XX_UART_DR_ADDR, c);
}

static PRINTK_HANDLER(printk_out)
{
  size_t i;

  for (i = 0; i < len; i++)
    {
      if (str[i] == '\n')
        printk_out_char('\r');
      printk_out_char(str[i]);
    }
}

static void power_domain_on(void)
{
  uint32_t reg;

  //peripheral power domain on
  reg = cpu_mem_read_32(CC26XX_PRCM_BASE + CC26XX_PRCM_PDCTL0PERIPH_ADDR);
  reg |= CC26XX_PRCM_PDCTL0PERIPH_ON;
  cpu_mem_write_32(CC26XX_PRCM_BASE + CC26XX_PRCM_PDCTL0PERIPH_ADDR, reg);

  //serial power domain on
  reg = cpu_mem_read_32(CC26XX_PRCM_BASE + CC26XX_PRCM_PDCTL0SERIAL_ADDR);
  reg |= CC26XX_PRCM_PDCTL0SERIAL_ON;
  cpu_mem_write_32(CC26XX_PRCM_BASE + CC26XX_PRCM_PDCTL0SERIAL_ADDR, reg);

  //waiting for power
  while (!(cpu_mem_read_32(CC26XX_PRCM_BASE + CC26XX_PRCM_PDSTAT0_ADDR) &
    CC26XX_PRCM_PDSTAT0_SERIAL_ON));
}

static void clk_enable(void)
{
  uint32_t reg;

  //gpio clock enable
  reg = cpu_mem_read_32(CC26XX_PRCM_BASE + CC26XX_PRCM_GPIOCLKGR_ADDR);
  reg |= CC26XX_PRCM_GPIOCLKGR_CLK_EN;
  cpu_mem_write_32(CC26XX_PRCM_BASE + CC26XX_PRCM_GPIOCLKGR_ADDR, reg);

  //uart clock enable
  reg = cpu_mem_read_32(CC26XX_PRCM_BASE + CC26XX_PRCM_UARTCLKGR_ADDR);
  reg |= CC26XX_PRCM_UARTCLKGR_CLK_EN;
  cpu_mem_write_32(CC26XX_PRCM_BASE + CC26XX_PRCM_UARTCLKGR_ADDR, reg);

  //loading clocks modif
  reg = CC26XX_PRCM_CLKLOADCTL_LOAD;
  cpu_mem_write_32(CC26XX_PRCM_NONBUF_BASE + CC26XX_PRCM_CLKLOADCTL_ADDR, reg);

  //waiting for clocks
  while (!(cpu_mem_read_32(CC26XX_PRCM_BASE + CC26XX_PRCM_CLKLOADCTL_ADDR) &
    CC26XX_PRCM_CLKLOADCTL_LOAD_DONE));
}

static void iomux_init(void)
{
  uint32_t reg;

  //DIO3 as UART0 TX
  reg = 0;
  CC26XX_IOC_IOCFG_PORT_ID_SET(reg, UART0_TX);
  CC26XX_IOC_IOCFG_PULL_CTL_SET(reg, DIS);
  reg &= ~CC26XX_IOC_IOCFG_IE;
  cpu_mem_write_32(CC26XX_IOC_BASE + CC26XX_IOC_IOCFG_ADDR(3), reg);

  reg = cpu_mem_read_32(CC26XX_GPIO_BASE + CC26XX_GPIO_DOE31_0_ADDR);
  //enable DIO3 output
  reg |= CC26XX_GPIO_DOE31_0_DIO(3);
  cpu_mem_write_32(CC26XX_GPIO_BASE + CC26XX_GPIO_DOE31_0_ADDR, reg);
}

void cc26xx_printk_init()
{
  uint32_t reg;

  power_domain_on();
  clk_enable();
  iomux_init();

  //disable the uart
  cpu_mem_write_32(CC26XX_UART0_BASE + CC26XX_UART_CTL_ADDR, 0);

  //write the integer portion of the BRD
  reg = 0;
  //CC26XX_UART_IBRD_DIVINT_SET(reg, 0x138); //9600
  CC26XX_UART_IBRD_DIVINT_SET(reg, 0x1A);  //115200
  cpu_mem_write_32(CC26XX_UART0_BASE + CC26XX_UART_IBRD_ADDR, reg);

  //write the fractionnal portion of the BRD
  reg = 0;
  //CC26XX_UART_FBRD_DIVFRAC_SET(reg, 0); //9600
  CC26XX_UART_FBRD_DIVFRAC_SET(reg, 3); //115200
  cpu_mem_write_32(CC26XX_UART0_BASE + CC26XX_UART_FBRD_ADDR, reg);

  //write the desired serial parameters
  reg = 0;
  CC26XX_UART_LCRH_WLEN_SET(reg, 8);
  CC26XX_UART_LCRH_PEN_SET(reg, DIS);
  reg &= ~CC26XX_UART_LCRH_STP2;
  CC26XX_UART_LCRH_FEN_SET(reg, DIS);
  cpu_mem_write_32(CC26XX_UART0_BASE + CC26XX_UART_LCRH_ADDR, reg);

  //enable the uart
  reg = 0;
  CC26XX_UART_CTL_TXE_SET(reg, EN);
  CC26XX_UART_CTL_UARTEN_SET(reg, EN);
  cpu_mem_write_32(CC26XX_UART0_BASE + CC26XX_UART_CTL_ADDR, reg);

  static struct printk_backend_s backend;
  printk_register(&backend, printk_out);
}


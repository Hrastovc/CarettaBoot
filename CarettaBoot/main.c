/****************************************************************************************
 *   _____                         _     _             ____                    _   
 *  / ____|                       | |   | |           |  _ \                  | |  
 * | |        __ _   _ __    ___  | |_  | |_    __ _  | |_) |   ___     ___   | |_ 
 * | |       / _` | | '__|  / _ \ | __| | __|  / _` | |  _ <   / _ \   / _ \  | __|
 * | |____  | (_| | | |    |  __/ | |_  | |_  | (_| | | |_) | | (_) | | (_) | | |_ 
 *  \_____|  \__,_| |_|     \___|  \__|  \__|  \__,_| |____/   \___/   \___/   \__|
 *
 ***************************************************************************************/

/************************************************************************************//**
* \file       main.c
* \brief      This is a bootloader for an open source project CarettaBMS.
*             Build upon Optiboot bootloader: https://github.com/Optiboot/optiboot.
****************************************************************************************/


/****************************************************************************************
* Include files
****************************************************************************************/
#include <avr/io.h>
#include <stdint.h>
#include <avr/signature.h>
#include <avr/pgmspace.h>
#include "command.h"


/****************************************************************************************
* Macro definitions
****************************************************************************************/
#define CARETTABOOT_MAJVER 0
#define CARETTABOOT_MINVER 1

#define UR_BOOT_FLAG_PTR  *(volatile uint8_t *)(&USERROW_USERROW31)
#define UR_MODULE_NUM_PTR *(volatile uint8_t *)(&USERROW_USERROW30)


/****************************************************************************************
* Function prototypes
****************************************************************************************/
void write(uint8_t ch);
uint8_t read(void) ;
uint8_t readWrite(void);
void writeIfBot(uint8_t ch);
void syncResponse(void);
void watchdogConfig(uint8_t x);
void readMultiple(uint8_t);
void writeUserRow(uint8_t offset, uint8_t val);
void writePageSPM(void);
uint8_t doCRCcheck();

typedef union {
  uint8_t  *bptr;
  uint16_t *wptr;
  uint16_t word;
  uint8_t bytes[2];
} addr16_t;


/************************************************************************************//**
** \brief     This is main program entry point
**
****************************************************************************************/
__attribute__((naked)) __attribute__((section(".ctors"))) void bootloader(void)
{
  /* Initialize system for AVR GCC support, expects r1 = 0 */
  asm volatile("clr r1");
  
  if((UR_BOOT_FLAG_PTR == 0xEA) && !(RSTCTRL_RSTFR & RSTCTRL_PORF_bm))
  {
    writeUserRow(31, 0xEB);
    if(doCRCcheck()) asm("jmp 0x0200");
    write(0xCE);
  }
  
  uint8_t ch;
  uint8_t length;
  uint8_t memtype;
  addr16_t address;
  
  watchdogConfig(WDT_PERIOD_1KCLK_gc);
  
  /* Clear Power-On Reset Flag */
  RSTCTRL_RSTFR = RSTCTRL_PORF_bm;
  VREF_CTRLA = VREF_DAC0REFSEL_4V34_gc;
  AC0_MUXCTRLA = AC_MUXNEG_VREF_gc;
  AC0_CTRLA = AC_HYSMODE_50mV_gc | AC_ENABLE_bm;
  PORTA_PIN6CTRL = PORT_INVEN_bm;
  USART0_BAUD = 1389; /* 9600 BAUD */
  USART0_CTRLB = USART_RXEN_bm | USART_TXEN_bm;
  PORTA_DIRSET = PIN6_bm;
  
  for(;;)
  {
    ch = readWrite();
    
    /*  */
    if(ch == Cmnd_STK_GET_PARAMETER)
    {
      ch = readWrite();
      syncResponse();
      if(ch == Parm_STK_SW_MINOR) writeIfBot(CARETTABOOT_MINVER);
      else if(ch == Parm_STK_SW_MAJOR) writeIfBot(CARETTABOOT_MAJVER);
      else writeIfBot(0x00);
    }
    
    /*  */
    else if(ch == Cmnd_STK_SET_PARAMETER)
    {
      if(readWrite() == Parm_STK_DEVICE)
      {
        ch = read() - 1;
        writeUserRow(30, ch);
        write(ch);
      }
      syncResponse();
    }
    
    /* SET DEVICE is ignored */
    else if(ch == Cmnd_STK_SET_DEVICE)
    {
      readMultiple(20);
      /* devicecode
       * revision
       * progtype
       * parmode
       * polling
       * selftimed
       * lockbytes
       * fusebytes
       * flashpollval1
       * flashpollval2
       * eeprompollval1
       * eeprompollval2
       * pagesizehigh
       * pagesizelow
       * eepromsizehigh
       * eepromsizelow
       * flashsize4
       * flashsize3
       * flashsize2
       * flashsize1
       * Sync_CRC_EOP
       */
    }
    
    /* SET DEVICE EXT is ignored */
    else if(ch == Cmnd_STK_SET_DEVICE_EXT)
    {
      readMultiple(4);
      /* commandsize
       * eeprompagesize
       * signalpagel
       * signalbs2
       * Sync_CRC_EOP
       */
    }
    
    /* LOAD ADDRESS */
    else if(ch == Cmnd_STK_LOAD_ADDRESS)
    {
      address.bytes[0] = readWrite();
      address.bytes[1] = readWrite();
      syncResponse();
    }
    
    /* UNIVERSAL command is ignored */
    else if(ch == Cmnd_STK_UNIVERSAL)
    {
      readMultiple(4);
      /* byte1
       * byte2
       * byte3
       * byte4
       */
      writeIfBot(0x00);
    }
    
    /* Write memory, length is big endian and is in bytes */
    else if(ch == Cmnd_STK_PROG_PAGE)
    {
      readWrite();
      length = readWrite();
      memtype = readWrite();
      if(memtype == 'F') address.word += MAPPED_PROGMEM_START;
      if(memtype == 'E') address.word += MAPPED_EEPROM_START;
      do *(address.bptr++) = readWrite();
      while(--length);
      syncResponse();
      writePageSPM();
    }
    
    /* Read memory block mode, length is big endian.  */
    else if(ch == Cmnd_STK_READ_PAGE)
    {
      readWrite();
      length = readWrite();
      readWrite();
      syncResponse();
      do writeIfBot(*(address.bptr++));
      while(--length);
    }

    /* Get device signature bytes  */
    else if(ch == Cmnd_STK_READ_SIGN)
    {
      syncResponse();
      writeIfBot(SIGNATURE_0);
      writeIfBot(SIGNATURE_1);
      writeIfBot(SIGNATURE_2);
    }  
      
    /*  */
    else if(ch == Cmnd_STK_LEAVE_PROGMODE)
    {
      syncResponse();
      if(!doCRCcheck()) while(1);
      writeUserRow(31, 0xEA);
    }
    
    /* Go to bootloader command */
    else if(ch == 'b')
    {
      /* Do not send anything, downstream bus is configured as 9O1! */
      while(1);
    }
    
    /*  */
    else
    {
      syncResponse();
    }
    
    writeIfBot(Resp_STK_OK);
  }
} /*** end of bootloader ***/


/************************************************************************************//**
** \brief     UART write byte
** \param     ch Byte to send
**
****************************************************************************************/
void write(uint8_t ch)
{
  while(!(USART0_STATUS & USART_DREIF_bm));
  USART0_TXDATAL = ch;
} /*** end of write ***/


/************************************************************************************//**
** \brief     UART read byte
** \return    USART0_RXDATAL Byte from UART RXD register
**
****************************************************************************************/
uint8_t read(void)
{
  do
  {
    if(!(AC0_STATUS & AC_STATE_bm)) PORTA_PIN7CTRL = 0;
    else PORTA_PIN7CTRL = PORT_INVEN_bm;
  }
  while(!(USART0_STATUS & USART_RXCIF_bm));
  asm("wdr");
  return USART0_RXDATAL;
} /*** end of read ***/


/************************************************************************************//**
** \brief     UART read byte and write it
** \return    ch Byte received
**
****************************************************************************************/
uint8_t readWrite(void)
{
  uint8_t ch = read();
  if(UR_MODULE_NUM_PTR) write(ch);
  return ch;
} /*** end of readWrite ***/


/************************************************************************************//**
** \brief     UART write byte if this is the bottom module
** \param     ch Byte to send
**
****************************************************************************************/
void writeIfBot(uint8_t ch)
{
  if(!UR_MODULE_NUM_PTR) write(ch);
} /*** end of writeIfBot ***/


/************************************************************************************//**
** \brief     UART read multiple bytes, but discard them
** \param     count of bytes to read
**
****************************************************************************************/
void readMultiple(uint8_t count)
{
  do readWrite();
  while(--count);
  syncResponse();
} /*** end of readMultiple ***/


/************************************************************************************//**
** \brief     Send sync response
**
****************************************************************************************/
void syncResponse(void)
{
  if(readWrite() != Sync_CRC_EOP)
  {
    watchdogConfig(WDT_PERIOD_8CLK_gc);
    while(1);
  }
  writeIfBot(Resp_STK_INSYNC);
} /*** end of syncResponse ***/


/************************************************************************************//**
** \brief     Configure watchdog timer
** \param     val for watchdog control A register
**
****************************************************************************************/
void watchdogConfig(uint8_t val)
{
  while(WDT_STATUS & WDT_SYNCBUSY_bm);
  _PROTECTED_WRITE(WDT_CTRLA, val);
} /*** end of watchdogConfig ***/


/************************************************************************************//**
** \brief     writeUserRow
** \param     position
** \param     val
**
****************************************************************************************/
void writeUserRow(uint8_t offset, uint8_t val)
{
  *(volatile uint8_t *)(&USERROW_USERROW0 + offset) = val;
  writePageSPM();
} /*** end of writeUserRow ***/


/************************************************************************************//**
** \brief     Write self programmable memory page
**
****************************************************************************************/
void writePageSPM(void)
{
  _PROTECTED_WRITE_SPM(NVMCTRL_CTRLA, NVMCTRL_CMD_PAGEERASEWRITE_gc);
  while(NVMCTRL_STATUS & (NVMCTRL_FBUSY_bm | NVMCTRL_EEBUSY_bm));
} /*** end of writePageSPM ***/


/************************************************************************************//**
** \brief     Do flash CRC check
** \return    CRCSCAN_STATUS CRC check status
**
****************************************************************************************/
uint8_t doCRCcheck()
{
  CRCSCAN_CTRLB = CRCSCAN_SRC_FLASH_gc;
  CRCSCAN_CTRLA = CRCSCAN_ENABLE_bm;
  while(CRCSCAN_STATUS & CRCSCAN_BUSY_bm);
  return CRCSCAN_STATUS;
}


/************************************ end of main.c ************************************/
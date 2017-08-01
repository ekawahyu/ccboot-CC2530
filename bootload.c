/*
 * bootload.c
 *
 * Created on: May 29, 2014
 *     Author: Ekawahyu Susilo
 *
 * Copyright (c) 2014, Chongqing Aisenke Electronic Technology Co., Ltd.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDER AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * The views and conclusions contained in the software and documentation are
 * those of the authors and should not be interpreted as representing official
 * policies, either expressed or implied, of the copyright holder.
 *
 */

#include "compiler.h"
#include "cc253x.h"
#include "sfr-bits.h"
#include "errnum.h"
#include "clock.h"
#include "uart.h"
#include "flash.h"
#include "radio.h"
#include "bootload.h"

/* string constants within code memory */

__code const uint8_t txt_space[] = " ";
__code const uint8_t txt_openbracket[] = "(";
__code const uint8_t txt_closebracket[] = ")";
__code const uint8_t txt_colon[] = ":";
__code const uint8_t txt_press_button[] = "passkey to flash ";
__code const uint8_t txt_running[] = "running...";
__code const uint8_t txt_ccboot[] = "CCBOOT v0.1 - CC253x";
__code const uint8_t txt_xmodem[] = "- xmodem 115200 bps";
__code const uint8_t txt_filetype[] = "- user program at 0x1000";
__code const uint8_t txt_please_send[] = "please send a file (ENTER to cancel)... ";
__code const uint8_t txt_rebooting[] = "rebooting...";
__code const uint8_t txt_done[] = "done!";
__code const uint8_t txt_remote[] = "waiting for remote flash...";
__code const uint8_t txt_remote_client[] = "waiting for remote client...";
__code const uint8_t txt_wait_file[] = "waiting for file...";
__code const uint8_t txt_connected[] = "connected!";
__code const uint8_t txt_noreply[] = "no reply!";

/* define the passkey and the length here, please a unique passkey
 * starting with character other than '0-9', 'A-Z', or 'a-z' like '@'
 */

#define PASSKEY_LENGTH 5
__code const uint8_t txt_passkey[] = "@boot";
#define RFPASSKEY_LENGTH 7
__code const uint8_t txt_rfpasskey[] = "@rfboot";

/* allocates variables in the upper XDATA memory area, see Makefile */

__xdata uint8_t flash_buffer[FLASH_PAGE_SIZE];
__xdata uint8_t rf_data[RFDATA_SIZE];
__xdata uint8_t xmodem_buffer[XMODEM_PAGE_SIZE];
__xdata uint8_t * pxmodem_buffer;
__xdata uint8_t * pflash_buffer;
__xdata uint8_t flash_page_number;
__xdata uint8_t flashing_mode;
__xdata uint8_t passkey_index;
__xdata uint8_t rfpasskey_index;
__xdata uint8_t interceptpasskey_index;
//__xdata uint8_t jmp_code[3];

__xdata uint8_t *macp = &X_IEEE_ADDR;

/* function prototypes */

void lnprint (uint8_t * ptext);

//---------------------------------------------------------------------------//
//  URX0_interceptor (void): serial port listener/interceptor                //
//---------------------------------------------------------------------------//
//    input:    none, it gets called by interrupt vector                     //
//    return:   if keypressed/passkey is matched, it invokes bootloader      //
//                                                                           //
//    this service routine listens/intercepts incoming serial data. if a     //
//    a key or a sequence of keys are matched, it gives a bootloader a full  //
//    control over the cpu.                                                  //
//---------------------------------------------------------------------------//

void URX0_interceptor (void)
{
  int16_t serial_data;

  serial_data = U0DBUF;
  if (serial_data == txt_passkey[passkey_index])
    passkey_index++;
  else
    passkey_index = 0;
  if (serial_data == txt_rfpasskey[rfpasskey_index])
    rfpasskey_index++;
  else
    rfpasskey_index = 0;

  if (passkey_index == PASSKEY_LENGTH) {
    passkey_index = 0;
    flashing_mode = LOCAL_FLASH;
    bootloader();
  }
  if (rfpasskey_index == RFPASSKEY_LENGTH) {
    rfpasskey_index = 0;
    flashing_mode = WIRELESS_FLASH;
    bootloader();
  }
}

//---------------------------------------------------------------------------//
//  RF_interceptor (void): radio packet listener/interceptor                 //
//---------------------------------------------------------------------------//
//    input:    none, it is called by interrupt vector                       //
//    return:   if keypressed/passkey is matched, it invokes rfbootloader    //
//                                                                           //
//    this service routine listens/intercepts incoming radio packet. if a    //
//    a key or a sequence of keys are matched, it gives a rfbootloader a     //
//    full control over the cpu. the cpu handover to user application        //
//    depends on bootloader decision.                                        //
//---------------------------------------------------------------------------//

void RF_interceptor(void) {
  __xdata uint8_t * prf_data;
  uint8_t length;
  uint8_t i;

  /* clear RF interrupt flag RFIF_0 and RFIF_1 */
  S1CON &= ~0x03;

  if (RFIRQF0 & FIFOP) {
    /* put RX data in RX buffer for user program to access
     * user program cannot get the data from RFD anymore
     */
    prf_data = rf_data;
    length = RFD;
    *prf_data++ = length;
    for (i = 0; i < length; i++)
    {
      *prf_data = RFD;
      if (*prf_data == txt_rfpasskey[interceptpasskey_index])
        interceptpasskey_index++;
      else
        interceptpasskey_index = 0;

      prf_data++;

      if (interceptpasskey_index == RFPASSKEY_LENGTH)
      {
        interceptpasskey_index = 0;
        flashing_mode = WIRELESS_FLASH;
        rfbootloader();
        return;
      }
    }

    //RFST = ISFLUSHRX; //-----> this can be done here or in user program

    /* clearing the FIFOP interrupt flag */
    //RFIRQF0 &= ~FIFOP;     //-------> has to be done in user program
  }
}

//---------------------------------------------------------------------------//
//  RFERR_interceptor (void): radio state machine error listener/interceptor //
//---------------------------------------------------------------------------//
//    input:    none, it is called by interrupt vector                       //
//    return:   none, only clearing interrupt flag and recover radio state   //
//                                                                           //
//    this service routine listens/intercepts incoming interrupt radio error.//
//    user program may include this as well, however it is unnecessary. you  //
//    could remove RFERR interceptor from the bootloader section only if you //
//    upload user program for the very first time through serial port.       //
//    otherwise the bootloader gets stuck without the error handler when the //
//    user program is empty.
//---------------------------------------------------------------------------//

void RFERR_interceptor (void) {

  // If RX overflow occurs, reset everything.
   if(FSMSTAT0 == 17) {
     RFST = ISRFOFF;
     RFST = ISFLUSHRX;
     RFST = ISFLUSHRX;
     RFST = ISRXON;
   }
   else if(FSMSTAT0 == 56) {
     RFST = ISFLUSHTX;
   }

   // clear RFERR interrupt flag
   RFERRIF = 0;
}

//---------------------------------------------------------------------------//
//  lnprint ("string"): put string to standard output                        //
//---------------------------------------------------------------------------//
//    input:   array/string pointer                                          //
//    return:  none                                                          //
//---------------------------------------------------------------------------//

void lnprint (uint8_t * ptext)
{
  putchar(0x0D);
  putchar(0x0A);
  do {
    putchar(*ptext);
    ptext++;
  } while (*ptext);
}

void print (uint8_t * ptext)
{
  do {
    putchar(*ptext);
    ptext++;
  } while (*ptext);
}

void printhex (uint8_t number)
{
  uint8_t high_number, low_number;

  high_number = (number & 0xF0) >> 4;
  low_number = number & 0x0F;

  if (high_number > 9) putchar(high_number + 55); else putchar(high_number + 48);
  if (low_number > 9) putchar(low_number + 55); else putchar(low_number + 48);
}

//---------------------------------------------------------------------------//
//  ccboot_getchar (timeout): get one character from serial port             //
//---------------------------------------------------------------------------//
//    input:   timeout period                                                //
//    return:  0-255 ascii received                                          //
//    return:  -1 timeout detected                                           //
//---------------------------------------------------------------------------//

int16_t ccboot_getchar (uint16_t timeout)
{
  uint8_t serial_data;

  U0CSR &= ~0x04;
  do {
    timeout--;
  } while (!(U0CSR & 0x04) && timeout);

  if (U0CSR & 0x04) {
    serial_data = U0DBUF;
    U0CSR &= ~0x04;
    return serial_data;
  }
  else {
    U0CSR &= ~0x04;
    return ERR_TIMEOUT_DETECTED;
  }
}

//---------------------------------------------------------------------------//
//  xmodem_wait_for_sender (timeout): waiting until sender ready to transfer //
//---------------------------------------------------------------------------//
//    input:   timeout period                                                //
//    return:  0-255 ascii received                                          //
//    return:  -1 timeout detected                                           //
//---------------------------------------------------------------------------//

int16_t xmodem_wait_for_sender(uint8_t timeout)
{
  int16_t serial_data;

  /* sender waits for NACK (XMODEM) to start data transmission */
  do {
    serial_data = ccboot_getchar(60000);
  } while (serial_data == ERR_TIMEOUT_DETECTED && timeout--);

  putchar(XMODEM_NACK);
  serial_data = ccboot_getchar(60000);

  return serial_data;
}

//---------------------------------------------------------------------------//
//  flash_packets (void)                                                     //
//---------------------------------------------------------------------------//
//    input:    none                                                         //
//    return:   erase, flash, and increase page number counter               //
//---------------------------------------------------------------------------//

void flash_packets (void)
{
  if (flash_page_number >= USER_PROGRAM_PAGE) {
    flash_erase_page(flash_page_number);
    flash_dma_write(flash_buffer, FLASH_PAGE_SIZE, flash_page_number << 11);
  }
  flash_page_number++;
}

//---------------------------------------------------------------------------//
//  send_flash_buffer_over_radio (packet status)                             //
//---------------------------------------------------------------------------//
//    input:    status from xmodem packet getter                             //
//    return:   straightly forward the status from xmodem packet getter      //
//              1 sending successful                                         //
//              0 end of sending                                             //
//              -1 sending error                                             //
//---------------------------------------------------------------------------//

int16_t send_flash_buffer_over_radio(int16_t xmodem_get_packets)
{
  return 0;
}

//---------------------------------------------------------------------------//
//  bootloader (void): bootloader program                                    //
//---------------------------------------------------------------------------//
//    input:    none, it gets called when a passkey is received              //
//    return:   jump to user program after flashing or canceled by           //
//              rebooting the system                                         //
//---------------------------------------------------------------------------//

void bootloader (void)
{
  int16_t serial_data = 0;
  int16_t flash_successful = 0;
  int16_t i, page_count = 0;
  uint8_t checksum = 0;

  clock_init();
  uart_init();
  EA = 0;     /* disable global interrupt */

  lnprint((uint8_t *) txt_space);
  lnprint((uint8_t *) txt_ccboot);

  macp += 7;
  print((uint8_t *) txt_space);
  print((uint8_t *) txt_openbracket);
  printhex(*macp--);
  print((uint8_t *) txt_colon);
  printhex(*macp--);
  print((uint8_t *) txt_colon);
  printhex(*macp--);
  print((uint8_t *) txt_colon);
  printhex(*macp--);
  print((uint8_t *) txt_colon);
  printhex(*macp--);
  print((uint8_t *) txt_colon);
  printhex(*macp--);
  print((uint8_t *) txt_colon);
  printhex(*macp--);
  print((uint8_t *) txt_colon);
  printhex(*macp--);
  print((uint8_t *) txt_closebracket);

  lnprint((uint8_t *) txt_xmodem);
  lnprint((uint8_t *) txt_filetype);
  lnprint((uint8_t *) txt_space);
  lnprint((uint8_t *) txt_please_send);

  pxmodem_buffer = xmodem_buffer;
  pflash_buffer = flash_buffer;
  flash_clear_buffer(flash_buffer);
  flash_page_number = 0;

  serial_data = xmodem_wait_for_sender(XMODEM_TIMEOUT);

  /* XMODEM_SOH is detected */
  if (serial_data == XMODEM_SOH) {

    /* put it in the xmodem buffer */
    *pxmodem_buffer = (uint8_t)serial_data;
    checksum += serial_data;
    pxmodem_buffer++;

    /* start receiving the rest of packets */
    do {
      serial_data = ccboot_getchar(60000);

      *pxmodem_buffer = (uint8_t)serial_data;
      pxmodem_buffer++;

      /* end of transmission */
      if (xmodem_buffer[0] == XMODEM_EOT) {
        pxmodem_buffer = xmodem_buffer;
        /* do write the flash buffer if there is left */
        /* TODO test for flash memory overflow */
        flash_packets();
        putchar(XMODEM_ACK);
        /* done, reboot the system */
        WDCTL = 0x0B;
        while(1);
      }

      /* when xmodem buffer is full */
      if (pxmodem_buffer == (xmodem_buffer + XMODEM_PAGE_SIZE)) {
        if (*(pxmodem_buffer-1) ==  checksum) {
          /* skip SOH and page numbering */
          pxmodem_buffer = xmodem_buffer + 3;
          /* transfer xmodem payload to flash buffer */
          for (i = 0; i < XMODEM_PAYLOAD_SIZE; i++) {
            *pflash_buffer++ = *pxmodem_buffer++;
          }
          /* do write when the flash buffer is full */
          if (pflash_buffer == (flash_buffer + FLASH_PAGE_SIZE)) {
            /* TODO test for flash memory overflow */
            flash_packets();
            pflash_buffer = flash_buffer;
            flash_clear_buffer(flash_buffer);
          }
          putchar(XMODEM_ACK);
        } else {
          putchar(XMODEM_NACK);
        }
        pxmodem_buffer = xmodem_buffer;
        checksum = 0;
      } else {
        checksum += serial_data;
      }
    } while(1);
  }
  else {
    /* nothing to do, reboot the system */
    WDCTL = 0x0B;
    while(1);
  }

  /* bootloader never arrives here, it exits to system reset vector
   * after flashing done/failed/canceled.
   */
}

//---------------------------------------------------------------------------//
//  rfbootloader (void): rfbootloader program                                    //
//---------------------------------------------------------------------------//
//    input:    none, it gets called when a rfpasskey is received              //
//    return:   jump to user program after flashing or canceled by           //
//              rebooting the system                                         //
//---------------------------------------------------------------------------//

void rfbootloader(void)
{

}

int8_t rfxmodem_handshake (void)
{
  return 0;
}

//---------------------------------------------------------------------------//
//  main program                                                             //
//---------------------------------------------------------------------------//
//    input:   none                                                          //
//    return:  never returns, it gives full control to user program          //
//                                                                           //
//    when no key or passkey are given, the bootloader with load             //
//    user program. if the serial interrrupt remains active, the bootloader  //
//    can be called within user application.                                 //
//---------------------------------------------------------------------------//

void main (void)
{
  uint8_t timeout = BOOTLOADER_TIMEOUT;

  clock_init();
  uart_init();
  EA = 1;     /* enable global interrupt */

  passkey_index = 0;

  lnprint((uint8_t *) txt_space);
  lnprint((uint8_t *) txt_press_button);

  do {
    ccboot_getchar(60000);
    putchar('*');
    timeout--;
  } while (timeout);

  lnprint((uint8_t *) txt_running);

  __asm_begin
  ASM (ljmp USER_PROGRAM_PAGE * 0x800)
  __asm_end;

  /* the bootloader never arrives to this area */
}

void isr_uart0_rx (void) __interrupt (INUM_URX0) __naked
{
  __asm_begin
  ASM (push acc)
  ASM (push b)
  ASM (push dpl)
  ASM (push dph)
  ASM (push 0x02)
  ASM (push 0x03)
  ASM (push 0x04)
  ASM (push 0x05)
  ASM (push 0x06)
  ASM (push 0x07)
  ASM (push 0x00)
  ASM (push 0x01)
  ASM (push psw)
  __asm_end;

     URX0_interceptor();

  __asm_begin
  ASM (pop psw)
  ASM (pop 0x01)
  ASM (pop 0x00)
  ASM (pop 0x07)
  ASM (pop 0x06)
  ASM (pop 0x05)
  ASM (pop 0x04)
  ASM (pop 0x03)
  ASM (pop 0x02)
  ASM (pop dph)
  ASM (pop dpl)
  ASM (pop b)
  ASM (pop acc)
  __asm_end;

  __asm_begin
  ASM (ljmp (USER_PROGRAM_PAGE * 0x800) + 0x13)
  __asm_end;
}

void isr_uart0_tx (void) __interrupt (INUM_UTX0)
{
  __asm_begin
  ASM (ljmp (USER_PROGRAM_PAGE * 0x800) + 0x3B)
  __asm_end;
}

void RFERR_VECTOR_redirected(void) __interrupt (INUM_RFERR)
{
  __asm_begin
  ASM (ljmp (USER_PROGRAM_PAGE * 0x800) + 0x03)
  __asm_end;
}

void ADC_VECTOR_redirected(void) __interrupt (INUM_ADC)
{
  __asm_begin
  ASM (ljmp (USER_PROGRAM_PAGE * 0x800) + 0x0B)
  __asm_end;
}

void URX1_VECTOR_redirected(void) __interrupt (INUM_URX1)
{
  __asm_begin
  ASM (ljmp (USER_PROGRAM_PAGE * 0x800) + 0x1B)
  __asm_end;
}

void ENC_VECTOR_redirected(void) __interrupt (INUM_ENC)
{
  __asm_begin
  ASM (ljmp (USER_PROGRAM_PAGE * 0x800) + 0x23)
  __asm_end;
}

void ST_VECTOR_redirected(void) __interrupt (INUM_ST)
{
  __asm_begin
  ASM (ljmp (USER_PROGRAM_PAGE * 0x800) + 0x2B)
  __asm_end;
}

void P2INT_VECTOR_redirected(void) __interrupt (INUM_P2INT)
{
  __asm_begin
  ASM (ljmp (USER_PROGRAM_PAGE * 0x800) + 0x33)
  __asm_end;
}

void DMA_VECTOR_redirected(void) __interrupt (INUM_DMA)
{
  __asm_begin
  ASM (ljmp (USER_PROGRAM_PAGE * 0x800) + 0x43)
  __asm_end;
}

void T1_VECTOR_redirected(void) __interrupt (INUM_T1)
{
  __asm_begin
  ASM (ljmp (USER_PROGRAM_PAGE * 0x800) + 0x4B)
  __asm_end;
}

void T2_VECTOR_redirected(void) __interrupt (INUM_T2)
{
  __asm_begin
  ASM (ljmp (USER_PROGRAM_PAGE * 0x800) + 0x53)
  __asm_end;
}

void T3_VECTOR_redirected(void) __interrupt (INUM_T3)
{
  __asm_begin
  ASM (ljmp (USER_PROGRAM_PAGE * 0x800) + 0x5B)
  __asm_end;
}

void T4_VECTOR_redirected(void) __interrupt (INUM_T4)
{
  __asm_begin
  ASM (ljmp (USER_PROGRAM_PAGE * 0x800) + 0x63)
  __asm_end;
}

void P0INT_VECTOR_redirected(void) __interrupt (INUM_P0INT)
{
  __asm_begin
  ASM (ljmp (USER_PROGRAM_PAGE * 0x800) + 0x6B)
  __asm_end;
}

void UTX1_VECTOR_redirected(void) __interrupt (INUM_UTX1)
{
  __asm_begin
  ASM (ljmp (USER_PROGRAM_PAGE * 0x800) + 0x73)
  __asm_end;
}

void P1INT_VECTOR_redirected(void) __interrupt (INUM_P1INT)
{
  __asm_begin
  ASM (ljmp (USER_PROGRAM_PAGE * 0x800) + 0x7B)
  __asm_end;
}

void RF_VECTOR_redirected(void) __interrupt (INUM_RF)
{
  __asm_begin
  ASM (ljmp (USER_PROGRAM_PAGE * 0x800) + 0x83)
  __asm_end;
}

void WDT_VECTOR_redirected(void) __interrupt (INUM_WDT)
{
  __asm_begin
  ASM (ljmp (USER_PROGRAM_PAGE * 0x800) + 0x8B)
  __asm_end;
}

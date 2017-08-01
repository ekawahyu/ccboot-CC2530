/*
 * bootload.h
 *
 * Created on: May 31, 2014
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

#ifndef BOOTLOAD_H_
#define BOOTLOAD_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define INUM_RFERR 0
#define INUM_ADC   1
#define INUM_URX0  2
#define INUM_URX1  3
#define INUM_ENC   4
#define INUM_ST    5
#define INUM_P2INT 6
#define INUM_UTX0  7
#define INUM_DMA   8
#define INUM_T1    9
#define INUM_T2    10
#define INUM_T3    11
#define INUM_T4    12
#define INUM_P0INT 13
#define INUM_UTX1  14
#define INUM_P1INT 15
#define INUM_RF    16
#define INUM_WDT   17

#define BOOTLOADER_TIMEOUT    10    /* min value is 1 = 0.1 sec */
#define XMODEM_TIMEOUT        150   /* minimum value is 1 = 0.1 sec */

#define XMODEM_SOH            0x01
#define XMODEM_EOT            0x04
#define XMODEM_ACK            0x06
#define XMODEM_NACK           0x15
#define XMODEM_ETB            0x17
#define XMODEM_CAN            0x18
#define XMODEM_C              0x43
#define XMODEM_BAUDRATE       BAUD115200

#define XMODEM_PAGE_SIZE      132
#define XMODEM_PAYLOAD_SIZE   128

#define USER_PROGRAM_PAGE     2

#define RFDATA_SIZE           128
#define RADIO_FRAME_SIZE      64    // < 114 bytes 802.15.4 DATA FRAME TYPE

#define LOCAL_FLASH           0
#define WIRELESS_FLASH        1

extern __xdata uint8_t flash_page_number;

void URX0_interceptor(void);
void RF_interceptor(void);
void RFERR_interceptor(void);
void bootloader(void);
void rfbootloader(void);
int8_t rfxmodem_handshake (void);

void lnprint (uint8_t * ptext);

#ifdef __cplusplus
}
#endif

#endif /* BOOTLOAD_H_ */

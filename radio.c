/*
 * radio.c
 *
 * Created on: Nov 4, 2014
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

#include <stdint.h>
#include "cc253x.h"
#include "sfr-bits.h"
#include "radio.h"

/* Local RF Flags */
#define RX_ACTIVE  0x80
#define WAS_OFF    0x10
#define RF_ON      0x01

extern __xdata uint8_t *macp;

static uint8_t rf_flags;

uint8_t radio_open (uint8_t channel, uint16_t panid, uint16_t address)
//----------------------------------------------------------------------------
// See radio.h for a description of this function.
//----------------------------------------------------------------------------
{
  // clear the RF interrupt flags and disable all mask
  RFIRQF0 = 0;
  RFIRQF1 = 0;
  // clear RFERR interrupt flag
  RFERRIF = 0;
  // clear RF interrupt flag RFIF_0 and RFIF_1
  S1CON &= ~0x03;
  // disable RF interrupt
  IEN2 &= ~0x01;
  // disable RFERR interrupt
  RFERRIE = 0;

  radio_set_channel (channel);

  //turning on power to analog part of radio and waiting for voltage regulator.
  radio_power_on();

  RXCTRL = 0x3F;
  FSCTRL = 0x55;

  // turning on Address Decoding ADDR_DECODE
  MDMCTRL0H |= 0x08;
  // turning off Address Decoding ADDR_DECODE
  //MDMCTRL0H &= ~0x08;
  // turning on AUTO_CRC and AUTO_ACK
  MDMCTRL0L |= 0x30;
  // set CCA on, while RSSI below threshold and not receiving packets
  MDMCTRL0L |= 0xC0;
  // use SLOTTED_ACK (auto acknowledge happens 12 -30 symbols later)
  //MDMCTRL1H |= 0x80;
  // turning on AUTO_TX2RX and off RX2RX 12 symbols delay
  FSMTC1 &= ~0x0C;
  // turning on ACK packet reception
  FSMTC1 |= 0x01;
  // turning off abortRxOnSrxon.
  FSMTC1 &= ~0x20;
  // control the transmitter power (default 0x7F)
  //TXCTRLL = 0xFF;

  // Setting the number of bytes to assert the FIFOP flag
  IOCFG0 = 0x7F;  // Put the RXFIFOP to maximum value so that it would raise
          // only when the buffer is full or frame is completed.

  // Flushing both TX and RXx FIFO. The flush-RX is issued twice to reset the SFD.
  // Calibrating the radio and turning on RX.
  RFST = SRXON;
  RFST = SFLUSHTX;
  RFST = SFLUSHRX;
  RFST = SFLUSHRX;
  RFST = STXCALN;
  RFST = ISSTART;

  // enable TXACK in IRQSRC to interrupt when the AUTO_ACK is sent
  //IRQSRC |=0x01;
  // enable FIFOP interrupt
  RFIM |= IM_FIFOP;
  // enable RF interrupt
  IEN2 |= 0x01;
  // enable RFERR interrupt
  RFERRIE = 1;

  PANIDH = (uint8_t) ((panid & 0xFF00) >> 8);
  PANIDL = (uint8_t) (panid & 0x00FF);
  SHORTADDRH = (uint8_t) ((address & 0xFF00) >> 8);
  SHORTADDRL = (uint8_t) (address & 0x00FF);

  return RADIO_SUCCESS;
}

void radio_power_on (void)
//----------------------------------------------------------------------------
// See radio.h for a description of this function.
//----------------------------------------------------------------------------
{
  if(!(rf_flags & RX_ACTIVE)) {
    RFST = ISFLUSHRX;
    RFST = ISFLUSHRX;
    RFST = ISRXON;

    rf_flags |= RX_ACTIVE;
  }
}

uint8_t radio_set_channel (uint8_t channel)
//----------------------------------------------------------------------------
// See radio.h for a description of this function.
//----------------------------------------------------------------------------
{
  //if (radio_status == RADIO_BUSY_TX | radio_status == RADIO_BUSY_RX |
  //    radio_status == RADIO_POWER_OFF) return RADIO_FAILED;

  // setting the frequency based on the channel number
  FREQCTRL = 11 + 5 * (channel - 11);

  return RADIO_SUCCESS;
}

//----------------------------------------------------------------------------
// See hal2430.h for a description of this function.
//----------------------------------------------------------------------------

void hal_radio_send_packet (__xdata uint8_t * pradio_packet, uint8_t length)
{
  uint8_t i;

  // clearing RF interrupt flags and enabling RF interrupt mask
  RFIF &= ~IRQ_TXDONE;
  //RFIM |= IM_TXDONE;
  // clear RF interrupt flag RFIF_0 and RFIF_1
  S1CON &= ~0x03;

  // filling the TXFIFO with data(s), the first byte is the message length.
  RFD = length;
  for (i=0; i<(length-2); i++) RFD = *pradio_packet++;

  // transmitting
  RFST = ISTXON;

  // polling the interrupt flag and clear things before leaving
  do {} while (!(RFIF & IRQ_TXDONE));
  RFST = ISFLUSHTX;
  RFIF &= ~IRQ_TXDONE;
}

//----------------------------------------------------------------------------
// See hal2430.h for a description of this function.
//----------------------------------------------------------------------------

int8_t hal_radio_send_data_packet (__xdata radio_data_packet_t * pradio_packet,
                                uint8_t length)
{
  if (length <= 114)
  {
    hal_radio_send_packet((xu8_t *) pradio_packet, length+13);
    return RADIO_SUCCESS;
  }
  else return RADIO_FAILED;
}

//----------------------------------------------------------------------------
// See hal2430.h for a description of this function.
//----------------------------------------------------------------------------

int16_t hal_radio_receive_packet (__xdata uint8_t * pradio_packet, uint16_t timeout)
{
  u8_t i, length;

  // clearing the FIFOP interrupt flag
  RFIF &= ~IRQ_FIFOP;

  do {
    timeout--;
  } while (!(RFIF & IRQ_FIFOP) && timeout);

  if (RFIF & IRQ_FIFOP)
  {
    // Get total length of radio message from radio buffer.
    length = RFD;
    for (i=0; i<(length-2); i++) *pradio_packet++ = RFD;
    *pradio_packet++ = RFD; // RSSI
    *pradio_packet = RFD; // FCS
    // clearing RX radio buffer
    RFST = ISFLUSHRX;
    // clearing the FIFOP interrupt flag
    RFIF &= ~IRQ_FIFOP;
    return length;
  }
  else
  {
    // clearing RX radio buffer
    RFST = ISFLUSHRX;
    // clearing the FIFOP interrupt flag
    RFIF &= ~IRQ_FIFOP;
    return TIMEOUT_DETECTED;
  }
}

//---------------------------------------------------------------------------//
//  radio_assign_mac_header(radio packet)                                    //
//---------------------------------------------------------------------------//
//    input:   pointer of radio packet to send (beacon, data, or command     //
//    return:  none                                                          //
//---------------------------------------------------------------------------//

void radio_assign_mac_header (__xdata radio_data_packet_t * pradio_packet)
{
  pradio_packet->header.FCF1 = 0x00;
  pradio_packet->header.FCF1 |= FT_DATA ;//| REQUEST_ACK;
  pradio_packet->header.FCF2 = 0x00;
  pradio_packet->header.FCF2 |= DEST_ADDR_16 | SRC_ADDR_16;

  pradio_packet->header.FSN = 100;

  pradio_packet->header.dest_PANIDH = PAN_ID1;
  pradio_packet->header.dest_PANIDL = PAN_ID0;
  pradio_packet->header.dest_ADDRH = 0xEE; // TODO assign as a variable
  pradio_packet->header.dest_ADDRL = 0x35;

  pradio_packet->header.src_PANIDH = PAN_ID1;
  pradio_packet->header.src_PANIDL = PAN_ID0;
  pradio_packet->header.src_ADDRH = *(macp+6);
  pradio_packet->header.src_ADDRL = *(macp+7);
}

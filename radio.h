/*
 * radio.h
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

#ifndef RADIO_H_
#define RADIO_H_

#ifdef __cplusplus
extern "C" {
#endif

// macro defining frame type under Frame Control Field (FCF1)
#define FT_BEACON     0x00
#define FT_DATA       0x01
#define FT_ACK        0x02
#define FT_COMMAND      0x03

// macro defining bit under Frame Control Field (FCF1)
#define SECURITY_ENABLE   0x08
#define FRAME_PENDING   0x10
#define REQUEST_ACK     0x20
#define PAN_COMPRESSION   0x40

// macro defining destination addressing mode under Frame Control Field (FCF2)
#define NO_DEST_ADDRESS   0x00
#define DEST_ADDR_16    0x08
#define DEST_ADDR_64    0x0C

// macro defining source addressing mode under Frame Control Field (FCF2)
#define NO_SRC_ADDRESS    0x00
#define SRC_ADDR_16     0x80
#define SRC_ADDR_64     0xC0

// macro defining Command Frame Identifier (CFI)
#define ASSOC_REQUEST   0x01  // TX
#define ASSOC_RESPONSE    0x02  // RX
#define DISASSOC_NOTIFY   0x03  // TX/RX
#define DATA_REQUEST    0x04  // TX
#define PANID_CONFLICT    0x05  // TX
#define ORPHAN_NOTIFY   0x06  // TX
#define BEACON_REQUEST    0x07  // N/A
#define COORD_REALIGN   0x08  // RX
#define GTS_REQUEST     0x09  // N/A

// macro defining interrupt flags
#define IRQ_TXDONE    0x40
#define IRQ_FIFOP   0x20

// macro defining interrupt masks
#define IM_TXDONE   0x40
#define IM_FIFOP    0x20

// location of IEEE long address in the flash
#define IEEE_ADDR_START 0xFFF8

#define FIFOP           0x04

// macro for CC2530 radio hardware state machine (RFST)
#define DECZ            0xC5
#define DECY            0xC4
#define DECX            0xC3
#define INCZ            0xC2
#define INCY            0xC1
#define INCX            0xC0
#define RANDXY          0xBD
#define INT             0xBA
#define WAITX           0xBC
#define SETCMP1         0xBE
#define WEVENT1         0xB8
#define WEVENT2         0xB9
#define LABEL           0xBB
#define STOP            0xD2
#define SNOP            0xD0
#define SRXON           0xD3
#define STXON           0xD9
#define STXONCCA        0xDA
#define SSAMPLECCA      0xDB
#define SRFOFF          0xDF
#define SFLUSHRX        0xDD
#define SFLUSHTX        0xDE
#define SACK            0xD6
#define SACKPEND        0xD7
#define SNACK           0xD8
#define SRXMASKBITSET   0xD4
#define SRXMASKBITCLR   0xD5
#define ISSTOP          0xE2
#define ISSTART         0xE1
#define ISRXON          0xE3
#define ISRXMASKBITSET  0xE4
#define ISRXMASKBITCLR  0xE5
#define ISTXON          0xE9
#define ISTXONCCA       0xEA
#define ISSAMPLECCA     0xEB
#define ISRFOFF         0xEF
#define ISFLUSHRX       0xED
#define ISFLUSHTX       0xEE
#define ISACK           0xE6
#define ISACKPEND       0xE7
#define ISNACK          0xE8
#define ISCLEAR         0xFF

typedef struct {
  unsigned char FCF1; //little endian
  unsigned char FCF2;
  unsigned char FSN;
  unsigned char dest_PANIDL;  //little endian
  unsigned char dest_PANIDH;
  unsigned char dest_ADDRL; //little endian
  unsigned char dest_ADDRH;
  unsigned char src_PANIDL; //little endian
  unsigned char src_PANIDH;
  unsigned char src_ADDRL;  //little endian
  unsigned char src_ADDRH;
} radio_data_frame_header_t; //STRUCT_RADIO_DATA_FRAME_HEADER;


typedef struct {
  unsigned char FCF1; //little endian
  unsigned char FCF2;
  unsigned char FSN;
  unsigned char dest_PANIDL;  //little endian
  unsigned char dest_PANIDH;
  unsigned char dest_ADDRL; //little endian
  unsigned char dest_ADDRH;
  unsigned char src_PANIDL; //little endian
  unsigned char src_PANIDH;
  unsigned char src_ADDRL;  //little endian
  unsigned char src_ADDRH;
} radio_command_frame_header_t; //STRUCT_RADIO_COMMAND_FRAME_HEADER;


typedef struct {
  unsigned char FCF1; //little endian
  unsigned char FCF2;
  unsigned char FSN;
  unsigned char src_PANIDL; //little endian
  unsigned char src_PANIDH;
  unsigned char src_ADDRL;  //little endian
  unsigned char src_ADDRH;
} radio_beacon_frame_header_t; //STRUCT_RADIO_BEACON_FRAME_HEADER;

typedef struct {
  unsigned char nodinoRSSI;
  unsigned char nodinoFrameCheck;
} radio_frame_footer_t; //STRUCT_RADIO_FRAME_FOOTER;


typedef struct {
  radio_data_frame_header_t header;
  unsigned char data_payload [114];
  radio_frame_footer_t footer;
} radio_data_packet_t; //STRUCT_RADIO_DATA_PACKET;

typedef struct {
  radio_command_frame_header_t header;
  unsigned char cfi;
  unsigned char command_payload [113];
  radio_frame_footer_t footer;
} radio_command_packet_t; //STRUCT_RADIO_COMMAND_PACKET;

typedef struct {
  radio_beacon_frame_header_t header;
  unsigned char SUPER_FRAMEL;
  unsigned char SUPER_FRAMEH;
  unsigned char beacon_payload [116];
  radio_frame_footer_t footer;
} radio_beacon_packet_t; //STRUCT_RADIO_BEACON_PACKET;

typedef struct {
  unsigned char FCF1;
  unsigned char FCF2;
  unsigned char FSN;
  unsigned char nodinoRSSI;
  unsigned char nodinoFrameCheck;
} radio_ack_packet_t; //STRUCT_RADIO_ACK_PACKET;

// RADIO_TRX_OFF is equal to RADIO_IDLE at FSMSTATE state machine
typedef enum {
  RADIO_BUSY,
  RADIO_BUSY_RX,
  RADIO_BUSY_TX,
  RADIO_FORCE_TRX_OFF,
  RADIO_IDLE,
  RADIO_INVALID_PARAMETER,
  RADIO_RX_ON,
  RADIO_SUCCESS,
  RADIO_TRX_OFF,
  RADIO_TX_ON,
  RADIO_UNSUPPORTED_ATTRIBUTE,
  RADIO_FAILED,
  RADIO_POWER_OFF,
  RADIO_TIMEOUT
} radio_status_enum_t;

typedef enum {
  RADIO_IS_UNLOCKED,
  RADIO_IS_LOCKED
} radio_lock_enum_t;

typedef enum {
  CH11 = 11,
  CH12,
  CH13,
  CH14,
  CH15,
  CH16,
  CH17,
  CH18,
  CH19,
  CH20,
  CH21,
  CH22,
  CH23,
  CH24,
  CH25,
  CH26
} radio_channel_alloc_enum_t;

uint8_t radio_set_channel (uint8_t channel);
void radio_power_on (void);

#ifdef __cplusplus
}
#endif

#endif /* RADIO_H_ */

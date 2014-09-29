/*
 * flash.c
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

#include "compiler.h"
#include "cc253x.h"
#include "flash.h"

void flash_clear_buffer(uint8_t * pflash_buffer)
{
  uint8_t * plimit = pflash_buffer+FLASH_PAGE_SIZE;

  do {
    *pflash_buffer++ = 0x00;
  } while (pflash_buffer <= plimit);
}

void flash_erase_page(uint8_t page)
{
  FADDRH = (page) << 1;
  FADDRL = 0x00;
  FLASH_CONFIG(ERASE);
  __asm_begin
  ASM(NOP)
  __asm_end;
}

void flash_dma_write(uint8_t *buffer, uint16_t length, uint16_t flashadr) // length is multiplication of 4
{
  dma_config_t dmaConfig0;

  dmaConfig0.src_h = ((uint16_t)buffer >> 8) & 0x00FF; /* source address high byte*/
  dmaConfig0.src_l = (uint16_t)buffer & 0x00FF; /* source address low byte*/
  dmaConfig0.dst_h = (((uint16_t)&FWDATA) >> 8) & 0x00FF; /* dest. address high byte*/
  dmaConfig0.dst_l = ((uint16_t)&FWDATA) & 0x00FF; /* dest. address low byte*/
  dmaConfig0.len_h = (length>>8) & 0x00FF; /* [7:5] VLEN, [4:0] length high byte, 5 lowest bits*/
  dmaConfig0.len_l = length & 0x00FF; /* length low byte*/
  dmaConfig0.wtt   = 18; /* 7: wordsize, [6:5] transfer mode, [4:0] trigger */
  dmaConfig0.inc_prio = 0x42; /* [7:6] src inc, [5:4] dst_inc, 3: IRQ, 2: M8(vlen), [1-0] prio */

  FLASH_BUSY_WAIT();

  FADDRH =   (flashadr >> 10) & 0x00FF;
  FADDRL =   (flashadr >>  2) & 0x00FF;
  DMA0CFGH = (((uint16_t)&dmaConfig0) >> 8) & 0x00FF;
  DMA0CFGL = ((uint16_t)&dmaConfig0) & 0x00FF;

  DMAARM |= 0x01; /* arm the DMA */
  FLASH_CONFIG(WRITE); /* trigger the DMA transfer */
  while (!(DMAIRQ & 0x01)); /* wait until write complete */
  DMAIRQ &= 0xFE; /* clear any DMA interrupt flag */

  FLASH_BUSY_WAIT();
}

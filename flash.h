/*
 * flash.h
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

#ifndef FLASH_H_
#define FLASH_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define FLASH_PAGE_SIZE 2048

#define READ_WHEN_NEED  0x00
#define CONTINOUS_READ  0x10
#define WRITE           0x02
#define ERASE           0x01
#define FLASH_FULL      0x40
#define FLASH_BUSY      0x80

#define FLASH_WRITE_PAGE(page)      \
  do{                             \
    FLASH_CONFIG(WRITE);        \
    FADDRH = ((page)+1) << 3;   \
    FADDRL = 0x00;              \
  }while (0)


#define FLASH_BUSY_WAIT()       \
  do{                         \
  }while (FCTL & FLASH_BUSY)


#define FLASH_FULL_WAIT()         \
  do{                             \
  }while (FCTL & FLASH_FULL)


#define FLASH_CONFIG(options)   \
  do {                        \
    FCTL = options;         \
  } while (0)

void flash_erase_page(unsigned char page);
void flash_clear_buffer(uint8_t * pflash_buffer);

/** DMA configuration structure */
typedef struct dma_config {
  uint8_t src_h; /* source address high byte*/
  uint8_t src_l; /* source address low byte*/
  uint8_t dst_h; /* dest. address high byte*/
  uint8_t dst_l; /* dest. address low byte*/
  uint8_t len_h; /* [7:5] VLEN, [4:0] length high byte, 5 lowest bits*/
  uint8_t len_l; /* length low byte*/
  uint8_t wtt;   /* 7: wordsize, [6:5] transfer mode, [4:0] trigger */
  /* [7:6] src inc, [5:4] dst_inc, 3: IRQ, 2: M8(vlen), [1-0] prio */
  uint8_t inc_prio;
} dma_config_t;

void flash_dma_write(uint8_t *buffer, uint16_t length, uint16_t flashadr);

#ifdef __cplusplus
}
#endif

#endif /* FLASH_H_ */

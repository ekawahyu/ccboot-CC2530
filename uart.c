/*
 * uart.c
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

#include "config.h"
#include "cc253x.h"
#include "sfr-bits.h"
#include "uart.h"
#include "errnum.h"

void uart_init (void)
{
#if (UART_STDOUT_PORT == 0)
    UART_SET_SPEED(0, UART_115_M, UART_115_E);
    PERCFG &= ~PERCFG_U0CFG;  /* alternative 1 = P0.2-5 */
    P0SEL |= 0x0C;            /* peripheral select for TX and RX */
    P0DIR |= 0x08;            /* TX out */
    P0DIR &= ~0x04;           /* RX in */
    U0UCR = 0x02;             /* defaults: 8N1, no flow, high stop bit */
    U0CSR = UCSR_MODE | UCSR_RE;  /* UART mode, RX enabled */
    U0UCR |= 0x80;            /* flush it */
    URX0IE = 1;               /* enable RX interrupt */
#elif (UART_STDOUT_PORT == 1)
    /* TODO */
#endif
}

void putchar (uint8_t byte)
{
#if (UART_STDOUT_PORT == 0)
  U0CSR &= ~UCSR_TX_BYTE; /* Clear TX_BYTE status */
  U0DBUF = byte;
  while(!(U0CSR & UCSR_TX_BYTE)); /* Wait until byte has been transmitted. */
  U0CSR &= ~UCSR_TX_BYTE; /* Clear TX_BYTE status */
#elif (UART_STDOUT_PORT == 1)
  U1CSR &= ~UCSR_TX_BYTE; /* Clear TX_BYTE status */
  U1DBUF = byte;
  while(!UTX1IF); /* Wait until byte has been transmitted. */
  while(!(U1CSR & UCSR_TX_BYTE)); /* Wait until byte has been transmitted. */
  U1CSR &= ~UCSR_TX_BYTE; /* Clear TX_BYTE status */
#endif
}

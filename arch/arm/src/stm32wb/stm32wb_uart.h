/****************************************************************************
 * arch/arm/src/stm32wb/stm32wb_uart.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#ifndef __ARCH_ARM_STC_STM32WB_STM32WB_UART_H
#define __ARCH_ARM_STC_STM32WB_STM32WB_UART_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/serial/serial.h>

#include "chip.h"
#include "hardware/stm32wb_uart.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Sanity checks */

#if !defined(CONFIG_STM32WB_LPUART1)
#  undef CONFIG_STM32WB_LPUART1_SERIALDRIVER
#  undef CONFIG_STM32WB_LPUART1_1WIREDRIVER
#endif
#if !defined(CONFIG_STM32WB_USART1)
#  undef CONFIG_STM32WB_USART1_SERIALDRIVER
#  undef CONFIG_STM32WB_USART1_1WIREDRIVER
#endif

/* Is there a USART enabled? */

#if defined(CONFIG_STM32WB_LPUART1) || defined(CONFIG_STM32WB_USART1)
#  define HAVE_UART 1
#endif

/* Is there a serial console? */

#if defined(CONFIG_LPUART1_SERIAL_CONSOLE) && defined(CONFIG_STM32WB_LPUART1_SERIALDRIVER)
#  undef CONFIG_USART1_SERIAL_CONSOLE
#  define CONSOLE_UART  1
#  define HAVE_CONSOLE  1
#elif defined(CONFIG_USART1_SERIAL_CONSOLE) && defined(CONFIG_STM32WB_USART1_SERIALDRIVER)
#  undef CONFIG_LPUART1_SERIAL_CONSOLE
#  define CONSOLE_UART  2
#  define HAVE_CONSOLE  1
#else
#  undef CONFIG_LPUART1_SERIAL_CONSOLE
#  undef CONFIG_USART1_SERIAL_CONSOLE
#  define CONSOLE_UART  0
#  undef HAVE_CONSOLE
#endif

/* DMA support is only provided if CONFIG_ARCH_DMA is in the NuttX
 * configuration
 */

#if !defined(HAVE_UART) || !defined(CONFIG_ARCH_DMA)
#  undef CONFIG_LPUART1_RXDMA
#  undef CONFIG_USART1_RXDMA
#endif

/* Disable the DMA configuration on all unused USARTs */

#ifndef CONFIG_STM32WB_LPUART1_SERIALDRIVER
#  undef CONFIG_LPUART1_RXDMA
#endif

#ifndef CONFIG_STM32WB_USART1_SERIALDRIVER
#  undef CONFIG_USART1_RXDMA
#endif

/* Is DMA available on any (enabled) USART? */

#undef SERIAL_HAVE_RXDMA
#if defined(CONFIG_LPUART1_RXDMA) || defined(CONFIG_USART1_RXDMA)
#  define SERIAL_HAVE_RXDMA 1
#endif

/* Is DMA used on the console UART? */

#undef SERIAL_HAVE_CONSOLE_DMA
#if defined(CONFIG_LPUART1_SERIAL_CONSOLE) && defined(CONFIG_LPUART1_RXDMA)
#  define SERIAL_HAVE_CONSOLE_DMA 1
#elif defined(CONFIG_USART1_SERIAL_CONSOLE) && defined(CONFIG_USART1_RXDMA)
#  define SERIAL_HAVE_CONSOLE_DMA 1
#endif

/* Is DMA used on all (enabled) USARTs */

#define SERIAL_HAVE_ONLY_DMA 1
#if defined(CONFIG_STM32WB_LPUART1_SERIALDRIVER) && !defined(CONFIG_LPUART1_RXDMA)
#  undef SERIAL_HAVE_ONLY_DMA
#elif defined(CONFIG_STM32WB_USART1_SERIALDRIVER) && !defined(CONFIG_USART1_RXDMA)
#  undef SERIAL_HAVE_ONLY_DMA
#endif

/* Is RS-485 used? */

#if defined(CONFIG_LPUART1_RS485) || defined(CONFIG_USART1_RS485)
#  define HAVE_RS485 1
#endif

#ifdef HAVE_RS485
#  define USART_CR1_USED_INTS    (USART_CR1_RXNEIE | USART_CR1_TXEIE | USART_CR1_PEIE | USART_CR1_TCIE)
#else
#  define USART_CR1_USED_INTS    (USART_CR1_RXNEIE | USART_CR1_TXEIE | USART_CR1_PEIE)
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: stm32wb_serial_dma_poll
 *
 * Description:
 *   Must be called periodically if any STM32WB UART is configured for DMA.
 *   The DMA callback is triggered for each fifo size/2 bytes, but this can
 *   result in some bytes being transferred but not collected if the incoming
 *   data is not a whole multiple of half the FIFO size.
 *
 *   May be safely called from either interrupt or thread context.
 *
 ****************************************************************************/

#ifdef SERIAL_HAVE_RXDMA
void stm32wb_serial_dma_poll(void);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_STC_STM32WB_STM32WB_UART_H */

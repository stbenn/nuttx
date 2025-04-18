/****************************************************************************
 * arch/arm/src/stm32h5/stm32_dma.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>

#include "arm_internal.h"
#include "sched/sched.h"
#include "stm32_dma.h"
#include "hardware/stm32_gpdma.h"
#include "hardware/stm32_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* For GPDMA peripheral, each channel has channel specific addresses that are
 * at a base offset based on channel. Channel reg references will be based
 * off this in this file.
 */

#define CH_BASE_OFFSET(ch)  (0x80*(ch)) 
#define CH_CxLBAR_OFFSET     0x50
#define CH_CxFCR_OFFSET      0x5C
#define CH_CxSR_OFFSET       0x60
#define CH_CxCR_OFFSET       0x64
#define CH_CxTR1_OFFSET      0x90
#define CH_CxTR2_OFFSET      0x94
#define CH_CxBR1_OFFSET      0x98
#define CH_CxSAR_OFFSET      0x9C
#define CH_CxDAR_OFFSET      0xA0
#define CH_CxTR3_OFFSET      0xA4
#define CH_CxBR2_OFFSET      0xA8
#define CH_CxLLR_OFFSET      0xCC

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct gpdma_ch_s
{
  uint8_t        irq;      /* DMA IRQ number */
  uint8_t        channel;  /* DMA Channel number (0-7) */
  uint32_t       base;     /* Channel base address */
  sem_t          sem;      /* Used to wait for DMA channel to become available */
  dma_callback_t callback; /* Callback invoked when DMA completes */
  void          *arg;      /* Argument passed to callback function */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct gpdma_ch_s g_chan[] = 
{
  {
    .channel = 0,
    .irq = STM32_IRQ_GPDMA1_CH0,
    .base = STM32_DMA1_BASE + CH_BASE_OFFSET(0),
  },
  {
    .channel = 1,
    .irq = STM32_IRQ_GPDMA1_CH1,
  },
  {
    .channel = 2,
    .irq = STM32_IRQ_GPDMA1_CH2,
  },
  {
    .channel = 3,
    .irq = STM32_IRQ_GPDMA1_CH3,
  },
  {
    .channel = 4,
    .irq = STM32_IRQ_GPDMA1_CH4,
  },
  {
    .channel = 5,
    .irq = STM32_IRQ_GPDMA1_CH5,
  },
  {
    .channel = 0,
    .irq = STM32_IRQ_GPDMA2_CH0,
    .base = STM32_DMA2_BASE + CH_BASE_OFFSET(0),
  },
  {
    .channel = 1,
    .irq = STM32_IRQ_GPDMA2_CH1,
  },
  {
    .channel = 2,
    .irq = STM32_IRQ_GPDMA2_CH2,
  },
  {
    .channel = 3,
    .irq = STM32_IRQ_GPDMA2_CH3,
  },
  {
    .channel = 4,
    .irq = STM32_IRQ_GPDMA2_CH4,
  },
  {
    .channel = 5,
    .irq = STM32_IRQ_GPDMA2_CH5,
  },
};
/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

 /****************************************************************************
 * Name: stm32_dmachannel
 *
 ****************************************************************************/

DMA_HANDLE stm32_dmachannel(uint16_t req)
{
  /* On the H5, peripherals can kind of be mapped to any number of different
     channels.

     P2M and M2P should be on channels with 8 byte FIFOs. GPDMA1/2 CH[0-3]

     Linear M2M should be on channels with 32 byte FIFOs. GPDMA1/2 CH[4-5]
     Some peripherals (e.g. OCTOSPI) can also use these for burst transfers.
  */
}
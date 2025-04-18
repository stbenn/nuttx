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
  uint8_t dma_instance; /* GPDMA1 or GPDMA2 */
  uint8_t channel;
  bool    free;         /* Is this channel free to use. */
  uint16_t request;     /* The request number tied to in use channel. */

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
    .dma_instance = 1,
    .channel = 0,
    .free = true
  },
  {
    .dma_instance = 1,
    .channel = 1,
    .free = true
  },
  {
    .dma_instance = 1,
    .channel = 2,
    .free = true
  },
  {
    .dma_instance = 1,
    .channel = 3,
    .free = true
  },
  {
    .dma_instance = 1,
    .channel = 4,
    .free = true
  },
  {
    .dma_instance = 1,
    .channel = 5,
    .free = true
  },
  {
    .dma_instance = 2,
    .channel = 0,
    .free = true
  },
  {
    .dma_instance = 2,
    .channel = 1,
    .free = true
  },
  {
    .dma_instance = 2,
    .channel = 2,
    .free = true
  },
  {
    .dma_instance = 2,
    .channel = 3,
    .free = true
  },
  {
    .dma_instance = 2,
    .channel = 4,
    .free = true
  },
  {
    .dma_instance = 2,
    .channel = 5,
    .free = true
  }

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

     TODO: Add better channel allocation. 
  */

  DMA_HANDLE handle = NULL;

  DEBUGASSERT(req < 0xffff); /* Currently no support for M2M mode. */

  // If it is P2M request, find first available 8 byte fifo channel
  for (int i = 0; i < (sizeof(g_chan) / sizeof(struct gpdma_ch_s)); i++)
    {
      struct gpdma_ch_s *chan = &g_chan[i];

      if (chan->free && chan->channel <= 3)
        {
          chan->free = false;
          chan->request = req;
          handle = (DMA_HANDLE)chan;
          break;
        }
    }

  return handle;
}

/****************************************************************************
 * Name: stm32_dmafree
 *
 * Description:
 *   Release a DMA channel and unmap DMAMUX if required.
 *
 *   NOTE:  The 'handle' used in this argument must NEVER be used again
 *   until stm32_dmachannel() is called again to re-gain access to the
 *   channel.
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   - The caller holds the DMA channel.
 *   - There is no DMA in progress
 *
 ****************************************************************************/

void stm32_dmafree(DMA_HANDLE handle)
{
  struct gpdma_ch_s *chan = (struct gpdma_ch_s *)handle;

  DEBUGASSERT(handle != NULL);

  chan->request = 0;
  chan->free = true;
}

/****************************************************************************
 * Name: stm32_dmasetup
 *
 * Description:
 *   Configure DMA before using
 *
 * Input Parameters:
 *   TODO: Figure out what the input parameter needs to be!!! H7 and F7 have
 *         very different implementations.
 *
 ****************************************************************************/

void stm32_dmasetup(DMA_HANDLE handle, stm32_gpdmacfg_t *cfg)
{
  struct gpdma_ch_s *chan = (struct gpdma_ch_s *)handle;

  DEBUGASSERT(handle != NULL);

}
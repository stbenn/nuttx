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
  uint32_t base;        /* Channel base address */

};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static inline uint32_t gpdmach_getreg(struct gpdma_ch_s *chan,
                                      uint32_t offset);
static inline void gpdmach_putreg(struct gpdma_ch_s *chan, uint32_t offset,
                                  uint32_t value);
static inline void gpdmach_modifyreg32(struct gpdma_ch_s *chan,
                                       uint32_t offset, uint32_t clrbits,
                                       uint32_t setbits);
static void gpdma_ch_abort(struct gpdma_ch_s *chan);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct gpdma_ch_s g_chan[] = 
{
  {
    .dma_instance = 1,
    .channel = 0,
    .free = true,
    .base = STM32_DMA1_BASE + CH_BASE_OFFSET(0)
  },
  {
    .dma_instance = 1,
    .channel = 1,
    .free = true,
    .base = STM32_DMA1_BASE + CH_BASE_OFFSET(1)
  },
  {
    .dma_instance = 1,
    .channel = 2,
    .free = true,
    .base = STM32_DMA1_BASE + CH_BASE_OFFSET(2)
  },
  {
    .dma_instance = 1,
    .channel = 3,
    .free = true,
    .base = STM32_DMA1_BASE + CH_BASE_OFFSET(3)
  },
  {
    .dma_instance = 1,
    .channel = 4,
    .free = true,
    .base = STM32_DMA1_BASE + CH_BASE_OFFSET(4)
  },
  {
    .dma_instance = 1,
    .channel = 5,
    .free = true,
    .base = STM32_DMA1_BASE + CH_BASE_OFFSET(5)
  },
  {
    .dma_instance = 2,
    .channel = 0,
    .free = true,
    .base = STM32_DMA2_BASE + CH_BASE_OFFSET(0)
  },
  {
    .dma_instance = 2,
    .channel = 1,
    .free = true,
    .base = STM32_DMA2_BASE + CH_BASE_OFFSET(1)
  },
  {
    .dma_instance = 2,
    .channel = 2,
    .free = true,
    .base = STM32_DMA2_BASE + CH_BASE_OFFSET(2)
  },
  {
    .dma_instance = 2,
    .channel = 3,
    .free = true,
    .base = STM32_DMA2_BASE + CH_BASE_OFFSET(3)
  },
  {
    .dma_instance = 2,
    .channel = 4,
    .free = true,
    .base = STM32_DMA2_BASE + CH_BASE_OFFSET(4)
  },
  {
    .dma_instance = 2,
    .channel = 5,
    .free = true,
    .base = STM32_DMA2_BASE + CH_BASE_OFFSET(5)
  }

};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline uint32_t gpdmach_getreg(struct gpdma_ch_s *chan,
                                      uint32_t offset)
{
  return getreg32(chan->base + offset);
}

static inline void gpdmach_putreg(struct gpdma_ch_s *chan, uint32_t offset,
                                  uint32_t value)
{
  putreg32(value, chan->base + offset);
}

static inline void gpdmach_modifyreg32(struct gpdma_ch_s *chan,
                                       uint32_t offset, uint32_t clrbits,
                                       uint32_t setbits)
{
  modifyreg32(chan->base + offset, clrbits, setbits);
}

/****************************************************************************
 * Name: gpdma_ch_abort
 *
 * Description:
 *   For the given channel, suspend and abort any ongoing channel transfers.
 *   Returns after the abort has complete and taken effect. 
 *
 ****************************************************************************/

static void gpdma_ch_abort(struct gpdma_ch_s *chan)
{
  // 1. Software writes 1 to the GPDMA_CxCR.SUSP bit
  if ((gpdmach_getreg(chan, CH_CxCR_OFFSET) & GPDMA_CXCR_EN) == 0)
    {
      return;
    }
  
  gpdmach_putreg(chan, CH_CxCR_OFFSET, GPDMA_CXCR_SUSP);

  // 2. Polls suspend flag GPDMA_CxSR.SUSPF until SUSPF = 1, or waits for an
  //    interrupt previously enabled by writing 1 to GPDMA_CxCR.SUSPIE.

  while ((gpdmach_getreg(chan, CH_CxSR_OFFSET) & GPDMA_CXSR_SUSPF) == 0)
    {
      up_udelay(1);
    }

  // 3. Reset chan by writing 1 to GPDMA_CxCR.RESET

  gpdmach_putreg(chan, CH_CxCR_OFFSET, GPDMA_CXCR_RESET);

  // 4. Wait for GPDMA_CxCR.EN and GPDMA_CxCR.SUSP bits to be reset

  while ((gpdmach_getreg(chan, CH_CxCR_OFFSET) &
          (GPDMA_CXCR_EN|GPDMA_CXCR_SUSP) == 0))
    {
      up_udelay(1);
    }
}

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

void stm32_dmasetup(DMA_HANDLE handle, stm32_gpdma_chcfg_t *cfg)
{
  struct gpdma_ch_s *chan = (struct gpdma_ch_s *)handle;
  uint32_t timeout;

  DEBUGASSERT(handle != NULL);

  /* Drivers using DMA should manage channel usage. If a DMA request is not
   * made on an error or an abort occurs, the driver should stop the DMA. If
   * it fails to do so, we can not just hang waiting on the HW that will not
   * change state.
   *
   * If at the end of waiting the HW is still not ready, there is a HW
   * or software usage problem.
   */

  if ((gpdmach_getreg(chan, CH_CxCR_OFFSET) & GPDMA_CXCR_EN) != 0)
    {
      gpdma_ch_abort(chan);
    }

  /* Clear any unhandled flags from previous transactions */

  gpdmach_putreg(chan, CH_CxFCR_OFFSET, 0x7f << 8);

  /* Set channel transfer priority */

  // gpdmach_modifyreg32(chan, CH_CxCR_OFFSET, GPDMA_CXCR_PRIO_MASK,
  //                    (cfg->priority & 0b11) << GPDMA_CXCR_PRIO_SHIFT);
}
/****************************************************************************
 * arch/arm/src/stm32h5/stm32_dma.h
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

#ifndef __ARCH_ARM_SRC_STM32H5_STM32_DMA_H
#define __ARCH_ARM_SRC_STM32H5_STM32_DMA_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>

#include <stdint.h>

#include "hardware/stm32_gpdma.h"

/* These definitions provide the bit encoding of the 'status' parameter
 * passed to the DMA callback function (see dma_callback_t).
 */

//  #define DMA_STATUS_FEIF           0                    /* Stream FIFO error (ignored) */
//  #define DMA_STATUS_DMEIF          DMA_STREAM_DMEIF_BIT /* Stream direct mode error */
//  #define DMA_STATUS_TEIF           DMA_STREAM_TEIF_BIT  /* Stream Transfer Error */
//  #define DMA_STATUS_HTIF           DMA_STREAM_HTIF_BIT  /* Stream Half Transfer */
//  #define DMA_STATUS_TCIF           DMA_STREAM_TCIF_BIT  /* Stream Transfer Complete */
 
//  #define DMA_STATUS_ERROR          (DMA_STATUS_FEIF | DMA_STATUS_DMEIF | DMA_STATUS_TEIF)
//  #define DMA_STATUS_SUCCESS        (DMA_STATUS_TCIF | DMA_STATUS_HTIF)

/* GPDMA Configuration Flags: WARNING!! NOT YET IMPLEMENTED! */

#define GPDMACFG_FLAG_CIRC    (1 << 0)  /* Enable Circular mode */
#define GPDMACFG_FLAG_PFC     (1 << 1)  /* Enable Peripheral flow control */
#define GPDMACFG_FLAG_DB      (1 << 2)  /* Enable Double buffer mode */

/****************************************************************************
 * Public Types
 ****************************************************************************/

enum gpdma_dir_e
{
  GPDMA_DIR_P2M = 0,      /* Peripheral to Memory transfer */
  GPDMA_DIR_M2P,          /* Memory to Peripheral transfer */
  GPDMA_DIR_M2M           /* Memory to Memory transfer */
};

enum gpdma_datawidth_e
{
  GPDMA_DW_BYTE = 0,  /* Byte */
  GPDMA_DW_HALF,      /* Half-word (2 Bytes) */
  GPDMA_DW_WORD       /* Word (4 Bytes) */
};

/* GPDMA Configuration structure: abstracts away register implementation into
 * fields to specify functionality. Driver handles translating this to
 * registers.
 */

struct stm32_gpdmacfg_s
{
  uint32_t src_addr;          /* Source address */
  bool     src_inc;           /* Source address increment */
  uint32_t dest_addr;         /* Destination address */
  bool     dest_inc;          /* Destination address increment */
  enum gpdma_dir_e dir;       /* Transfer direction */
  enum gpdma_datawidth_e dw;  /* Data width of src and dest */

  /* Burst length applies to both source and dest. The driver currently
   * only supports consistent data formats between src and dest.
   */

  uint8_t  burst_len;

  /* Specialized config flags defined by GPDMACFG_FLAG_X (see above) */

  uint16_t flags;

};

/* DMA_HANDLE Provides an opaque reference that can be used to represent a
 * DMA stream.
 */

typedef void *DMA_HANDLE;

/* Description:
 *   This is the type of the callback that is used to inform the user of the
 *   completion of the DMA.  NOTE:  The DMA module does *NOT* perform any
 *   cache operations.  It is the responsibility of the DMA client to
 *   invalidate DMA buffers after completion of the DMA RX operations.
 *
 * Input Parameters:
 *   handle - Refers to the DMA channel or stream
 *   status - A bit encoded value that provides the completion status.  See
 *            the DMASTATUS_* definitions above.
 *   arg    - A user-provided value that was provided when stm32_dmastart()
 *            was called.
 */

typedef void (*dma_callback_t)(DMA_HANDLE handle, uint8_t status, void *arg);

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
 * Name: stm32_dmachannel
 *
 * Description:
 *   Allocate a DMA channel based on provided request signal. If no channel
 *   is available, will wait until one becomes available.
 *
 *   For request signal definitions, see GPDMA_REQ_ definitions in
 *   hardware/stm32h56x_dmasigmap.h (For H56x H57x chips)
 *
 * Input Parameters:
 *   req - GPDMA request signal number or 0xFFFF for software triggered
 *         memory-to-memory transfer.
 *
 * Returned Value:
 *   On success, this function returns a non-NULL, void* DMA channel handle.
 *   NULL is returned on any failure.  This function can fail only if no DMA
 *   channel is available.
 *
 * Assumptions:
 *   - The caller does not hold he DMA channel.
 *   - The caller can wait for the DMA channel to be freed if it is no
 *     available.
 *
 ****************************************************************************/

DMA_HANDLE stm32_dmachannel(uint16_t req);

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

void stm32_dmafree(DMA_HANDLE handle);

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

void stm32_dmasetup(DMA_HANDLE handle, stm32_dmacfg_t *cfg);

/****************************************************************************
 * Name: stm32_dmastart
 *
 * Description:
 *   Start the DMA transfer.  NOTE:  The DMA module does *NOT* perform any
 *   cache operations.  It is the responsibility of the DMA client to clean
 *   DMA buffers after starting of the DMA TX operations.
 *
 * Assumptions:
 *   - DMA handle allocated by stm32_dmachannel()
 *   - No DMA in progress
 *
 ****************************************************************************/

void stm32_dmastart(DMA_HANDLE handle, dma_callback_t callback, void *arg,
                    bool half);

/****************************************************************************
 * Name: stm32_dmastop
 *
 * Description:
 *   Cancel the DMA.  After stm32_dmastop() is called, the DMA channel is
 *   reset and stm32_dmasetup() must be called before stm32_dmastart() can be
 *   called again
 *
 * Assumptions:
 *   - DMA handle allocated by stm32_dmachannel()
 *
 ****************************************************************************/

void stm32_dmastop(DMA_HANDLE handle);

/****************************************************************************
 * Name: stm32_dmaresidual
 *
 * Description:
 *   Returns the number of bytes remaining to be transferred
 *
 * Assumptions:
 *   - DMA handle allocated by stm32_dmachannel()
 *
 ****************************************************************************/

size_t stm32_dmaresidual(DMA_HANDLE handle);

/****************************************************************************
 * Name: stm32_dmacapable
 *
 * Description:
 *   Check if the DMA controller can transfer data to/from given memory
 *   address with the given configuration. This depends on the internal
 *   connections in the ARM bus matrix of the processor. Note that this only
 *   applies to memory addresses, it will return false for any peripheral
 *   address.
 *
 * Input Parameters:
 *
 *   maddr - starting memory address
 *   count - number of unit8 or uint16 or uint32 items as defined by MSIZE
 *           of ccr.
 *   ccr   - DMA stream configuration register
 *
 * Returned Value:
 *   True, if transfer is possible.
 *
 ****************************************************************************/

#ifdef CONFIG_STM32H7_DMACAPABLE
bool stm32_dmacapable(DMA_HANDLE handle, stm32_dmacfg_t *cfg);
#else
#  define stm32_dmacapable(handle, cfg) (true)
#endif

/****************************************************************************
 * Name: stm32_dmasample
 *
 * Description:
 *   Sample DMA register contents
 *
 * Input Parameters:
 *   TODO: Figure these out!! Not sure if I need the struct like F7 or not?
 *
 * Assumptions:
 *   - DMA handle allocated by stm32_dmachannel()
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_DMA_INFO
void stm32_dmasample(DMA_HANDLE handle, struct stm32_dmaregs_s *regs);
#else
#  define stm32_dmasample(handle,regs)
#endif

/****************************************************************************
 * Name: stm32_dmadump
 *
 * Description:
 *   Dump previously sampled DMA register contents
 *
 * Assumptions:
 *   - DMA handle allocated by stm32_dmachannel()
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_DMA_INFO
void stm32_dmadump(DMA_HANDLE handle, const char *msg);
#else
#  define stm32_dmadump(handle,msg)
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* !__ASSEMBLY__*/
#endif /* __ARCH_ARM_SRC_STM32H5_STM32_DMA_H*/

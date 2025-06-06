/****************************************************************************
 * arch/arm/src/imxrt/imxrt_edma.h
 *
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: 2018 Gregory Nutt. All rights reserved.
 * SPDX-FileCopyrightText: 2016-2017 NXP
 * SPDX-FileCopyrightText: 2015, Freescale Semiconductor, Inc.
 * SPDX-FileContributor: Gregory Nutt <gnutt@nuttx.org>
 * All rights reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_IMXRT_IMXRT_EDMAC_H
#define __ARCH_ARM_SRC_IMXRT_IMXRT_EDMAC_H

/* General Usage:
 *
 * 1. Allocate a DMA channel
 *
 *      DMACH_HANDLE handle;
 *      handle = edma_dmach_alloc(dmamux, dchpri);
 *
 *    Where 'dmamux' is the channel DMAMUX configuration register setting and
 *    'dchpri' is the channel DCHPRIO priority register setting.
 *
 * 2. Create the transfer configuration:
 *
 *      struct imxrt_edma_xfrconfig_s config;
 *      config.saddr = ..;
 *      config.daddr = ..;
 *      etc.
 *
 * 3. Setup the transfer in hardware:
 *
 *      int ret;
 *      ret = imxrt_dmach_xfrsetup(handle, &config);
 *
 * 4. If you are setting up a scatter gather DMA
 *    (with CONFIG_IMXRT_EDMA_NTCD > 0),
 *    then repeat steps 2 and 3 for each segment of the transfer.
 *
 * 5. Start the DMA:
 *
 *      ret = imxrt_dmach_start(handle, my_callback_func, priv);
 *
 *    Where my_callback_func() is called when the DMA completes or an error
 *    occurs. 'priv' represents some internal driver state that will be
 *    provided with the callback.
 *
 * 6. If you need to stop the DMA and free resources (such as if a timeout
 *    occurs), then:
 *
 *     i mxrt_dmach_stop(handle);
 *
 * 7. The callback will be received when the DMA completes (or an error
 *    occurs). After that, you may free  the DMA channel, or reuse it on
 *    subsequent DMAs.
 *
 *      imxrt_dmach_free(handle);
 *
 * Almost non-invasive debug instrumentation is available.  You may call
 * imxrt_dmasample() to save the current state of the eDMA registers at any
 * given point in time.  At some later, postmortem analysis, you can dump the
 * content of the buffered registers with imxrt_dmadump().  imxrt_dmasample()
 * is also available for monitoring DMA progress.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include "hardware/imxrt_edma.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration flags.
 *
 * REVISIT:  Many missing options that should be represented as flags:
 * 1. Bandwidth
 * 2. Source/Destination modulo
 */

#define EDMA_CONFIG_LINKTYPE_SHIFT       (0) /* Bits 0-1: Link type */
#define EDMA_CONFIG_LINKTYPE_MASK        (3 << EDMA_CONFIG_LINKTYPE_SHIFT)
#  define EDMA_CONFIG_LINKTYPE_LINKNONE  (0 << EDMA_CONFIG_LINKTYPE_SHIFT) /* No channel link */
#  define EDMA_CONFIG_LINKTYPE_MINORLINK (1 << EDMA_CONFIG_LINKTYPE_SHIFT) /* Channel link after each minor loop */
#  define EDMA_CONFIG_LINKTYPE_MAJORLINK (2 << EDMA_CONFIG_LINKTYPE_SHIFT) /* Channel link when major loop count exhausted */

#define EDMA_CONFIG_LOOP_SHIFT           (2) /* Bits 2-3: Loop type */
#define EDMA_CONFIG_LOOP_MASK            (3 << EDMA_CONFIG_LOOP_SHIFT)
#  define EDMA_CONFIG_LOOPNONE           (0 << EDMA_CONFIG_LOOP_SHIFT) /* No looping */
#  define EDMA_CONFIG_LOOPSRC            (1 << EDMA_CONFIG_LOOP_SHIFT) /* Source looping */
#  define EDMA_CONFIG_LOOPDEST           (2 << EDMA_CONFIG_LOOP_SHIFT) /* Dest looping */

#define EDMA_CONFIG_INTHALF              (1 << 4) /* Bits 4: Int on HALF */
#define EDMA_CONFIG_INTMAJOR             (1 << 5) /* Bits 5: Int on all Major completion
                                                   * Default is only on last completion
                                                   * if using scatter gather
                                                   */

/****************************************************************************
 * Public Types
 ****************************************************************************/

typedef void *DMACH_HANDLE;
typedef void (*edma_callback_t)(DMACH_HANDLE handle,
                                void *arg, bool done, int result);

/* eDMA transfer sizes */

enum kinetis_edma_sizes_e
{
  EDMA_8BIT    = 0,      /* Transfer data size 8 */
  EDMA_16BIT   = 1,      /* Transfer data size 16 */
  EDMA_32BIT   = 2,      /* Transfer data size 32 */
};

/* This structure holds the source/destination transfer attribute
 * configuration.
 */

struct imxrt_edma_xfrconfig_s
{
  uint32_t saddr;      /* Source data address. */
  uint32_t daddr;      /* Destination data address. */
  int16_t  soff;       /* Sign-extended offset for current source address. */
  int16_t  doff;       /* Sign-extended offset for current destination address. */
  uint16_t iter;       /* Major loop iteration count. */
  uint8_t  flags;      /* See EDMA_CONFIG_* definitions */
  uint8_t  ssize;      /* Source data transfer size (see TCD_ATTR_SIZE_* definitions in hardware/. */
  uint8_t  dsize;      /* Destination data transfer size. */
#ifdef CONFIG_IMXRT_EDMA_EMLIM
  uint16_t nbytes;     /* Bytes to transfer in a minor loop */
#else
  uint32_t nbytes;     /* Bytes to transfer in a minor loop */
#endif
#ifdef CONFIG_IMXRT_EDMA_ELINK
  DMACH_HANDLE linkch; /* Link channel (With EDMA_CONFIG_LINKTYPE_* flags) */
#endif
};

/* The following is used for sampling DMA registers
 * when CONFIG DEBUG_DMA is selected
 */

#ifdef CONFIG_DEBUG_DMA
struct imxrt_dmaregs_s
{
  uint8_t chan;          /* Sampled channel */

  /* eDMA Global Registers */

  uint32_t cr;           /* Control */
  uint32_t es;           /* Error Status */
  uint32_t erq;          /* Enable Request */
  uint32_t req;          /* Interrupt Request */
  uint32_t err;          /* Error */
  uint32_t hrs;          /* Hardware Request Status */
  uint32_t ears;         /* Enable Asynchronous Request in Stop */

  /* eDMA Channel registers */

  uint8_t dchpri;        /* Channel priority */

  /* eDMA TCD */

  uint32_t saddr;        /* TCD Source Address */
  uint16_t soff;         /* TCD Signed Source Address Offset */
  uint16_t attr;         /* TCD Transfer Attributes */
  uint32_t nbml;         /* TCD Signed Minor Loop Offset / Byte Count */
  uint32_t slast;        /* TCD Last Source Address Adjustment */
  uint32_t daddr;        /* TCD Destination Address */
  uint16_t doff;         /* TCD Signed Destination Address Offset */
  uint16_t citer;        /* TCD Current Minor Loop Link, Major Loop Count */
  uint32_t dlastsga;     /* TCD Last Destination Address Adjustment/Scatter Gather Address */
  uint16_t csr;          /* TCD Control and Status */
  uint16_t biter;        /* TCD Beginning Minor Loop Link, Major Loop Count */

  /* DMAMUX registers */

  uint32_t dmamux;       /* Channel configuration */
};
#endif /* CONFIG_DEBUG_DMA */

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Data
 ****************************************************************************/

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
 * Name: imxrt_dmach_alloc
 *
 *   Allocate a DMA channel.  This function sets aside a DMA channel,
 *   initializes the DMAMUX for the channel, then gives the caller exclusive
 *   access to the DMA channel.
 *
 * Input Parameters:
 *   dmamux - DMAMUX configuration see DMAMUX channel configuration register
 *            bit-field definitions in hardware/imxrt_dmamux.h.
 *            Settings include:
 *
 *            DMAMUX_CHCFG_SOURCE     Chip-specific DMA source (required)
 *            DMAMUX_CHCFG_AON        DMA Channel Always Enable (optional)
 *            DMAMUX_CHCFG_TRIG       DMA Channel Trigger Enable (optional)
 *            DMAMUX_CHCFG_ENBL       DMA Mux Channel Enable (required)
 *
 *            A value of zero will disable the DMAMUX channel.
 *   dchpri - DCHPRI channel priority configuration.  See DCHPRI channel
 *            configuration register bit-field definitions in
 *            hardware/imxrt_edma.h.  Meaningful settings include:
 *
 *            EDMA_DCHPRI_CHPRI       Channel Arbitration Priority
 *            DCHPRI_DPA              Disable Preempt Ability
 *            DCHPRI_ECP              Enable Channel Preemption
 *
 *            The power-on default, 0x05, is a reasonable choice.
 *
 * Returned Value:
 *   If a DMA channel is available, this function returns a non-NULL, void*
 *   DMA channel handle.  NULL is returned on any failure.
 *
 ****************************************************************************/

DMACH_HANDLE imxrt_dmach_alloc(uint32_t dmamux, uint8_t dchpri);

/****************************************************************************
 * Name: imxrt_dmach_free
 *
 * Description:
 *   Release a DMA channel.
 *   NOTE:  The 'handle' used in this argument must NEVER be used again until
 *   imxrt_dmach_alloc() is called again to re-gain a valid handle.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void imxrt_dmach_free(DMACH_HANDLE handle);

/****************************************************************************
 * Name: imxrt_dmach_xfrsetup
 *
 * Description:
 *   This function adds the eDMA transfer to the DMA sequence.  The request
 *   is setup according to the content of the transfer configuration
 *   structure.  For "normal" DMA, imxrt_dmach_xfrsetup is called only once.
 *   Scatter/gather DMA is accomplished by calling this function repeatedly,
 *   once for each transfer in the sequence.  Scatter/gather DMA processing
 *   is enabled automatically when the second transfer configuration is
 *   received.
 *
 *   This function may be called multiple times to handle multiple,
 *   discontinuous transfers (scatter-gather)
 *
 * Input Parameters:
 *   handle - DMA channel handle created by imxrt_dmach_alloc()
 *   config - A DMA transfer configuration instance, populated by the
 *            The content of 'config' describes the transfer
 *
 * Returned Value
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

int imxrt_dmach_xfrsetup(DMACH_HANDLE handle,
                         const struct imxrt_edma_xfrconfig_s *config);

/****************************************************************************
 * Name: imxrt_dmach_start
 *
 * Description:
 *   Start the DMA transfer by enabling the channel DMA request.  This
 *   function should be called after the final call to imxrt_dmasetup() in
 *   order to avoid race conditions.
 *
 *   At the conclusion of each major DMA loop, a callback to the
 *   user-provided function is made:  |For "normal" DMAs, this will
 *   correspond to the DMA DONE interrupt; for scatter gather DMAs, multiple
 *   interrupts will be generated with the final being the DONE interrupt.
 *
 *   At the conclusion of the DMA, the DMA channel is reset, all TCDs are
 *   freed, and the callback function is called with the the success/fail
 *   result of the DMA.
 *
 *   NOTE: On Rx DMAs (peripheral-to-memory or memory-to-memory), it is
 *   necessary to invalidate the destination memory.  That is not done
 *   automatically by the DMA module.  Invalidation of the destination memory
 *   regions is the responsibility of the caller.
 *
 * Input Parameters:
 *   handle   - DMA channel handle created by imxrt_dmach_alloc()
 *   callback - The callback to be invoked when the DMA is completes or is
 *              aborted.
 *   arg      - An argument that accompanies the callback
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

int imxrt_dmach_start(DMACH_HANDLE handle,
                      edma_callback_t callback, void *arg);

/****************************************************************************
 * Name: imxrt_dmach_stop
 *
 * Description:
 *   Cancel the DMA.  After imxrt_dmach_stop() is called, the DMA channel is
 *   reset, all TCDs are freed, and imxrt_dmarx/txsetup() must be called
 *   before imxrt_dmach_start() can be called again
 *
 * Input Parameters:
 *   handle   - DMA channel handle created by imxrt_dmach_alloc()
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void imxrt_dmach_stop(DMACH_HANDLE handle);

/****************************************************************************
 * Name: imxrt_dmach_getcount
 *
 * Description:
 *   This function checks the TCD (Task Control Descriptor) status for a
 *   specified eDMA channel and returns the the number of major loop counts
 *   that have not finished.
 *
 *   NOTES:
 *   1. This function can only be used to get unfinished major loop count of
 *      transfer without the next TCD, or it might be inaccuracy.
 *   2. The unfinished/remaining transfer bytes cannot be obtained directly
 *      from registers while the channel is running.
 *
 *   Because to calculate the remaining bytes, the initial NBYTES configured
 *   in DMA_TCDn_NBYTES_MLNO register is needed while the eDMA IP does not
 *   support getting it while a channel is active.  In another words, the
 *   NBYTES value reading is always the actual (decrementing) NBYTES value
 *   the dma_engine is working with while a channel is running.
 *   Consequently, to get the remaining transfer bytes, a software-saved
 *   initial value of NBYTES (for example copied before enabling the channel)
 *   is needed. The formula to calculate it is shown below:
 *
 *     RemainingBytes = RemainingMajorLoopCount * NBYTES
 *                                               (initially configured)
 *
 * Input Parameters:
 *   handle  - DMA channel handle created by imxrt_dmach_alloc()
 *
 * Returned Value:
 *   Major loop count which has not been transferred yet for the current TCD.
 *
 ****************************************************************************/

unsigned int imxrt_dmach_getcount(DMACH_HANDLE handle);

/****************************************************************************
 * Name: imxrt_dmach_idle
 *
 * Description:
 *   This function checks if the dma is idle
 *
 * Returned Value:
 *   0  - if idle
 *   !0 - not
 *
 ****************************************************************************/

unsigned int imxrt_dmach_idle(DMACH_HANDLE handle);

/****************************************************************************
 * Name: imxrt_dmasample
 *
 * Description:
 *   Sample DMA register contents
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_DMA
void imxrt_dmasample(DMACH_HANDLE handle, struct imxrt_dmaregs_s *regs);
#else
#  define imxrt_dmasample(handle,regs)
#endif

/****************************************************************************
 * Name: imxrt_dmadump
 *
 * Description:
 *   Dump previously sampled DMA register contents
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_DMA
void imxrt_dmadump(const struct imxrt_dmaregs_s *regs, const char *msg);
#else
#  define imxrt_dmadump(handle,regs,msg)
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_IMXRT_IMXRT_EDMAC_H */

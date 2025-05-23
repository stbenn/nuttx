/****************************************************************************
 * arch/arm/src/lpc31xx/lpc31_allocateheap.c
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

#include <sys/types.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/kmalloc.h>
#include <arch/board/board.h>

#include "arm.h"
#include "chip.h"
#include "arm_internal.h"
#include "lpc31_memorymap.h"

#ifdef CONFIG_LEGACY_PAGING
#  include <nuttx/page.h>
#  include "pg_macros.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Terminology.  In the flat build (CONFIG_BUILD_FLAT=y), there is only a
 * single heap access with the standard allocations (malloc/free).  This
 * heap is referred to as the user heap.  In the protected build
 * (CONFIG_BUILD_PROTECTED=y) where an MPU is used to protect a region of
 * otherwise flat memory, there will be two allocators:  One that allocates
 * protected (kernel) memory and one that allocates unprotected (user)
 * memory.  These are referred to as the kernel and user heaps,
 * respectively.
 *
 * The ARM has no MPU but does have an MMU.  With this MMU, it can support
 * the kernel build (CONFIG_BUILD_KERNEL=y).  In this configuration, there
 * is one kernel heap but multiple user heaps:  One per task group.  However,
 * in this case, we need only be concerned about initializing the single
 * kernel heap here.
 */

#if defined(CONFIG_BUILD_KERNEL)
#  define MM_ADDREGION kmm_addregion
#else
#  define MM_ADDREGION umm_addregion
#endif

/* Some sanity checking.  If external memory regions are defined, verify
 * that CONFIG_MM_REGIONS is set to match, exactly, the number of external
 * memory regions that we have been asked to add to the heap.
 */

#if defined(CONFIG_LPC31_EXTSRAM0) && defined(CONFIG_LPC31_EXTSRAM0HEAP)
#  if defined(CONFIG_LPC31_EXTSRAM1) && defined(CONFIG_LPC31_EXTSRAM1HEAP)
#    if defined(CONFIG_LPC31_EXTDRAM) && defined(CONFIG_LPC31_EXTDRAMHEAP)
#      /* SRAM+EXTSRAM0+EXTSRAM1+EXTSDRAM */
#      define LPC31_NEXT_REGIONS 4
#    else
#      /* SRAM+EXTSRAM0+EXTSRAM1 */
#      define LPC31_NEXT_REGIONS 3
#    endif
#  elif defined(CONFIG_LPC31_EXTDRAM) && defined(CONFIG_LPC31_EXTDRAMHEAP)
#      /* SRAM+EXTSRAM0+EXTSDRAM */
#      define LPC31_NEXT_REGIONS 3
#  else
#      /* SRAM+EXTSRAM0 */
#      define LPC31_NEXT_REGIONS 2
#  endif
#elif defined(CONFIG_LPC31_EXTSRAM1) && defined(CONFIG_LPC31_EXTSRAM1HEAP)
#  if defined(CONFIG_LPC31_EXTDRAM) && defined(CONFIG_LPC31_EXTDRAMHEAP)
#      /* SRAM+EXTSRAM1+EXTSDRAM */
#      define LPC31_NEXT_REGIONS 3
#  else
#      /* SRAM+EXTSRAM1 */
#      define LPC31_NEXT_REGIONS 2
#  endif
#elif defined(CONFIG_LPC31_EXTDRAM) && defined(CONFIG_LPC31_EXTDRAMHEAP)
#      /* SRAM+EXTSDRAM */
#      define LPC31_NEXT_REGIONS 2
#else
#      /* SRAM */
#      define LPC31_NEXT_REGIONS 1
#endif

#if CONFIG_MM_REGIONS != LPC31_NEXT_REGIONS
#  if CONFIG_MM_REGIONS < LPC31_NEXT_REGIONS
#    error "CONFIG_MM_REGIONS is not large enough for the selected memory regions"
#  else
#    error "CONFIG_MM_REGIONS is too large for the selected memory regions"
#  endif
#  if defined(CONFIG_LPC31_EXTSRAM0) && defined(CONFIG_LPC31_EXTSRAM0HEAP)
#    error "External SRAM0 is selected for heap"
#  endif
#  if defined(CONFIG_LPC31_EXTSRAM1) && defined(CONFIG_LPC31_EXTSRAM1HEAP)
#    error "External SRAM1 is selected for heap"
#  endif
#  if defined(CONFIG_LPC31_EXTDRAM) && defined(CONFIG_LPC31_EXTDRAMHEAP)
#    error "External DRAM is selected for heap"
#  endif
#endif

/* The following determines the end+1 address the heap (in SRAM0 or SRAM1)
 * and that, in turn, determines the size of the heap.  Specifically, this
 * logic it checks if a page table has been allocated at the end of SRAM
 * and, if so, subtracts that the size of the page table from the end+1
 * address of heap.
 *
 * If the page table was not allocated at the end of SRAM, then this logic
 * will let the heap run all the way to the end of SRAM.
 */

#ifdef CONFIG_LEGACY_PAGING
#  ifdef PGTABLE_IN_HIGHSRAM
#    define LPC31_HEAP_VEND (PG_LOCKED_VBASE + PG_TOTAL_VSIZE - PGTABLE_SIZE)
#  else
#    define LPC31_HEAP_VEND (PG_LOCKED_VBASE + PG_TOTAL_VSIZE)
#  endif
#else
#  ifdef PGTABLE_IN_HIGHSRAM
#    define LPC31_HEAP_VEND (LPC31_INTSRAM_VSECTION + LPC31_ISRAM_SIZE - PGTABLE_SIZE)
#  else
#    define LPC31_HEAP_VEND (LPC31_INTSRAM_VSECTION + LPC31_ISRAM_SIZE)
#  endif
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_allocate_heap
 *
 * Description:
 *   This function will be called to dynamically set aside the heap region.
 *
 *   For the kernel build (CONFIG_BUILD_KERNEL=y) with both kernel- and
 *   user-space heaps (CONFIG_MM_KERNEL_HEAP=y), this function provides the
 *   size of the unprotected, user-space heap.
 *
 *   If a protected kernel-space heap is provided, the kernel heap must be
 *   allocated (and protected) by an analogous up_allocate_kheap().
 *
 *   SRAM layout:
 *   Start of SRAM:   .data
 *                    .bss
 *                    IDLE thread stack
 *   End of SRAm:     heap
 *
 *   NOTE: Ignore the erroneous nomenclature DRAM and SDRAM.  That names
 *   date back to an earlier platform that had SDRAM.
 *
 ****************************************************************************/

void up_allocate_heap(void **heap_start, size_t *heap_size)
{
  board_autoled_on(LED_HEAPALLOCATE);
  *heap_start = (void *)g_idle_topstack;
  *heap_size  = LPC31_HEAP_VEND - g_idle_topstack;
}

/****************************************************************************
 * Name: arm_addregion
 *
 * Description:
 *   Memory may be added in non-contiguous chunks.  Additional chunks are
 *   added by calling this function.
 *
 ****************************************************************************/

#if CONFIG_MM_REGIONS > 1
void arm_addregion(void)
{
#if defined(CONFIG_LPC31_EXTSRAM0) && defined(CONFIG_LPC31_EXTSRAM0HEAP)
  MM_ADDREGION((void *)LPC31_EXTSRAM0_VSECTION,
                CONFIG_LPC31_EXTSRAM0SIZE);
#endif

#if defined(CONFIG_LPC31_EXTSRAM1) && defined(CONFIG_LPC31_EXTSRAM1HEAP)
  MM_ADDREGION((void *)LPC31_EXTSRAM1_VSECTION,
                CONFIG_LPC31_EXTSRAM1SIZE);
#endif

#if defined(CONFIG_LPC31_EXTDRAM) && defined(CONFIG_LPC31_EXTDRAMHEAP)
  MM_ADDREGION((void *)LPC31_EXTSDRAM_VSECTION,
                CONFIG_LPC31_EXTDRAMSIZE);
#endif
}
#endif

/****************************************************************************
 * mm/iob/iob_free.c
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

#include <stdbool.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#ifdef CONFIG_IOB_ALLOC
#  include <nuttx/kmalloc.h>
#endif
#include <nuttx/mm/iob.h>

#include "iob.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_IOB_NOTIFIER
#  if !defined(CONFIG_IOB_NOTIFIER_DIV) || CONFIG_IOB_NOTIFIER_DIV < 2
#    define IOB_DIVIDER 1
#  elif CONFIG_IOB_NOTIFIER_DIV < 4
#    define IOB_DIVIDER 2
#  elif CONFIG_IOB_NOTIFIER_DIV < 8
#    define IOB_DIVIDER 4
#  elif CONFIG_IOB_NOTIFIER_DIV < 16
#    define IOB_DIVIDER 8
#  elif CONFIG_IOB_NOTIFIER_DIV < 32
#    define IOB_DIVIDER 16
#  elif CONFIG_IOB_NOTIFIER_DIV < 64
#    define IOB_DIVIDER 32
#  else
#    define IOB_DIVIDER 64
#  endif
#endif

#define IOB_MASK      (IOB_DIVIDER - 1)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: iob_free
 *
 * Description:
 *   Free the I/O buffer at the head of a buffer chain returning it to the
 *   free list.  The link to  the next I/O buffer in the chain is return.
 *
 ****************************************************************************/

FAR struct iob_s *iob_free(FAR struct iob_s *iob)
{
  FAR struct iob_s *next = iob->io_flink;
  irqstate_t flags;
#ifdef CONFIG_IOB_NOTIFIER
  int16_t navail;
#endif

  iobinfo("iob=%p io_pktlen=%u io_len=%u next=%p\n",
          iob, iob->io_pktlen, iob->io_len, next);

  /* Copy the data that only exists in the head of a I/O buffer chain into
   * the next entry.
   */

  if (next != NULL)
    {
      /* Copy and decrement the total packet length, being careful to
       * do nothing too crazy.
       */

      if (iob->io_pktlen > iob->io_len)
        {
          /* Adjust packet length and move it to the next entry */

          next->io_pktlen = iob->io_pktlen - iob->io_len;
          DEBUGASSERT(next->io_pktlen >= next->io_len);
        }
      else
        {
          /* This can only happen if the free entry isn't first entry in the
           * chain...
           */

          next->io_pktlen = 0;
        }

      iobinfo("next=%p io_pktlen=%u io_len=%u\n",
              next, next->io_pktlen, next->io_len);
    }

#ifdef CONFIG_IOB_ALLOC
  if (iob->io_free != NULL)
    {
      iob->io_free(iob->io_data);
      kmm_free(iob);
      return next;
    }
#endif

  /* Free the I/O buffer by adding it to the head of the free or the
   * committed list. We don't know what context we are called from so
   * we use extreme measures to protect the free list:  We disable
   * interrupts very briefly.
   */

  flags = spin_lock_irqsave(&g_iob_lock);

  /* Which list?  If there is a task waiting for an IOB, then put
   * the IOB on either the free list or on the committed list where
   * it is reserved for that allocation (and not available to
   * iob_tryalloc()). This is true for both throttled and non-throttled
   * cases.
   */

  if (g_iob_count < 0)
    {
      g_iob_count++;
      iob->io_flink   = g_iob_committed;
      g_iob_committed = iob;
      spin_unlock_irqrestore(&g_iob_lock, flags);
      nxsem_post(&g_iob_sem);
    }
#if CONFIG_IOB_THROTTLE > 0
  else if (g_throttle_wait > 0 && g_iob_count >= CONFIG_IOB_THROTTLE)
    {
      iob->io_flink   = g_iob_committed;
      g_iob_committed = iob;
      g_throttle_wait--;
      spin_unlock_irqrestore(&g_iob_lock, flags);
      nxsem_post(&g_throttle_sem);
    }
#endif
  else
    {
      g_iob_count++;
      iob->io_flink   = g_iob_freelist;
      g_iob_freelist  = iob;
      spin_unlock_irqrestore(&g_iob_lock, flags);
    }

  DEBUGASSERT(g_iob_count <= CONFIG_IOB_NBUFFERS);

#ifdef CONFIG_IOB_NOTIFIER
  /* Check if the IOB was claimed by a thread that is blocked waiting
   * for an IOB.
   */

  navail = iob_navail(false);
  if (navail > 0 && (navail & IOB_MASK) == 0)
    {
      /* Signal any threads that have requested a signal notification
       * when an IOB becomes available.
       */

      iob_notifier_signal();
    }
#endif

  /* And return the I/O buffer after the one that was freed */

  return next;
}

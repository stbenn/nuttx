/****************************************************************************
 * arch/arm/src/mx8mp/mx8mp_clrpend.c
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

#include <arch/irq.h>

#include "nvic.h"
#include "arm_internal.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mx8mp_clrpend
 *
 * Description:
 *   Clear a pending interrupt at the NVIC.  This does not seem to be
 *   required for most interrupts.  Don't know why...
 *
 *   I keep it in a separate file so that it will not increase the footprint
 *   on mx8mp platforms that do not need this function.
 *
 ****************************************************************************/

void mx8mp_clrpend(int irq)
{
  /* Check for external interrupt */

  if (irq >= MX8MP_IRQ_FIRST)
    {
      if (irq < (MX8MP_IRQ_FIRST + 32))
        {
          putreg32(1 << (irq - MX8MP_IRQ_FIRST),
                   NVIC_IRQ0_31_CLRPEND);
        }
      else if (irq < (MX8MP_IRQ_FIRST + 64))
        {
          putreg32(1 << (irq - MX8MP_IRQ_FIRST - 32),
                   NVIC_IRQ32_63_CLRPEND);
        }
      else if (irq < (MX8MP_IRQ_FIRST +96))
        {
          putreg32(1 << (irq - MX8MP_IRQ_FIRST - 64),
                   NVIC_IRQ64_95_CLRPEND);
        }
      else if (irq < (MX8MP_IRQ_FIRST +128))
        {
          putreg32(1 << (irq - MX8MP_IRQ_FIRST - 96),
                   NVIC_IRQ96_127_CLRPEND);
        }
      else if (irq < NR_IRQS)
        {
          putreg32(1 << (irq - MX8MP_IRQ_FIRST - 128),
                   NVIC_IRQ128_159_CLRPEND);
        }
    }
}

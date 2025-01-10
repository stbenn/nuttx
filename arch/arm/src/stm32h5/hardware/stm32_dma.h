/****************************************************************************
 * arch/arm/src/stm32h5/hardware/stm32_dma.h
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

#ifndef __ARCH_ARM_SRC_STM32H5_HARDWARE_STM32_DMA_H
#define __ARCH_ARM_SRC_STM32H5_HARDWARE_STM32_DMA_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* 2 DMA controllers */

#define DMA1                       (0)
#define DMA2                       (1)

/* The STM32 H5 family has 16 channels total:
 * 8 DMA1 channels(1-8) and 8 DMA2 channels (1-8).
 */

#define DMA_CHAN1                  (0)
#define DMA_CHAN2                  (1)
#define DMA_CHAN3                  (2)
#define DMA_CHAN4                  (3)
#define DMA_CHAN5                  (4)
#define DMA_CHAN6                  (5)
#define DMA_CHAN7                  (6)
#define DMA_CHAN8                  (7)

/* Register Offsets *********************************************************/


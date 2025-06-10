/****************************************************************************
 * arch/arm/src/stm32f0l0g0/hardware/stm32g0_dmamux.h
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

#ifndef __ARCH_ARM_SRC_STM32F0L0G0_HARDWARE_STM32G0_DMAMUX_H
#define __ARCH_ARM_SRC_STM32F0L0G0_HARDWARE_STM32G0_DMAMUX_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* DMAMUX1 mapping **********************************************************/

/* NOTE: DMAMUX1 channels 0 to 7 are connected to DMA1 channels 0 to 7 */

#define DMAMUX1_REQ_GEN0       (1)
#define DMAMUX1_REQ_GEN1       (2)
#define DMAMUX1_REQ_GEN2       (3)
#define DMAMUX1_REQ_GEN3       (4)
#define DMAMUX1_ADC1           (5)
#define DMAMUX1_AES_IN         (6)
#define DMAMUX1_AES_OUT        (7)

/* TODO: ... */

/* DMAP for DMA1 */

#define DMAMAP_DMA1_REQGEN0     DMAMAP_MAP(DMA1, DMAMUX1_REQ_GEN0)
#define DMAMAP_DMA1_REQGEN1     DMAMAP_MAP(DMA1, DMAMUX1_REQ_GEN1)
#define DMAMAP_DMA1_REQGEN2     DMAMAP_MAP(DMA1, DMAMUX1_REQ_GEN2)
#define DMAMAP_DMA1_REQGEN3     DMAMAP_MAP(DMA1, DMAMUX1_REQ_GEN3)
#define DMAMAP_DMA1_ADC1        DMAMAP_MAP(DMA1, DMAMUX1_ADC1)

/* TODO: ... */

#define DMAMUX1_TIM1_CH1         (0)
#define DMAMAP_DMA1_TIM1_CH1         DMAMAP_MAP(DMA1, DMAMUX1_TIM1_CH1)
#define DMAMUX1_TIM1_CH2         (1)
#define DMAMAP_DMA1_TIM1_CH2         DMAMAP_MAP(DMA1, DMAMUX1_TIM1_CH2)
#define DMAMUX1_TIM1_CH3         (2)
#define DMAMAP_DMA1_TIM1_CH3         DMAMAP_MAP(DMA1, DMAMUX1_TIM1_CH3)
#define DMAMUX1_TIM1_CH4         (3)
#define DMAMAP_DMA1_TIM1_CH4         DMAMAP_MAP(DMA1, DMAMUX1_TIM1_CH4)
#define DMAMUX1_TIM1_UP          (4)
#define DMAMAP_DMA1_TIM1_UP          DMAMAP_MAP(DMA1, DMAMUX1_TIM1_UP)
#define DMAMUX1_TIM1_TRIG        (5)
#define DMAMAP_DMA1_TIM1_TRIG        DMAMAP_MAP(DMA1, DMAMUX1_TIM1_TRIG)
#define DMAMUX1_TIM1_COM         (6)
#define DMAMAP_DMA1_TIM1_COM         DMAMAP_MAP(DMA1, DMAMUX1_TIM1_COM)
#define DMAMUX1_TIM2_CH1         (7)
#define DMAMAP_DMA1_TIM2_CH1         DMAMAP_MAP(DMA1, DMAMUX1_TIM2_CH1)
#define DMAMUX1_TIM2_CH2         (8)
#define DMAMAP_DMA1_TIM2_CH2         DMAMAP_MAP(DMA1, DMAMUX1_TIM2_CH2)
#define DMAMUX1_TIM2_CH3         (9)
#define DMAMAP_DMA1_TIM2_CH3         DMAMAP_MAP(DMA1, DMAMUX1_TIM2_CH3)
#define DMAMUX1_TIM2_CH4         (10)
#define DMAMAP_DMA1_TIM2_CH4         DMAMAP_MAP(DMA1, DMAMUX1_TIM2_CH4)
#define DMAMUX1_TIM2_UP          (11)
#define DMAMAP_DMA1_TIM2_UP          DMAMAP_MAP(DMA1, DMAMUX1_TIM2_UP)
#define DMAMUX1_TIM3_CH1         (12)
#define DMAMAP_DMA1_TIM3_CH1         DMAMAP_MAP(DMA1, DMAMUX1_TIM3_CH1)
#define DMAMUX1_TIM3_CH2         (13)
#define DMAMAP_DMA1_TIM3_CH2         DMAMAP_MAP(DMA1, DMAMUX1_TIM3_CH2)
#define DMAMUX1_TIM3_CH3         (14)
#define DMAMAP_DMA1_TIM3_CH3         DMAMAP_MAP(DMA1, DMAMUX1_TIM3_CH3)
#define DMAMUX1_TIM3_CH4         (15)
#define DMAMAP_DMA1_TIM3_CH4         DMAMAP_MAP(DMA1, DMAMUX1_TIM3_CH4)
#define DMAMUX1_TIM3_UP          (16)
#define DMAMAP_DMA1_TIM3_UP          DMAMAP_MAP(DMA1, DMAMUX1_TIM3_UP)
#define DMAMUX1_TIM3_TRIG        (17)
#define DMAMAP_DMA1_TIM3_TRIG        DMAMAP_MAP(DMA1, DMAMUX1_TIM3_TRIG)
#define DMAMUX1_TIM6_UP          (18)
#define DMAMAP_DMA1_TIM6_UP          DMAMAP_MAP(DMA1, DMAMUX1_TIM6_UP)
#define DMAMUX1_TIM7_UP          (19)
#define DMAMAP_DMA1_TIM7_UP          DMAMAP_MAP(DMA1, DMAMUX1_TIM7_UP)
#define DMAMUX1_USART1_RX        (20)
#define DMAMAP_DMA1_USART1_RX        DMAMAP_MAP(DMA1, DMAMUX1_USART1_RX)
#define DMAMUX1_USART1_TX        (21)
#define DMAMAP_DMA1_USART1_TX        DMAMAP_MAP(DMA1, DMAMUX1_USART1_TX)
#define DMAMUX1_USART2_RX        (22)
#define DMAMAP_DMA1_USART2_RX        DMAMAP_MAP(DMA1, DMAMUX1_USART2_RX)
#define DMAMUX1_USART2_TX        (23)
#define DMAMAP_DMA1_USART2_TX        DMAMAP_MAP(DMA1, DMAMUX1_USART2_TX)
#define DMAMUX1_LPUART1_RX       (24)
#define DMAMAP_DMA1_LPUART1_RX       DMAMAP_MAP(DMA1, DMAMUX1_LPUART1_RX)
#define DMAMUX1_LPUART1_TX       (25)
#define DMAMAP_DMA1_LPUART1_TX       DMAMAP_MAP(DMA1, DMAMUX1_LPUART1_TX)
#define DMAMUX1_SPI1_RX          (26)
#define DMAMAP_DMA1_SPI1_RX          DMAMAP_MAP(DMA1, DMAMUX1_SPI1_RX)
#define DMAMUX1_SPI1_TX          (27)
#define DMAMAP_DMA1_SPI1_TX          DMAMAP_MAP(DMA1, DMAMUX1_SPI1_TX)
#define DMAMUX1_SPI2_RX          (28)
#define DMAMAP_DMA1_SPI2_RX          DMAMAP_MAP(DMA1, DMAMUX1_SPI2_RX)
#define DMAMUX1_SPI2_TX          (29)
#define DMAMAP_DMA1_SPI2_TX          DMAMAP_MAP(DMA1, DMAMUX1_SPI2_TX)
#define DMAMUX1_I2C1_RX          (30)
#define DMAMAP_DMA1_I2C1_RX          DMAMAP_MAP(DMA1, DMAMUX1_I2C1_RX)
#define DMAMUX1_I2C1_TX          (31)
#define DMAMAP_DMA1_I2C1_TX          DMAMAP_MAP(DMA1, DMAMUX1_I2C1_TX)
#define DMAMUX1_I2C2_RX          (32)
#define DMAMAP_DMA1_I2C2_RX          DMAMAP_MAP(DMA1, DMAMUX1_I2C2_RX)
#define DMAMUX1_I2C2_TX          (33)
#define DMAMAP_DMA1_I2C2_TX          DMAMAP_MAP(DMA1, DMAMUX1_I2C2_TX)
#define DMAMUX1_ADC1             (34)
#define DMAMAP_DMA1_ADC1             DMAMAP_MAP(DMA1, DMAMUX1_ADC1)
#define DMAMUX1_DAC_CH1          (35)
#define DMAMAP_DMA1_DAC_CH1          DMAMAP_MAP(DMA1, DMAMUX1_DAC_CH1)
#define DMAMUX1_DAC_CH2          (36)
#define DMAMAP_DMA1_DAC_CH2          DMAMAP_MAP(DMA1, DMAMUX1_DAC_CH2)
#define DMAMUX1_AES_IN           (37)
#define DMAMAP_DMA1_AES_IN           DMAMAP_MAP(DMA1, DMAMUX1_AES_IN)
#define DMAMUX1_AES_OUT          (38)
#define DMAMAP_DMA1_AES_OUT          DMAMAP_MAP(DMA1, DMAMUX1_AES_OUT)
#define DMAMUX1_SUBGHZSPI_RX     (39)
#define DMAMAP_DMA1_SUBGHZSPI_RX     DMAMAP_MAP(DMA1, DMAMUX1_SUBGHZSPI_RX)
#define DMAMUX1_SUBGHZSPI_TX     (40)
#define DMAMAP_DMA1_SUBGHZSPI_TX     DMAMAP_MAP(DMA1, DMAMUX1_SUBGHZSPI_TX)
#define DMAMUX1_AES_IN_ALT       (41)
#define DMAMAP_DMA1_AES_IN_ALT       DMAMAP_MAP(DMA1, DMAMUX1_AES_IN_ALT)
#define DMAMUX1_AES_OUT_ALT      (42)
#define DMAMAP_DMA1_AES_OUT_ALT      DMAMAP_MAP(DMA1, DMAMUX1_AES_OUT_ALT)
#define DMAMUX1_DMAMUX1_REQGEN0  (48)
#define DMAMAP_DMA1_DMAMUX1_REQGEN0  DMAMAP_MAP(DMA1, DMAMUX1_DMAMUX1_REQGEN0)
#define DMAMUX1_DMAMUX1_REQGEN1  (49)
#define DMAMAP_DMA1_DMAMUX1_REQGEN1  DMAMAP_MAP(DMA1, DMAMUX1_DMAMUX1_REQGEN1)
#define DMAMUX1_DMAMUX1_REQGEN2  (50)
#define DMAMAP_DMA1_DMAMUX1_REQGEN2  DMAMAP_MAP(DMA1, DMAMUX1_DMAMUX1_REQGEN2)
#define DMAMUX1_DMAMUX1_REQGEN3  (51)
#define DMAMAP_DMA1_DMAMUX1_REQGEN3  DMAMAP_MAP(DMA1, DMAMUX1_DMAMUX1_REQGEN3)


#endif /* __ARCH_ARM_SRC_STM32F0L0G0_HARDWARE_STM32G0_DMAMUX_H */

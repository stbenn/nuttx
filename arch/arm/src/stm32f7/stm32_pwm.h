/****************************************************************************
 * arch/arm/src/stm32f7/stm32_pwm.h
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

#ifndef __ARCH_ARM_SRC_STM32F7_STM32_PWM_H
#define __ARCH_ARM_SRC_STM32F7_STM32_PWM_H

/* The STM32 does not have dedicated PWM hardware.  Rather, pulsed output
 * control is a capability of the STM32 timers.  The logic in this file
 * implements the lower half of the standard, NuttX PWM interface using the
 * STM32 timers.  That interface is described in include/nuttx/timers/pwm.h.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/timers/pwm.h>

#include "chip.h"

#ifdef CONFIG_STM32F7_PWM
#  include <arch/board/board.h>
#  include "hardware/stm32_tim.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Timer devices may be used for different purposes.  One special purpose is
 * to generate modulated outputs for such things as motor control.
 * If CONFIG_STM32F7_TIMn is defined then the CONFIG_STM32F7_TIMn_PWM must
 * also be defined to indicate that timer "n" is intended to be used for
 * pulsed output signal generation.
 */

#ifndef CONFIG_STM32F7_TIM1
#  undef CONFIG_STM32F7_TIM1_PWM
#endif
#ifndef CONFIG_STM32F7_TIM2
#  undef CONFIG_STM32F7_TIM2_PWM
#endif
#ifndef CONFIG_STM32F7_TIM3
#  undef CONFIG_STM32F7_TIM3_PWM
#endif
#ifndef CONFIG_STM32F7_TIM4
#  undef CONFIG_STM32F7_TIM4_PWM
#endif
#ifndef CONFIG_STM32F7_TIM5
#  undef CONFIG_STM32F7_TIM5_PWM
#endif
#ifndef CONFIG_STM32F7_TIM8
#  undef CONFIG_STM32F7_TIM8_PWM
#endif
#ifndef CONFIG_STM32F7_TIM9
#  undef CONFIG_STM32F7_TIM9_PWM
#endif
#ifndef CONFIG_STM32F7_TIM10
#  undef CONFIG_STM32F7_TIM10_PWM
#endif
#ifndef CONFIG_STM32F7_TIM11
#  undef CONFIG_STM32F7_TIM11_PWM
#endif
#ifndef CONFIG_STM32F7_TIM12
#  undef CONFIG_STM32F7_TIM12_PWM
#endif
#ifndef CONFIG_STM32F7_TIM13
#  undef CONFIG_STM32F7_TIM13_PWM
#endif
#ifndef CONFIG_STM32F7_TIM14
#  undef CONFIG_STM32F7_TIM14_PWM
#endif

/* The basic timers (timer 6 and 7) are not capable of generating output
 * pulses
 */

#undef CONFIG_STM32F7_TIM6_PWM
#undef CONFIG_STM32F7_TIM7_PWM

/* Check if PWM support for any channel is enabled. */

#ifdef CONFIG_STM32F7_PWM

/* PWM driver channels configuration */

#ifdef CONFIG_STM32F7_PWM_MULTICHAN

#ifdef CONFIG_STM32F7_TIM1_CHANNEL1
#  define PWM_TIM1_CHANNEL1 1
#else
#  define PWM_TIM1_CHANNEL1 0
#endif
#ifdef CONFIG_STM32F7_TIM1_CHANNEL2
#  define PWM_TIM1_CHANNEL2 1
#else
#  define PWM_TIM1_CHANNEL2 0
#endif
#ifdef CONFIG_STM32F7_TIM1_CHANNEL3
#  define PWM_TIM1_CHANNEL3 1
#else
#  define PWM_TIM1_CHANNEL3 0
#endif
#ifdef CONFIG_STM32F7_TIM1_CHANNEL4
#  define PWM_TIM1_CHANNEL4 1
#else
#  define PWM_TIM1_CHANNEL4 0
#endif
#ifdef CONFIG_STM32F7_TIM1_CHANNEL5
#  define PWM_TIM1_CHANNEL5 1
#else
#  define PWM_TIM1_CHANNEL5 0
#endif
#ifdef CONFIG_STM32F7_TIM1_CHANNEL6
#  define PWM_TIM1_CHANNEL6 1
#else
#  define PWM_TIM1_CHANNEL6 0
#endif
#define PWM_TIM1_NCHANNELS (PWM_TIM1_CHANNEL1 + PWM_TIM1_CHANNEL2 + \
                            PWM_TIM1_CHANNEL3 + PWM_TIM1_CHANNEL4 + \
                            PWM_TIM1_CHANNEL5 + PWM_TIM1_CHANNEL6)

#ifdef CONFIG_STM32F7_TIM2_CHANNEL1
#  define PWM_TIM2_CHANNEL1 1
#else
#  define PWM_TIM2_CHANNEL1 0
#endif
#ifdef CONFIG_STM32F7_TIM2_CHANNEL2
#  define PWM_TIM2_CHANNEL2 1
#else
#  define PWM_TIM2_CHANNEL2 0
#endif
#ifdef CONFIG_STM32F7_TIM2_CHANNEL3
#  define PWM_TIM2_CHANNEL3 1
#else
#  define PWM_TIM2_CHANNEL3 0
#endif
#ifdef CONFIG_STM32F7_TIM2_CHANNEL4
#  define PWM_TIM2_CHANNEL4 1
#else
#  define PWM_TIM2_CHANNEL4 0
#endif
#define PWM_TIM2_NCHANNELS (PWM_TIM2_CHANNEL1 + PWM_TIM2_CHANNEL2 + \
                            PWM_TIM2_CHANNEL3 + PWM_TIM2_CHANNEL4)

#ifdef CONFIG_STM32F7_TIM3_CHANNEL1
#  define PWM_TIM3_CHANNEL1 1
#else
#  define PWM_TIM3_CHANNEL1 0
#endif
#ifdef CONFIG_STM32F7_TIM3_CHANNEL2
#  define PWM_TIM3_CHANNEL2 1
#else
#  define PWM_TIM3_CHANNEL2 0
#endif
#ifdef CONFIG_STM32F7_TIM3_CHANNEL3
#  define PWM_TIM3_CHANNEL3 1
#else
#  define PWM_TIM3_CHANNEL3 0
#endif
#ifdef CONFIG_STM32F7_TIM3_CHANNEL4
#  define PWM_TIM3_CHANNEL4 1
#else
#  define PWM_TIM3_CHANNEL4 0
#endif
#define PWM_TIM3_NCHANNELS (PWM_TIM3_CHANNEL1 + PWM_TIM3_CHANNEL2 + \
                            PWM_TIM3_CHANNEL3 + PWM_TIM3_CHANNEL4)

#ifdef CONFIG_STM32F7_TIM4_CHANNEL1
#  define PWM_TIM4_CHANNEL1 1
#else
#  define PWM_TIM4_CHANNEL1 0
#endif
#ifdef CONFIG_STM32F7_TIM4_CHANNEL2
#  define PWM_TIM4_CHANNEL2 1
#else
#  define PWM_TIM4_CHANNEL2 0
#endif
#ifdef CONFIG_STM32F7_TIM4_CHANNEL3
#  define PWM_TIM4_CHANNEL3 1
#else
#  define PWM_TIM4_CHANNEL3 0
#endif
#ifdef CONFIG_STM32F7_TIM4_CHANNEL4
#  define PWM_TIM4_CHANNEL4 1
#else
#  define PWM_TIM4_CHANNEL4 0
#endif
#define PWM_TIM4_NCHANNELS (PWM_TIM4_CHANNEL1 + PWM_TIM4_CHANNEL2 + \
                            PWM_TIM4_CHANNEL3 + PWM_TIM4_CHANNEL4)

#ifdef CONFIG_STM32F7_TIM5_CHANNEL1
#  define PWM_TIM5_CHANNEL1 1
#else
#  define PWM_TIM5_CHANNEL1 0
#endif
#ifdef CONFIG_STM32F7_TIM5_CHANNEL2
#  define PWM_TIM5_CHANNEL2 1
#else
#  define PWM_TIM5_CHANNEL2 0
#endif
#ifdef CONFIG_STM32F7_TIM5_CHANNEL3
#  define PWM_TIM5_CHANNEL3 1
#else
#  define PWM_TIM5_CHANNEL3 0
#endif
#ifdef CONFIG_STM32F7_TIM5_CHANNEL4
#  define PWM_TIM5_CHANNEL4 1
#else
#  define PWM_TIM5_CHANNEL4 0
#endif
#define PWM_TIM5_NCHANNELS (PWM_TIM5_CHANNEL1 + PWM_TIM5_CHANNEL2 + \
                            PWM_TIM5_CHANNEL3 + PWM_TIM5_CHANNEL4)

#ifdef CONFIG_STM32F7_TIM8_CHANNEL1
#  define PWM_TIM8_CHANNEL1 1
#else
#  define PWM_TIM8_CHANNEL1 0
#endif
#ifdef CONFIG_STM32F7_TIM8_CHANNEL2
#  define PWM_TIM8_CHANNEL2 1
#else
#  define PWM_TIM8_CHANNEL2 0
#endif
#ifdef CONFIG_STM32F7_TIM8_CHANNEL3
#  define PWM_TIM8_CHANNEL3 1
#else
#  define PWM_TIM8_CHANNEL3 0
#endif
#ifdef CONFIG_STM32F7_TIM8_CHANNEL4
#  define PWM_TIM8_CHANNEL4 1
#else
#  define PWM_TIM8_CHANNEL4 0
#endif
#ifdef CONFIG_STM32F7_TIM8_CHANNEL5
#  define PWM_TIM8_CHANNEL5 1
#else
#  define PWM_TIM8_CHANNEL5 0
#endif
#ifdef CONFIG_STM32F7_TIM8_CHANNEL6
#  define PWM_TIM8_CHANNEL6 1
#else
#  define PWM_TIM8_CHANNEL6 0
#endif
#define PWM_TIM8_NCHANNELS (PWM_TIM8_CHANNEL1 + PWM_TIM8_CHANNEL2 + \
                            PWM_TIM8_CHANNEL3 + PWM_TIM8_CHANNEL4 + \
                            PWM_TIM8_CHANNEL5 + PWM_TIM8_CHANNEL6)

#ifdef CONFIG_STM32F7_TIM9_CHANNEL1
#  define PWM_TIM9_CHANNEL1 1
#else
#  define PWM_TIM9_CHANNEL1 0
#endif
#ifdef CONFIG_STM32F7_TIM9_CHANNEL2
#  define PWM_TIM9_CHANNEL2 1
#else
#  define PWM_TIM9_CHANNEL2 0
#endif
#define PWM_TIM9_NCHANNELS (PWM_TIM9_CHANNEL1 + PWM_TIM9_CHANNEL2)

#ifdef CONFIG_STM32F7_TIM10_CHANNEL1
#  define PWM_TIM10_CHANNEL1 1
#else
#  define PWM_TIM10_CHANNEL1 0
#endif
#define PWM_TIM10_NCHANNELS (PWM_TIM10_CHANNEL1)

#ifdef CONFIG_STM32F7_TIM11_CHANNEL1
#  define PWM_TIM11_CHANNEL1 1
#else
#  define PWM_TIM11_CHANNEL1 0
#endif
#define PWM_TIM11_NCHANNELS (PWM_TIM11_CHANNEL1)

#ifdef CONFIG_STM32F7_TIM12_CHANNEL1
#  define PWM_TIM12_CHANNEL1 1
#else
#  define PWM_TIM12_CHANNEL1 0
#endif
#ifdef CONFIG_STM32F7_TIM12_CHANNEL2
#  define PWM_TIM12_CHANNEL2 1
#else
#  define PWM_TIM12_CHANNEL2 0
#endif
#define PWM_TIM12_NCHANNELS (PWM_TIM12_CHANNEL1 + PWM_TIM12_CHANNEL2)

#ifdef CONFIG_STM32F7_TIM13_CHANNEL1
#  define PWM_TIM13_CHANNEL1 1
#else
#  define PWM_TIM13_CHANNEL1 0
#endif
#define PWM_TIM13_NCHANNELS (PWM_TIM13_CHANNEL1)

#ifdef CONFIG_STM32F7_TIM14_CHANNEL1
#  define PWM_TIM14_CHANNEL1 1
#else
#  define PWM_TIM14_CHANNEL1 0
#endif
#define PWM_TIM14_NCHANNELS (PWM_TIM14_CHANNEL1)

#else  /* !CONFIG_PWM_MULTICHAN */

/* For each timer that is enabled for PWM usage, we need the following
 * additional configuration settings:
 *
 * CONFIG_STM32F7_TIMx_CHANNEL - Specifies the timer output channel {1,..,4}
 * PWM_TIMx_CHn - One of the values defined in chip/stm32*_pinmap.h.  In the
 * case where there are multiple pin selections, the correct setting must be
 * provided in the arch/board/board.h file.
 *
 * NOTE: The STM32 timers are each capable of generating different signals on
 * each of the four channels with different duty cycles.  That capability is
 * not supported by this driver:  Only one output channel per timer.
 */

#ifdef CONFIG_STM32F7_TIM1_PWM
#  if !defined(CONFIG_STM32F7_TIM1_CHANNEL)
#    error "CONFIG_STM32F7_TIM1_CHANNEL must be provided"
#  elif CONFIG_STM32F7_TIM1_CHANNEL == 1
#    define CONFIG_STM32F7_TIM1_CHANNEL1 1
#    define CONFIG_STM32F7_TIM1_CH1MODE  CONFIG_STM32F7_TIM1_CHMODE
#  elif CONFIG_STM32F7_TIM1_CHANNEL == 2
#    define CONFIG_STM32F7_TIM1_CHANNEL2 1
#    define CONFIG_STM32F7_TIM1_CH2MODE  CONFIG_STM32F7_TIM1_CHMODE
#  elif CONFIG_STM32F7_TIM1_CHANNEL == 3
#    define CONFIG_STM32F7_TIM1_CHANNEL3 1
#    define CONFIG_STM32F7_TIM1_CH3MODE  CONFIG_STM32F7_TIM1_CHMODE
#  elif CONFIG_STM32F7_TIM1_CHANNEL == 4
#    define CONFIG_STM32F7_TIM1_CHANNEL4 1
#    define CONFIG_STM32F7_TIM1_CH4MODE  CONFIG_STM32F7_TIM1_CHMODE
#  else
#    error "Unsupported value of CONFIG_STM32F7_TIM1_CHANNEL"
#  endif
#  define PWM_TIM1_NCHANNELS 1
#endif

#ifdef CONFIG_STM32F7_TIM2_PWM
#  if !defined(CONFIG_STM32F7_TIM2_CHANNEL)
#    error "CONFIG_STM32F7_TIM2_CHANNEL must be provided"
#  elif CONFIG_STM32F7_TIM2_CHANNEL == 1
#    define CONFIG_STM32F7_TIM2_CHANNEL1 1
#    define CONFIG_STM32F7_TIM2_CH1MODE  CONFIG_STM32F7_TIM2_CHMODE
#  elif CONFIG_STM32F7_TIM2_CHANNEL == 2
#    define CONFIG_STM32F7_TIM2_CHANNEL2 1
#    define CONFIG_STM32F7_TIM2_CH2MODE  CONFIG_STM32F7_TIM2_CHMODE
#  elif CONFIG_STM32F7_TIM2_CHANNEL == 3
#    define CONFIG_STM32F7_TIM2_CHANNEL3 1
#    define CONFIG_STM32F7_TIM2_CH3MODE  CONFIG_STM32F7_TIM2_CHMODE
#  elif CONFIG_STM32F7_TIM2_CHANNEL == 4
#    define CONFIG_STM32F7_TIM2_CHANNEL4 1
#    define CONFIG_STM32F7_TIM2_CH4MODE  CONFIG_STM32F7_TIM2_CHMODE
#  else
#    error "Unsupported value of CONFIG_STM32F7_TIM2_CHANNEL"
#  endif
#  define PWM_TIM2_NCHANNELS 1
#endif

#ifdef CONFIG_STM32F7_TIM3_PWM
#  if !defined(CONFIG_STM32F7_TIM3_CHANNEL)
#    error "CONFIG_STM32F7_TIM3_CHANNEL must be provided"
#  elif CONFIG_STM32F7_TIM3_CHANNEL == 1
#    define CONFIG_STM32F7_TIM3_CHANNEL1 1
#    define CONFIG_STM32F7_TIM3_CH1MODE  CONFIG_STM32F7_TIM3_CHMODE
#  elif CONFIG_STM32F7_TIM3_CHANNEL == 2
#    define CONFIG_STM32F7_TIM3_CHANNEL2 1
#    define CONFIG_STM32F7_TIM3_CH2MODE  CONFIG_STM32F7_TIM3_CHMODE
#  elif CONFIG_STM32F7_TIM3_CHANNEL == 3
#    define CONFIG_STM32F7_TIM3_CHANNEL3 1
#    define CONFIG_STM32F7_TIM3_CH3MODE  CONFIG_STM32F7_TIM3_CHMODE
#  elif CONFIG_STM32F7_TIM3_CHANNEL == 4
#    define CONFIG_STM32F7_TIM3_CHANNEL4 1
#    define CONFIG_STM32F7_TIM3_CH4MODE  CONFIG_STM32F7_TIM3_CHMODE
#  else
#    error "Unsupported value of CONFIG_STM32F7_TIM3_CHANNEL"
#  endif
#  define PWM_TIM3_NCHANNELS 1
#endif

#ifdef CONFIG_STM32F7_TIM4_PWM
#  if !defined(CONFIG_STM32F7_TIM4_CHANNEL)
#    error "CONFIG_STM32F7_TIM4_CHANNEL must be provided"
#  elif CONFIG_STM32F7_TIM4_CHANNEL == 1
#    define CONFIG_STM32F7_TIM4_CHANNEL1 1
#    define CONFIG_STM32F7_TIM4_CH1MODE  CONFIG_STM32F7_TIM4_CHMODE
#  elif CONFIG_STM32F7_TIM4_CHANNEL == 2
#    define CONFIG_STM32F7_TIM4_CHANNEL2 1
#    define CONFIG_STM32F7_TIM4_CH2MODE  CONFIG_STM32F7_TIM4_CHMODE
#  elif CONFIG_STM32F7_TIM4_CHANNEL == 3
#    define CONFIG_STM32F7_TIM4_CHANNEL3 1
#    define CONFIG_STM32F7_TIM4_CH3MODE  CONFIG_STM32F7_TIM4_CHMODE
#  elif CONFIG_STM32F7_TIM4_CHANNEL == 4
#    define CONFIG_STM32F7_TIM4_CHANNEL4 1
#    define CONFIG_STM32F7_TIM4_CH4MODE  CONFIG_STM32F7_TIM4_CHMODE
#  else
#    error "Unsupported value of CONFIG_STM32F7_TIM4_CHANNEL"
#  endif
#  define PWM_TIM4_NCHANNELS 1
#endif

#ifdef CONFIG_STM32F7_TIM5_PWM
#  if !defined(CONFIG_STM32F7_TIM5_CHANNEL)
#    error "CONFIG_STM32F7_TIM5_CHANNEL must be provided"
#  elif CONFIG_STM32F7_TIM5_CHANNEL == 1
#    define CONFIG_STM32F7_TIM5_CHANNEL1 1
#    define CONFIG_STM32F7_TIM5_CH1MODE  CONFIG_STM32F7_TIM5_CHMODE
#  elif CONFIG_STM32F7_TIM5_CHANNEL == 2
#    define CONFIG_STM32F7_TIM5_CHANNEL2 1
#    define CONFIG_STM32F7_TIM5_CH2MODE  CONFIG_STM32F7_TIM5_CHMODE
#  elif CONFIG_STM32F7_TIM5_CHANNEL == 3
#    define CONFIG_STM32F7_TIM5_CHANNEL3 1
#    define CONFIG_STM32F7_TIM5_CH3MODE  CONFIG_STM32F7_TIM5_CHMODE
#  elif CONFIG_STM32F7_TIM5_CHANNEL == 4
#    define CONFIG_STM32F7_TIM5_CHANNEL4 1
#    define CONFIG_STM32F7_TIM5_CH4MODE  CONFIG_STM32F7_TIM5_CHMODE
#  else
#    error "Unsupported value of CONFIG_STM32F7_TIM5_CHANNEL"
#  endif
#  define PWM_TIM5_NCHANNELS 1
#endif

#ifdef CONFIG_STM32F7_TIM8_PWM
#  if !defined(CONFIG_STM32F7_TIM8_CHANNEL)
#    error "CONFIG_STM32F7_TIM8_CHANNEL must be provided"
#  elif CONFIG_STM32F7_TIM8_CHANNEL == 1
#    define CONFIG_STM32F7_TIM8_CHANNEL1 1
#    define CONFIG_STM32F7_TIM8_CH1MODE  CONFIG_STM32F7_TIM8_CHMODE
#  elif CONFIG_STM32F7_TIM8_CHANNEL == 2
#    define CONFIG_STM32F7_TIM8_CHANNEL2 1
#    define CONFIG_STM32F7_TIM8_CH2MODE  CONFIG_STM32F7_TIM8_CHMODE
#  elif CONFIG_STM32F7_TIM8_CHANNEL == 3
#    define CONFIG_STM32F7_TIM8_CHANNEL3 1
#    define CONFIG_STM32F7_TIM8_CH3MODE  CONFIG_STM32F7_TIM8_CHMODE
#  elif CONFIG_STM32F7_TIM8_CHANNEL == 4
#    define CONFIG_STM32F7_TIM8_CHANNEL4 1
#    define CONFIG_STM32F7_TIM8_CH4MODE  CONFIG_STM32F7_TIM8_CHMODE
#  else
#    error "Unsupported value of CONFIG_STM32F7_TIM8_CHANNEL"
#  endif
#  define PWM_TIM8_NCHANNELS 1
#endif

#ifdef CONFIG_STM32F7_TIM9_PWM
#  if !defined(CONFIG_STM32F7_TIM9_CHANNEL)
#    error "CONFIG_STM32F7_TIM9_CHANNEL must be provided"
#  elif CONFIG_STM32F7_TIM9_CHANNEL == 1
#    define CONFIG_STM32F7_TIM9_CHANNEL1 1
#    define CONFIG_STM32F7_TIM9_CH1MODE  CONFIG_STM32F7_TIM9_CHMODE
#  elif CONFIG_STM32F7_TIM9_CHANNEL == 2
#    define CONFIG_STM32F7_TIM9_CHANNEL2 1
#    define CONFIG_STM32F7_TIM9_CH2MODE  CONFIG_STM32F7_TIM9_CHMODE
#  else
#    error "Unsupported value of CONFIG_STM32F7_TIM9_CHANNEL"
#  endif
#  define PWM_TIM9_NCHANNELS 1
#endif

#ifdef CONFIG_STM32F7_TIM10_PWM
#  if !defined(CONFIG_STM32F7_TIM10_CHANNEL)
#    error "CONFIG_STM32F7_TIM10_CHANNEL must be provided"
#  elif CONFIG_STM32F7_TIM10_CHANNEL == 1
#    define CONFIG_STM32F7_TIM10_CHANNEL1 1
#    define CONFIG_STM32F7_TIM10_CH1MODE  CONFIG_STM32F7_TIM10_CHMODE
#  else
#    error "Unsupported value of CONFIG_STM32F7_TIM10_CHANNEL"
#  endif
#  define PWM_TIM10_NCHANNELS 1
#endif

#ifdef CONFIG_STM32F7_TIM11_PWM
#  if !defined(CONFIG_STM32F7_TIM11_CHANNEL)
#    error "CONFIG_STM32F7_TIM11_CHANNEL must be provided"
#  elif CONFIG_STM32F7_TIM11_CHANNEL == 1
#    define CONFIG_STM32F7_TIM11_CHANNEL1 1
#    define CONFIG_STM32F7_TIM11_CH1MODE  CONFIG_STM32F7_TIM11_CHMODE
#  else
#    error "Unsupported value of CONFIG_STM32F7_TIM11_CHANNEL"
#  endif
#  define PWM_TIM11_NCHANNELS 1
#endif

#ifdef CONFIG_STM32F7_TIM12_PWM
#  if !defined(CONFIG_STM32F7_TIM12_CHANNEL)
#    error "CONFIG_STM32F7_TIM12_CHANNEL must be provided"
#  elif CONFIG_STM32F7_TIM12_CHANNEL == 1
#    define CONFIG_STM32F7_TIM12_CHANNEL1 1
#    define CONFIG_STM32F7_TIM12_CH1MODE  CONFIG_STM32F7_TIM12_CHMODE
#  elif CONFIG_STM32F7_TIM12_CHANNEL == 2
#    define CONFIG_STM32F7_TIM12_CHANNEL2 1
#    define CONFIG_STM32F7_TIM12_CH2MODE  CONFIG_STM32F7_TIM12_CHMODE
#  else
#    error "Unsupported value of CONFIG_STM32F7_TIM12_CHANNEL"
#  endif
#  define PWM_TIM12_NCHANNELS 1
#endif

#ifdef CONFIG_STM32F7_TIM13_PWM
#  if !defined(CONFIG_STM32F7_TIM13_CHANNEL)
#    error "CONFIG_STM32F7_TIM13_CHANNEL must be provided"
#  elif CONFIG_STM32F7_TIM13_CHANNEL == 1
#    define CONFIG_STM32F7_TIM13_CHANNEL1 1
#    define CONFIG_STM32F7_TIM13_CH1MODE  CONFIG_STM32F7_TIM13_CHMODE
#  else
#    error "Unsupported value of CONFIG_STM32F7_TIM13_CHANNEL"
#  endif
#  define PWM_TIM13_NCHANNELS 1
#endif

#ifdef CONFIG_STM32F7_TIM14_PWM
#  if !defined(CONFIG_STM32F7_TIM14_CHANNEL)
#    error "CONFIG_STM32F7_TIM14_CHANNEL must be provided"
#  elif CONFIG_STM32F7_TIM14_CHANNEL == 1
#    define CONFIG_STM32F7_TIM14_CHANNEL1 1
#    define CONFIG_STM32F7_TIM14_CH1MODE  CONFIG_STM32F7_TIM14_CHMODE
#  else
#    error "Unsupported value of CONFIG_STM32F7_TIM14_CHANNEL"
#  endif
#  define PWM_TIM14_NCHANNELS 1
#endif

#endif /* CONFIG_STM32F7_PWM_MULTICHAN */

#ifdef CONFIG_STM32F7_TIM1_CH1OUT
#  define PWM_TIM1_CH1CFG GPIO_TIM1_CH1OUT
#else
#  define PWM_TIM1_CH1CFG 0
#endif
#ifdef CONFIG_STM32F7_TIM1_CH1NOUT
#  define PWM_TIM1_CH1NCFG GPIO_TIM1_CH1NOUT
#else
#  define PWM_TIM1_CH1NCFG 0
#endif
#ifdef CONFIG_STM32F7_TIM1_CH2OUT
#  define PWM_TIM1_CH2CFG GPIO_TIM1_CH2OUT
#else
#  define PWM_TIM1_CH2CFG 0
#endif
#ifdef CONFIG_STM32F7_TIM1_CH2NOUT
#  define PWM_TIM1_CH2NCFG GPIO_TIM1_CH2NOUT
#else
#  define PWM_TIM1_CH2NCFG 0
#endif
#ifdef CONFIG_STM32F7_TIM1_CH3OUT
#  define PWM_TIM1_CH3CFG GPIO_TIM1_CH3OUT
#else
#  define PWM_TIM1_CH3CFG 0
#endif
#ifdef CONFIG_STM32F7_TIM1_CH3NOUT
#  define PWM_TIM1_CH3NCFG GPIO_TIM1_CH3NOUT
#else
#  define PWM_TIM1_CH3NCFG 0
#endif
#ifdef CONFIG_STM32F7_TIM1_CH4OUT
#  define PWM_TIM1_CH4CFG GPIO_TIM1_CH4OUT
#else
#  define PWM_TIM1_CH4CFG 0
#endif

#ifdef CONFIG_STM32F7_TIM2_CH1OUT
#  define PWM_TIM2_CH1CFG GPIO_TIM2_CH1OUT
#else
#  define PWM_TIM2_CH1CFG 0
#endif
#ifdef CONFIG_STM32F7_TIM2_CH2OUT
#  define PWM_TIM2_CH2CFG GPIO_TIM2_CH2OUT
#else
#  define PWM_TIM2_CH2CFG 0
#endif
#ifdef CONFIG_STM32F7_TIM2_CH3OUT
#  define PWM_TIM2_CH3CFG GPIO_TIM2_CH3OUT
#else
#  define PWM_TIM2_CH3CFG 0
#endif
#ifdef CONFIG_STM32F7_TIM2_CH4OUT
#  define PWM_TIM2_CH4CFG GPIO_TIM2_CH4OUT
#else
#  define PWM_TIM2_CH4CFG 0
#endif

#ifdef CONFIG_STM32F7_TIM3_CH1OUT
#  define PWM_TIM3_CH1CFG GPIO_TIM3_CH1OUT
#else
#  define PWM_TIM3_CH1CFG 0
#endif
#ifdef CONFIG_STM32F7_TIM3_CH2OUT
#  define PWM_TIM3_CH2CFG GPIO_TIM3_CH2OUT
#else
#  define PWM_TIM3_CH2CFG 0
#endif
#ifdef CONFIG_STM32F7_TIM3_CH3OUT
#  define PWM_TIM3_CH3CFG GPIO_TIM3_CH3OUT
#else
#  define PWM_TIM3_CH3CFG 0
#endif
#ifdef CONFIG_STM32F7_TIM3_CH4OUT
#  define PWM_TIM3_CH4CFG GPIO_TIM3_CH4OUT
#else
#  define PWM_TIM3_CH4CFG 0
#endif

#ifdef CONFIG_STM32F7_TIM4_CH1OUT
#  define PWM_TIM4_CH1CFG GPIO_TIM4_CH1OUT
#else
#  define PWM_TIM4_CH1CFG 0
#endif
#ifdef CONFIG_STM32F7_TIM4_CH2OUT
#  define PWM_TIM4_CH2CFG GPIO_TIM4_CH2OUT
#else
#  define PWM_TIM4_CH2CFG 0
#endif
#ifdef CONFIG_STM32F7_TIM4_CH3OUT
#  define PWM_TIM4_CH3CFG GPIO_TIM4_CH3OUT
#else
#  define PWM_TIM4_CH3CFG 0
#endif
#ifdef CONFIG_STM32F7_TIM4_CH4OUT
#  define PWM_TIM4_CH4CFG GPIO_TIM4_CH4OUT
#else
#  define PWM_TIM4_CH4CFG 0
#endif

#ifdef CONFIG_STM32F7_TIM5_CH1OUT
#  define PWM_TIM5_CH1CFG GPIO_TIM5_CH1OUT
#else
#  define PWM_TIM5_CH1CFG 0
#endif
#ifdef CONFIG_STM32F7_TIM5_CH2OUT
#  define PWM_TIM5_CH2CFG GPIO_TIM5_CH2OUT
#else
#  define PWM_TIM5_CH2CFG 0
#endif
#ifdef CONFIG_STM32F7_TIM5_CH3OUT
#  define PWM_TIM5_CH3CFG GPIO_TIM5_CH3OUT
#else
#  define PWM_TIM5_CH3CFG 0
#endif
#ifdef CONFIG_STM32F7_TIM5_CH4OUT
#  define PWM_TIM5_CH4CFG GPIO_TIM5_CH4OUT
#else
#  define PWM_TIM5_CH4CFG 0
#endif

#ifdef CONFIG_STM32F7_TIM8_CH1OUT
#  define PWM_TIM8_CH1CFG GPIO_TIM8_CH1OUT
#else
#  define PWM_TIM8_CH1CFG 0
#endif
#ifdef CONFIG_STM32F7_TIM8_CH1NOUT
#  define PWM_TIM8_CH1NCFG GPIO_TIM8_CH1NOUT
#else
#  define PWM_TIM8_CH1NCFG 0
#endif
#ifdef CONFIG_STM32F7_TIM8_CH2OUT
#  define PWM_TIM8_CH2CFG GPIO_TIM8_CH2OUT
#else
#  define PWM_TIM8_CH2CFG 0
#endif
#ifdef CONFIG_STM32F7_TIM8_CH2NOUT
#  define PWM_TIM8_CH2NCFG GPIO_TIM8_CH2NOUT
#else
#  define PWM_TIM8_CH2NCFG 0
#endif
#ifdef CONFIG_STM32F7_TIM8_CH3OUT
#  define PWM_TIM8_CH3CFG GPIO_TIM8_CH3OUT
#else
#  define PWM_TIM8_CH3CFG 0
#endif
#ifdef CONFIG_STM32F7_TIM8_CH3NOUT
#  define PWM_TIM8_CH3NCFG GPIO_TIM8_CH3NOUT
#else
#  define PWM_TIM8_CH3NCFG 0
#endif
#ifdef CONFIG_STM32F7_TIM8_CH4OUT
#  define PWM_TIM8_CH4CFG GPIO_TIM8_CH4OUT
#else
#  define PWM_TIM8_CH4CFG 0
#endif

#ifdef CONFIG_STM32F7_TIM9_CH1OUT
#  define PWM_TIM9_CH1CFG GPIO_TIM9_CH1OUT
#else
#  define PWM_TIM9_CH1CFG 0
#endif

#ifdef CONFIG_STM32F7_TIM9_CH2OUT
#  define PWM_TIM9_CH2CFG GPIO_TIM9_CH2OUT
#else
#  define PWM_TIM9_CH2CFG 0
#endif

#ifdef CONFIG_STM32F7_TIM10_CH1OUT
#  define PWM_TIM10_CH1CFG GPIO_TIM10_CH1OUT
#else
#  define PWM_TIM10_CH1CFG 0
#endif

#ifdef CONFIG_STM32F7_TIM11_CH1OUT
#  define PWM_TIM11_CH1CFG GPIO_TIM11_CH1OUT
#else
#  define PWM_TIM11_CH1CFG 0
#endif

#ifdef CONFIG_STM32F7_TIM12_CH1OUT
#  define PWM_TIM12_CH1CFG GPIO_TIM12_CH1OUT
#else
#  define PWM_TIM12_CH1CFG 0
#endif
#ifdef CONFIG_STM32F7_TIM12_CH2OUT
#  define PWM_TIM12_CH2CFG GPIO_TIM12_CH2OUT
#else
#  define PWM_TIM12_CH2CFG 0
#endif

#ifdef CONFIG_STM32F7_TIM13_CH1OUT
#  define PWM_TIM13_CH1CFG GPIO_TIM13_CH1OUT
#else
#  define PWM_TIM13_CH1CFG 0
#endif

#ifdef CONFIG_STM32F7_TIM14_CH1OUT
#  define PWM_TIM14_CH1CFG GPIO_TIM14_CH1OUT
#else
#  define PWM_TIM14_CH1CFG 0
#endif

/* Complementary outputs support */

#if defined(CONFIG_STM32F7_TIM1_CH1NOUT) || defined(CONFIG_STM32F7_TIM1_CH2NOUT) || \
    defined(CONFIG_STM32F7_TIM1_CH3NOUT)
#  define HAVE_TIM1_COMPLEMENTARY
#endif
#if defined(CONFIG_STM32F7_TIM8_CH1NOUT) || defined(CONFIG_STM32F7_TIM8_CH2NOUT) || \
    defined(CONFIG_STM32F7_TIM8_CH3NOUT)
#  define HAVE_TIM8_COMPLEMENTARY
#endif
#if defined(HAVE_TIM1_COMPLEMENTARY) || defined(HAVE_TIM8_COMPLEMENTARY)
#  define HAVE_PWM_COMPLEMENTARY
#endif

/* Low-level ops helpers ****************************************************/

#ifdef CONFIG_STM32F7_PWM_LL_OPS

/* NOTE:
 * low-level ops accept pwm_lowerhalf_s as first argument, but llops access
 *       can be found in stm32_pwm_dev_s
 */

#define PWM_SETUP(dev)                                                             \
        (dev)->ops->setup((struct pwm_lowerhalf_s *)dev)
#define PWM_SHUTDOWN(dev)                                                          \
        (dev)->ops->shutdown((struct pwm_lowerhalf_s *)dev)
#define PWM_CCR_UPDATE(dev, index, ccr)                                            \
        (dev)->llops->ccr_update((struct pwm_lowerhalf_s *)dev, index, ccr)
#define PWM_MODE_UPDATE(dev, index, mode)                                          \
        (dev)->llops->mode_update((struct pwm_lowerhalf_s *)dev, index, mode)
#define PWM_CCR_GET(dev, index)                                                    \
        (dev)->llops->ccr_get((struct pwm_lowerhalf_s *)dev, index)
#define PWM_ARR_UPDATE(dev, arr)                                                   \
        (dev)->llops->arr_update((struct pwm_lowerhalf_s *)dev, arr)
#define PWM_ARR_GET(dev)                                                           \
        (dev)->llops->arr_get((struct pwm_lowerhalf_s *)dev)
#define PWM_RCR_UPDATE(dev, rcr)                                                   \
        (dev)->llops->rcr_update((struct pwm_lowerhalf_s *)dev, rcr)
#define PWM_RCR_GET(dev)                                                           \
        (dev)->llops->rcr_get((struct pwm_lowerhalf_s *)dev)
#ifdef CONFIG_STM32F7_PWM_TRGO
#  define PWM_TRGO_SET(dev, trgo)                                                  \
        (dev)->llops->trgo_set((struct pwm_lowerhalf_s *)dev, trgo)
#endif
#define PWM_OUTPUTS_ENABLE(dev, out, state)                                        \
        (dev)->llops->outputs_enable((struct pwm_lowerhalf_s *)dev, out, state)
#define PWM_SOFT_UPDATE(dev)                                                       \
        (dev)->llops->soft_update((struct pwm_lowerhalf_s *)dev)
#define PWM_CONFIGURE(dev)                                                         \
        (dev)->llops->configure((struct pwm_lowerhalf_s *)dev)
#define PWM_SOFT_BREAK(dev, state)                                                 \
        (dev)->llops->soft_break((struct pwm_lowerhalf_s *)dev, state)
#define PWM_FREQ_UPDATE(dev, freq)                                                 \
        (dev)->llops->freq_update((struct pwm_lowerhalf_s *)dev, freq)
#define PWM_TIM_ENABLE(dev, state)                                                 \
        (dev)->llops->tim_enable((struct pwm_lowerhalf_s *)dev, state)
#ifdef CONFIG_DEBUG_PWM_INFO
#  define PWM_DUMP_REGS(dev, msg)                                                  \
        (dev)->llops->dump_regs((struct pwm_lowerhalf_s *)dev, msg)
#else
#  define PWM_DUMP_REGS(dev, msg)
#endif
#define PWM_DT_UPDATE(dev, dt)                                                     \
        (dev)->llops->dt_update((struct pwm_lowerhalf_s *)dev, dt)
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Timer mode */

enum stm32_pwm_tim_mode_e
{
  STM32_TIMMODE_COUNTUP   = 0,
  STM32_TIMMODE_COUNTDOWN = 1,
  STM32_TIMMODE_CENTER1   = 2,
  STM32_TIMMODE_CENTER2   = 3,
  STM32_TIMMODE_CENTER3   = 4,
};

/* Timer output polarity */

enum stm32_pwm_pol_e
{
  STM32_POL_POS  = 0,
  STM32_POL_NEG  = 1,
};

/* Timer output IDLE state */

enum stm32_pwm_idle_e
{
  STM32_IDLE_INACTIVE = 0,
  STM32_IDLE_ACTIVE   = 1
};

/* PWM channel mode */

enum stm32_pwm_chanmode_e
{
  STM32_CHANMODE_FRZN        = 0,  /* CCRx matches has no effects on outputs */
  STM32_CHANMODE_CHACT       = 1,  /* OCxREF active on match */
  STM32_CHANMODE_CHINACT     = 2,  /* OCxREF inactive on match */
  STM32_CHANMODE_OCREFTOG    = 3,  /* OCxREF toggles when TIMy_CNT=TIMyCCRx */
  STM32_CHANMODE_OCREFLO     = 4,  /* OCxREF is forced low */
  STM32_CHANMODE_OCREFHI     = 5,  /* OCxREF is forced high */
  STM32_CHANMODE_PWM1        = 6,  /* PWM mode 1 */
  STM32_CHANMODE_PWM2        = 7,  /* PWM mode 2 */
  STM32_CHANMODE_COMBINED1   = 8,  /* Combined PWM mode 1 */
  STM32_CHANMODE_COMBINED2   = 9,  /* Combined PWM mode 2 */
  STM32_CHANMODE_ASYMMETRIC1 = 10, /* Asymmetric PWM mode 1 */
  STM32_CHANMODE_ASYMMETRIC2 = 11, /* Asymmetric PWM mode 2 */
};

/* PWM timer channel */

enum stm32_pwm_chan_e
{
  STM32_PWM_CHAN1  = 1,
  STM32_PWM_CHAN2  = 2,
  STM32_PWM_CHAN3  = 3,
  STM32_PWM_CHAN4  = 4,
  STM32_PWM_CHAN5  = 5,
  STM32_PWM_CHAN6  = 6,
};

/* PWM timer channel output */

enum stm32_pwm_output_e
{
  STM32_PWM_OUT1  = (1 << 0),
  STM32_PWM_OUT1N = (1 << 1),
  STM32_PWM_OUT2  = (1 << 2),
  STM32_PWM_OUT2N = (1 << 3),
  STM32_PWM_OUT3  = (1 << 4),
  STM32_PWM_OUT3N = (1 << 5),
  STM32_PWM_OUT4  = (1 << 6),

  /* 1 << 7 reserved - no complementary output for CH4 */

  /* Only available inside micro */

  STM32_PWM_OUT5  = (1 << 8),

  /* 1 << 9 reserved - no complementary output for CH5 */

  STM32_PWM_OUT6  = (1 << 10),

  /* 1 << 11 reserved - no complementary output for CH6 */
};

#ifdef CONFIG_STM32F7_PWM_LL_OPS

/* This structure provides the publicly visible representation of the
 * "lower-half" PWM driver structure.
 */

struct stm32_pwm_dev_s
{
  /* The first field of this state structure must be a pointer to the PWM
   * callback structure to be consistent with upper-half PWM driver.
   */

  const struct pwm_ops_s *ops;

  /* Publicly visible portion of the "lower-half" PWM driver structure */

  const struct stm32_pwm_ops_s *llops;

  /* Require cast-compatibility with private "lower-half" PWM structure */
};

/* Low-level operations for PWM */

struct pwm_lowerhalf_s;
struct stm32_pwm_ops_s
{
  /* Update CCR register */

  int (*ccr_update)(struct pwm_lowerhalf_s *dev,
                    uint8_t index, uint32_t ccr);

  /* Update PWM mode */

  int (*mode_update)(struct pwm_lowerhalf_s *dev,
                     uint8_t index, uint32_t mode);

  /* Get CCR register */

  uint32_t (*ccr_get)(struct pwm_lowerhalf_s *dev, uint8_t index);

  /* Update ARR register */

  int (*arr_update)(struct pwm_lowerhalf_s *dev, uint32_t arr);

  /* Get ARR register */

  uint32_t (*arr_get)(struct pwm_lowerhalf_s *dev);

  /* Update RCR register */

  int (*rcr_update)(struct pwm_lowerhalf_s *dev, uint16_t rcr);

  /* Get RCR register */

  uint16_t (*rcr_get)(struct pwm_lowerhalf_s *dev);

#ifdef CONFIG_STM32F7_PWM_TRGO
  /* Set TRGO/TRGO2 register */

  int (*trgo_set)(struct pwm_lowerhalf_s *dev, uint8_t trgo);
#endif

  /* Enable outputs */

  int (*outputs_enable)(struct pwm_lowerhalf_s *dev, uint16_t outputs,
                        bool state);

  /* Software update */

  int (*soft_update)(struct pwm_lowerhalf_s *dev);

  /* PWM configure */

  int (*configure)(struct pwm_lowerhalf_s *dev);

  /* Software break */

  int (*soft_break)(struct pwm_lowerhalf_s *dev, bool state);

  /* Update frequency */

  int (*freq_update)(struct pwm_lowerhalf_s *dev, uint32_t frequency);

  /* Enable timer counter */

  int (*tim_enable)(struct pwm_lowerhalf_s *dev, bool state);

#ifdef CONFIG_DEBUG_PWM_INFO
  /* Dump timer registers */

  void (*dump_regs)(struct pwm_lowerhalf_s *dev, const char *msg);
#endif

#ifdef HAVE_PWM_COMPLEMENTARY
  /* Deadtime update */

  int (*dt_update)(struct pwm_lowerhalf_s *dev, uint8_t dt);
#endif
};

#endif /* CONFIG_STM32F7_PWM_LL_OPS */

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
 * Name: stm32_pwminitialize
 *
 * Description:
 *   Initialize one timer for use with the upper_level PWM driver.
 *
 * Input Parameters:
 *   timer - A number identifying the timer use.  The number of valid timer
 *     IDs varies with the STM32 MCU and MCU family but is somewhere in
 *     the range of {1,..,17}.
 *
 * Returned Value:
 *   On success, a pointer to the STM32 lower half PWM driver is returned.
 *   NULL is returned on any failure.
 *
 ****************************************************************************/

struct pwm_lowerhalf_s *stm32_pwminitialize(int timer);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* CONFIG_STM32F7_PWM */
#endif /* __ARCH_ARM_SRC_STM32F7_STM32_PWM_H */

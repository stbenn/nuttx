/****************************************************************************
 * arch/arm/src/nuc1xx/nuc_config.h
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

#ifndef __ARCH_ARM_SRC_NUC1XX_NUC_CONFIG_H
#define __ARCH_ARM_SRC_NUC1XX_NUC_CONFIG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <arch/nuc1xx/chip.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Limit the number of enabled UARTs to those supported by the chip
 * architecture
 */

#if NUC_NUARTS < 3
#  undef CONFIG_NUC_UART2
#endif

#if NUC_NUARTS < 2
#  undef CONFIG_NUC_UART1
#endif

#if NUC_NUARTS < 1
#  undef CONFIG_NUC_UART0
#endif

/* Are any UARTs enabled? */

#undef HAVE_UART
#if defined(CONFIG_NUC_UART0) || defined(CONFIG_NUC_UART1) || defined(CONFIG_NUC_UART2)
#  define HAVE_UART 1
#endif

/* Make sure all features are disabled for disabled U[S]ARTs.
 *  This simplifies checking later.
 */

#ifndef CONFIG_NUC_UART0
#  undef CONFIG_UART0_SERIAL_CONSOLE
#  undef CONFIG_UART0_FLOWCONTROL
#  undef CONFIG_UART0_IRDAMODE
#  undef CONFIG_UART0_RS485MODE
#endif

#ifndef CONFIG_NUC_UART1
#  undef CONFIG_UART1_SERIAL_CONSOLE
#  undef CONFIG_UART1_FLOWCONTROL
#  undef CONFIG_UART1_IRDAMODE
#  undef CONFIG_UART1_RS485MODE
#endif

#undef CONFIG_UART2_FLOWCONTROL  /* UART2 does not support flow control */
#ifndef CONFIG_NUC_UART2
#  undef CONFIG_UART2_SERIAL_CONSOLE
#  undef CONFIG_UART2_IRDAMODE
#  undef CONFIG_UART2_RS485MODE
#endif

/* Is there a serial console? There should be at most one defined.
 * It could be on any UARTn, n=0,1,2 - OR - there might not be any serial
 * console at all.
 */

#if defined(CONFIG_UART0_SERIAL_CONSOLE)
#  undef CONFIG_UART1_SERIAL_CONSOLE
#  undef CONFIG_UART2_SERIAL_CONSOLE
#  define HAVE_SERIAL_CONSOLE 1
#elif defined(CONFIG_UART1_SERIAL_CONSOLE)
#  undef CONFIG_UART0_SERIAL_CONSOLE
#  undef CONFIG_UART2_SERIAL_CONSOLE
#  define HAVE_SERIAL_CONSOLE 1
#elif defined(CONFIG_UART2_SERIAL_CONSOLE)
#  undef CONFIG_UART0_SERIAL_CONSOLE
#  undef CONFIG_UART1_SERIAL_CONSOLE
#  define HAVE_SERIAL_CONSOLE 1
#else
#  undef CONFIG_UART0_SERIAL_CONSOLE
#  undef CONFIG_UART1_SERIAL_CONSOLE
#  undef CONFIG_UART2_SERIAL_CONSOLE
#  undef HAVE_SERIAL_CONSOLE
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_NUC1XX_NUC_CONFIG_H */

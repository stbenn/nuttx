/****************************************************************************
 * arch/arm/src/stm32h5/hardware/stm32_qspi.h
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

#ifndef __ARCH_ARM_SRC_STM32H5_HARDWARE_STM32_OCTOSPI_H
#define __ARCH_ARM_SRC_STM32H5_HARDWARE_STM32_OCTOSPI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <arch/stm32h5/chip.h>

#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* General Characteristics **************************************************/

#define STM32H5_OCTOSPI_MINBITS          8         /* Minimum word width */
#define STM32H5_OCTOSPI_MAXBITS          32        /* Maximum word width */

/* OCTOSPI register offsets ****************************************************/

#define STM32_OCTOSPI_CR_OFFSET       0x0000    /* Control Register */
#define STM32_OCTOSPI_DCR1_OFFSET     0x0008    /* Device Configuration Register 1 */
#define STM32_OCTOSPI_DCR2_OFFSET     0x000c    /* Device Configuration Register 2 */
#define STM32_OCTOSPI_DCR3_OFFSET     0x0010    /* Device Configuration Register 3 */
#define STM32_OCTOSPI_DCR4_OFFSET     0x0014    /* Device Configuration Register 4 */
#define STM32_OCTOSPI_SR_OFFSET       0x0020    /* Status Register */
#define STM32_OCTOSPI_FCR_OFFSET      0x0024    /* Flag Clear Register */
#define STM32_OCTOSPI_DLR_OFFSET      0x0040    /* Data Length Register */
#define STM32_OCTOSPI_AR_OFFSET       0x0048    /* Address Register */
#define STM32_OCTOSPI_DR_OFFSET       0x0050    /* Data Register */
#define STM32_OCTOSPI_PSMKR_OFFSET    0x0080    /* Polling Status mask Register */
#define STM32_OCTOSPI_PSMAR_OFFSET    0x0088    /* Polling Status match Register */
#define STM32_OCTOSPI_PIR_OFFSET      0x0090    /* Polling Interval Register */
#define STM32_OCTOSPI_CCR_OFFSET      0x0100    /* Communication Configuration Register */
#define STM32_OCTOSPI_TCR_OFFSET      0x0108    /* Timing Configuration Register */
#define STM32_OCTOSPI_IR_OFFSET       0x0110    /* Instruction Register */
#define STM32_OCTOSPI_ABR_OFFSET      0x0120    /* Alternate Bytes Register */
#define STM32_OCTOSPI_LPTR_OFFSET     0x0130    /* Low-Power Timeout Register */
#define STM32_OCTOSPI_WPCCR_OFFSET    0x0140    /* Wrap Communication Configuration Register */
#define STM32_OCTOSPI_WPTCR_OFFSET    0x0148    /* Wrap Timing Configuration Register */
#define STM32_OCTOSPI_WPIR_OFFSET     0x0150    /* Wrap Instruction Register */
#define STM32_OCTOSPI_WPABR_OFFSET    0x0160    /* Wrap Alternate Bytes Register */
#define STM32_OCTOSPI_WCCR_OFFSET     0x0180    /* Write Communication Configuration Register */
#define STM32_OCTOSPI_WTCR_OFFSET     0x0188    /* Write Configuration Register */
#define STM32_OCTOSPI_WIR_OFFSET      0x0190    /* Write Instruction Register */
#define STM32_OCTOSPI_WABR_OFFSET     0x01a0    /* Write Alternate Bytes Register */
#define STM32_OCTOSPI_HLCR_OFFSET     0x0200    /* HyperBus Latency Configuration Register */

/* OCTOSPI register addresses **************************************************/

#define STM32_OCTOSPI_CR     (STM32_OCTOSPI_BASE+STM32_OCTOSPI_CR_OFFSET)     /* Control Register */
#define STM32_OCTOSPI_DCR1   (STM32_OCTOSPI_BASE+STM32_OCTOSPI_DCR1_OFFSET)   /* Device Configuration Register 1 */
#define STM32_OCTOSPI_DCR2   (STM32_OCTOSPI_BASE+STM32_OCTOSPI_DCR2_OFFSET)   /* Device Configuration Register 2 */
#define STM32_OCTOSPI_DCR3   (STM32_OCTOSPI_BASE+STM32_OCTOSPI_DCR3_OFFSET)   /* Device Configuration Register 3 */
#define STM32_OCTOSPI_DCR4   (STM32_OCTOSPI_BASE+STM32_OCTOSPI_DCR4_OFFSET)   /* Device Configuration Register 4 */
#define STM32_OCTOSPI_SR     (STM32_OCTOSPI_BASE+STM32_OCTOSPI_SR_OFFSET)     /* Status Register */
#define STM32_OCTOSPI_FCR    (STM32_OCTOSPI_BASE+STM32_OCTOSPI_FCR_OFFSET)    /* Flag Clear Register */
#define STM32_OCTOSPI_DLR    (STM32_OCTOSPI_BASE+STM32_OCTOSPI_DLR_OFFSET)    /* Data Length Register */
#define STM32_OCTOSPI_AR     (STM32_OCTOSPI_BASE+STM32_OCTOSPI_AR_OFFSET)     /* Address Register */
#define STM32_OCTOSPI_DR     (STM32_OCTOSPI_BASE+STM32_OCTOSPI_DR_OFFSET)     /* Data Register */
#define STM32_OCTOSPI_PSMKR  (STM32_OCTOSPI_BASE+STM32_OCTOSPI_PSMKR_OFFSET)  /* Polling Status mask Register */
#define STM32_OCTOSPI_PSMAR  (STM32_OCTOSPI_BASE+STM32_OCTOSPI_PSMAR_OFFSET)  /* Polling Status match Register */
#define STM32_OCTOSPI_PIR    (STM32_OCTOSPI_BASE+STM32_OCTOSPI_PIR_OFFSET)    /* Polling Interval Register */
#define STM32_OCTOSPI_CCR    (STM32_OCTOSPI_BASE+STM32_OCTOSPI_CCR_OFFSET)    /* Communication Configuration Register */
#define STM32_OCTOSPI_TCR    (STM32_OCTOSPI_BASE+STM32_OCTOSPI_TCR_OFFSET)    /* Timing Configuration Register */
#define STM32_OCTOSPI_IR     (STM32_OCTOSPI_BASE+STM32_OCTOSPI_IR_OFFSET)     /* Instruction Register */
#define STM32_OCTOSPI_ABR    (STM32_OCTOSPI_BASE+STM32_OCTOSPI_ABR_OFFSET)    /* Alternate Bytes Register */
#define STM32_OCTOSPI_LPTR   (STM32_OCTOSPI_BASE+STM32_OCTOSPI_LPTR_OFFSET)   /* Low-Power Timeout Register */
#define STM32_OCTOSPI_WPCCR  (STM32_OCTOSPI_BASE+STM32_OCTOSPI_WPCCR_OFFSET)  /* Wrap Communication Configuration Register */
#define STM32_OCTOSPI_WPTCR  (STM32_OCTOSPI_BASE+STM32_OCTOSPI_WPTCR_OFFSET)  /* Wrap Timing Configuration Register */
#define STM32_OCTOSPI_WPIR   (STM32_OCTOSPI_BASE+STM32_OCTOSPI_WPIR_OFFSET)   /* Wrap Instruction Register */
#define STM32_OCTOSPI_WPABR  (STM32_OCTOSPI_BASE+STM32_OCTOSPI_WPABR_OFFSET)  /* Wrap Alternate Bytes Register */
#define STM32_OCTOSPI_WCCR   (STM32_OCTOSPI_BASE+STM32_OCTOSPI_WCCR_OFFSET)   /* Write Communication Configuration Register */
#define STM32_OCTOSPI_WTCR   (STM32_OCTOSPI_BASE+STM32_OCTOSPI_WTCR_OFFSET)   /* Write Configuration Register */
#define STM32_OCTOSPI_WIR    (STM32_OCTOSPI_BASE+STM32_OCTOSPI_WIR_OFFSET)    /* Write Instruction Register */
#define STM32_OCTOSPI_WABR   (STM32_OCTOSPI_BASE+STM32_OCTOSPI_WABR_OFFSET)   /* Write Alternate Bytes Register */
#define STM32_OCTOSPI_HLCR   (STM32_OCTOSPI_BASE+STM32_OCTOSPI_HLCR_OFFSET)   /* HyperBus Latency Configuration Register */

/* OCTOSPI register bit definitions ********************************************/

/* Control Register */

#define OCTOSPI_CR_EN                 (1 << 0)   /* Bit 0:  OCTOSPI Enable */
#define OCTOSPI_CR_ABORT              (1 << 1)   /* Bit 1:  Abort request */
#define OCTOSPI_CR_TCEN               (1 << 3)   /* Bit 3:  Timeout counter enable */
#define OCTOSPI_CR_SSHIFT             (1 << 4)   /* Bit 4:  Sample shift */
#define OCTOSPI_CR_DFM                (1 << 6)   /* Bit 6:  DFM: Dual-flash mode */
#define OCTOSPI_CR_FSEL               (1 << 7)   /* Bit 7:  FSEL: Flash memory selection */
#define OCTOSPI_CR_FTHRES_SHIFT       (8)        /* Bits 8-11: FIFO threshold level */
#define OCTOSPI_CR_FTHRES_MASK        (0x0f << OCTOSPI_CR_FTHRES_SHIFT)
#define OCTOSPI_CR_TEIE               (1 << 16)  /* Bit 16:  Transfer error interrupt enable */
#define OCTOSPI_CR_TCIE               (1 << 17)  /* Bit 17:  Transfer complete interrupt enable */
#define OCTOSPI_CR_FTIE               (1 << 18)  /* Bit 18:  FIFO threshold interrupt enable */
#define OCTOSPI_CR_SMIE               (1 << 19)  /* Bit 19:  Status match interrupt enable */
#define OCTOSPI_CR_TOIE               (1 << 20)  /* Bit 20:  TimeOut interrupt enable */
#define OCTOSPI_CR_APMS               (1 << 22)  /* Bit 22:  Automatic poll mode stop */
#define OCTOSPI_CR_PMM                (1 << 23)  /* Bit 23:  Polling match mode */

#define OCTOSPI_CR_FMODE_SHIFT       (28)        /* Bits 28-29: Functional mode */
#define OCTOSPI_CR_FMODE_MASK        (0x3 << OCTOSPI_CR_FMODE_SHIFT)
#  define OCTOSPI_CR_FMODE(n)        ((uint32_t)(n) << OCTOSPI_CR_FMODE_SHIFT)

#define CR_FMODE_INDWR     0   /* Indirect write mode */
#define CR_FMODE_INDRD     1   /* Indirect read mode */
#define CR_FMODE_AUTOPOLL  2   /* Automatic polling mode */
#define CR_FMODE_MEMMAP    3   /* Memory-mapped mode */

/* Device Configuration Register */

#define OCTOSPI_DCR_CKMODE            (1 << 0)   /* Bit 0:  Mode 0 / mode 3 */
#define OCTOSPI_DCR_CSHT_SHIFT        (8)        /* Bits 8-10: Chip select high time */
#define OCTOSPI_DCR_CSHT_MASK         (0x7 << OCTOSPI_DCR_CSHT_SHIFT)
#define OCTOSPI_DCR_FSIZE_SHIFT       (16)       /* Bits 16-20: Flash memory size */
#define OCTOSPI_DCR_FSIZE_MASK        (0x1f << OCTOSPI_DCR_FSIZE_SHIFT)

#define OCTOSPI_DCR2_PRESCALER_SHIFT    (24)       /* Bits 24-31: Clock prescaler */
#define OCTOSPI_DCR2_PRESCALER_MASK     (0xff << OCTOSPI_CR_PRESCALER_SHIFT)

/* Status Register */

#define OCTOSPI_SR_TEF                (1 << 0)   /* Bit 0:  Transfer error flag */
#define OCTOSPI_SR_TCF                (1 << 1)   /* Bit 1:  Transfer complete flag */
#define OCTOSPI_SR_FTF                (1 << 2)   /* Bit 2:  FIFO threshold flag */
#define OCTOSPI_SR_SMF                (1 << 3)   /* Bit 3:  Status match flag */
#define OCTOSPI_SR_TOF                (1 << 4)   /* Bit 4:  Timeout flag */
#define OCTOSPI_SR_BUSY               (1 << 5)   /* Bit 5:  Busy */
#define OCTOSPI_SR_FLEVEL_SHIFT       (8)        /* Bits 8-12: FIFO threshold level */
#define OCTOSPI_SR_FLEVEL_MASK        (0x1f << OCTOSPI_SR_FLEVEL_SHIFT)

/* Flag Clear Register */

#define OCTOSPI_FCR_CTEF              (1 << 0)   /* Bit 0:  Clear Transfer error flag */
#define OCTOSPI_FCR_CTCF              (1 << 1)   /* Bit 1:  Clear Transfer complete flag */
#define OCTOSPI_FCR_CSMF              (1 << 3)   /* Bit 3:  Clear Status match flag */
#define OCTOSPI_FCR_CTOF              (1 << 4)   /* Bit 4:  Clear Timeout flag */

/* Data Length Register */

/* Communication Configuration Register */

#define CCR_IMODE_NONE      0   /* No instruction */
#define CCR_IMODE_SINGLE    1   /* Instruction on a single line */
#define CCR_IMODE_DUAL      2   /* Instruction on two lines */
#define CCR_IMODE_QUAD      3   /* Instruction on four lines */

#define CCR_ADMODE_NONE     0   /* No address */
#define CCR_ADMODE_SINGLE   1   /* Address on a single line */
#define CCR_ADMODE_DUAL     2   /* Address on two lines */
#define CCR_ADMODE_QUAD     3   /* Address on four lines */

#define CCR_ADSIZE_8        0   /* 8-bit address */
#define CCR_ADSIZE_16       1   /* 16-bit address */
#define CCR_ADSIZE_24       2   /* 24-bit address */
#define CCR_ADSIZE_32       3   /* 32-bit address */

#define CCR_ABMODE_NONE     0   /* No alternate bytes */
#define CCR_ABMODE_SINGLE   1   /* Alternate bytes on a single line */
#define CCR_ABMODE_DUAL     2   /* Alternate bytes on two lines */
#define CCR_ABMODE_QUAD     3   /* Alternate bytes on four lines */

#define CCR_ABSIZE_8        0   /* 8-bit alternate byte */
#define CCR_ABSIZE_16       1   /* 16-bit alternate bytes */
#define CCR_ABSIZE_24       2   /* 24-bit alternate bytes */
#define CCR_ABSIZE_32       3   /* 32-bit alternate bytes */

#define CCR_DMODE_NONE      0   /* No data */
#define CCR_DMODE_SINGLE    1   /* Data on a single line */
#define CCR_DMODE_DUAL      2   /* Data on two lines */
#define CCR_DMODE_QUAD      3   /* Data on four lines */

#define OCTOSPI_CCR_INSTRUCTION_SHIFT (0)        /* Bits 0-7: Instruction */
#define OCTOSPI_CCR_INSTRUCTION_MASK  (0xff << OCTOSPI_CCR_INSTRUCTION_SHIFT)
#  define OCTOSPI_CCR_INST(n)         ((uint32_t)(n) << OCTOSPI_CCR_INSTRUCTION_SHIFT)
#define OCTOSPI_CCR_IMODE_SHIFT       (8)        /* Bits 8-9: Instruction mode */
#define OCTOSPI_CCR_IMODE_MASK        (0x3 << OCTOSPI_CCR_IMODE_SHIFT)
#  define OCTOSPI_CCR_IMODE(n)        ((uint32_t)(n) << OCTOSPI_CCR_IMODE_SHIFT)
#define OCTOSPI_CCR_ADMODE_SHIFT      (10)        /* Bits 10-11: Address mode */
#define OCTOSPI_CCR_ADMODE_MASK       (0x3 << OCTOSPI_CCR_ADMODE_SHIFT)
#  define OCTOSPI_CCR_ADMODE(n)       ((uint32_t)(n) << OCTOSPI_CCR_ADMODE_SHIFT)
#define OCTOSPI_CCR_ADSIZE_SHIFT      (12)        /* Bits 12-13: Address size */
#define OCTOSPI_CCR_ADSIZE_MASK       (0x3 << OCTOSPI_CCR_ADSIZE_SHIFT)
#  define OCTOSPI_CCR_ADSIZE(n)       ((uint32_t)(n) << OCTOSPI_CCR_ADSIZE_SHIFT)
#define OCTOSPI_CCR_ABMODE_SHIFT      (14)        /* Bits 14-15: Alternate bytes mode */
#define OCTOSPI_CCR_ABMODE_MASK       (0x3 << OCTOSPI_CCR_ABMODE_SHIFT)
#  define OCTOSPI_CCR_ABMODE(n)       ((uint32_t)(n) << OCTOSPI_CCR_ABMODE_SHIFT)
#define OCTOSPI_CCR_ABSIZE_SHIFT      (16)        /* Bits 16-17: Alternate bytes size */
#define OCTOSPI_CCR_ABSIZE_MASK       (0x3 << OCTOSPI_CCR_ABSIZE_SHIFT)
#  define OCTOSPI_CCR_ABSIZE(n)       ((uint32_t)(n) << OCTOSPI_CCR_ABSIZE_SHIFT)
#define OCTOSPI_CCR_DCYC_SHIFT        (18)        /* Bits 18-23: Number of dummy cycles */
#define OCTOSPI_CCR_DCYC_MASK         (0x1f << OCTOSPI_CCR_DCYC_SHIFT)
#  define OCTOSPI_CCR_DCYC(n)         ((uint32_t)(n) << OCTOSPI_CCR_DCYC_SHIFT)
#define OCTOSPI_CCR_DMODE_SHIFT       (24)        /* Bits 24-25: Data mode */
#define OCTOSPI_CCR_DMODE_MASK        (0x3 << OCTOSPI_CCR_DMODE_SHIFT)
#  define OCTOSPI_CCR_DMODE(n)        ((uint32_t)(n) << OCTOSPI_CCR_DMODE_SHIFT)
#define OCTOSPI_CCR_SIOO              (1 << 28)   /* Bit 28:  Send instruction only once mode */
#define OCTOSPI_CCR_FRCM              (1 << 29)   /* Bit 28:  Enters Free running clock mode */
#define OCTOSPI_CCR_DDRM              (1 << 31)   /* Bit 31:  Double data rate mode */

/* Address Register */

/* Alternate Bytes Register */

/* Data Register */

/* Polling Status mask Register */

/* Polling Status match Register */

/* Polling Interval Register */

#define OCTOSPI_PIR_INTERVAL_SHIFT    (0)        /* Bits 0-15: Polling interval */
#define OCTOSPI_PIR_INTERVAL_MASK     (0xFFff << OCTOSPI_PIR_INTERVAL_SHIFT)

/* Low-Power Timeout Register */

#define OCTOSPI_LPTR_TIMEOUT_SHIFT    (0)        /* Bits 0-15: Timeout period */
#define OCTOSPI_LPTR_TIMEOUT_MASK     (0xFFff << OCTOSPI_LPTR_TIMEOUT_SHIFT)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_STM32H5_HARDWARE_STM32_OCTOSPI_H */

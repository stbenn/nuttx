/****************************************************************************
 * arch/arm/src/stm32h5/stm32h563xx_flash.c
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

/* Provides standard flash access functions, to be used by the flash mtd
 * driver.  The interface is defined in the include/nuttx/progmem.h
 *
 * Requirements during write/erase operations on FLASH:
 *  - HSI must be ON.
 *  - Low Power Modes are not permitted during write/erase
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/arch.h>
#include <nuttx/mutex.h>
#include <arch/barriers.h>

#include <stdbool.h>
#include <assert.h>
#include <errno.h>

#include "hardware/stm32_flash.h"
#include "hardware/stm32_memorymap.h"
#include "arm_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define _K(x) ((x)*1024)
#define FLASH_SECTOR_SIZE _K(8)
#define FLASH_PAGE_SIZE     16

/**
 * NOTES:
 *  - This file will only be valid for H56xxx and H57xxx. May be valid for 
 *    H53xxx and H52xxx but that is not verified.
 *  - STM Documentation uses "Flash Sectors" instead of "Blocks". They refer
 *    to the same thing in this file.
 *  - This file ASSUMES DUAL BANK flash memory. 
 */


#if !defined(CONFIG_STM32H5_FLASH_OVERRIDE_DEFAULT) && \
    !defined(CONFIG_STM32H5_FLASH_OVERRIDE_B) && \
    !defined(CONFIG_STM32H5_FLASH_OVERRIDE_C) && \
    !defined(CONFIG_STM32H5_FLASH_OVERRIDE_E) && \
    !defined(CONFIG_STM32H5_FLASH_OVERRIDE_G) && \
    !defined(CONFIG_STM32H5_FLASH_OVERRIDE_I) && \
    !defined(CONFIG_STM32H5_FLASH_CONFIG_B) && \
    !defined(CONFIG_STM32H5_FLASH_CONFIG_C) && \
    !defined(CONFIG_STM32H5_FLASH_CONFIG_E) && \
    !defined(CONFIG_STM32H5_FLASH_CONFIG_G) && \
    !defined(CONFIG_STM32H5_FLASH_CONFIG_I)
#  define CONFIG_STM32H5_FLASH_OVERRIDE_E
#  warning "Flash size not defined defaulting to 512KiB (E)"
#endif

/* Override of the Flash has been chosen */

// TODO: This needs to be updated to take care of all possibilities
#if !defined(CONFIG_STM32H5_FLASH_OVERRIDE_DEFAULT)
#  undef CONFIG_STM32H5_FLASH_CONFIG_C
#  undef CONFIG_STM32H5_FLASH_CONFIG_E
#  if defined(CONFIG_STM32H5_FLASH_OVERRIDE_C)
#    define CONFIG_STM32H5_FLASH_CONFIG_C
#  elif defined(CONFIG_STM32H5_FLASH_OVERRIDE_E)
#    define CONFIG_STM32H5_FLASH_CONFIG_E
#  endif
#endif

#if defined(CONFIG_STM32H5_FLASH_CONFIG_I)
#  define H5_FLASH_BANK_NBLOCKS    128
#elif defined(CONFIG_STM32H5_FLASH_CONFIG_G)
#  define H5_FLASH_BANK_NBLOCKS    64
#elif defined(CONFIG_STM32H5_FLASH_CONFIG_E)
#  define H5_FLASH_BANK_NBLOCKS    32
#elif defined(CONFIG_STM32H5_FLASH_CONFIG_C)
#  define H5_FLASH_BANK_NBLOCKS    16
#elif defined(CONFIG_STM32H5_FLASH_CONFIG_B)
#  define H5_FLASH_BANK_NBLOCKS    8
#else
#  warning "No valid STM32H5_FLASH_CONFIG_x defined."
#endif

#define H5_FLASH_BANKSIZE   (FLASH_SECTOR_SIZE * H5_FLASH_BANK_NBLOCKS)
#define H5_FLASH_NBLOCKS    (2 * H5_FLASH_BANK_NBLOCKS)
#define H5_FLASH_TOTALSIZE  (2 * H5_FLASH_BANKSIZE)
#define H5_FLASH_NPAGES     (H5_FLASH_TOTALSIZE / FLASH_PAGE_SIZE)

/* Define the valid configuration  */

#define FLASH_KEY1      0x45670123
#define FLASH_KEY2      0xCDEF89AB
#define FLASH_OPTKEY1   0x08192A3B
#define FLASH_OPTKEY2   0x4C5D6E7F
#define FLASH_OBKKEY1   0x192A083B
#define FLASH_OBKKEY2   0x5E7F4C5D

#define FLASH_ERASEDVALUE     0xffu
#define FLASH_TIMEOUT_VALUE   5000000   /* 5s */
/****************************************************************************
 * Private Types
 ****************************************************************************/

struct stm32h5_flash_priv_s
{
  uint32_t base;    /* FLASH base address */
  uint32_t stblock; /* The first block number */
  uint32_t stpage;  /* The first page number */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct stm32h5_flash_priv_s flash_bank1_priv =
{
  .base    = STM32_FLASH_BANK1,
  .stblock = 0,
  .stpage  = 0
};
static struct stm32h5_flash_priv_s flash_bank2_priv =
{
  .base    = STM32_FLASH_BANK2,
  .stblock = (H5_FLASH_NBLOCKS / 2),
  .stpage  = (H5_FLASH_NPAGES / 2),
};

static mutex_t g_lock = NXMUTEX_INITIALIZER;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline
struct stm32h5_flash_priv_s * flash_bank(size_t address)
{
  struct stm32h5_flash_priv_s *priv = NULL;

  if (address >= flash_bank1_priv.base &&
      address < flash_bank1_priv.base + H5_FLASH_BANKSIZE)
    {
      priv = &flash_bank1_priv;
    }
  else if (address >= flash_bank2_priv.base &&
           address < flash_bank2_priv.base + H5_FLASH_BANKSIZE)
    {
      priv = &flash_bank2_priv;
    }
  
  return priv;
}

/****************************************************************************
 * Name: flash_unlock_nscr
 * 
 * Description:
 *    Unlock the non-secure control register.
 ****************************************************************************/

static void flash_unlock_nscr(void)
{
  while (getreg32(STM32_FLASH_NSSR) & FLASH_NSSR_BSY)
    {
    }
  
  if (getreg32(STM32_FLASH_NSCR) & FLASH_NSCR_LOCK)
    {
      putreg32(FLASH_KEY1, STM32_FLASH_NSKEYR);
      putreg32(FLASH_KEY2, STM32_FLASH_NSKEYR);
    }
}

/****************************************************************************
 * Name: flash_lock_nscr
 * 
 * Description:
 *    Lock the non-secure control register.
 ****************************************************************************/

static void flash_lock_nscr(void)
{
  modifyreg32(STM32_FLASH_NSCR, 0, FLASH_NSCR_LOCK);
}

/****************************************************************************
 * Name: stm32h5_israngeerased
 *
 * Description:
 *   Returns count of non-erased words
 *
 ****************************************************************************/

static int stm32h5_israngeerased(size_t startaddress, size_t size)
{
  #warning "stm32h5_israngerased() not implemented yet"
}

/****************************************************************************
 * Name: flash_wait_for_operation()
 *
 * Description:
 *   Wait for last write/erase operation to finish
 *   Return error in case of timeout
 *
 * Returned Value:
 *     Zero or error value
 *
 *     ETIME:  Timeout while waiting for previous write/erase operation to
 *             complete
 *
 ****************************************************************************/

static int flash_wait_for_operation(void)
{
  int i;
  bool timeout = true;

  UP_DSB();

  for (i = 0; i < FLASH_TIMEOUT_VALUE; i++)
    {
      if (!(getreg32(STM32_FLASH_NSSR) &
          (FLASH_NSSR_BSY | FLASH_NSSR_DBNE | FLASH_NSSR_WBNE)))
        {
          timeout = false;
          break;
        }
      up_udelay(1);
    }

  if (timeout)
    {
      return -EBUSY;
    }

  return 0;
}

/****************************************************************************
 * Name: flash_unlock_opt
 *
 * Description:
 *   Unlock the flash option bytes
 *
 ****************************************************************************/

static void flash_unlock_opt(void)
{
  while (getreg32(STM32_FLASH_NSSR) & FLASH_NSSR_BSY)
    {
    }

  if (getreg32(STM32_FLASH_OPTCR) & FLASH_OPTCR_OPTLOCK)
    {
      putreg32(FLASH_OPTKEY1, STM32_FLASH_OPTKEYR);
      putreg32(FLASH_OPTKEY2, STM32_FLASH_OPTKEYR);
    }
}

/****************************************************************************
 * Name: flash_lock_opt
 *
 * Description:
 *   Lock the flash option bytes
 *
 ****************************************************************************/

static void flash_lock_opt(void)
{
  modifyreg32(STM32_FLASH_OPTCR, 0, FLASH_OPTCR_OPTLOCK);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32h5_flash_unlock
 *
 * Description:
 *   Unlock flash control
 ****************************************************************************/

void stm32h5_flash_unlock(void)
{
  nxmutex_lock(&g_lock);
  flash_unlock_nscr();
  nxmutex_unlock(&g_lock);
}

/****************************************************************************
 * Name: stm32h5_flash_lock
 *
 * Description:
 *   Lock flash control
 ****************************************************************************/

void stm32h5_flash_lock(void)
{
  nxmutex_lock(&g_lock);
  flash_lock_nscr();
  nxmutex_unlock(&g_lock);
}

/****************************************************************************
 * Name: stm32h5_flash_getopt
 *
 * Description:
 *   Read the current flash option bytes from FLASH_OPTSR_CUR and
 *   FLASH_OPTSR2_CUR registers.
 * 
 * Input Parameters:
 *   opt1 - result from FLASH_OPTSR_CUR
 *   opt2 - result from FLASH_OPTSR2_CUR
 *
 ****************************************************************************/

void stm32h5_flash_getopt(uint32_t *opt1, uint32_t *opt2)
{
  *opt1 = getreg32(STM32_FLASH_OPTSR_CUR);
  *opt2 = getreg32(STM32_FLASH_OPTSR2_CUR);
}

/****************************************************************************
 * Name: stm32h5_flash_optmodify
 *
 * Description:
 *   Modifies the current flash option bytes, given bits to set and clear.
 * 
 * Input Parameters:
 *   clear1 - clear bits for FLASH_OPTSR
 *   set1   - set bits for FLASH_OPTSR
 *   clear2 - clear bits for FLASH_OPTSR2
 *   set2   - set bits for FLASH_OPTSR2
 * 
 * Returned Value:
 *   Zero or error value
 * 
 *     -EBUSY: Timeout occurred waiting for previous FLASH operation to occur,
 *            or there was data in the flash data buffer.
 *   
 ****************************************************************************/

int stm32h5_flash_optmodify(uint32_t clear1, uint32_t set1,
                             uint32_t clear2, uint32_t set2)
{
  int ret;
  uint32_t reg;

  ret = flash_wait_for_operation();
  if (ret != 0)
    {
      -EBUSY;
    }
    
  reg = getreg32(STM32_FLASH_NSSR);
  if (reg & FLASH_NSSR_DBNE)
    {
      -EBUSY;
    }

  flash_unlock_opt();

  modifyreg32(STM32_FLASH_OPTSR_PRG, clear1, set1);
  modifyreg32(STM32_FLASH_OPTSR2_PRG, clear2, set2);

  modifyreg32(STM32_FLASH_OPTCR, 0, FLASH_OPTCR_OPTSTRT);

  while (getreg32(STM32_FLASH_NSSR) & FLASH_NSSR_BSY)
    {
    }

}

/****************************************************************************
 * Name: stm32h5_flash_swapbanks
 *
 * Description:
 *   Swaps banks 1 and 2 in the processor's memory map.  Takes effect
 *   the next time the system is reset.
 *
 * Returned Value:
 *      Zero or error value
 * 
 *      -ETIMEDOUT: Timeout occurred waiting for previous operation to occur.
 ****************************************************************************/

int stm32h5_flash_swapbanks(void)
{
  uint32_t reg;

  if (flash_wait_for_operation())
    {
      return -ETIMEDOUT;
    }
  
  flash_unlock_opt();

  reg = getreg32(STM32_FLASH_OPTSR_PRG);
  reg ^= FLASH_OPTSR_PRG_SWAP_BANK;
  putreg32(reg, STM32_FLASH_OPTSR_PRG);

  modifyreg32(STM32_FLASH_OPTCR, 0, FLASH_OPTCR_OPTSTRT);

  /* Once completed, FLASH_OPTSR_CUR contains the expected SWAP_BANK value */

  while ((getreg32(STM32_FLASH_OPTSR_CUR) >> 31) != (reg >> 31))
    {
    }

  return 0;
}

#ifdef CONFIG_ARCH_HAVE_PROGMEM

/* up_progmem_x functions defined in nuttx/include/nuttx/progmem.h
 *
 * Notes on Implementation:
 *   - The driver implementations DO NOT enforce memory address boundaries.
 *     For processors with less than 2MB flash, the user is responsible for
 *     not writing to memory between banks.
 *     
 */

size_t up_progmem_pagesize(size_t page)
{
  return FLASH_PAGE_SIZE;
}

ssize_t up_progmem_getpage(size_t addr)
{
  struct stm32h5_flash_priv_s *priv;

  priv = flash_bank(addr);

  if (priv == NULL)
    {
      return -EFAULT;
    }

  return priv->stpage + ((addr - priv->base) / FLASH_PAGE_SIZE);
}

size_t up_progmem_getaddress(size_t page)
{

}

size_t up_progmem_neraseblocks(void)
{
  return H5_FLASH_NBLOCKS;
}

bool up_progmem_isuniform(void)
{
  return true;
}

ssize_t up_progmem_ispageerased(size_t page)
{

}

size_t up_progmem_erasesize(size_t block)
{
  return FLASH_SECTOR_SIZE;
}

ssize_t up_progmem_eraseblock(size_t block)
{
}

ssize_t up_progmem_write(size_t addr, const void *buf, size_t count)
{
}

uint8_t up_progmem_erasestate(void)
{
  return FLASH_ERASEDVALUE;
}

#endif /* CONFIG_ARCH_HAVE_PROGMEM */

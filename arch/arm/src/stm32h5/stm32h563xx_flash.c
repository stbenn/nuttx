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
#include "arm_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct stm32h5_flash_priv_s
{
  mutex_t  lock;    /* Bank exclusive */
  uint32_t ifbase;  /* FLASHIF interface base address */
  uint32_t base;    /* FLASH base address */
  uint32_t stblock; /* The first Block Number */
  uint32_t stpage;  /* The first Page Number */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/


/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32h5_flash_getreg32
 *
 * Description:
 *   Get a 32-bit register value by offset
 *
 ****************************************************************************/

static inline uint32_t stm32h5_flash_getreg32(struct stm32h5_flash_priv_s
                                            *priv, uint32_t offset)
{
  return getreg32(priv->ifbase + offset);
}

/****************************************************************************
 * Name: stm32h5_flash_putreg32
 *
 * Description:
 *  Put a 32-bit register value by offset
 *
 ****************************************************************************/

static inline void stm32h5_flash_putreg32(struct stm32h5_flash_priv_s
                                          *priv, uint32_t offset,
                                          uint32_t value)
{
  putreg32(value, priv->ifbase + offset);
}

/****************************************************************************
 * Name: stm32h5_flash_modifyreg32
 *
 * Description:
 *   Modify a 32-bit register value by offset
 *
 ****************************************************************************/

static inline void stm32h5_flash_modifyreg32(struct stm32h5_flash_priv_s
                                             *priv, uint32_t offset,
                                             uint32_t clearbits,
                                             uint32_t setbits)
{
  modifyreg32(priv->ifbase + offset, clearbits, setbits);
}

/****************************************************************************
 * Name: stm32h5_unlock_flash
 *
 * Description:
 *   Unlock the Bank
 *
 ****************************************************************************/

static void stm32h5_unlock_flash(struct stm32h5_flash_priv_s *priv)
{
  #warning "stm32h5_unlock_flash() not implemented yet."
}

/****************************************************************************
 * Name: stm32h5_lock_flash
 *
 * Description:
 *   Lock the Bank
 *
 ****************************************************************************/

static void stm32h5_lock_flash(struct stm32h5_flash_priv_s *priv)
{
  #warning "stm32h5_lock_flash() not implemented yet"
}

/****************************************************************************
 * Name: stm32h5_flash_size
 *
 * Description:
 *   Returns the size in bytes of FLASH
 *
 ****************************************************************************/

static inline uint32_t stm32h5_flash_size(
    struct stm32h5_flash_priv_s *priv)
{
  #warning "stm32h5_flash_size() not implemented yet"
}

/****************************************************************************
 * Name: stm32h5_flash_bank
 *
 * Description:
 *   Returns the priv pointer to the correct bank
 *
 ****************************************************************************/

static inline
struct stm32h5_flash_priv_s * stm32h5_flash_bank(size_t address)
{
  #warning "stm32h5_flash_bank() not implemented yet"
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
 * Name: stm32h5_wait_for_last_operation()
 *
 * Description:
 *   Wait for last write/erase operation to finish
 *   Return error in case of timeout
 *
 * Input Parameters:
 *   priv  - Flash bank based config
 *
 * Returned Value:
 *     Zero or error value
 *
 *     ETIME:  Timeout while waiting for previous write/erase operation to
 *             complete
 *
 ****************************************************************************/

static int stm32h5_wait_for_last_operation(struct stm32h5_flash_priv_s
                                           *priv)
{
  #warning "stm32h5_wait_for_last_operation() not implemented yet"
}

/****************************************************************************
 * Name: stm32h5_unlock_flashopt
 *
 * Description:
 *   Unlock the flash option bytes
 *   Returns true if the flash was locked before, false otherwise
 *
 ****************************************************************************/

static bool stm32h5_unlock_flashopt(struct stm32h5_flash_priv_s *priv)
{
  #warning "stm32h5_unlock_flashopt() not implemented yet"
}

/****************************************************************************
 * Name: stm32h5_lock_flashopt
 *
 * Description:
 *   Lock the flash option bytes
 *
 ****************************************************************************/

static void stm32h5_lock_flashopt(struct stm32h5_flash_priv_s *priv)
{
  stm32h5_flash_modifyreg32(priv, STM32_FLASH_OPTCR_OFFSET, 0,
                            FLASH_OPTCR_OPTLOCK);
}

/****************************************************************************
 * Name: stm32h5_save_flashopt
 *
 * Description:
 *   Save the flash option bytes to non-volatile storage.
 *
 ****************************************************************************/

static void stm32h5_save_flashopt(struct stm32h5_flash_priv_s *priv)
{
  #warning "stm32h5_save_flashopt() not implemented yet"
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32h5_flash_unlock
 *
 * Description:
 *   Unlocks a bank
 *
 ****************************************************************************/

int stm32h5_flash_unlock(size_t addr)
{
  #warning "stm32h5_flash_unlock() not implemented"
}

/****************************************************************************
 * Name: stm32h5_flash_lock
 *
 * Description:
 *   Locks a bank
 *
 ****************************************************************************/

int stm32h5_flash_lock(size_t addr)
{
  #warning "stm32h5_flash_lock() not implemented yet"
}

/****************************************************************************
 * Name: stm32h5_flash_writeprotect
 *
 * Description:
 *   Enable or disable the write protection of a flash sector.
 *
 ****************************************************************************/

int stm32h5_flash_writeprotect(size_t block, bool enabled)
{
  #warning "stm32h5_flash_writeprotect() not implemented"
}

/****************************************************************************
 * Name: stm32h5_flash_getopt
 *
 * Description:
 *   Returns the current flash option bytes from the FLASH_OPTSR_CR register.
 *
 ****************************************************************************/

uint32_t stm32h5_flash_getopt(void)
{
  #warning "stm32h5_flash_getopt() not implemented"
}

/****************************************************************************
 * Name: stm32h5_flash_optmodify
 *
 * Description:
 *   Modifies the current flash option bytes, given bits to set and clear.
 *
 ****************************************************************************/

void stm32h5_flash_optmodify(uint32_t clear, uint32_t set)
{
  #warning "stm32h5_flash_optmodify() not implemented."
}

/****************************************************************************
 * Name: stm32h5_flash_swapbanks
 *
 * Description:
 *   Swaps banks 1 and 2 in the processor's memory map.  Takes effect
 *   the next time the system is reset.
 *
 ****************************************************************************/

void stm32h5_flash_swapbanks(void)
{
  #warning "stm32h5_flash_swapbanks() is not implemented."
}

size_t up_progmem_pagesize(size_t page)
{
  return FLASH_PAGE_SIZE;
}

ssize_t up_progmem_getpage(size_t addr)
{
  struct stm32h5_flash_priv_s *priv;

  priv = stm32h5_flash_bank(addr);

  if (priv == NULL)
    {
      return -EFAULT;
    }

  return  priv->stpage + ((addr - priv->base) / FLASH_PAGE_SIZE);
}

size_t up_progmem_getaddress(size_t page)
{
  struct stm32h5_flash_priv_s *priv;
  if (page >= FLASH_NPAGES)
    {
      return SIZE_MAX;
    }

  priv = stm32h5_flash_bank(STM32_FLASH_BANK1 + (page * FLASH_PAGE_SIZE));
  return priv->base + (page - priv->stpage) * FLASH_PAGE_SIZE;
}

size_t up_progmem_neraseblocks(void)
{
  return PROGMEM_NBLOCKS;
}

bool up_progmem_isuniform(void)
{
  return true;
}

ssize_t up_progmem_ispageerased(size_t page)
{
  size_t addr;
  size_t count;
  size_t bwritten = 0;

  if (page >= FLASH_NPAGES)
    {
      return -EFAULT;
    }

  /* Verify */

  for (addr = up_progmem_getaddress(page), count = up_progmem_pagesize(page);
       count; count--, addr++)
    {
      if (getreg8(addr) != FLASH_ERASEDVALUE)
        {
          bwritten++;
        }
    }

  return bwritten;
}

size_t up_progmem_erasesize(size_t block)
{
  return FLASH_SECTOR_SIZE;
}

ssize_t up_progmem_eraseblock(size_t block)
{
  struct stm32h5_flash_priv_s *priv;
  int ret;
  size_t block_address = STM32_FLASH_BANK1 + (block * FLASH_SECTOR_SIZE);

  if (block >= PROGMEM_NBLOCKS)
    {
      return -EFAULT;
    }

  priv = stm32h5_flash_bank(block_address);

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return (ssize_t)ret;
    }

  if (stm32h5_wait_for_last_operation(priv))
    {
      ret = -EIO;
      goto exit_with_lock;
    }

  /* Get flash ready and begin erasing single block */

  stm32h5_unlock_flash(priv);

  stm32h5_flash_modifyreg32(priv, STM32_FLASH_CR1_OFFSET, 0, FLASH_CR_SER);
  stm32h5_flash_modifyreg32(priv, STM32_FLASH_CR1_OFFSET, FLASH_CR_SNB_MASK,
                            FLASH_CR_SNB(block - priv->stblock));

  stm32h5_flash_modifyreg32(priv, STM32_FLASH_CR1_OFFSET, 0, FLASH_CR_START);

  /* Wait for erase operation to complete */

  if (stm32h5_wait_for_last_operation(priv))
    {
      ret = -EIO;
      goto exit_with_unlock;
    }

  stm32h5_flash_modifyreg32(priv, STM32_FLASH_CR1_OFFSET, FLASH_CR_SER, 0);
  stm32h5_flash_modifyreg32(priv, STM32_FLASH_CR1_OFFSET, FLASH_CR_SNB_MASK,
                            0);

  ret = 0;

  up_invalidate_dcache(block_address, block_address + FLASH_SECTOR_SIZE);

exit_with_unlock:
  stm32h5_lock_flash(priv);

exit_with_lock:
  nxmutex_unlock(&priv->lock);

  /* Verify */

  if (ret == 0 &&
      stm32h5_israngeerased(block_address, up_progmem_erasesize(block)) == 0)
    {
      ret = up_progmem_erasesize(block); /* success */
    }
  else
    {
      ret = -EIO; /* failure */
    }

  return ret;
}

ssize_t up_progmem_write(size_t addr, const void *buf, size_t count)
{
  struct stm32h5_flash_priv_s *priv;
  uint32_t     *fp;
  uint32_t     *rp;
  uint32_t     *ll        = (uint32_t *)buf;
  size_t       faddr;
  size_t       written    = count;
  int          ret;
  const size_t pagesize   = up_progmem_pagesize(0); /* 256 bit, 32 bytes per page */
  const size_t llperpage  = pagesize / sizeof(uint32_t);
  size_t       pcount     = count / pagesize;
  uint32_t     sr;

  priv = stm32h5_flash_bank(addr);

  if (priv == NULL)
    {
      return -EFAULT;
    }

  /* Check for valid address range */

  if (addr < priv->base ||
      addr + count > priv->base + (STM32_FLASH_SIZE / 2))
    {
      return -EFAULT;
    }

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return (ssize_t)ret;
    }

  /* Check address and count alignment */

  DEBUGASSERT(!(addr % pagesize));
  DEBUGASSERT(!(count % pagesize));

  if (stm32h5_wait_for_last_operation(priv))
    {
      written = -EIO;
      goto exit_with_lock;
    }

  /* Get flash ready and begin flashing */

  stm32h5_unlock_flash(priv);

  stm32h5_flash_modifyreg32(priv, STM32_FLASH_CR1_OFFSET,
                            FLASH_CR_PSIZE_MASK, FLASH_CR_PSIZE);

  stm32h5_flash_modifyreg32(priv, STM32_FLASH_CR1_OFFSET, 0, FLASH_CR_PG);

  for (ll = (uint32_t *)buf, faddr = addr; pcount;
      pcount -= 1, ll += llperpage, faddr += pagesize)
    {
      fp = (uint32_t *)faddr;
      rp = ll;

      UP_MB();

      /* Write 8 32 bit word and wait to complete */

      *fp++ = *rp++;
      *fp++ = *rp++;
      *fp++ = *rp++;
      *fp++ = *rp++;
      *fp++ = *rp++;
      *fp++ = *rp++;
      *fp++ = *rp++;
      *fp++ = *rp++;

      /* Data synchronous Barrier (DSB) just after the write operation. This
       * will force the CPU to respect the sequence of instruction (no
       * optimization).
       */

      UP_MB();

      if (stm32h5_wait_for_last_operation(priv))
        {
          written = -EIO;
          goto exit_with_unlock;
        }

      sr = stm32h5_flash_getreg32(priv, STM32_FLASH_SR1_OFFSET);
      if (sr & (FLASH_SR_SNECCERR | FLASH_SR_DBECCERR))
        {
          stm32h5_flash_modifyreg32(priv, STM32_FLASH_CR1_OFFSET,
                                    FLASH_CR_PG,
                                    0);

          stm32h5_flash_modifyreg32(priv, STM32_FLASH_CCR1_OFFSET,
                                    0, ~0);
          ret = -EIO;
          goto exit_with_unlock;
        }
    }

  stm32h5_flash_modifyreg32(priv, STM32_FLASH_CR1_OFFSET, FLASH_CR_PG, 0);

  stm32h5_flash_modifyreg32(priv, STM32_FLASH_CCR1_OFFSET,
                            0, ~0);
exit_with_unlock:
  stm32h5_lock_flash(priv);

  /* Verify */

  if (written > 0)
    {
      for (ll = (uint32_t *)buf, faddr = addr, pcount = count / pagesize;
          pcount; pcount -= 1, ll += llperpage, faddr += pagesize)
        {
          fp = (uint32_t *)faddr;
          rp = ll;

          stm32h5_flash_modifyreg32(priv, STM32_FLASH_CCR1_OFFSET,
                                    0, ~0);
          if ((*fp++ != *rp++) ||
              (*fp++ != *rp++) ||
              (*fp++ != *rp++) ||
              (*fp++ != *rp++) ||
              (*fp++ != *rp++) ||
              (*fp++ != *rp++) ||
              (*fp++ != *rp++) ||
              (*fp++ != *rp++))
            {
              written = -EIO;
              break;
            }

          sr = stm32h5_flash_getreg32(priv, STM32_FLASH_SR1_OFFSET);
          if (sr & (FLASH_SR_SNECCERR | FLASH_SR_DBECCERR))
            {
              written = -EIO;
              break;
            }
        }

      stm32h5_flash_modifyreg32(priv, STM32_FLASH_CCR1_OFFSET,
                                0, ~0);
    }

exit_with_lock:
  nxmutex_unlock(&priv->lock);
  return written;
}

uint8_t up_progmem_erasestate(void)
{
  return FLASH_ERASEDVALUE;
}

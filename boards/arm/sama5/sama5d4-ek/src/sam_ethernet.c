/****************************************************************************
 * boards/arm/sama5/sama5d4-ek/src/sam_ethernet.c
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

/* Force verbose debug on in this file only to support unit-level testing. */

#ifdef CONFIG_NETDEV_PHY_DEBUG
#  undef  CONFIG_DEBUG_INFO
#  define CONFIG_DEBUG_INFO 1
#  undef  CONFIG_DEBUG_NET
#  define CONFIG_DEBUG_NET 1
#endif

#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/spinlock.h>

#include "sam_pio.h"
#include "sam_ethernet.h"

#include "sama5d4-ek.h"

#ifdef HAVE_NETWORK

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_SAMA5_EMAC0
#  undef CONFIG_SAMA5_EMAC0_ISETH0
#endif

#ifdef CONFIG_SAMA5_EMAC0_ISETH0
#  define SAMA5_EMAC0_DEVNAME "eth0"
#  define SAMA5_EMAC1_DEVNAME "eth1"
#else
#  define SAMA5_EMAC0_DEVNAME "eth1"
#  define SAMA5_EMAC1_DEVNAME "eth0"
#endif

/* Debug ********************************************************************/

/* Extra, in-depth debug output that is only available if
 * CONFIG_NETDEV_PHY_DEBUG us defined.
 */

#ifdef CONFIG_NETDEV_PHY_DEBUG
#  define phyerr    _err
#  define phywarn   _warn
#  define phyinfo   _info
#else
#  define phyerr(x...)
#  define phywarn(x...)
#  define phyinfo(x...)
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_SAMA5_PIOE_IRQ
static spinlock_t g_phy_lock = SP_UNLOCKED;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_emac0/1_phy_enable
 ****************************************************************************/

#ifdef CONFIG_SAMA5_PIOE_IRQ
#ifdef CONFIG_SAMA5_EMAC0
static void sam_emac0_phy_enable(bool enable)
{
  phyinfo("IRQ%d: enable=%d\n", IRQ_INT_ETH0, enable);
  if (enable)
    {
      sam_pioirqenable(IRQ_INT_ETH0);
    }
  else
    {
      sam_pioirqdisable(IRQ_INT_ETH0);
    }
}

#endif

#ifdef CONFIG_SAMA5_EMAC1
static void sam_emac1_phy_enable(bool enable)
{
  phyinfo("IRQ%d: enable=%d\n", IRQ_INT_ETH1, enable);
  if (enable)
    {
      sam_pioirqenable(IRQ_INT_ETH1);
    }
  else
    {
      sam_pioirqdisable(IRQ_INT_ETH1);
    }
}
#endif
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_netinitialize
 *
 * Description:
 *   Configure board resources to support networking.
 *
 ****************************************************************************/

void weak_function sam_netinitialize(void)
{
#ifdef CONFIG_SAMA5_EMAC0
  phyinfo("Configuring %08x\n", PIO_INT_ETH0);
  sam_configpio(PIO_INT_ETH0);
#endif

#ifdef CONFIG_SAMA5_EMAC1
  phyinfo("Configuring %08x\n", PIO_INT_ETH1);
  sam_configpio(PIO_INT_ETH1);
#endif
}

/****************************************************************************
 * Name: arch_phy_irq
 *
 * Description:
 *   This function may be called to register an interrupt handler that will
 *   be called when a PHY interrupt occurs.  This function both attaches
 *   the interrupt handler and enables the interrupt if 'handler' is non-
 *   NULL.  If handler is NULL, then the interrupt is detached and disabled
 *   instead.
 *
 *   The PHY interrupt is always disabled upon return.  The caller must
 *   call back through the enable function point to control the state of
 *   the interrupt.
 *
 *   This interrupt may or may not be available on a given platform depending
 *   on how the network hardware architecture is implemented.  In a typical
 *   case, the PHY interrupt is provided to board-level logic as a GPIO
 *   interrupt (in which case this is a board-specific interface and really
 *   should be called board_phy_irq()); In other cases, the PHY interrupt
 *   may be cause by the chip's MAC logic (in which case arch_phy_irq()) is
 *   an appropriate name.  Other other boards, there may be no PHY interrupts
 *   available at all.  If client attachable PHY interrupts are available
 *   from the board or from the chip, then CONFIG_ARCH_PHY_INTERRUPT should
 *   be defined to indicate that fact.
 *
 *   Typical usage:
 *   a. OS service logic (not application logic*) attaches to the PHY
 *      PHY interrupt and enables the PHY interrupt.
 *   b. When the PHY interrupt occurs:  (1) the interrupt should be
 *      disabled and () work should be scheduled on the worker thread (or
 *      perhaps a dedicated application thread).
 *   c. That worker thread should use the SIOCGMIIPHY, SIOCGMIIREG,
 *      and SIOCSMIIREG ioctl calls** to communicate with the PHY,
 *      determine what network event took place (Link Up/Down?), and
 *      take the appropriate actions.
 *   d. It should then interact the PHY to clear any pending
 *      interrupts, then re-enable the PHY interrupt.
 *
 *    * This is an OS internal interface and should not be used from
 *      application space.  Rather applications should use the SIOCMIISIG
 *      ioctl to receive a signal when a PHY event occurs.
 *   ** This interrupt is really of no use if the Ethernet MAC driver
 *      does not support these ioctl calls.
 *
 * Input Parameters:
 *   intf    - Identifies the network interface.  For example "eth0".  Only
 *             useful on platforms that support multiple Ethernet interfaces
 *             and, hence, multiple PHYs and PHY interrupts.
 *   handler - The client interrupt handler to be invoked when the PHY
 *             asserts an interrupt.  Must reside in OS space, but can
 *             signal tasks in user space.  A value of NULL can be passed
 *             in order to detach and disable the PHY interrupt.
 *   arg     - The argument that will accompany the interrupt
 *   enable  - A function pointer that be unused to enable or disable the
 *             PHY interrupt.
 *
 * Returned Value:
 *   Zero (OK) returned on success; a negated errno value is returned on
 *   failure.
 *
 ****************************************************************************/

#ifdef CONFIG_SAMA5_PIOE_IRQ
int arch_phy_irq(const char *intf, xcpt_t handler, void *arg,
                 phy_enable_t *enable)
{
  irqstate_t flags;
  pio_pinset_t pinset;
  phy_enable_t enabler;
  int irq;

  DEBUGASSERT(intf);

  ninfo("%s: handler=%p\n", intf, handler);
#ifdef CONFIG_SAMA5_EMAC0
  phyinfo("EMAC0: devname=%s\n", SAMA5_EMAC0_DEVNAME);
#endif
#ifdef CONFIG_SAMA5_EMAC1
  phyinfo("EMAC1: devname=%s\n", SAMA5_EMAC1_DEVNAME);
#endif

#ifdef CONFIG_SAMA5_EMAC0
  if (strcmp(intf, SAMA5_EMAC0_DEVNAME) == 0)
    {
      phyinfo("Select EMAC0\n");
      pinset   = PIO_INT_ETH0;
      irq      = IRQ_INT_ETH0;
      enabler  = sam_emac0_phy_enable;
    }
  else
#endif
#ifdef CONFIG_SAMA5_EMAC1
  if (strcmp(intf, SAMA5_EMAC1_DEVNAME) == 0)
    {
      phyinfo("Select EMAC1\n");
      pinset   = PIO_INT_ETH1;
      irq      = IRQ_INT_ETH1;
      enabler  = sam_emac1_phy_enable;
    }
  else
#endif
    {
      nerr("ERROR: Unsupported interface: %s\n", intf);
      return -EINVAL;
    }

  /* Disable interrupts until we are done.  This guarantees that the
   * following operations are atomic.
   */

  flags = spin_lock_irqsave(&g_phy_lock);

  /* Configure the interrupt */

  if (handler)
    {
      phyinfo("Configure pin: %08x\n", pinset);
      sam_pioirq(pinset);

      phyinfo("Attach IRQ%d\n", irq);
      irq_attach(irq, handler, arg);
    }
  else
    {
      phyinfo("Detach IRQ%d\n", irq);
      irq_detach(irq);
      enabler = NULL;
    }

  /* Return with the interrupt disabled in either case */

  sam_pioirqdisable(irq);

  /* Return the enabling function pointer */

  if (enable)
    {
      *enable = enabler;
    }

  /* Return the old handler (so that it can be restored) */

  spin_unlock_irqrestore(&g_phy_lock, flags);
  return OK;
}
#endif /* CONFIG_SAMA5_PIOE_IRQ */

#endif /* HAVE_NETWORK */

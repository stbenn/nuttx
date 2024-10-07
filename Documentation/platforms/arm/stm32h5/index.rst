==========
ST STM32H5
==========

This is a port of NuttX to the STM32H5 Family

Used development board is the Nucleo H563ZI.

Most code is copied and adapted from the STM32L5 port.

The only supported STM32H5 family currently is:

================ ======= ============================
NuttX config      Manual Chips
================ ======= ============================
STM32H5           RM0481 STM32H52x, STM32H53x, 
                         STM32H56x, STM32H57x
================ ======= ============================

TODO list
---------

Extensive testing.  Only initial sniff tests have been done.
A proper TODO list should be generated.

Supported MCUs
==============

TODO

Peripheral Support
==================

RCC
PWR
Serial

Supported Boards
================

.. toctree::
   :glob:
   :maxdepth: 1

   boards/*/*

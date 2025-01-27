/****************************************************************************
 * arch/risc-v/src/esp32c3-legacy/esp32c3_i2c.h
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

#ifndef __ARCH_RISCV_SRC_ESP32C3_LEGACY_ESP32C3_I2C_H
#define __ARCH_RISCV_SRC_ESP32C3_LEGACY_ESP32C3_I2C_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/i2c/i2c_master.h>

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
 * Name: esp32c3_i2cbus_initialize
 *
 * Description:
 *   Initialize the selected I2C port. And return a pointer to an unique
 *   instance of struct i2c_master_s. This function may be called to obtain
 *   multiple instances of the interface, each of which may be set up with a
 *   different frequency and slave address.
 *
 * Input Parameters:
 *   port        - Port number of the I2C interface to be initialized.
 *
 * Returned Value:
 *   Pointer to valid I2C device structure is returned on success.
 *   A NULL pointer is returned on failure.
 *
 ****************************************************************************/

struct i2c_master_s *esp32c3_i2cbus_initialize(int port);

/****************************************************************************
 * Name: esp32c3_i2cbus_uninitialize
 *
 * Description:
 *   De-initialize the selected I2C port and power down the device.
 *
 * Input Parameters:
 *   dev         - Device structure as returned by
 *                 esp32c3_i2cbus_initialize()
 *
 * Returned Value:
 *   OK is returned on success. ERROR is returned when internal reference
 *   count mismatches or dev points to invalid hardware device.
 *
 ****************************************************************************/

int esp32c3_i2cbus_uninitialize(struct i2c_master_s *dev);

#ifdef __cplusplus
}
#endif
#undef EXTERN

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_RISCV_SRC_ESP32C3_LEGACY_ESP32C3_I2C_H */

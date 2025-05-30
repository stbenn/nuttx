/****************************************************************************
 * drivers/sensors/bmp180.c
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

/* Character driver for the Freescale BMP1801 Barometer Sensor */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "bmp180_base.h"

#if defined(CONFIG_I2C) && defined(CONFIG_SENSORS_BMP180)

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Character driver methods */

static ssize_t bmp180_read(FAR struct file *filep, FAR char *buffer,
                           size_t buflen);
static ssize_t bmp180_write(FAR struct file *filep, FAR const char *buffer,
                            size_t buflen);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_bmp180fops =
{
  NULL,                         /* open */
  NULL,                         /* close */
  bmp180_read,                  /* read */
  bmp180_write,                 /* write */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bmp180_read
 ****************************************************************************/

static ssize_t bmp180_read(FAR struct file *filep, FAR char *buffer,
                           size_t buflen)
{
  FAR struct inode        *inode = filep->f_inode;
  FAR struct bmp180_dev_s *priv  = inode->i_private;
  FAR uint32_t            *press = (FAR uint32_t *) buffer;

  if (!buffer)
    {
      snerr("ERROR: Buffer is null\n");
      return -1;
    }

  if (buflen != 4)
    {
      snerr("ERROR: You can't read something "
            "other than 32 bits (4 bytes)\n");
      return -1;
    }

  /* Get the pressure compensated */

  *press = (int32_t) bmp180_getpressure(priv, NULL);

  /* Return size of uint32_t (4 bytes) */

  return 4;
}

/****************************************************************************
 * Name: bmp180_write
 ****************************************************************************/

static ssize_t bmp180_write(FAR struct file *filep, FAR const char *buffer,
                            size_t buflen)
{
  return -ENOSYS;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bmp180_register
 *
 * Description:
 *   Register the BMP180 character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/press0"
 *   i2c     - An instance of the I2C interface to use to communicate with
 *             BMP180
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int bmp180_register(FAR const char *devpath, FAR struct i2c_master_s *i2c)
{
  FAR struct bmp180_dev_s *priv;
  int ret;

  /* Initialize the BMP180 device structure */

  priv = (FAR struct bmp180_dev_s *)kmm_malloc(sizeof(struct bmp180_dev_s));
  if (!priv)
    {
      snerr("ERROR: Failed to allocate instance\n");
      return -ENOMEM;
    }

  priv->i2c = i2c;
  priv->addr = BMP180_ADDR;
  priv->freq = BMP180_FREQ;

  /* Check Device ID */

  ret = bmp180_checkid(priv);
  if (ret < 0)
    {
      snerr("ERROR: Failed to register driver: %d\n", ret);
      kmm_free(priv);
      return ret;
    }

  /* Read the coefficient value */

  bmp180_updatecaldata(priv);

  /* Register the character driver */

  ret = register_driver(devpath, &g_bmp180fops, 0666, priv);
  if (ret < 0)
    {
      snerr("ERROR: Failed to register driver: %d\n", ret);
      kmm_free(priv);
    }

  sninfo("BMP180 driver loaded successfully!\n");
  return ret;
}

#endif /* CONFIG_I2C && CONFIG_SENSORS_BMP180 */

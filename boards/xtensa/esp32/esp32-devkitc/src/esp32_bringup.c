/****************************************************************************
 * boards/xtensa/esp32/esp32-devkitc/src/esp32_bringup.c
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

#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <debug.h>
#include <errno.h>
#if defined(CONFIG_ESP32_EFUSE)
#include <nuttx/efuse/efuse.h>
#endif
#include <nuttx/fs/fs.h>
#include <nuttx/himem/himem.h>
#include <nuttx/board.h>

#if defined(CONFIG_ESP32_EFUSE)
#include "esp32_efuse.h"
#endif
#include "esp32_partition.h"

#ifdef CONFIG_USERLED
#  include <nuttx/leds/userled.h>
#endif

#ifdef CONFIG_CAN_MCP2515
#  include "esp32_mcp2515.h"
#endif

#ifdef CONFIG_TIMER
#  include <esp32_tim_lowerhalf.h>
#endif

#ifdef CONFIG_ONESHOT
#  include "esp32_board_oneshot.h"
#endif

#ifdef CONFIG_WATCHDOG
#  include "esp32_board_wdt.h"
#endif

#ifdef CONFIG_ESP32_SPIFLASH
#  include "esp32_board_spiflash.h"
#endif

#ifdef CONFIG_ESPRESSIF_BLE
#  include "esp32_ble.h"
#endif

#ifdef CONFIG_ESPRESSIF_WLAN
#  include "esp32_board_wlan.h"
#endif

#ifdef CONFIG_ESPRESSIF_WIFI_BT_COEXIST
#  include "esp32_wifi_adapter.h"
#endif

#ifdef CONFIG_ESP32_I2C
#  include "esp32_board_i2c.h"
#endif

#ifdef CONFIG_ESPRESSIF_I2S
#  include "espressif/esp_i2s.h"
#endif

#ifdef CONFIG_ESPRESSIF_I2S
#  include "espressif/esp_i2s.h"
#endif

#ifdef CONFIG_ESP_PCNT
#  include "espressif/esp_pcnt.h"
#  include "esp32_board_pcnt.h"
#endif

#ifdef CONFIG_I2CMULTIPLEXER_TCA9548A
#  include "esp32_tca9548a.h"
#endif

#ifdef CONFIG_SENSORS_APDS9960
#include "esp32_board_apds9960.h"
#endif

#ifdef CONFIG_SENSORS_BMP180
#  include "esp32_bmp180.h"
#endif

#ifdef CONFIG_SENSORS_BMP280
#  include "esp32_bmp280.h"
#endif

#ifdef CONFIG_SENSORS_SHT3X
#  include "esp32_sht3x.h"
#endif

#ifdef CONFIG_SENSORS_MS56XX
#  include "esp32_ms5611.h"
#endif

#ifdef CONFIG_LCD_HT16K33
#  include "esp32_ht16k33.h"
#endif

#ifdef CONFIG_ESP32_AES_ACCELERATOR
#  include "esp32_aes.h"
#endif

#ifdef CONFIG_ESP32_RT_TIMER
#  include "esp32_rt_timer.h"
#endif

#ifdef CONFIG_INPUT_BUTTONS
#  include <nuttx/input/buttons.h>
#endif

#ifdef CONFIG_LCD_DEV
#  include <nuttx/lcd/lcd_dev.h>
#endif

#ifdef CONFIG_VIDEO_FB
#  include <nuttx/video/fb.h>
#endif

#ifdef CONFIG_RTC_DRIVER
#  include "esp32_rtc_lowerhalf.h"
#endif

#ifdef CONFIG_SPI_DRIVER
#  include "esp32_spi.h"
#endif

#ifdef CONFIG_SPI_SLAVE_DRIVER
#  include "esp32_board_spislavedev.h"
#endif

#ifdef CONFIG_LCD_BACKPACK
#  include "esp32_lcd_backpack.h"
#endif

#ifdef CONFIG_SENSORS_MAX6675
#  include "esp32_max6675.h"
#endif

#ifdef CONFIG_DAC
#  include "esp32_board_dac.h"
#endif

#ifdef CONFIG_ESP_RMT
#  include "esp32_board_rmt.h"
#endif

#ifdef CONFIG_ESP_MCPWM
#  include "esp32_board_mcpwm.h"
#endif

#ifdef CONFIG_SYSTEM_NXDIAG_ESPRESSIF_CHIP_WO_TOOL
#  include "espressif/esp_nxdiag.h"
#endif

#ifdef CONFIG_ESPRESSIF_ESPNOW_PKTRADIO
#  include "espressif/esp_espnow_pktradio.h"
#endif

#ifdef CONFIG_ESPRESSIF_ADC
#  include "esp32_board_adc.h"
#endif

#ifdef CONFIG_MMCSD_SPI
#  include "esp32_board_sdmmc.h"
#endif

#include "esp32-devkitc.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32_bringup
 *
 * Description:
 *   Perform architecture-specific initialization
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=y :
 *     Called from board_late_initialize().
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=n && CONFIG_BOARDCTL=y :
 *     Called from the NSH library
 *
 ****************************************************************************/

int esp32_bringup(void)
{
  int ret;

#ifdef CONFIG_ESP32_AES_ACCELERATOR
  ret = esp32_aes_init();
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to initialize AES: %d\n", ret);
    }
#endif

#if defined(CONFIG_ESP32_SPIRAM) && \
    defined(CONFIG_ESP32_SPIRAM_BANKSWITCH_ENABLE)
  ret = esp_himem_init();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to init HIMEM: %d\n", ret);
    }
#endif

#if defined(CONFIG_ESP32_EFUSE)
  ret = esp32_efuse_initialize("/dev/efuse");
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to init EFUSE: %d\n", ret);
    }
#endif

#ifdef CONFIG_FS_PROCFS
  /* Mount the procfs file system */

  ret = nx_mount(NULL, "/proc", "procfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to mount procfs at /proc: %d\n", ret);
    }
#endif

#ifdef CONFIG_LCD_BACKPACK
  /* slcd:0, i2c:0, rows=2, cols=16 */

  ret = board_lcd_backpack_init(0, 0, 2, 16);
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize PCF8574 LCD, error %d\n", ret);
    }
#endif

#ifdef CONFIG_FS_TMPFS
  /* Mount the tmpfs file system */

  ret = nx_mount(NULL, CONFIG_LIBC_TMPDIR, "tmpfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to mount tmpfs at %s: %d\n",
             CONFIG_LIBC_TMPDIR, ret);
    }
#endif

#ifdef CONFIG_MMCSD_SPI
  ret = board_sdmmc_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize SD slot: %d\n", ret);
    }
#endif

#ifdef CONFIG_ESP32_SPIFLASH
  ret = board_spiflash_init();
  if (ret)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize SPI Flash\n");
    }
#endif

#ifdef CONFIG_ESP32_PARTITION_TABLE
  ret = esp32_partition_init();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize partition error=%d\n",
             ret);
    }
#endif

#ifdef CONFIG_ESP32_LEDC
  ret = esp32_pwm_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: esp32_pwm_setup() failed: %d\n", ret);
    }
#endif /* CONFIG_ESP32_LEDC */

#ifdef CONFIG_ESP_MCPWM_CAPTURE
  ret = board_capture_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: board_capture_initialize failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_ESP_MCPWM_MOTOR_BDC
  ret = board_motor_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: board_motor_initialize failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_SENSORS_MAX6675
  ret = board_max6675_initialize(0, 2);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: MAX6675 initialization failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_ESP32_TWAI

  /* Initialize TWAI and register the TWAI driver. */

  ret = esp32_twai_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: esp32_twai_setup failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_ESP32_RT_TIMER
  ret = esp32_rt_timer_init();
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize RT timer: %d\n", ret);
    }
#endif

#ifdef CONFIG_ESPRESSIF_WIFI_BT_COEXIST
  ret = esp32_wifi_bt_coexist_init();
  if (ret)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize Wi-Fi and BT "
                      "coexistence support\n");
    }
#endif

#ifdef CONFIG_ESPRESSIF_BLE
  ret = esp32_ble_initialize();
  if (ret)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize BLE: %d\n", ret);
    }
#endif

#ifdef CONFIG_ESPRESSIF_WLAN
  ret = board_wlan_init();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize wlan subsystem=%d\n",
             ret);
    }
#endif

#ifdef CONFIG_ESPRESSIF_ESPNOW_PKTRADIO
  ret = pktradio_espnow();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize ESPNOW pktradio=%d\n",
             ret);
    }
#endif

#ifdef CONFIG_ESP32_OPENETH
  ret = esp_openeth_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize Open ETH ethernet.\n");
    }
#endif

  /* First, register the timer drivers and let timer 1 for oneshot
   * if it is enabled.
   */

#ifdef CONFIG_TIMER

#if defined(CONFIG_ESP32_TIMER0) && !defined(CONFIG_ESP32_RT_TIMER)
  ret = esp32_timer_initialize("/dev/timer0", TIMER0);
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to initialize timer driver: %d\n",
             ret);
    }
#endif

#if defined(CONFIG_ESP32_TIMER1) && !defined(CONFIG_ONESHOT)
  ret = esp32_timer_initialize("/dev/timer1", TIMER1);
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to initialize timer driver: %d\n",
             ret);
    }
#endif

#ifdef CONFIG_ESP32_TIMER2
  ret = esp32_timer_initialize("/dev/timer2", TIMER2);
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to initialize timer driver: %d\n",
             ret);
    }
#endif

#ifdef CONFIG_ESP32_TIMER3
  ret = esp32_timer_initialize("/dev/timer3", TIMER3);
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to initialize timer driver: %d\n",
             ret);
    }
#endif

#endif /* CONFIG_TIMER */

  /* Now register one oneshot driver */

#if defined(CONFIG_ONESHOT) && defined(CONFIG_ESP32_TIMER1)

  ret = esp32_oneshot_init(ONESHOT_TIMER, ONESHOT_RESOLUTION_US);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: esp32_oneshot_init() failed: %d\n", ret);
    }

#endif /* CONFIG_ONESHOT */

#ifdef CONFIG_USERLED
  /* Register the LED driver */

  ret = userled_lower_initialize("/dev/userleds");
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: userled_lower_initialize() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_CAN_MCP2515
  /* Configure and initialize the MCP2515 CAN device */

  ret = board_mcp2515_initialize(0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: board_mcp2515_initialize() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_WATCHDOG
  /* Configure watchdog timer */

  ret = board_wdt_init();
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to initialize watchdog drivers: %d\n",
             ret);
    }
#endif

#ifdef CONFIG_DEV_GPIO
  ret = esp32_gpio_init();
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize GPIO Driver: %d\n", ret);
    }
#endif

#ifdef CONFIG_ESP_PCNT
  ret = board_pcnt_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: board_pcnt_initialize failed: %d\n", ret);
    }
#endif

  /* Register the TCA9548A Multiplexer before others I2C drivers to allow it
   * be used by other drivers. Look at esp32_ms5611.c how to use it.
   */

#ifdef CONFIG_I2CMULTIPLEXER_TCA9548A
  /* Add the TCA9548A Mux as device 0 (0x70) in I2C Bus 0 */

  ret = board_tca9548a_initialize(0, 0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize TCA9548A driver: %d\n", ret);
      return ret;
    }
#endif

#ifdef CONFIG_I2C_DRIVER

#ifdef CONFIG_ESP32_I2C0
  ret = esp32_i2c_register(0);

  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize I2C Driver for I2C0: %d\n", ret);
    }
#endif

#ifdef CONFIG_ESP32_I2C1
  ret = esp32_i2c_register(1);

  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize I2C Driver for I2C1: %d\n", ret);
    }
#endif

#endif

#ifdef CONFIG_SENSORS_BMP180
  /* Try to register BMP180 device in I2C0 */

  ret = board_bmp180_initialize(0, 0);

  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize BMP180 driver: %d\n", ret);
    }
#endif

#ifdef CONFIG_SENSORS_BMP280
  /* Try to register BMP280 device in I2C0 */

  ret = board_bmp280_initialize(0, 0);

  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize BMP280 driver: %d\n", ret);
    }
#endif

#if defined(CONFIG_ESPRESSIF_I2S) || defined(CONFIG_ESPRESSIF_I2S)

#if defined(CONFIG_ESPRESSIF_I2S0) && !defined(CONFIG_AUDIO_CS4344) || \
    defined(CONFIG_ESPRESSIF_I2S1)
  bool i2s_enable_tx;
  bool i2s_enable_rx;
#endif

#if defined(CONFIG_ESPRESSIF_I2S0) || defined(CONFIG_ESPRESSIF_I2S0)

  /* Configure I2S0 */

#ifdef CONFIG_AUDIO_CS4344

  /* Configure CS4344 audio on I2S0 */

  ret = esp32_cs4344_initialize(ESP32_I2S0);
  if (ret != OK)
    {
      syslog(LOG_ERR, "Failed to initialize CS4344 audio: %d\n", ret);
    }
#else

#if defined(CONFIG_ESPRESSIF_I2S0_TX) || defined (CONFIG_ESPRESSIF_I2S0_TX)
  i2s_enable_tx = true;
#else
  i2s_enable_tx = false;
#endif /* CONFIG_ESPRESSIF_I2S0_TX || CONFIG_ESPRESSIF_I2S0_TX */

#if defined(CONFIG_ESPRESSIF_I2S0_RX) || defined (CONFIG_ESPRESSIF_I2S0_RX)
    i2s_enable_rx = true;
#else
    i2s_enable_rx = false;
#endif /* CONFIG_ESPRESSIF_I2S0_RX || CONFIG_ESPRESSIF_I2S0_RX */

  /* Configure I2S generic audio on I2S0 */

  ret = board_i2sdev_initialize(ESP32_I2S0, i2s_enable_tx, i2s_enable_rx);
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize I2S%d driver: %d\n",
             ESP32_I2S0, ret);
    }

#endif /* CONFIG_AUDIO_CS4344 */

#endif /* CONFIG_ESPRESSIF_I2S0 || CONFIG_ESPRESSIF_I2S0 */

#if defined(CONFIG_ESPRESSIF_I2S1) || defined(CONFIG_ESPRESSIF_I2S1)

#if defined(CONFIG_ESPRESSIF_I2S1_TX) || defined (CONFIG_ESPRESSIF_I2S1_TX)
  i2s_enable_tx = true;
#else
  i2s_enable_tx = false;
#endif /* CONFIG_ESPRESSIF_I2S1_TX || CONFIG_ESPRESSIF_I2S1_TX */

#if defined(CONFIG_ESPRESSIF_I2S1_RX) || defined (CONFIG_ESPRESSIF_I2S1_RX)
    i2s_enable_rx = true;
#else
    i2s_enable_rx = false;
#endif /* CONFIG_ESPRESSIF_I2S1_RX || CONFIG_ESPRESSIF_I2S1_RX */

  /* Configure I2S generic audio on I2S1 */

  ret = board_i2sdev_initialize(ESP32_I2S1, i2s_enable_tx, i2s_enable_rx);
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize I2S%d driver: %d\n",
             ESP32_I2S1, ret);
    }

#endif /* CONFIG_ESPRESSIF_I2S1 || CONFIG_ESPRESSIF_I2S1 */

#endif /* CONFIG_ESPRESSIF_I2S || CONFIG_ESPRESSIF_I2S */

#ifdef CONFIG_SENSORS_SHT3X
  /* Try to register SHT3x device in I2C0 */

  ret = board_sht3x_initialize(0, 0);

  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize SHT3X driver: %d\n", ret);
    }
#endif

#ifdef CONFIG_SENSORS_MS56XX
  /* Try to register MS5611 device in I2C0 as device 0: I2C addr 0x77 */

  ret = board_ms5611_initialize(0, 0);

  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize MS5611 driver: %d\n", ret);
      return ret;
    }
#endif

#ifdef CONFIG_LCD_HT16K33
  /* Try to register HT16K33 in the I2C0 */

  ret = board_ht16k33_initialize(0, 0);

  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize HT16K33 driver: %d\n", ret);
    }
#endif

#ifdef CONFIG_INPUT_BUTTONS
  /* Register the BUTTON driver */

  ret = btn_lower_initialize("/dev/buttons");
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: btn_lower_initialize() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_LCD_DEV
  ret = board_lcd_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: board_lcd_initialize() failed: %d\n", ret);
    }

  ret = lcddev_register(0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: lcddev_register() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_DAC
  ret = board_dac_initialize(CONFIG_ESP32_DAC_DEVPATH);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: board_dac_initialize(0) failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_ESP_RMT
  ret = board_rmt_txinitialize(RMT_TXCHANNEL, RMT_OUTPUT_PIN);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: board_rmt_txinitialize() failed: %d\n", ret);
    }

  ret = board_rmt_rxinitialize(RMT_RXCHANNEL, RMT_INPUT_PIN);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: board_rmt_txinitialize() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_RTC_DRIVER
  /* Instantiate the ESP32 RTC driver */

  ret = esp32_rtc_driverinit();
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to Instantiate the RTC driver: %d\n", ret);
    }
#endif

#if defined(CONFIG_ESP32_SPI2) && defined(CONFIG_SPI_DRIVER)
  ret = board_spidev_initialize(ESP32_SPI2);
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize SPI%d driver: %d\n",
             ESP32_SPI2, ret);
    }
#endif

# if defined(CONFIG_ESP32_SPI2) && defined(CONFIG_SPI_SLAVE_DRIVER)
  ret = board_spislavedev_initialize(ESP32_SPI2);
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize SPI%d Slave driver: %d\n",
              ESP32_SPI2, ret);
    }
#endif

#ifdef CONFIG_WS2812
#  ifndef CONFIG_WS2812_NON_SPI_DRIVER
  ret = board_ws2812_initialize(0, ESP32_SPI3, CONFIG_WS2812_LED_COUNT);
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize ws2812 driver\n");
    }
#  endif
#endif

#ifdef CONFIG_VIDEO_FB
  /* Initialize and register the framebuffer driver */

  ret = fb_register(0, 0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: fb_register() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_SENSORS_APDS9960
  /* Register the APDS-9960 gesture sensor */

  ret = board_apds9960_initialize(0, 0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: board_apds9960_initialize() failed: %d\n",
             ret);
    }
#endif

#ifdef CONFIG_SYSTEM_NXDIAG_ESPRESSIF_CHIP_WO_TOOL
  ret = esp_nxdiag_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: esp_nxdiag_initialize failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_ESPRESSIF_ADC
  ret = board_adc_init();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: board_adc_init failed: %d\n", ret);
    }
#endif

  /* If we got here then perhaps not all initialization was successful, but
   * at least enough succeeded to bring-up NSH with perhaps reduced
   * capabilities.
   */

  UNUSED(ret);
  return OK;
}

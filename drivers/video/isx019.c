/****************************************************************************
 * drivers/video/isx019.c
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
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/signal.h>
#include <nuttx/clock.h>
#include <arch/board/board.h>
#include <nuttx/video/isx019.h>
#include <nuttx/video/imgsensor.h>
#include <math.h>
#include <nuttx/mutex.h>

#include "isx019_reg.h"
#include "isx019_range.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Wait time on power on sequence. */

#define TRANSITION_TIME_TO_STARTUP   (130 * USEC_PER_MSEC) /* unit : usec */
#define TRANSITION_TIME_TO_STREAMING (40 * USEC_PER_MSEC)  /* unit : usec */
#define DELAY_TIME_JPEGDQT_SWAP      (35 * USEC_PER_MSEC)  /* unit : usec */

/* For get_supported_value() I/F */

#define SET_RANGE(range, min, max, s, def) \
        do                                 \
          {                                \
            (range).minimum       = (min); \
            (range).maximum       = (max); \
            (range).step          = (s);   \
            (range).default_value = (def); \
          }                                \
        while (0);

#define SET_DISCRETE(disc, nr, val, def)   \
        do                                 \
          {                                \
            (disc).nr_values     = (nr);   \
            (disc).values        = (val);  \
            (disc).default_value = (def);  \
          }                                \
        while (0);

#define SET_ELEMS(elem, nr, min, max, s)   \
        do                                 \
          {                                \
            (elem).nr_elems      = (nr);   \
            (elem).minimum       = (min);  \
            (elem).maximum       = (max);  \
            (elem).step          = (s);    \
          }                                \
        while (0);

#define COMPARE_FRAMESIZE(w, h, sup_w, sup_h)  (((w) == (sup_w)) && \
                                                ((h) == (sup_h)))

#define VALIDATE_FRAMESIZE(w, h) (COMPARE_FRAMESIZE((w), (h), 1280, 960) || \
                                  COMPARE_FRAMESIZE((w), (h), 1280, 720) || \
                                  COMPARE_FRAMESIZE((w), (h),  640, 480) || \
                                  COMPARE_FRAMESIZE((w), (h),  640, 360) || \
                                  COMPARE_FRAMESIZE((w), (h),  480, 360) || \
                                  COMPARE_FRAMESIZE((w), (h),  320, 240) || \
                                  COMPARE_FRAMESIZE((w), (h),  160, 120))

#define VALIDATE_THUMNAIL_SIZE(m, s) (((s) != 0) && \
                                      ((m) % (s) == 0) && \
                                      ((m) / (s) < 5)  && \
                                      ((m) / (s) > 0))

/* For set_value() and get_value() I/F */

#define SET_REGINFO(a, c, o, t, s) do \
                                  {                      \
                                    (a)->category = (c); \
                                    (a)->offset   = (o); \
                                    (a)->type     = (t); \
                                    (a)->size     = (s); \
                                  }                      \
                                while (0);

/* Register type, which represents the number of bits and
 * whether it is signed or unsigned.
 */

#define ISX019_REGTYPE_INT8   (0)
#define ISX019_REGTYPE_UINT8  (1)
#define ISX019_REGTYPE_INT16  (2)

#define SET_REGINFO_INT8(a, c, o) SET_REGINFO(a, c, o, ISX019_REGTYPE_INT8, 1)
#define SET_REGINFO_UINT8(a, c, o) SET_REGINFO(a, c, o, ISX019_REGTYPE_UINT8, 1)
#define SET_REGINFO_INT16(a, c, o) SET_REGINFO(a, c, o, ISX019_REGTYPE_INT16, 2)

#define VALIDATE_RANGE(v, min, max, step) (((v) >= (min)) && \
                                           ((v) <= (max)) && \
                                           (((v) - (min)) % (step) == 0))

/* Offset for IMGSENSOR_ID_3A_PARAMETER control */

#define OFFSET_3APARAMETER_AWB_R      (0)
#define OFFSET_3APARAMETER_AWB_B      (2)
#define OFFSET_3APARAMETER_AE_SHTTIME (4)
#define OFFSET_3APARAMETER_AE_GAIN    (8)

/* Index of array for drive mode setting */

#define INDEX_SENS     (0)
#define INDEX_POST     (1)
#define INDEX_SENSPOST (2)
#define INDEX_IO       (3)

/* Timer value for power on control */

#define ISX019_ACCESSIBLE_WAIT_SEC    (0)
#define ISX019_ACCESSIBLE_WAIT_USEC   (200000)
#define FPGA_ACCESSIBLE_WAIT_SEC      (1)
#define FPGA_ACCESSIBLE_WAIT_USEC     (0)

/* Array size of DQT setting for JPEG quality */

#define JPEG_DQT_ARRAY_SIZE (64)

/* ISX019 standard master clock */

#define ISX019_STANDARD_MASTER_CLOCK (27000000)

/* Vivid colors setting */

#define VIVID_COLORS_SATURATION (0xf0)
#define VIVID_COLORS_SHARPNESS  (0x20)

/* Black white colors setting */

#define BW_COLORS_SATURATION (0x00)

/* Definition for calculation of extended frame number */

#define VTIME_PER_FRAME    (30518)
#define INTERVAL_PER_FRAME (33333)

/* ISX019 image sensor output frame size. */

#define ISX019_WIDTH  (1280)
#define ISX019_HEIGHT (960)

/* The number of whole image splits for spot position decision. */

#define ISX019_SPOT_POSITION_SPLIT_NUM_X (9)
#define ISX019_SPOT_POSITION_SPLIT_NUM_Y (7)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct isx019_default_value_s
{
  int32_t brightness;
  int32_t contrast;
  int32_t saturation;
  int32_t hue;
  int32_t awb;
  int32_t gamma;
  int32_t ev;
  int32_t hflip_video;
  int32_t vflip_video;
  int32_t hflip_still;
  int32_t vflip_still;
  int32_t sharpness;
  int32_t ae;
  int32_t wbmode;
  int32_t hdr;
  int32_t iso_auto;
  int32_t meter;
  int32_t spot_pos;
  int32_t threealock;
  int32_t jpgquality;
};

typedef struct isx019_default_value_s isx019_default_value_t;

struct isx019_rect_s
{
  int32_t left;
  int32_t top;
  uint32_t width;
  uint32_t height;
};

typedef struct isx019_rect_s isx019_rect_t;

struct isx019_dev_s
{
  struct imgsensor_s sensor;
  mutex_t fpga_lock;
  mutex_t i2c_lock;
  FAR struct i2c_master_s *i2c;
  struct i2c_config_s i2c_cfg;
  float clock_ratio;
  isx019_default_value_t  default_value;
  imgsensor_stream_type_t stream;
  imgsensor_white_balance_t wb_mode;
  uint8_t flip_video;
  uint8_t flip_still;
  isx019_rect_t clip_video;
  isx019_rect_t clip_still;
  double  gamma;
  int32_t jpg_quality;
  int32_t hue;
  imgsensor_colorfx_t colorfx;
};

typedef struct isx019_dev_s isx019_dev_t;

typedef CODE int32_t (*convert_t)(int32_t value32);

typedef CODE int (*setvalue_t)(FAR isx019_dev_t *priv,
                               imgsensor_value_t value);
typedef CODE int (*getvalue_t)(FAR isx019_dev_t *priv,
                               FAR imgsensor_value_t *value);

struct isx019_reginfo_s
{
  uint16_t category;
  uint16_t offset;
  uint8_t  type;
  uint8_t  size;
};

typedef struct isx019_reginfo_s isx019_reginfo_t;

struct isx019_fpga_jpg_quality_s
{
  /* JPEG quality */

  int quality;

  /* DQT header setting for y component */

  uint8_t y_head[JPEG_DQT_ARRAY_SIZE];

  /* DQT calculation data for y component */

  uint8_t y_calc[JPEG_DQT_ARRAY_SIZE];

  /* DQT header setting for c component */

  uint8_t c_head[JPEG_DQT_ARRAY_SIZE];

  /* DQT calculation data for c component */

  uint8_t c_calc[JPEG_DQT_ARRAY_SIZE];
};

typedef struct isx019_fpga_jpg_quality_s isx019_fpga_jpg_quality_t;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static bool isx019_is_available(FAR struct imgsensor_s *sensor);
static int isx019_init(FAR struct imgsensor_s *sensor);
static int isx019_uninit(FAR struct imgsensor_s *sensor);
static FAR const char *
isx019_get_driver_name(FAR struct imgsensor_s *sensor);
static int isx019_validate_frame_setting(FAR struct imgsensor_s *sensor,
                                         imgsensor_stream_type_t type,
                                         uint8_t nr_datafmt,
                                         FAR imgsensor_format_t *datafmts,
                                         FAR imgsensor_interval_t *interval);
static int isx019_start_capture(FAR struct imgsensor_s *sensor,
                                imgsensor_stream_type_t type,
                                uint8_t nr_datafmt,
                                FAR imgsensor_format_t *datafmts,
                                FAR imgsensor_interval_t *interval);
static int isx019_stop_capture(FAR struct imgsensor_s *sensor,
                               imgsensor_stream_type_t type);
static int isx019_get_frame_interval(FAR struct imgsensor_s *sensor,
                                     imgsensor_stream_type_t type,
                                     FAR imgsensor_interval_t *interval);
static int isx019_get_supported_value(FAR struct imgsensor_s *sensor,
                                      uint32_t id,
                                     FAR imgsensor_supported_value_t *value);
static int isx019_get_value(FAR struct imgsensor_s *sensor,
                            uint32_t id, uint32_t size,
                            FAR imgsensor_value_t *value);
static int isx019_set_value(FAR struct imgsensor_s *sensor,
                            uint32_t id, uint32_t size,
                            imgsensor_value_t value);
static int initialize_jpg_quality(FAR isx019_dev_t *priv);
static void initialize_wbmode(FAR isx019_dev_t *priv);
static int send_read_cmd(FAR isx019_dev_t *priv,
                         FAR const struct i2c_config_s *config,
                         uint8_t cat,
                         uint16_t addr,
                         uint8_t size);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct imgsensor_ops_s g_isx019_ops =
{
  isx019_is_available,
  isx019_init,
  isx019_uninit,
  isx019_get_driver_name,
  isx019_validate_frame_setting,
  isx019_start_capture,
  isx019_stop_capture,
  isx019_get_frame_interval,
  isx019_get_supported_value,
  isx019_get_value,
  isx019_set_value,
};

static isx019_dev_t g_isx019_private =
{
  {
    &g_isx019_ops
  },
  NXMUTEX_INITIALIZER,
  NXMUTEX_INITIALIZER,
};

static const isx019_fpga_jpg_quality_t g_isx019_jpg_quality[] =
{
  {
    10,
      {
         21,  16,  16,  26,  18,  26,  43,  21,
         21,  43,  43,  43,  32,  43,  43,  43,
         43,  43,  43,  43,  43,  64,  43,  43,
         43,  43,  43,  64,  64,  64,  64,  64,
         64,  64,  64,  64,  64,  64,  64,  64,
         64,  64,  64,  64,  64,  64,  64,  64,
         64,  64,  64,  64,  64,  64,  64,  64,
         64,  64,  64,  64,  64,  64,  64,  64,
      },
      {
          3,   4, 133, 131, 131, 131,   1,   1,
          4, 135,   3, 131, 131, 131,   1,   1,
        133,   3,   2, 131, 131,   1,   1,   1,
        131, 131, 131, 131,   1,   1,   1,   1,
        131, 131, 131,   1,   1,   1,   1,   1,
        131, 131,   1,   1,   1,   1,   1,   1,
          1,   1,   1,   1,   1,   1,   1,   1,
          1,   1,   1,   1,   1,   1,   1,   1,
      },
      {
         21,  26,  26,  32,  26,  32,  43,  26,
         26,  43,  64,  43,  32,  43,  64,  64,
         64,  43,  43,  64,  64,  64,  64,  64,
         43,  64,  64,  64,  64,  64,  64,  64,
         64,  64,  64,  64,  64,  64,  64,  64,
         64,  64,  64,  64,  64,  64,  64,  64,
         64,  64,  64,  64,  64,  64,  64,  64,
         64,  64,  64,  64,  64,  64,  64,  64,
      },
      {
          3, 133,   2, 131,   1,   1,   1,   1,
        133, 133, 133, 131,   1,   1,   1,   1,
          2, 133,   2, 131,   1,   1,   1,   1,
        131, 131, 131, 131,   1,   1,   1,   1,
          1,   1,   1,   1,   1,   1,   1,   1,
          1,   1,   1,   1,   1,   1,   1,   1,
          1,   1,   1,   1,   1,   1,   1,   1,
          1,   1,   1,   1,   1,   1,   1,   1,
      }
  },
  {
    20,
      {
         18,  14,  14,  14,  16,  14,  21,  16,
         16,  21,  32,  21,  16,  21,  32,  32,
         26,  21,  21,  26,  32,  32,  26,  26,
         26,  26,  26,  32,  43,  32,  32,  32,
         32,  32,  32,  43,  43,  43,  43,  43,
         43,  43,  43,  64,  64,  64,  64,  64,
         64,  64,  64,  64,  64,  64,  64,  64,
         64,  64,  64,  64,  64,  64,  64,  64,
      },
      {
        135, 137, 137,   3,   2,   2,   2, 131,
        137,   4,   4,   3, 133, 133,   2, 131,
        137,   4,   4,   3, 133,   2, 131,   1,
          3,   3,   3, 133,   2, 131,   1,   1,
          2, 133, 133,   2, 131,   1,   1,   1,
          2, 133,   2, 131,   1,   1,   1,   1,
          2,   2, 131,   1,   1,   1,   1,   1,
        131, 131,   1,   1,   1,   1,   1,   1,
      },
      {
         21,  21,  21,  21,  26,  21,  26,  21,
         21,  26,  26,  21,  26,  21,  26,  32,
         26,  26,  26,  26,  32,  43,  32,  32,
         32,  32,  32,  43,  64,  43,  43,  43,
         43,  43,  43,  64,  64,  64,  43,  43,
         43,  64,  64,  64,  64,  64,  64,  64,
         64,  64,  64,  64,  64,  64,  64,  64,
         64,  64,  64,  64,  64,  64,  64,  64,
      },
      {
          3,   3,   3, 133, 133,   2, 131,   1,
          3, 133,   3,   3, 133,   2, 131,   1,
          3,   3, 133, 133,   2, 131,   1,   1,
        133,   3, 133,   2, 131, 131,   1,   1,
        133, 133,   2, 131, 131,   1,   1,   1,
          2,   2, 131, 131,   1,   1,   1,   1,
        131, 131,   1,   1,   1,   1,   1,   1,
          1,   1,   1,   1,   1,   1,   1,   1,
      }
  },
  {
    30,
      {
         16,  11,  11,  11,  12,  11,  16,  12,
         12,  16,  21,  14,  13,  14,  21,  26,
         21,  16,  16,  21,  26,  32,  21,  21,
         21,  21,  21,  32,  32,  21,  26,  26,
         26,  26,  21,  32,  32,  32,  32,  43,
         32,  32,  32,  43,  43,  43,  43,  43,
         43,  64,  64,  64,  64,  64,  64,  64,
         64,  64,  64,  64,  64,  64,  64,  64,
      },
      {
          4,   6,   6,   4,   3, 133,   2,   2,
          6, 139, 139, 137,   3,   3,   3,   2,
          6, 139,   5,   4,   3, 133,   2, 131,
          4, 137,   4,   3, 133,   2, 131,   1,
          3,   3,   3, 133, 131, 131,   1,   1,
        133,   3, 133,   2, 131,   1,   1,   1,
          2,   3,   2, 131,   1,   1,   1,   1,
          2,   2, 131,   1,   1,   1,   1,   1,
      },
      {
         16,  14,  14,  16,  18,  16,  21,  18,
         18,  21,  21,  16,  21,  16,  21,  26,
         21,  21,  21,  21,  26,  43,  26,  26,
         26,  26,  26,  43,  43,  32,  32,  32,
         32,  32,  32,  43,  43,  43,  43,  43,
         43,  43,  43,  43,  43,  43,  43,  43,
         43,  64,  64,  64,  64,  64,  64,  64,
         64,  64,  64,  64,  64,  64,  64,  64,
      },
      {
          4, 137,   4,   3,   3, 133, 131, 131,
        137, 135, 135,   4,   3, 133,   2, 131,
          4, 135,   3,   3, 133,   2, 131, 131,
          3,   4,   3, 133,   2, 131, 131,   1,
          3,   3, 133,   2, 131, 131,   1,   1,
        133, 133,   2, 131, 131,   1,   1,   1,
        131,   2, 131, 131,   1,   1,   1,   1,
        131, 131, 131,   1,   1,   1,   1,   1,
      }
  },
  {
    40,
      {
         12,   8,   8,   8,   9,   8,  12,   9,
          9,  12,  16,  11,  10,  11,  16,  21,
         14,  12,  12,  14,  21,  26,  18,  18,
         21,  18,  18,  26,  21,  18,  21,  21,
         21,  21,  18,  21,  21,  26,  26,  32,
         26,  26,  21,  32,  32,  43,  43,  32,
         32,  43,  43,  43,  43,  43,  64,  64,
         64,  64,  64,  64,  64,  64,  64,  64,
      },
      {
        139,   8,   8, 139,   4,   3, 133,   3,
          8,   7,   7,   6, 137, 135, 135,   3,
          8,   7, 141, 139, 135,   3, 133,   2,
        139,   6, 139,   3,   3, 133,   2, 131,
          4, 137, 135,   3,   2, 131, 131,   1,
          3, 135,   3, 133, 131, 131,   1,   1,
        133, 135, 133,   2, 131,   1,   1,   1,
          3,   3,   2, 131,   1,   1,   1,   1,
      },
      {
         13,  11,  11,  13,  14,  13,  16,  14,
         14,  16,  21,  14,  14,  14,  21,  21,
         16,  16,  16,  16,  21,  26,  21,  21,
         21,  21,  21,  26,  32,  26,  21,  21,
         21,  21,  26,  32,  32,  32,  32,  32,
         32,  32,  32,  43,  43,  32,  32,  43,
         43,  43,  43,  43,  43,  43,  64,  64,
         64,  64,  64,  64,  64,  64,  64,  64,
      },
      {
          5,   6,   5,   4,   3,   3, 133,   2,
          6, 137, 137, 137,   4,   3, 133,   2,
          5, 137, 137,   4,   3,   3,   2, 131,
          4, 137,   4,   3,   3,   2, 131, 131,
          3,   4,   3,   3,   2,   2, 131,   1,
          3,   3,   3,   2,   2, 131,   1,   1,
        133, 133,   2, 131, 131,   1,   1,   1,
          2,   2, 131, 131,   1,   1,   1,   1,
      }
  },
  {
    50,
      {
          8,   6,   6,   6,   6,   6,   8,   6,
          6,   8,  12,   8,   7,   8,  12,  14,
         10,   8,   8,  10,  14,  16,  13,  13,
         14,  13,  13,  16,  16,  12,  14,  13,
         13,  14,  12,  16,  14,  18,  18,  21,
         18,  18,  14,  26,  26,  26,  26,  26,
         26,  32,  32,  32,  32,  32,  43,  43,
         43,  43,  43,  43,  43,  43,  43,  43,
      },
      {
          8,  11,  11,   8, 139, 137,   4,   4,
         11,  11,  11,   8, 141,   5, 139, 137,
         11,  11,   9,   8,   5, 137, 135, 133,
          8,   8,   8, 137,   5, 135, 133,   2,
        139, 141,   5,   5,   3, 133,   2, 131,
        137,   5, 137, 135, 133,   2, 131, 131,
          4, 139, 135, 133,   2, 131, 131, 131,
          4, 137, 133,   2, 131, 131, 131, 131,
      },
      {
          9,   8,   8,   9,  10,   9,  11,   9,
          9,  11,  14,  11,  13,  11,  14,  16,
         14,  14,  14,  14,  16,  18,  13,  13,
         14,  13,  13,  18,  26,  16,  14,  14,
         14,  14,  16,  26,  21,  21,  21,  21,
         21,  21,  21,  26,  26,  26,  26,  26,
         26,  32,  32,  32,  32,  32,  43,  43,
         43,  43,  43,  43,  43,  43,  43,  43,
      },
      {
          7,   8,   7,   6, 137,   4, 135, 133,
          8, 141,   7,   6, 137,   5,   4,   3,
          7,   7,   5, 137,   5, 137,   3, 133,
          6,   6, 137, 137, 137,   3, 133,   2,
        137, 137,   5, 137,   3, 133,   2, 131,
          4,   5, 137,   3, 133,   2, 131, 131,
        135,   4,   3, 133,   2, 131, 131, 131,
        133,   3, 133,   2, 131, 131, 131, 131,
      }
  },
  {
    60,
      {
          6,   4,   4,   4,   5,   4,   6,   5,
          5,   6,   9,   6,   5,   6,   9,  11,
          8,   6,   6,   8,  11,  12,  10,  10,
         11,  10,  10,  12,  16,  12,  12,  12,
         12,  12,  12,  16,  12,  14,  14,  16,
         14,  14,  12,  18,  18,  21,  21,  18,
         18,  26,  26,  26,  26,  26,  32,  32,
         32,  32,  32,  32,  32,  32,  32,  32,
      },
      {
         11,  16,  16,  11,   7,   6, 139,   4,
         16,  13,  13,  11,   8, 141, 139, 139,
         16,  13,  13,  11, 141, 139, 137, 135,
         11,  11,  11,   6, 139, 137, 135, 133,
          7,   8, 141, 139,   4,   3, 133,   2,
          6, 141, 139, 137,   3, 133,   2,   2,
        139, 139, 137, 135, 133,   2,   2,   2,
          4, 139, 135, 133,   2,   2,   2,   2,
      },
      {
          7,   7,   7,  13,  12,  13,  26,  16,
         16,  26,  26,  21,  16,  21,  26,  32,
         32,  32,  32,  32,  32,  32,  32,  32,
         32,  32,  32,  32,  32,  32,  32,  32,
         32,  32,  32,  32,  32,  32,  32,  32,
         32,  32,  32,  32,  32,  32,  32,  32,
         32,  32,  32,  32,  32,  32,  32,  32,
         32,  32,  32,  32,  32,  32,  32,  32,
      },
      {
          9,   9,   5, 133, 133,   2,   2,   2,
          9, 139,   4,   3,   2,   2,   2,   2,
          5,   4,   4,   2,   2,   2,   2,   2,
        133,   3,   2,   2,   2,   2,   2,   2,
        133,   2,   2,   2,   2,   2,   2,   2,
          2,   2,   2,   2,   2,   2,   2,   2,
          2,   2,   2,   2,   2,   2,   2,   2,
          2,   2,   2,   2,   2,   2,   2,   2,
      }
  },
  {
    70,
      {
          4,   3,   3,   3,   3,   3,   4,   3,
          3,   4,   6,   4,   3,   4,   6,   7,
          5,   4,   4,   5,   7,   8,   6,   6,
          7,   6,   6,   8,  10,   8,   9,   9,
          9,   9,   8,  10,  10,  12,  12,  12,
         12,  12,  10,  12,  12,  13,  13,  12,
         12,  16,  16,  16,  16,  16,  21,  21,
         21,  21,  21,  21,  21,  21,  21,  21,
      },
      {
         16,  21,  21,  16,  11,   9,   8, 141,
         21,  21,  21,  16,  13,  11,   8, 141,
         21,  21,  21,  16,  11,   7, 139, 139,
         16,  16,  16,   9,   7, 139, 139,   4,
         11,  13,  11,   7, 139,   5,   4,   3,
          9,  11,   7, 139,   5,   4,   3,   3,
          8,   8, 139, 139,   4,   3,   3,   3,
        141, 141, 139,   4,   3,   3,   3,   3,
      },
      {
          4,   5,   5,   8,   7,   8,  14,  10,
         10,  14,  21,  14,  14,  14,  21,  21,
         21,  21,  21,  21,  21,  21,  21,  21,
         21,  21,  21,  21,  21,  21,  21,  21,
         21,  21,  21,  21,  21,  21,  21,  21,
         21,  21,  21,  21,  21,  21,  21,  21,
         21,  21,  21,  21,  21,  21,  21,  21,
         21,  21,  21,  21,  21,  21,  21,  21,
      },
      {
         16,  13,   8, 137,   3,   3,   3,   3,
         13,   9, 141, 137,   3,   3,   3,   3,
          8, 141, 137,   3,   3,   3,   3,   3,
        137, 137,   3,   3,   3,   3,   3,   3,
          3,   3,   3,   3,   3,   3,   3,   3,
          3,   3,   3,   3,   3,   3,   3,   3,
          3,   3,   3,   3,   3,   3,   3,   3,
          3,   3,   3,   3,   3,   3,   3,   3,
      }
  },
  {
    80,
      {
          2,   2,   2,   2,   2,   2,   2,   2,
          2,   2,   3,   2,   2,   2,   3,   4,
          3,   2,   2,   3,   4,   5,   4,   4,
          4,   4,   4,   5,   6,   5,   5,   5,
          5,   5,   5,   6,   6,   7,   7,   8,
          7,   7,   6,   9,   9,  10,  10,   9,
          9,  12,  12,  12,  12,  12,  12,  12,
         12,  12,  12,  12,  12,  12,  12,  12,
      },
      {
         32,  32,  32,  32,  21,  16,  13,  11,
         32,  32,  32,  32,  21,  16,  13,  11,
         32,  32,  32,  32,  16,  13,   9,   7,
         32,  32,  32,  16,  13,   9,   7, 139,
         21,  21,  16,  13,   8, 141, 139, 139,
         16,  16,  13,   9, 141, 139, 139, 139,
         13,  13,   9,   7, 139, 139, 139, 139,
         11,  11,   7, 139, 139, 139, 139, 139,
      },
      {
          3,   3,   3,   5,   4,   5,   9,   6,
          6,   9,  13,  11,   9,  11,  13,  14,
         14,  14,  14,  14,  14,  14,  12,  12,
         12,  12,  12,  14,  14,  12,  12,  12,
         12,  12,  12,  14,  12,  12,  12,  12,
         12,  12,  12,  12,  12,  12,  12,  12,
         12,  12,  12,  12,  12,  12,  12,  12,
         12,  12,  12,  12,  12,  12,  12,  12,
      },
      {
         21,  21,  13,   7,   5, 137, 137, 137,
         21,  16,  11,   6, 137, 139, 139, 139,
         13,  11,   7, 137, 139, 139, 139, 139,
          7,   6, 137, 139, 139, 139, 139, 139,
          5, 137, 139, 139, 139, 139, 139, 139,
        137, 139, 139, 139, 139, 139, 139, 139,
        137, 139, 139, 139, 139, 139, 139, 139,
        137, 139, 139, 139, 139, 139, 139, 139,
      }
  },
  {
    90,
      {
          1,   1,   1,   1,   1,   1,   1,   1,
          1,   1,   2,   1,   1,   1,   2,   2,
          2,   1,   1,   2,   2,   2,   2,   2,
          2,   2,   2,   2,   3,   2,   3,   3,
          3,   3,   2,   3,   3,   4,   4,   4,
          4,   4,   3,   5,   5,   5,   5,   5,
          5,   7,   7,   7,   7,   7,   8,   8,
          8,   8,   8,   8,   8,   8,   8,   8,
      },
      {
         64,  64,  64,  64,  32,  32,  32,  21,
         64,  64,  64,  64,  32,  32,  32,  21,
         64,  64,  64,  64,  32,  21,  16,  13,
         64,  64,  64,  32,  21,  16,  13,   9,
         32,  32,  32,  21,  16,  13,   9,   8,
         32,  32,  21,  16,  13,   9,   8,   8,
         32,  32,  16,  13,   9,   8,   8,   8,
         21,  21,  13,   9,   8,   8,   8,   8,
      },
      {
          1,   1,   1,   2,   2,   2,   5,   3,
          3,   5,   7,   5,   4,   5,   7,   8,
          8,   8,   8,   8,   8,   8,   8,   8,
          8,   8,   8,   8,   8,   8,   8,   8,
          8,   8,   8,   8,   8,   8,   8,   8,
          8,   8,   8,   8,   8,   8,   8,   8,
          8,   8,   8,   8,   8,   8,   8,   8,
          8,   8,   8,   8,   8,   8,   8,   8,
      },
      {
         64,  64,  32,  13,   9,   8,   8,   8,
         64,  32,  21,  13,   8,   8,   8,   8,
         32,  21,  16,   8,   8,   8,   8,   8,
         13,  13,   8,   8,   8,   8,   8,   8,
          9,   8,   8,   8,   8,   8,   8,   8,
          8,   8,   8,   8,   8,   8,   8,   8,
          8,   8,   8,   8,   8,   8,   8,   8,
          8,   8,   8,   8,   8,   8,   8,   8,
      }
  },
  {
    100,
      {
          1,   1,   1,   1,   1,   1,   1,   1,
          1,   1,   1,   1,   1,   1,   1,   1,
          1,   1,   1,   1,   1,   1,   1,   1,
          1,   1,   1,   1,   1,   1,   1,   1,
          1,   1,   1,   1,   1,   1,   1,   1,
          1,   1,   1,   2,   2,   2,   2,   2,
          2,   2,   2,   2,   2,   2,   3,   3,
          3,   3,   3,   3,   3,   3,   3,   3,
      },
      {
         64,  64,  64,  64,  64,  64,  64,  64,
         64,  64,  64,  64,  64,  64,  64,  64,
         64,  64,  64,  64,  64,  64,  64,  32,
         64,  64,  64,  64,  64,  64,  32,  32,
         64,  64,  64,  64,  64,  32,  32,  21,
         64,  64,  64,  64,  32,  32,  21,  21,
         64,  64,  64,  32,  32,  21,  21,  21,
         64,  64,  32,  32,  21,  21,  21,  21,
      },
      {
          1,   1,   1,   1,   1,   1,   1,   1,
          1,   1,   2,   2,   1,   2,   2,   3,
          3,   3,   3,   3,   3,   3,   3,   3,
          3,   3,   3,   3,   3,   3,   3,   3,
          3,   3,   3,   3,   3,   3,   3,   3,
          3,   3,   3,   3,   3,   3,   3,   3,
          3,   3,   3,   3,   3,   3,   3,   3,
          3,   3,   3,   3,   3,   3,   3,   3,
      },
      {
         64,  64,  64,  64,  32,  21,  21,  21,
         64,  64,  64,  32,  21,  21,  21,  21,
         64,  64,  64,  21,  21,  21,  21,  21,
         64,  32,  21,  21,  21,  21,  21,  21,
         32,  21,  21,  21,  21,  21,  21,  21,
         21,  21,  21,  21,  21,  21,  21,  21,
         21,  21,  21,  21,  21,  21,  21,  21,
         21,  21,  21,  21,  21,  21,  21,  21,
      }
  }
};

#define NR_JPGSETTING_TBL \
        (sizeof(g_isx019_jpg_quality) / sizeof(isx019_fpga_jpg_quality_t))

static const int32_t g_isx019_colorfx[] =
{
  IMGSENSOR_COLORFX_NONE,
  IMGSENSOR_COLORFX_BW,
  IMGSENSOR_COLORFX_VIVID,
};

#define NR_COLORFX (sizeof(g_isx019_colorfx) / sizeof(int32_t))

static const int32_t g_isx019_wbmode[] =
{
  IMGSENSOR_WHITE_BALANCE_AUTO,
  IMGSENSOR_WHITE_BALANCE_INCANDESCENT,
  IMGSENSOR_WHITE_BALANCE_FLUORESCENT,
  IMGSENSOR_WHITE_BALANCE_DAYLIGHT,
  IMGSENSOR_WHITE_BALANCE_CLOUDY,
  IMGSENSOR_WHITE_BALANCE_SHADE,
};

#define NR_WBMODE (sizeof(g_isx019_wbmode) / sizeof(int32_t))

static const int32_t g_isx019_iso[] =
{
  1000,     /* ISO1 */
  1200,     /* ISO1.2 */
  1600,     /* ISO1.6 */
  2000,     /* ISO2 */
  2500,     /* ISO2.5 */
  3000,     /* ISO3 */
  4000,     /* ISO4 */
  5000,     /* ISO5 */
  6000,     /* ISO6 */
  8000,     /* ISO8 */
  10000,    /* ISO10 */
  12000,    /* ISO12 */
  16000,    /* ISO16 */
  20000,    /* ISO20 */
  25000,    /* ISO25 */
  32000,    /* ISO32 */
  40000,    /* ISO40 */
  50000,    /* ISO50 */
  64000,    /* ISO64 */
  80000,    /* ISO80 */
  100000,   /* ISO100 */
  125000,   /* ISO125 */
  160000,   /* ISO160 */
  200000,   /* ISO200 */
  250000,   /* ISO250 */
  320000,   /* ISO320 */
  400000,   /* ISO400 */
  500000,   /* ISO500 */
  640000,   /* ISO640 */
  800000,   /* ISO800 */
  1000000,  /* ISO1000 */
  1250000,  /* ISO1250 */
  1600000,  /* ISO1600 */
  2000000,  /* ISO2000 */
  2500000,  /* ISO2500 */
  3200000,  /* ISO3200 */
  4000000,  /* ISO4000 */
  5000000,  /* ISO5000 */
};

#define NR_ISO (sizeof(g_isx019_iso) / sizeof(int32_t))

/* Gain values corresponding to each element of g_isx019_iso table.
 * This needs to have the same size as g_isx019_iso.
 */

static const uint8_t g_isx019_gain[] =
{
  1,   /* gain for ISO1 */
  2,   /* gain for ISO1.2 */
  3,   /* gain for ISO1.6 */
  4,   /* gain for ISO2 */
  5,   /* gain for ISO2.5 */
  6,   /* gain for ISO3 */
  7,   /* gain for ISO4 */
  8,   /* gain for ISO5 */
  9,   /* gain for ISO6 */
  10,  /* gain for ISO8 */
  11,  /* gain for ISO10 */
  12,  /* gain for ISO12 */
  13,  /* gain for ISO16 */
  14,  /* gain for ISO20 */
  15,  /* gain for ISO25 */
  16,  /* gain for ISO32 */
  17,  /* gain for ISO40 */
  18,  /* gain for ISO50 */
  19,  /* gain for ISO64 */
  20,  /* gain for ISO80 */
  21,  /* gain for ISO100 */
  22,  /* gain for ISO125 */
  23,  /* gain for ISO160 */
  24,  /* gain for ISO200 */
  25,  /* gain for ISO250 */
  26,  /* gain for ISO320 */
  27,  /* gain for ISO400 */
  28,  /* gain for ISO500 */
  29,  /* gain for ISO640 */
  30,  /* gain for ISO800 */
  31,  /* gain for ISO1000 */
  32,  /* gain for ISO1250 */
  33,  /* gain for ISO1600 */
  34,  /* gain for ISO2000 */
  35,  /* gain for ISO2500 */
  36,  /* gain for ISO3200 */
  37,  /* gain for ISO4000 */
  38,  /* gain for ISO5000 */
};

static const int32_t g_isx019_iso_auto[] =
{
  IMGSENSOR_ISO_SENSITIVITY_MANUAL,
  IMGSENSOR_ISO_SENSITIVITY_AUTO,
};

#define NR_ISO_AUTO (sizeof(g_isx019_iso_auto) / sizeof(int32_t))

static const int32_t g_isx019_metering[] =
{
  IMGSENSOR_EXPOSURE_METERING_AVERAGE,
  IMGSENSOR_EXPOSURE_METERING_CENTER_WEIGHTED,
  IMGSENSOR_EXPOSURE_METERING_SPOT,
  IMGSENSOR_EXPOSURE_METERING_MATRIX,
};

#define NR_METERING (sizeof(g_isx019_metering) / sizeof(int32_t))

static const int32_t g_isx019_ae[] =
{
  IMGSENSOR_EXPOSURE_AUTO,
  IMGSENSOR_EXPOSURE_MANUAL,
};

#define NR_AE (sizeof(g_isx019_ae) / sizeof(int32_t))

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int fpga_i2c_write(FAR isx019_dev_t *priv, uint8_t addr,
                          FAR const void *data, uint8_t size)
{
  static uint8_t buf[FPGA_I2C_REGSIZE_MAX + FPGA_I2C_REGADDR_LEN];
  int ret;

  DEBUGASSERT(size <= FPGA_I2C_REGSIZE_MAX);

  priv->i2c_cfg.address = FPGA_I2C_SLVADDR;

  nxmutex_lock(&priv->i2c_lock);
  buf[FPGA_I2C_OFFSET_ADDR] = addr;
  memcpy(&buf[FPGA_I2C_OFFSET_WRITEDATA], data, size);
  ret = i2c_write(priv->i2c,
                  &priv->i2c_cfg,
                  buf,
                  size + FPGA_I2C_REGADDR_LEN);
  nxmutex_unlock(&priv->i2c_lock);

  return ret;
}

static int fpga_i2c_read(FAR isx019_dev_t *priv, uint8_t addr,
                         FAR void *data, uint8_t size)
{
  int ret;

  DEBUGASSERT(size <= FPGA_I2C_REGSIZE_MAX);

  priv->i2c_cfg.address   = FPGA_I2C_SLVADDR;

  nxmutex_lock(&priv->i2c_lock);
  ret = i2c_write(priv->i2c,
                  &priv->i2c_cfg,
                  &addr,
                  FPGA_I2C_REGADDR_LEN);
  if (ret >= 0)
    {
      ret = i2c_read(priv->i2c, &priv->i2c_cfg, data, size);
    }

  nxmutex_unlock(&priv->i2c_lock);
  return ret;
}

static void fpga_activate_setting(FAR isx019_dev_t *priv)
{
  uint8_t regval = FPGA_ACTIVATE_REQUEST;
  fpga_i2c_write(priv, FPGA_ACTIVATE, &regval, 1);

  while (1)
    {
      fpga_i2c_read(priv, FPGA_ACTIVATE, &regval, 1);
      if (regval == 0)
        {
          break;
        }
    }
}

static uint8_t calc_isx019_chksum(FAR const uint8_t *data, uint8_t len)
{
  int i;
  uint8_t chksum = 0;

  for (i = 0; i < len; i++)
    {
      /* ISX019 checksum is lower 8bit of addition result.
       * So, no problem even if overflow occur.
       */

      chksum += data[i];
    }

  return chksum;
}

static bool validate_isx019_chksum(FAR const uint8_t *data, uint8_t len)
{
  uint8_t chksum;

  chksum = calc_isx019_chksum(data, len - 1);

  return data[len - 1] == chksum;
}

static int recv_write_response(FAR isx019_dev_t *priv,
                               FAR const struct i2c_config_s *config)
{
  int ret;
  uint8_t buf[ISX019_I2C_WRRES_TOTALLEN];

  ret = i2c_read(priv->i2c, config, buf, sizeof(buf));
  if (ret < 0)
    {
      return ret;
    }

  if ((buf[ISX019_I2C_OFFSET_TOTALLEN] != ISX019_I2C_WRRES_TOTALLEN) ||
      (buf[ISX019_I2C_OFFSET_CMDNUM]   != 1) ||
      (buf[ISX019_I2C_OFFSET_CMDLEN]   != ISX019_I2C_WRRES_LEN) ||
      (buf[ISX019_I2C_OFFSET_STS]      != ISX019_I2C_STS_OK) ||
      !validate_isx019_chksum(buf, ISX019_I2C_WRRES_TOTALLEN))
    {
      return -EPROTO;
    }

  return OK;
}

static int recv_read_response(FAR isx019_dev_t *priv,
                              FAR const struct i2c_config_s *config,
                              FAR void *data,
                              uint8_t size)
{
  int ret;
  uint8_t buf[ISX019_I2C_WRREQ_TOTALLEN(ISX019_I2C_REGSIZE_MAX)] =
    {
      0
    };

  DEBUGASSERT(size <= ISX019_I2C_REGSIZE_MAX);

  ret = i2c_read(priv->i2c,
                 config, buf, ISX019_I2C_RDRES_TOTALLEN(size));
  if (ret < 0)
    {
      return ret;
    }

  if ((buf[ISX019_I2C_OFFSET_TOTALLEN] != ISX019_I2C_RDRES_TOTALLEN(size)) ||
      (buf[ISX019_I2C_OFFSET_CMDNUM]   != 1) ||
      (buf[ISX019_I2C_OFFSET_CMDLEN]   != ISX019_I2C_RDRES_LEN(size)) ||
      (buf[ISX019_I2C_OFFSET_STS]      != ISX019_I2C_STS_OK) ||
      !validate_isx019_chksum(buf, ISX019_I2C_RDRES_TOTALLEN(size)))
    {
      return -EPROTO;
    }

  memcpy(data, &buf[ISX019_I2C_OFFSET_READDATA], size);

  return OK;
}

static int send_write_cmd(FAR isx019_dev_t *priv,
                          FAR const struct i2c_config_s *config,
                          uint8_t cat,
                          uint16_t addr,
                          FAR const void *data,
                          uint8_t size)
{
  int len;
  uint8_t buf[ISX019_I2C_WRREQ_TOTALLEN(ISX019_I2C_REGSIZE_MAX)] =
    {
      0
    };

  DEBUGASSERT(size <= ISX019_I2C_REGSIZE_MAX);

  buf[ISX019_I2C_OFFSET_TOTALLEN]  = ISX019_I2C_WRREQ_TOTALLEN(size);
  buf[ISX019_I2C_OFFSET_CMDNUM]    = 1;
  buf[ISX019_I2C_OFFSET_CMDLEN]    = ISX019_I2C_WRREQ_LEN(size);
  buf[ISX019_I2C_OFFSET_CMD]       = ISX019_I2C_CMD_WRITE;
  buf[ISX019_I2C_OFFSET_CATEGORY]  = cat;
  buf[ISX019_I2C_OFFSET_ADDRESS_H] = addr >> 8;
  buf[ISX019_I2C_OFFSET_ADDRESS_L] = addr & 0xff;

  memcpy(&buf[ISX019_I2C_OFFSET_WRITEDATA], data, size);

  len = ISX019_I2C_OFFSET_WRITEDATA + size;
  buf[len] = calc_isx019_chksum(buf, len);
  len++;

  return i2c_write(priv->i2c, config, buf, len);
}

static int isx019_i2c_write(FAR isx019_dev_t *priv,
                            uint8_t cat,
                            uint16_t addr,
                            FAR const void *data,
                            uint8_t size)
{
  int ret;

  DEBUGASSERT(size <= ISX019_I2C_REGSIZE_MAX);

  priv->i2c_cfg.address   = ISX019_I2C_SLVADDR;

  nxmutex_lock(&priv->i2c_lock);

  ret = send_write_cmd(priv, &priv->i2c_cfg, cat, addr, data, size);
  if (ret == OK)
    {
      ret = recv_write_response(priv, &priv->i2c_cfg);
    }

  nxmutex_unlock(&priv->i2c_lock);
  return ret;
}

static int send_read_cmd(FAR isx019_dev_t *priv,
                         FAR const struct i2c_config_s *config,
                         uint8_t cat,
                         uint16_t addr,
                         uint8_t size)
{
  int ret;
  int len;
  uint8_t buf[ISX019_I2C_RDREQ_TOTALLEN] =
    {
      0
    };

  buf[ISX019_I2C_OFFSET_TOTALLEN]  = ISX019_I2C_RDREQ_TOTALLEN;
  buf[ISX019_I2C_OFFSET_CMDNUM]    = 1;
  buf[ISX019_I2C_OFFSET_CMDLEN]    = ISX019_I2C_RDREQ_LEN;
  buf[ISX019_I2C_OFFSET_CMD]       = ISX019_I2C_CMD_READ;
  buf[ISX019_I2C_OFFSET_CATEGORY]  = cat;
  buf[ISX019_I2C_OFFSET_ADDRESS_H] = addr >> 8;
  buf[ISX019_I2C_OFFSET_ADDRESS_L] = addr & 0xff;
  buf[ISX019_I2C_OFFSET_READSIZE]  = size;

  len = ISX019_I2C_OFFSET_READSIZE + 1;
  buf[len] = calc_isx019_chksum(buf, len);
  len++;

  ret = i2c_write(priv->i2c, config, buf, len);
  return ret;
}

static int isx019_i2c_read(FAR isx019_dev_t *priv,
                           uint8_t cat,
                           uint16_t addr,
                           FAR void *data,
                           uint8_t size)
{
  int ret;

  DEBUGASSERT(size <= ISX019_I2C_REGSIZE_MAX);

  priv->i2c_cfg.address   = ISX019_I2C_SLVADDR;

  nxmutex_lock(&priv->i2c_lock);

  ret = send_read_cmd(priv, &priv->i2c_cfg, cat, addr, size);
  if (ret == OK)
    {
      ret = recv_read_response(priv, &priv->i2c_cfg, data, size);
    }

  nxmutex_unlock(&priv->i2c_lock);
  return ret;
}

static void fpga_init(FAR isx019_dev_t *priv)
{
  uint8_t regval;

  regval = FPGA_RESET_ENABLE;
  fpga_i2c_write(priv, FPGA_RESET, &regval, 1);
  regval = FPGA_DATA_OUTPUT_STOP;
  fpga_i2c_write(priv, FPGA_DATA_OUTPUT, &regval, 1);
  fpga_activate_setting(priv);
  regval = FPGA_RESET_RELEASE;
  fpga_i2c_write(priv, FPGA_RESET, &regval, 1);
  fpga_activate_setting(priv);
}

static int set_drive_mode(FAR isx019_dev_t *priv)
{
  uint8_t drv[] =
    {
#ifdef CONFIG_VIDEO_ISX019_DOL2
      DOL2_30FPS_SENS, DOL2_30FPS_POST, DOL2_30FPS_SENSPOST, DOL2_30FPS_IO
#else
      DOL3_30FPS_SENS, DOL3_30FPS_POST, DOL3_30FPS_SENSPOST, DOL3_30FPS_IO
#endif
    };

  nxsig_usleep(TRANSITION_TIME_TO_STARTUP);

  isx019_i2c_write(priv, CAT_CONFIG, MODE_SENSSEL,      &drv[INDEX_SENS], 1);
  isx019_i2c_write(priv, CAT_CONFIG, MODE_POSTSEL,      &drv[INDEX_POST], 1);
  isx019_i2c_write(priv,
    CAT_CONFIG, MODE_SENSPOST_SEL, &drv[INDEX_SENSPOST], 1);

  nxsig_usleep(TRANSITION_TIME_TO_STREAMING);

  return OK;
}

static bool try_repeat(int sec, int usec, FAR isx019_dev_t *priv,
                       CODE int (*trial_func)(FAR isx019_dev_t *))
{
  int ret;
  struct timespec start;
  struct timespec now;
  struct timespec delta;

  ret = clock_systime_timespec(&start);
  if (ret < 0)
    {
      return false;
    }

  while (1)
    {
      ret = trial_func(priv);
      if (ret >= 0)
        {
          return true;
        }
      else
        {
          ret = clock_systime_timespec(&now);
          if (ret < 0)
            {
              return false;
            }

          clock_timespec_subtract(&now, &start, &delta);
          if ((delta.tv_sec > sec) ||
              ((delta.tv_sec == sec) &&
               (delta.tv_nsec > (usec * NSEC_PER_USEC))))
            {
              break;
            }
        }
    };

  return false;
}

static int try_isx019_i2c(FAR isx019_dev_t *priv)
{
  uint8_t buf;
  return isx019_i2c_read(priv, CAT_SYSCOM, DEVSTS, &buf, 1);
}

static int try_fpga_i2c(FAR isx019_dev_t *priv)
{
  uint8_t buf;
  return fpga_i2c_read(priv, FPGA_VERSION, &buf, 1);
}

static void power_on(FAR isx019_dev_t *priv)
{
  priv->i2c = board_isx019_initialize();
  priv->i2c_cfg.frequency = ISX019_I2C_FREQUENCY;
  priv->i2c_cfg.addrlen   = ISX019_I2C_SLVADDR_LEN;
  board_isx019_power_on();
  board_isx019_release_reset();
}

static void power_off(FAR isx019_dev_t *priv)
{
  board_isx019_set_reset();
  board_isx019_power_off();
  board_isx019_uninitialize(priv->i2c);
}

static bool isx019_is_available(FAR struct imgsensor_s *sensor)
{
  FAR isx019_dev_t *priv = (FAR isx019_dev_t *)sensor;
  bool ret;

  power_on(priv);

  /* Try to access via I2C
   * about both ISX019 image sensor and FPGA.
   */

  ret = false;
  if (try_repeat(ISX019_ACCESSIBLE_WAIT_SEC,
                ISX019_ACCESSIBLE_WAIT_USEC,
                priv, try_isx019_i2c))
    {
      if (try_repeat(FPGA_ACCESSIBLE_WAIT_SEC,
                     FPGA_ACCESSIBLE_WAIT_USEC,
                     priv, try_fpga_i2c))
        {
          ret = true;
        }
    }

  power_off(priv);

  return ret;
}

static int32_t get_value32(FAR isx019_dev_t *priv, uint32_t id)
{
  imgsensor_value_t val;
  isx019_get_value(&priv->sensor, id, 0, &val);
  return val.value32;
}

static void store_default_value(FAR isx019_dev_t *priv)
{
  FAR isx019_default_value_t *def = &priv->default_value;

  def->brightness   = get_value32(priv, IMGSENSOR_ID_BRIGHTNESS);
  def->contrast     = get_value32(priv, IMGSENSOR_ID_CONTRAST);
  def->saturation   = get_value32(priv, IMGSENSOR_ID_SATURATION);
  def->hue          = get_value32(priv, IMGSENSOR_ID_HUE);
  def->awb          = get_value32(priv, IMGSENSOR_ID_AUTO_WHITE_BALANCE);
  def->gamma        = get_value32(priv, IMGSENSOR_ID_GAMMA);
  def->ev           = get_value32(priv, IMGSENSOR_ID_EXPOSURE);
  def->hflip_video  = get_value32(priv, IMGSENSOR_ID_HFLIP_VIDEO);
  def->vflip_video  = get_value32(priv, IMGSENSOR_ID_VFLIP_VIDEO);
  def->hflip_still  = get_value32(priv, IMGSENSOR_ID_HFLIP_STILL);
  def->vflip_still  = get_value32(priv, IMGSENSOR_ID_VFLIP_STILL);
  def->sharpness    = get_value32(priv, IMGSENSOR_ID_SHARPNESS);
  def->ae           = get_value32(priv, IMGSENSOR_ID_EXPOSURE_AUTO);
  def->wbmode       = get_value32(priv, IMGSENSOR_ID_AUTO_N_PRESET_WB);
  def->hdr          = get_value32(priv, IMGSENSOR_ID_WIDE_DYNAMIC_RANGE);
  def->iso_auto     = get_value32(priv, IMGSENSOR_ID_ISO_SENSITIVITY_AUTO);
  def->meter        = get_value32(priv, IMGSENSOR_ID_EXPOSURE_METERING);
  def->spot_pos     = get_value32(priv, IMGSENSOR_ID_SPOT_POSITION);
  def->threealock   = get_value32(priv, IMGSENSOR_ID_3A_LOCK);
  def->jpgquality   = get_value32(priv, IMGSENSOR_ID_JPEG_QUALITY);
}

static int isx019_init(FAR struct imgsensor_s *sensor)
{
  FAR isx019_dev_t *priv = (FAR isx019_dev_t *)sensor;
  uint32_t clk;

  power_on(priv);
  set_drive_mode(priv);
  fpga_init(priv);
  initialize_wbmode(priv);
  initialize_jpg_quality(priv);

  /* Set initial gamma value for getting current value API. */

  priv->gamma = 1000;

  /* Because store_default_value() needs the clock ratio,
   * clock_ratio has to be calculated first.
   */

  clk = board_isx019_get_master_clock();
  priv->clock_ratio = (float)clk / ISX019_STANDARD_MASTER_CLOCK;
  store_default_value(priv);

  /* Store initial HUE value for getting current value API. */

  priv->hue = priv->default_value.hue;

  return OK;
}

static int isx019_uninit(FAR struct imgsensor_s *sensor)
{
  FAR isx019_dev_t *priv = (FAR isx019_dev_t *)sensor;

  power_off(priv);
  return OK;
}

static FAR const char *isx019_get_driver_name(FAR struct imgsensor_s *sensor)
{
#ifdef CONFIG_VIDEO_ISX019_NAME_WITH_VERSION
  FAR isx019_dev_t *priv = (FAR isx019_dev_t *)sensor;
  static char name[20];
  uint8_t f_ver = 0;
  uint16_t is_ver = 0;

  isx019_i2c_read(priv, CAT_VERSION, ROM_VERSION, &is_ver, 2);
  fpga_i2c_read(priv, FPGA_VERSION, &f_ver, 1);
  snprintf(name, sizeof(name), "ISX019 v%04x_%02d", is_ver, f_ver);

  return name;
#else
  return "ISX019";
#endif
}

static int validate_format(int nr_fmt, FAR imgsensor_format_t *fmt)
{
  int ret;

  uint16_t main_w;
  uint16_t main_h;
  uint16_t sub_w;
  uint16_t sub_h;

  if ((nr_fmt < 1) || (nr_fmt > 2))
    {
      return -EINVAL;
    }

  if (fmt == NULL)
    {
      return -EINVAL;
    }

  main_w = fmt[IMGSENSOR_FMT_MAIN].width;
  main_h = fmt[IMGSENSOR_FMT_MAIN].height;

  switch (fmt[IMGSENSOR_FMT_MAIN].pixelformat)
    {
      case IMGSENSOR_PIX_FMT_UYVY:             /* YUV 4:2:2 */
      case IMGSENSOR_PIX_FMT_RGB565:           /* RGB565 */
      case IMGSENSOR_PIX_FMT_JPEG:             /* JPEG */
      case IMGSENSOR_PIX_FMT_JPEG_WITH_SUBIMG: /* JPEG + sub image */

        if (!VALIDATE_FRAMESIZE(main_w, main_h))
          {
            ret = -EINVAL;
            break;
          }

        if (nr_fmt > 1)
          {
            sub_w = fmt[IMGSENSOR_FMT_SUB].width;
            sub_h = fmt[IMGSENSOR_FMT_SUB].height;
            if (!VALIDATE_THUMNAIL_SIZE(main_w, sub_w) ||
                !VALIDATE_THUMNAIL_SIZE(main_h, sub_h))
              {
                ret = -EINVAL;
                break;
              }
          }

        ret = OK;
        break;

      default: /* otherwise */
        ret = -EINVAL;
        break;
    }

  return ret;
}

static int validate_frameinterval(FAR imgsensor_interval_t *interval)
{
  int ret = -EINVAL;

  if (interval == NULL)
    {
      return -EINVAL;
    }

  /* Avoid multiplication overflow */

  if ((interval->denominator * 10) / 10 != interval->denominator)
    {
      return -EINVAL;
    }

  /* Avoid division by zero */

  if (interval->numerator == 0)
    {
      return -EINVAL;
    }

  switch ((interval->denominator * 10) / interval->numerator)
    {
      case 300:  /* 30FPS  */
      case 150:  /* 15FPS  */
      case 100:  /* 10FPS  */
      case 75:   /* 7.5FPS */
        ret = OK;
        break;

      default:  /* otherwise  */
        ret = -EINVAL;
        break;
    }

  return ret;
}

static int isx019_validate_frame_setting(FAR struct imgsensor_s *sensor,
                                         imgsensor_stream_type_t type,
                                         uint8_t nr_fmt,
                                         FAR imgsensor_format_t *fmt,
                                         FAR imgsensor_interval_t *interval)
{
  int ret = OK;

  ret = validate_format(nr_fmt, fmt);
  if (ret != OK)
    {
      return ret;
    }

  return validate_frameinterval(interval);
}

static int activate_flip(FAR isx019_dev_t *priv,
                         imgsensor_stream_type_t type)
{
  uint8_t flip;

  flip = (type == IMGSENSOR_STREAM_TYPE_VIDEO) ?
         priv->flip_video : priv->flip_still;

  return isx019_i2c_write(priv, CAT_CONFIG, REVERSE, &flip, 1);
}

static int activate_clip(FAR isx019_dev_t *priv,
                         imgsensor_stream_type_t type,
                         uint16_t w,
                         uint16_t h)
{
  FAR isx019_rect_t *clip;
  uint8_t size;
  uint8_t top;
  uint8_t left = 0;

  clip = (type == IMGSENSOR_STREAM_TYPE_VIDEO) ?
         &priv->clip_video : &priv->clip_still;

  switch (w)
    {
      case 1280:
        if (clip->width == 640) /* In this case, c_h == 360 */
          {
            size = FPGA_CLIP_640_360;
            top  = clip->top / FPGA_CLIP_UNIT;
            left = clip->left / FPGA_CLIP_UNIT;

            if (h == 720)
              {
                /* Shift (960 - 720) / 2 lines */

                top  += 120 / FPGA_CLIP_UNIT;
              }
          }
        else if (clip->width == 1280)
          {
            /* In this case, clip->height == 720 */

            size = FPGA_CLIP_1280_720;
            top  = clip->top / FPGA_CLIP_UNIT;
            left = clip->left / FPGA_CLIP_UNIT;
          }
        else /* no clip */
          {
            if (h == 720)
              {
                size = FPGA_CLIP_1280_720;

                /* Shift (960 - 720) / 2 lines */

                top = 120 / FPGA_CLIP_UNIT;
              }
            else
              {
                size = FPGA_CLIP_NON;
                top  = 0;
                left = 0;
              }
          }

        break;

      case 640:
        if (clip->width == 640)
          {
            /* In this case, clip->height == 360 */

            size = FPGA_CLIP_640_360;
            top  = clip->top / FPGA_CLIP_UNIT;
            left = clip->left / FPGA_CLIP_UNIT;
          }
        else /* no clip */
          {
            if (h == 360)
              {
                size = FPGA_CLIP_640_360;

                /* Shift (480 - 360) / 2 lines */

                top  = 60 / FPGA_CLIP_UNIT;
              }
            else
              {
                size = FPGA_CLIP_NON;
                top  = 0;
                left = 0;
              }
          }

        break;

      default: /* Otherwise, clear clip setting. */
        size = FPGA_CLIP_NON;
        top  = 0;
        left = 0;

        break;
    }

  fpga_i2c_write(priv, FPGA_CLIP_SIZE, &size, 1);
  fpga_i2c_write(priv, FPGA_CLIP_TOP,  &top, 1);
  fpga_i2c_write(priv, FPGA_CLIP_LEFT, &left, 1);

  return OK;
}

static int isx019_start_capture(FAR struct imgsensor_s *sensor,
                                imgsensor_stream_type_t type,
                                uint8_t nr_fmt,
                                FAR imgsensor_format_t *fmt,
                                FAR imgsensor_interval_t *interval)
{
  FAR isx019_dev_t *priv = (FAR isx019_dev_t *)sensor;
  int ret;
  uint8_t regval = 0;

  ret = isx019_validate_frame_setting(sensor, type, nr_fmt, fmt, interval);
  if (ret != OK)
    {
      return ret;
    }

  ret = activate_flip(priv, type);
  if (ret != OK)
    {
      return ret;
    }

  nxmutex_lock(&priv->fpga_lock);

  /* Update FORMAT_AND_SCALE register of FPGA */

  switch (fmt[IMGSENSOR_FMT_MAIN].pixelformat)
    {
      case IMGSENSOR_PIX_FMT_RGB565:
        regval |= FPGA_FORMAT_RGB;
        break;

      case IMGSENSOR_PIX_FMT_UYVY:
        regval |= FPGA_FORMAT_YUV;
        break;

      case IMGSENSOR_PIX_FMT_JPEG:
        regval |= FPGA_FORMAT_JPEG;
        break;

      default: /* IMGSENSOR_PIX_FMT_JPEG_WITH_SUBIMG */
        if (nr_fmt == 1)
          {
            regval |= FPGA_FORMAT_JPEG;
          }
        else
          {
            regval |= FPGA_FORMAT_THUMBNAIL;
          }

        break;
    }

  switch (fmt[IMGSENSOR_FMT_MAIN].width)
    {
      case 1280:
        regval |= FPGA_SCALE_1280_960;
        break;

      case 640:
        regval |= FPGA_SCALE_640_480;
        break;

      case 320:
        regval |= FPGA_SCALE_320_240;
        break;

      default: /* 160 */
        regval |= FPGA_SCALE_160_120;
        break;
    }

  activate_clip(priv, type,
                fmt[IMGSENSOR_FMT_MAIN].width,
                fmt[IMGSENSOR_FMT_MAIN].height);

  fpga_i2c_write(priv, FPGA_FORMAT_AND_SCALE, &regval, 1);

  /* Update FPS_AND_THUMBNAIL register of FPGA */

  regval = 0;

  if (nr_fmt == 2)
    {
      switch (fmt[IMGSENSOR_FMT_MAIN].width / fmt[IMGSENSOR_FMT_SUB].width)
        {
          case 1 : /* 1/1 */
            regval |= FPGA_THUMBNAIL_SCALE_1_1;
            break;

          case 2 : /* 1/2 */
            regval |= FPGA_THUMBNAIL_SCALE_1_2;
            break;

          case 4 : /* 1/4 */
            regval |= FPGA_THUMBNAIL_SCALE_1_4;
            break;

          default: /* 1/8 */
            regval |= FPGA_THUMBNAIL_SCALE_1_8;
            break;
        }
    }

  switch ((interval->denominator * 10) / interval->numerator)
    {
      case 300:  /* 30FPS */
        regval |= FPGA_FPS_1_1;
        break;

      case 150:  /* 15FPS */
        regval |= FPGA_FPS_1_2;
        break;

      case 100:  /* 10FPS */
        regval |= FPGA_FPS_1_3;
        break;

      case 75:   /* 7.5FPS */
        regval |= FPGA_FPS_1_4;
        break;

      default:   /* otherwise */

        /* It may not come here because the value has already been validated
         * in validate_frameinterval().
         */

        break;
    }

  fpga_i2c_write(priv, FPGA_FPS_AND_THUMBNAIL, &regval, 1);

  regval = FPGA_DATA_OUTPUT_START;
  fpga_i2c_write(priv, FPGA_DATA_OUTPUT, &regval, 1);

  fpga_activate_setting(priv);
  nxmutex_unlock(&priv->fpga_lock);
  priv->stream = type;

  return OK;
}

static int isx019_stop_capture(FAR struct imgsensor_s *sensor,
                               imgsensor_stream_type_t type)
{
  return OK;
}

static int calc_gcm(int a, int b)
{
  int r;

  DEBUGASSERT((a != 0) && (b != 0));

  while ((r = a % b) != 0)
    {
      a = b;
      b = r;
    }

  return b;
}

static int isx019_get_frame_interval(FAR struct imgsensor_s *sensor,
                                     imgsensor_stream_type_t type,
                                     FAR imgsensor_interval_t *interval)
{
  FAR isx019_dev_t *priv = (FAR isx019_dev_t *)sensor;
  uint32_t vtime = VTIME_PER_FRAME;
  uint32_t frame = 1;
  uint8_t  fps   = FPGA_FPS_1_1;
  int decimation = 1;
  int gcm;

  if (interval == NULL)
    {
      return -EINVAL;
    }

  /* ISX019's base frame interval = 1/30. */

  interval->denominator = 30;
  interval->numerator = 1;

  /* ISX019 has the frame extension feature, which automatically
   * exposes longer than one frame in dark environment.
   * The number of extended frame is calculated from V_TIME register,
   * which has the value
   *   VTIME_PER_FRAME + INTERVAL_PER_FRAME * (frame number - 1)
   */

  isx019_i2c_read(priv, CAT_AESOUT, V_TIME, &vtime, 4);
  frame = 1 + (vtime - VTIME_PER_FRAME) / INTERVAL_PER_FRAME;
  interval->numerator *= frame;

  /* Also, consider frame decimation by FPGA.
   * decimation amount is gotten from FPGA register.
   */

  fpga_i2c_read(priv, FPGA_FPS_AND_THUMBNAIL, &fps, 1);
  switch (fps & FPGA_FPS_BITS)
    {
      case FPGA_FPS_1_1:
        decimation = 1;
        break;

      case FPGA_FPS_1_2:
        decimation = 2;
        break;

      case FPGA_FPS_1_3:
        decimation = 3;
        break;

      default: /* FPGA_FPS_1_4 */
        decimation = 4;
        break;
    }

  interval->numerator *= decimation;

  /* Reduce the fraction. */

  gcm = calc_gcm(30, frame * decimation);
  interval->denominator /= gcm;
  interval->numerator   /= gcm;

  return OK;
}

static int isx019_get_supported_value(FAR struct imgsensor_s *sensor,
                                      uint32_t id,
                                      FAR imgsensor_supported_value_t *val)
{
  FAR isx019_dev_t *priv = (FAR isx019_dev_t *)sensor;
  FAR struct isx019_default_value_s *def = &priv->default_value;
  int ret = OK;

  DEBUGASSERT(val);

  switch (id)
    {
      case IMGSENSOR_ID_BRIGHTNESS:
        val->type = IMGSENSOR_CTRL_TYPE_INTEGER;
        SET_RANGE(val->u.range, MIN_BRIGHTNESS, MAX_BRIGHTNESS,
                                STEP_BRIGHTNESS, def->brightness);
        break;

      case IMGSENSOR_ID_CONTRAST:
        val->type = IMGSENSOR_CTRL_TYPE_INTEGER;
        SET_RANGE(val->u.range, MIN_CONTRAST, MAX_CONTRAST,
                                STEP_CONTRAST, def->contrast);
        break;

      case IMGSENSOR_ID_SATURATION:
        val->type = IMGSENSOR_CTRL_TYPE_INTEGER;
        SET_RANGE(val->u.range, MIN_SATURATION, MAX_SATURATION,
                                STEP_SATURATION, def->saturation);
        break;

      case IMGSENSOR_ID_HUE:
        val->type = IMGSENSOR_CTRL_TYPE_INTEGER;
        SET_RANGE(val->u.range, MIN_HUE, MAX_HUE,
                                STEP_HUE, def->hue);
        break;

      case IMGSENSOR_ID_AUTO_WHITE_BALANCE:
        val->type = IMGSENSOR_CTRL_TYPE_BOOLEAN;
        SET_RANGE(val->u.range, MIN_AWB, MAX_AWB,
                                STEP_AWB, def->awb);
        break;

      case IMGSENSOR_ID_GAMMA:
        val->type = IMGSENSOR_CTRL_TYPE_INTEGER;
        SET_RANGE(val->u.range, MIN_GAMMA, MAX_GAMMA,
                                STEP_GAMMA, def->gamma);
        break;

      case IMGSENSOR_ID_EXPOSURE:
        val->type = IMGSENSOR_CTRL_TYPE_INTEGER;
        SET_RANGE(val->u.range, MIN_EXPOSURE, MAX_EXPOSURE,
                                STEP_EXPOSURE, def->ev);
        break;

      case IMGSENSOR_ID_HFLIP_VIDEO:
        val->type = IMGSENSOR_CTRL_TYPE_BOOLEAN;
        SET_RANGE(val->u.range, MIN_HFLIP, MAX_HFLIP,
                                STEP_HFLIP, def->hflip_video);
        break;

      case IMGSENSOR_ID_VFLIP_VIDEO:
        val->type = IMGSENSOR_CTRL_TYPE_BOOLEAN;
        SET_RANGE(val->u.range, MIN_VFLIP, MAX_VFLIP,
                                STEP_VFLIP, def->vflip_video);
        break;

      case IMGSENSOR_ID_HFLIP_STILL:
        val->type = IMGSENSOR_CTRL_TYPE_BOOLEAN;
        SET_RANGE(val->u.range, MIN_HFLIP, MAX_HFLIP,
                                STEP_HFLIP, def->hflip_still);
        break;

      case IMGSENSOR_ID_VFLIP_STILL:
        val->type = IMGSENSOR_CTRL_TYPE_BOOLEAN;
        SET_RANGE(val->u.range, MIN_VFLIP, MAX_VFLIP,
                                STEP_VFLIP, def->hflip_still);
        break;

      case IMGSENSOR_ID_SHARPNESS:
        val->type = IMGSENSOR_CTRL_TYPE_INTEGER;
        SET_RANGE(val->u.range, MIN_SHARPNESS, MAX_SHARPNESS,
                                STEP_SHARPNESS, def->sharpness);
        break;

      case IMGSENSOR_ID_COLORFX:
        val->type = IMGSENSOR_CTRL_TYPE_INTEGER_MENU;
        SET_DISCRETE(val->u.discrete,
                     NR_COLORFX,
                     g_isx019_colorfx,
                     IMGSENSOR_COLORFX_NONE);
        break;

      case IMGSENSOR_ID_EXPOSURE_AUTO:
        val->type = IMGSENSOR_CTRL_TYPE_INTEGER_MENU;
        SET_DISCRETE(val->u.discrete,
                     NR_AE,
                     g_isx019_ae,
                     IMGSENSOR_EXPOSURE_AUTO);
        break;

      case IMGSENSOR_ID_EXPOSURE_ABSOLUTE:
        val->type = IMGSENSOR_CTRL_TYPE_INTEGER;
        SET_RANGE(val->u.range, MIN_EXPOSURETIME, MAX_EXPOSURETIME,
                                STEP_EXPOSURETIME, 0); /* 0 means undefined */
        break;

      case IMGSENSOR_ID_AUTO_N_PRESET_WB:
        val->type = IMGSENSOR_CTRL_TYPE_INTEGER_MENU;
        SET_DISCRETE(val->u.discrete,
                     NR_WBMODE,
                     g_isx019_wbmode,
                     IMGSENSOR_WHITE_BALANCE_AUTO);
        break;

      case IMGSENSOR_ID_WIDE_DYNAMIC_RANGE:
        val->type = IMGSENSOR_CTRL_TYPE_INTEGER;
        SET_RANGE(val->u.range, MIN_HDR, MAX_HDR,
                                STEP_HDR, def->hdr);
        break;

      case IMGSENSOR_ID_ISO_SENSITIVITY:
        val->type = IMGSENSOR_CTRL_TYPE_INTEGER_MENU;
        SET_DISCRETE(val->u.discrete,
                     NR_ISO,
                     g_isx019_iso,
                     0); /* 0 means undefined */
        break;

      case IMGSENSOR_ID_ISO_SENSITIVITY_AUTO:
        val->type = IMGSENSOR_CTRL_TYPE_INTEGER_MENU;
        SET_DISCRETE(val->u.discrete,
                     NR_ISO_AUTO,
                     g_isx019_iso_auto,
                     IMGSENSOR_ISO_SENSITIVITY_AUTO);
        break;

      case IMGSENSOR_ID_EXPOSURE_METERING:
        val->type = IMGSENSOR_CTRL_TYPE_INTEGER_MENU;
        SET_DISCRETE(val->u.discrete,
                     NR_METERING,
                     g_isx019_metering,
                     IMGSENSOR_EXPOSURE_METERING_AVERAGE);
        break;

      case IMGSENSOR_ID_SPOT_POSITION:
        val->type = IMGSENSOR_CTRL_TYPE_INTEGER;
        SET_RANGE(val->u.range, MIN_SPOTPOS, MAX_SPOTPOS,
                                STEP_SPOTPOS, def->spot_pos);
        break;

      case IMGSENSOR_ID_3A_LOCK:
        val->type = IMGSENSOR_CTRL_TYPE_BITMASK;
        SET_RANGE(val->u.range, MIN_3ALOCK, MAX_3ALOCK,
                                STEP_3ALOCK, def->threealock);
        break;

      case IMGSENSOR_ID_3A_PARAMETER:
        val->type = IMGSENSOR_CTRL_TYPE_U8;
        SET_ELEMS(val->u.elems, NRELEM_3APARAM, MIN_3APARAM, MAX_3APARAM,
                                STEP_3APARAM);
        break;

      case IMGSENSOR_ID_3A_STATUS:
        val->type = IMGSENSOR_CTRL_TYPE_INTEGER;
        SET_RANGE(val->u.range, MIN_3ASTATUS,
                                MAX_3ASTATUS, STEP_3ASTATUS,
                                IMGSENSOR_3A_STATUS_AE_OPERATING
                                | IMGSENSOR_3A_STATUS_AWB_OPERATING);
        break;

      case IMGSENSOR_ID_JPEG_QUALITY:
        val->type = IMGSENSOR_CTRL_TYPE_INTEGER;
        SET_RANGE(val->u.range, MIN_JPGQUALITY, MAX_JPGQUALITY,
                                STEP_JPGQUALITY, def->jpgquality);
        break;

      case IMGSENSOR_ID_CLIP_VIDEO:
      case IMGSENSOR_ID_CLIP_STILL:
        val->type = IMGSENSOR_CTRL_TYPE_U32;
        SET_ELEMS(val->u.elems, NRELEM_CLIP, MIN_CLIP, MAX_CLIP,
                                STEP_CLIP);
        break;

      default:
        ret = -EINVAL;
        break;
    }

  return ret;
}

static int32_t not_convert(int32_t val)
{
  return val;
}

static int32_t convert_brightness_is2reg(int32_t val)
{
  return (val * 4);
}

static int32_t convert_brightness_reg2is(int32_t val)
{
  return (val / 4);
}

static int32_t convert_hdr_is2reg(int32_t val)
{
  int32_t ret = AEWDMODE_AUTO;

  switch (val)
    {
      case 0:
        ret = AEWDMODE_NORMAL;
        break;

      case 1:
        ret = AEWDMODE_AUTO;
        break;

      case 2:
        ret = AEWDMODE_HDR;
        break;

      default:
        /* It may not come here because the value has already been validated
         * in validate_value().
         */

        break;
    }

  return ret;
}

static int32_t convert_hdr_reg2is(int32_t val)
{
  int32_t ret;

  switch (val)
    {
      case AEWDMODE_NORMAL:
        ret = 0;
        break;

      case AEWDMODE_AUTO:
        ret = 1;
        break;

      default: /* AEWDMODE_HDR */
        ret = 2;
        break;
    }

  return ret;
}

static convert_t get_reginfo(uint32_t id, bool is_set,
                             FAR isx019_reginfo_t *reg)
{
  convert_t cvrt = NULL;

  DEBUGASSERT(reg);

  switch (id)
    {
      case IMGSENSOR_ID_BRIGHTNESS:
        SET_REGINFO_INT16(reg, CAT_PICTTUNE, UIBRIGHTNESS);
        cvrt = is_set ? convert_brightness_is2reg
                      : convert_brightness_reg2is;
        break;

      case IMGSENSOR_ID_CONTRAST:
        SET_REGINFO_UINT8(reg, CAT_PICTTUNE, UICONTRAST);
        cvrt = not_convert;
        break;

      case IMGSENSOR_ID_SATURATION:
        SET_REGINFO_UINT8(reg, CAT_PICTTUNE, UISATURATION);
        cvrt = not_convert;
        break;

      case IMGSENSOR_ID_EXPOSURE:
        SET_REGINFO_INT8(reg, CAT_AEDGRM, EVSEL);
        cvrt = not_convert;
        break;

      case IMGSENSOR_ID_SHARPNESS:
        SET_REGINFO_UINT8(reg, CAT_PICTTUNE, UISHARPNESS);
        cvrt = not_convert;
        break;

      case IMGSENSOR_ID_WIDE_DYNAMIC_RANGE:
        SET_REGINFO_UINT8(reg, CAT_AEWD, AEWDMODE);
        cvrt = is_set ? convert_hdr_is2reg : convert_hdr_reg2is;
        break;

      default:
        break;
    }

  return cvrt;
}

static int set_hue(FAR isx019_dev_t *priv,
                   imgsensor_value_t val)
{
  int ret;
  int val32 = val.value32 * 90 / 128;

  ret = isx019_i2c_write(priv, CAT_PICTTUNE, UIHUE, &val32, 1);
  if (ret == OK)
    {
      /* Store value before conversion for get_hue(). */

      priv->hue = val.value32;
    }

  return ret;
}

static void set_flip(FAR uint8_t *flip, uint8_t direction, int32_t val)
{
  DEBUGASSERT(flip);

  *flip = (val == 0) ? (*flip & ~direction) : (*flip | direction);
}

static int set_hflip_video(FAR isx019_dev_t *priv,
                           imgsensor_value_t val)
{
  set_flip(&priv->flip_video, H_REVERSE, val.value32);
  if (priv->stream == IMGSENSOR_STREAM_TYPE_VIDEO)
    {
      activate_flip(priv, IMGSENSOR_STREAM_TYPE_VIDEO);
    }

  return OK;
}

static int set_vflip_video(FAR isx019_dev_t *priv,
                           imgsensor_value_t val)
{
  set_flip(&priv->flip_video, V_REVERSE, val.value32);
  if (priv->stream == IMGSENSOR_STREAM_TYPE_VIDEO)
    {
      activate_flip(priv, IMGSENSOR_STREAM_TYPE_VIDEO);
    }

  return OK;
}

static int set_hflip_still(FAR isx019_dev_t *priv,
                           imgsensor_value_t val)
{
  set_flip(&priv->flip_still, H_REVERSE, val.value32);
  if (priv->stream == IMGSENSOR_STREAM_TYPE_STILL)
    {
      activate_flip(priv, IMGSENSOR_STREAM_TYPE_STILL);
    }

  return OK;
}

static int set_vflip_still(FAR isx019_dev_t *priv,
                           imgsensor_value_t val)
{
  set_flip(&priv->flip_still, V_REVERSE, val.value32);
  if (priv->stream == IMGSENSOR_STREAM_TYPE_STILL)
    {
      activate_flip(priv, IMGSENSOR_STREAM_TYPE_STILL);
    }

  return OK;
}

static int set_colorfx(FAR isx019_dev_t *priv,
                       imgsensor_value_t val)
{
  int ret = -EINVAL;
  FAR isx019_default_value_t *def = &priv->default_value;
  int32_t sat;
  int32_t sharp;

  /* ISX019 realize color effects by adjusting saturation and sharpness. */

  switch (val.value32)
    {
      case IMGSENSOR_COLORFX_NONE:

        sat   = def->saturation;
        sharp = def->sharpness;
        break;

      case IMGSENSOR_COLORFX_BW:

        sat   = BW_COLORS_SATURATION;
        sharp = def->sharpness;
        break;

      case IMGSENSOR_COLORFX_VIVID:

        sat   = VIVID_COLORS_SATURATION;
        sharp = VIVID_COLORS_SHARPNESS;
        break;

      default:

        /* It may not come here because the value has already been validated
         * in validate_value().
         */

        break;
    }

  ret = isx019_i2c_write(priv, CAT_PICTTUNE, UISATURATION, &sat, 1);
  if (ret == OK)
    {
      ret = isx019_i2c_write(priv, CAT_PICTTUNE, UISHARPNESS, &sharp, 1);
      if (ret == OK)
        {
          priv->colorfx = val.value32;
        }
    }

  return ret;
}

static int set_ae(FAR isx019_dev_t *priv,
                  imgsensor_value_t val)
{
  uint32_t regval = 0;

  if (val.value32 == IMGSENSOR_EXPOSURE_AUTO)
    {
      regval = 0;
    }
  else
    {
      isx019_i2c_read(priv, CAT_AESOUT, SHT_TIME, &regval, 4);
    }

  return isx019_i2c_write(priv, CAT_CATAE, SHT_PRIMODE, &regval, 4);
}

static int set_exptime(FAR isx019_dev_t *priv,
                       imgsensor_value_t val)
{
  uint32_t regval;

  /* Take into account the master clock and convert unit.
   *   image sensor I/F : 100usec
   *   register         :   1usec
   */

  regval = val.value32 * 100 * priv->clock_ratio;

  return isx019_i2c_write(priv, CAT_CATAE, SHT_PRIMODE, &regval, 4);
}

static int set_awb_hold(FAR isx019_dev_t *priv)
{
  uint8_t mode = AWBMODE_HOLD;
  return isx019_i2c_write(priv, CAT_CATAWB, AWBMODE, &mode, 1);
}

static void initialize_wbmode(FAR isx019_dev_t *priv)
{
  priv->wb_mode = IMGSENSOR_WHITE_BALANCE_AUTO;
}

static int update_wbmode_reg(FAR isx019_dev_t *priv,
                             int32_t val)
{
  /*  AWBMODE     mode0 : auto, mode4 : user defined white balance
   *  AWBUSER_NO  definition number for AWBMODE = mode4
   *  USER0_R, USER1_B : R,B value for AWBUSER_NO = 0
   *  USER1_R, USER1_B : R,B value for AWBUSER_NO = 1
   */

  uint8_t  mode;
  uint16_t r_addr;
  uint16_t b_addr;
  uint16_t r;
  uint16_t b;
  static bool toggle = false;

  if (toggle)
    {
      r_addr = USER0_R;
      b_addr = USER0_B;
      toggle = false;
    }
  else
    {
      r_addr = USER1_R;
      b_addr = USER1_B;
      toggle = true;
    }

  switch (val)
    {
      case IMGSENSOR_WHITE_BALANCE_AUTO:
        mode = AWBMODE_AUTO;
        break;

      case IMGSENSOR_WHITE_BALANCE_INCANDESCENT:
        r = RED_INCANDESCENT;
        b = BLUE_INCANDESCENT;
        mode = AWBMODE_MANUAL;
        break;

      case IMGSENSOR_WHITE_BALANCE_FLUORESCENT:
        r = RED_FLUORESCENT;
        b = BLUE_FLUORESCENT;
        mode = AWBMODE_MANUAL;
        break;

      case IMGSENSOR_WHITE_BALANCE_DAYLIGHT:
        r = RED_DAYLIGHT;
        b = BLUE_DAYLIGHT;
        mode = AWBMODE_MANUAL;
        break;

      case IMGSENSOR_WHITE_BALANCE_CLOUDY:
        r = RED_CLOUDY;
        b = BLUE_CLOUDY;
        mode = AWBMODE_MANUAL;
        break;

      default: /* IMGSENSOR_WHITE_BALANCE_SHADE */
        r = RED_SHADE;
        b = BLUE_SHADE;
        mode = AWBMODE_MANUAL;
        break;
    }

  isx019_i2c_write(priv, CAT_AWB_USERTYPE, r_addr, &r, 2);
  isx019_i2c_write(priv, CAT_AWB_USERTYPE, b_addr, &b, 2);
  isx019_i2c_write(priv, CAT_CATAWB, AWBUSER_NO, &toggle, 1);
  isx019_i2c_write(priv, CAT_CATAWB, AWBMODE, &mode, 1);

  return OK;
}

static bool is_awb_enable(FAR isx019_dev_t *priv)
{
  uint8_t mode = AWBMODE_AUTO;

  isx019_i2c_read(priv, CAT_CATAWB, AWBMODE, &mode, 1);

  return mode != AWBMODE_HOLD;
}

static int set_wbmode(FAR isx019_dev_t *priv,
                      imgsensor_value_t val)
{
  /* Update register only if IMGSENSOR_ID_AUTO_WHITE_BALANCE = 1. */

  if (is_awb_enable(priv))
    {
      update_wbmode_reg(priv, val.value32);
    }

  priv->wb_mode = val.value32;
  return OK;
}

static int set_awb(FAR isx019_dev_t *priv,
                   imgsensor_value_t val)
{
  /* true  -> false : Update register to HOLD
   * false -> true  : Update register
   *                  with IMGSENSOR_ID_AUTO_N_PRESET_WB setting
   * otherwise      : Nothing to do
   */

  if (is_awb_enable(priv))
    {
      if (val.value32 == 0)
        {
          set_awb_hold(priv);
        }
    }
  else
    {
      if (val.value32 == 1)
        {
          update_wbmode_reg(priv, priv->wb_mode);
        }
    }

  return OK;
}

static int set_meter(FAR isx019_dev_t *priv,
                     imgsensor_value_t val)
{
  uint8_t normal;
  uint8_t hdr;

  switch (val.value32)
    {
      case IMGSENSOR_EXPOSURE_METERING_AVERAGE:
        normal = AEWEIGHT_AVERAGE;
        hdr    = AEWEIGHTHDR_AVERAGE;
        break;

      case IMGSENSOR_EXPOSURE_METERING_CENTER_WEIGHTED:
        normal = AEWEIGHT_CENTER;
        hdr    = AEWEIGHTHDR_CENTER;
        break;

      case IMGSENSOR_EXPOSURE_METERING_SPOT:
        normal = AEWEIGHT_SPOT;
        hdr    = AEWEIGHTHDR_SPOT;
        break;

      default: /* IMGSENSOR_EXPOSURE_METERING_MATRIX */
        normal = AEWEIGHT_MATRIX;
        hdr    = AEWEIGHTHDR_MATRIX;
        break;
    }

  isx019_i2c_write(priv, CAT_AUTOCTRL, AEWEIGHTMODE, &normal, 1);
  isx019_i2c_write(priv, CAT_AEWD, AEWEIGHTMODE_WD, &hdr, 1);

  return OK;
}

static void get_current_framesize(FAR isx019_dev_t *priv,
                                  FAR uint16_t *w, FAR uint16_t *h)
{
  uint8_t frmsz;

  DEBUGASSERT(w && h);

  fpga_i2c_read(priv, FPGA_FORMAT_AND_SCALE, &frmsz, 1);

  switch (frmsz & 0xf0)
    {
      case FPGA_SCALE_1280_960:
        *w = 1280;
        *h = 960;
        break;

      case FPGA_SCALE_640_480:
        *w = 640;
        *h = 480;
        break;

      case FPGA_SCALE_320_240:
        *w = 320;
        *h = 240;
        break;

      case FPGA_SCALE_160_120:
        *w = 160;
        *h = 120;
        break;

      default:

        /* It may not come here due to register specification */

        break;
    }
}

static void get_current_clip_setting(FAR isx019_dev_t *priv,
                                     FAR uint16_t *w,
                                     FAR uint16_t *h,
                                     FAR uint16_t *offset_x,
                                     FAR uint16_t *offset_y)
{
  uint8_t sz;
  uint8_t top;
  uint8_t left;

  fpga_i2c_read(priv, FPGA_CLIP_SIZE, &sz, 1);
  fpga_i2c_read(priv, FPGA_CLIP_TOP,  &top, 1);
  fpga_i2c_read(priv, FPGA_CLIP_LEFT, &left, 1);

  *offset_x = left * FPGA_CLIP_UNIT;
  *offset_y = top  * FPGA_CLIP_UNIT;

  switch (sz)
    {
      case FPGA_CLIP_NON:
        *w = 0;
        *h = 0;
        *offset_x = 0;
        *offset_y = 0;
        break;

      case FPGA_CLIP_1280_720:
        *w = 1280;
        *h = 720;
        break;

      case FPGA_CLIP_640_360:
        *w = 640;
        *h = 360;
        break;

      default:

        /* It may not come here due to register specification */

        break;
    }
}

static int calc_spot_position_regval(uint16_t val,
                                     uint16_t basis,
                                     uint16_t sz,
                                     uint16_t offset,
                                     int      split)
{
  int ret;
  int ratio;

  /* Change basis from `sz` to `basis` about `val` and `offset`. */

  ratio = basis / sz;
  ret = val * ratio;
  ret += (offset * FPGA_CLIP_UNIT * ratio);

  return (ret * split) / basis;
}

static int set_spot_position(FAR isx019_dev_t *priv,
                             imgsensor_value_t val)
{
  uint8_t regval;
  uint8_t reg_x;
  uint8_t reg_y;
  uint16_t w;
  uint16_t h;
  uint16_t clip_w;
  uint16_t clip_h;
  uint16_t offset_x;
  uint16_t offset_y;
  uint16_t x = (uint16_t)(val.value32 >> 16);
  uint16_t y = (uint16_t)(val.value32 & 0xffff);
  int split;

  /* Spot position of ISX019 is divided into 9x7 sections.
   * - Horizontal direction is divided into 9 sections.
   * - Vertical  direction is divided into 7 sections.
   * The register value 0 means left top.
   * The register value 62 means right bottom.
   * Then, the following ISX019 board flow.
   * - image sensor output the 1280x960 image
   * - FPGA scale
   * - FPGA clipping
   */

  get_current_framesize(priv, &w, &h);
  if ((x >= w) || (y >= h))
    {
      return -EINVAL;
    }

  get_current_clip_setting(priv, &clip_w, &clip_h, &offset_x, &offset_y);
  if ((clip_w != 0) && (clip_h != 0))
    {
      if ((x >= clip_w) || (y >= clip_h))
        {
          return -EINVAL;
        }
    }

  split = ISX019_SPOT_POSITION_SPLIT_NUM_X;
  reg_x = calc_spot_position_regval(x, 1280, w, offset_x, split);
  split = ISX019_SPOT_POSITION_SPLIT_NUM_Y;
  reg_y = calc_spot_position_regval(y,  960, h, offset_y, split);

  regval = reg_y * ISX019_SPOT_POSITION_SPLIT_NUM_X + reg_x;
  return isx019_i2c_write(priv, CAT_CATAE, SPOT_FRM_NUM, &regval, 1);
}

static int set_3alock(FAR isx019_dev_t *priv,
                      imgsensor_value_t val)
{
  uint8_t regval;

  regval = (val.value32 & IMGSENSOR_LOCK_WHITE_BALANCE) ? AWBMODE_HOLD
                                                        : AWBMODE_AUTO;
  isx019_i2c_write(priv, CAT_CATAWB, AWBMODE, &regval, 1);

  regval = (val.value32 & IMGSENSOR_LOCK_EXPOSURE) ? AEMODE_HOLD
                                                   : AEMODE_AUTO;
  isx019_i2c_write(priv, CAT_CATAE,  AEMODE,  &regval, 1);

  return OK;
}

static int set_3aparameter(FAR isx019_dev_t *priv,
                           imgsensor_value_t val)
{
  uint16_t gain;
  uint8_t regval;

  if (val.p_u8 == NULL)
    {
      return -EINVAL;
    }

  /* Convert unit
   *  GAIN_LEVEL register(accessed in get_3aparameter()) : 0.3dB
   *  GAIN_PRIMODE register(accessed in this function)   : 0.1dB
   */

  gain = val.p_u8[OFFSET_3APARAMETER_AE_GAIN] * 3;

  isx019_i2c_write(priv,
    CAT_AWB_USERTYPE, USER4_R, &val.p_u8[OFFSET_3APARAMETER_AWB_R], 2);
  isx019_i2c_write(priv,
    CAT_AWB_USERTYPE, USER4_B, &val.p_u8[OFFSET_3APARAMETER_AWB_B], 2);

  regval = 4;
  isx019_i2c_write(priv, CAT_CATAWB, AWBUSER_NO, &regval, 1);

  regval = AWBMODE_MANUAL;
  isx019_i2c_write(priv, CAT_CATAWB, AWBMODE, &regval, 1);

  isx019_i2c_write(priv,
    CAT_CATAE, SHT_PRIMODE, &val.p_u8[OFFSET_3APARAMETER_AE_SHTTIME], 4);
  isx019_i2c_write(priv, CAT_CATAE, GAIN_PRIMODE, &gain, 2);

  return OK;
}

static uint16_t get_gain_from_iso(int32_t iso)
{
  int i;

  for (i = 0; i < NR_ISO; i++)
    {
      if (g_isx019_iso[i] == iso)
        {
          break;
        }
    }

  /* Return gain corresponding to specified ISO sensitivity. */

  return (uint16_t)g_isx019_gain[i];
}

static int set_iso(FAR isx019_dev_t *priv,
                   imgsensor_value_t val)
{
  uint16_t gain;

  /* ISX019 has not ISO sensitivity register but gain register.
   * So, calculate gain from ISO sensitivity.
   */

  gain = get_gain_from_iso(val.value32) * 10;
  isx019_i2c_write(priv, CAT_CATAE, GAIN_PRIMODE, &gain, 2);

  return OK;
}

static int set_iso_auto(FAR isx019_dev_t *priv,
                        imgsensor_value_t val)
{
  uint8_t  buf;
  uint16_t gain;

  if (val.value32 == IMGSENSOR_ISO_SENSITIVITY_AUTO)
    {
      gain = 0;
    }
  else /* IMGSENSOR_ISO_SENSITIVITY_MANUAL */
    {
      isx019_i2c_read(priv, CAT_CATAE, GAIN_PRIMODE, &gain, 2);

      if (gain == 0)
        {
          /* gain = 0 means auto adjustment mode.
           * In such a case, apply current auto adjustment value
           * as manual setting.
           * Note : auto adjustment value register has the unit 0.3dB.
           *        So, convert the unit to 0.1dB.
           */

          isx019_i2c_read(priv, CAT_AECOM, GAIN_LEVEL, &buf, 1);
          gain = buf * 3;
        }
    }

  return isx019_i2c_write(priv, CAT_CATAE, GAIN_PRIMODE, &gain, 2);
}

static uint16_t calc_gamma_regval(double in, double gamma)
{
  double out;

  /* 1) Calculate the normalized result.
   *    formula :  output = input^gamma
   * 2) Perform scaling for ISX019 register.
   * 3) Change the format from the floating-point number type
   *    to the fixed-point number type according to the register.
   */

  out = pow(in, gamma);
  out *= GAM_OUTPUT_SCALE;

  return ((uint8_t)out) << 2;
}

static int set_gamma(FAR isx019_dev_t *priv,
                     imgsensor_value_t val)
{
  int i;
  uint16_t regval;
  uint16_t offset;
  double gamma;

  gamma = (double)val.value32 / 1000;

  /* ISX019 gamma adjustment feature is constructed by
   * registers for low-input and registers for high-input.
   */

  offset = GAM_KNOT_C0;

  for (i = 0; i < NR_GAM_KNOT_LOWINPUT; i++)
    {
      regval = calc_gamma_regval((double)i * GAM_LOWINPUT_INTERVAL, gamma);
      isx019_i2c_write(priv, CAT_PICTGAMMA, offset, &regval, 2);
      offset += 2;
    }

  offset = GAM_KNOT_C11;

  for (i = 0; i < NR_GAM_KNOT_HIGHINPUT; i++)
    {
      regval = calc_gamma_regval
               ((double)(i + 1) * GAM_HIGHINPUT_INTERVAL, gamma);
      isx019_i2c_write(priv, CAT_PICTGAMMA, offset, &regval, 2);
      offset += 2;
    }

  /* Special register setting.
   * GAM_KNOT_C9 and GAM_KNOT_C10 need to be set
   * to be continuous.
   * So, this driver set GAM_KNOT_C10 = GAM_KNOT_C8,
   * GAM_KNOT_C9 = GAM_KNOT_C11.
   */

  isx019_i2c_read(priv, CAT_PICTGAMMA,  GAM_KNOT_C8, &regval, 2);
  isx019_i2c_write(priv, CAT_PICTGAMMA, GAM_KNOT_C10, &regval, 2);
  isx019_i2c_read(priv, CAT_PICTGAMMA,  GAM_KNOT_C11, &regval, 2);
  isx019_i2c_write(priv, CAT_PICTGAMMA, GAM_KNOT_C9, &regval, 2);

  priv->gamma = val.value32;
  return OK;
}

static void search_dqt_data(int32_t quality,
                            FAR const uint8_t **y_head,
                            FAR const uint8_t **y_calc,
                            FAR const uint8_t **c_head,
                            FAR const uint8_t **c_calc)
{
  int i;
  FAR const isx019_fpga_jpg_quality_t *jpg = &g_isx019_jpg_quality[0];

  *y_head = NULL;
  *y_calc = NULL;
  *c_head = NULL;
  *c_calc = NULL;

  /* Search approximate DQT data from a table by rounding quality. */

  quality = ((quality + 5) / 10) * 10;
  if (quality == 0)
    {
      /* Set the minimum value of quality to 10. */

      quality = 10;
    }

  for (i = 0; i < NR_JPGSETTING_TBL; i++)
    {
      if (quality == jpg->quality)
        {
          *y_head = jpg->y_head;
          *y_calc = jpg->y_calc;
          *c_head = jpg->c_head;
          *c_calc = jpg->c_calc;
          break;
        }

      jpg++;
    }
}

int set_dqt(FAR isx019_dev_t *priv, uint8_t component,
            uint8_t target, FAR const uint8_t *buf)
{
  int i;
  uint8_t addr;
  uint8_t select;
  uint8_t data;
  uint8_t regval;

  if (target == FPGA_DQT_DATA)
    {
      addr   = FPGA_DQT_ADDRESS;
      select = FPGA_DQT_SELECT;
      data   = FPGA_DQT_DATA;
    }
  else
    {
      addr   = FPGA_DQT_CALC_ADDRESS;
      select = FPGA_DQT_CALC_SELECT;
      data   = FPGA_DQT_CALC_DATA;
    }

  fpga_i2c_write(priv, select, &component, 1);
  for (i = 0; i < JPEG_DQT_ARRAY_SIZE; i++)
    {
      regval = i | FPGA_DQT_WRITE | FPGA_DQT_BUFFER;
      fpga_i2c_write(priv, addr, &regval, 1);
      fpga_i2c_write(priv, data, &buf[i], 1);
    }

  return OK;
}

static int set_jpg_quality(FAR isx019_dev_t *priv,
                           imgsensor_value_t val)
{
  FAR const uint8_t *y_head;
  FAR const uint8_t *y_calc;
  FAR const uint8_t *c_head;
  FAR const uint8_t *c_calc;

  /* Set JPEG quality by setting DQT information to FPGA. */

  search_dqt_data(val.value32, &y_head, &y_calc, &c_head, &c_calc);
  if ((y_head == NULL) ||
      (y_calc == NULL) ||
      (c_head == NULL) ||
      (c_calc == NULL))
    {
      return -EINVAL;
    }

  nxmutex_lock(&priv->fpga_lock);

  /* Update DQT data and activate them. */

  set_dqt(priv, FPGA_DQT_LUMA,   FPGA_DQT_DATA, y_head);
  set_dqt(priv, FPGA_DQT_CHROMA, FPGA_DQT_DATA, c_head);
  set_dqt(priv, FPGA_DQT_LUMA,   FPGA_DQT_CALC_DATA, y_calc);
  set_dqt(priv, FPGA_DQT_CHROMA, FPGA_DQT_CALC_DATA, c_calc);
  fpga_activate_setting(priv);

  /* Wait for swap of non-active side and active side. */

  nxsig_usleep(DELAY_TIME_JPEGDQT_SWAP);

  /* Update non-active side in preparation for other activation trigger. */

  set_dqt(priv, FPGA_DQT_LUMA,   FPGA_DQT_DATA, y_head);
  set_dqt(priv, FPGA_DQT_CHROMA, FPGA_DQT_DATA, c_head);
  set_dqt(priv, FPGA_DQT_LUMA,   FPGA_DQT_CALC_DATA, y_calc);
  set_dqt(priv, FPGA_DQT_CHROMA, FPGA_DQT_CALC_DATA, c_calc);

  nxmutex_unlock(&priv->fpga_lock);

  priv->jpg_quality = val.value32;
  return OK;
}

static int initialize_jpg_quality(FAR isx019_dev_t *priv)
{
  imgsensor_value_t val;

  val.value32 = CONFIG_VIDEO_ISX019_INITIAL_JPEG_QUALITY;
  return set_jpg_quality(priv, val);
}

static bool validate_clip_setting(FAR uint32_t *clip)
{
  bool ret = false;
  uint32_t w;
  uint32_t h;

  DEBUGASSERT(clip);

  w = clip[IMGSENSOR_CLIP_INDEX_WIDTH];
  h = clip[IMGSENSOR_CLIP_INDEX_HEIGHT];

  if (((w == 1280) && (h == 720)) ||
      ((w ==  640) && (h == 360)) ||
      ((w ==  0) && (h == 0)))
    {
      ret = true;
    }

  return ret;
}

static int set_clip(FAR uint32_t *val, FAR isx019_rect_t *target)
{
  if (val == NULL)
    {
      return -EINVAL;
    }

  if (!validate_clip_setting(val))
    {
      return -EINVAL;
    }

  target->left   = (int32_t)val[IMGSENSOR_CLIP_INDEX_LEFT];
  target->top    = (int32_t)val[IMGSENSOR_CLIP_INDEX_TOP];
  target->width  = val[IMGSENSOR_CLIP_INDEX_WIDTH];
  target->height = val[IMGSENSOR_CLIP_INDEX_HEIGHT];

  return OK;
}

static int set_clip_video(FAR isx019_dev_t *priv,
                          imgsensor_value_t val)
{
  return set_clip(val.p_u32, &priv->clip_video);
}

static int set_clip_still(FAR isx019_dev_t *priv,
                          imgsensor_value_t val)
{
  return set_clip(val.p_u32, &priv->clip_still);
}

static setvalue_t set_value_func(uint32_t id)
{
  setvalue_t func = NULL;

  switch (id)
    {
      case IMGSENSOR_ID_HUE:
        func = set_hue;
        break;

      case IMGSENSOR_ID_GAMMA:
        func = set_gamma;
        break;

      case IMGSENSOR_ID_HFLIP_VIDEO:
        func = set_hflip_video;
        break;

      case IMGSENSOR_ID_VFLIP_VIDEO:
        func = set_vflip_video;
        break;

      case IMGSENSOR_ID_HFLIP_STILL:
        func = set_hflip_still;
        break;

      case IMGSENSOR_ID_VFLIP_STILL:
        func = set_vflip_still;
        break;

      case IMGSENSOR_ID_COLORFX:
        func = set_colorfx;
        break;

      case IMGSENSOR_ID_EXPOSURE_AUTO:
        func = set_ae;
        break;

      case IMGSENSOR_ID_EXPOSURE_ABSOLUTE:
        func = set_exptime;
        break;

      case IMGSENSOR_ID_AUTO_WHITE_BALANCE:
        func = set_awb;
        break;

      case IMGSENSOR_ID_AUTO_N_PRESET_WB:
        func = set_wbmode;
        break;

      case IMGSENSOR_ID_ISO_SENSITIVITY:
        func = set_iso;
        break;

      case IMGSENSOR_ID_ISO_SENSITIVITY_AUTO:
        func = set_iso_auto;
        break;

      case IMGSENSOR_ID_EXPOSURE_METERING:
        func = set_meter;
        break;

      case IMGSENSOR_ID_SPOT_POSITION:
        func = set_spot_position;
        break;

      case IMGSENSOR_ID_3A_LOCK:
        func = set_3alock;
        break;

      case IMGSENSOR_ID_3A_PARAMETER:
        func = set_3aparameter;
        break;

      case IMGSENSOR_ID_JPEG_QUALITY:
        func = set_jpg_quality;
        break;

      case IMGSENSOR_ID_CLIP_VIDEO:
        func = set_clip_video;
        break;

      case IMGSENSOR_ID_CLIP_STILL:
        func = set_clip_still;
        break;

      default:
        break;
    }

  return func;
}

static int get_hue(FAR isx019_dev_t *priv,
                   FAR imgsensor_value_t *val)
{
  if (val == NULL)
    {
      return -EINVAL;
    }

  /* Return stored value without reading register. */

  val->value32 = priv->hue;

  return OK;
}

static int32_t get_flip(FAR uint8_t *flip, uint8_t direction)
{
  DEBUGASSERT(flip);

  return (*flip & direction) ? 1 : 0;
}

static int get_hflip_video(FAR isx019_dev_t *priv,
                           FAR imgsensor_value_t *val)
{
  if (val == NULL)
    {
      return -EINVAL;
    }

  val->value32 = get_flip(&priv->flip_video, H_REVERSE);
  return OK;
}

static int get_vflip_video(FAR isx019_dev_t *priv,
                           FAR imgsensor_value_t *val)
{
  if (val == NULL)
    {
      return -EINVAL;
    }

  val->value32 = get_flip(&priv->flip_video, V_REVERSE);
  return OK;
}

static int get_hflip_still(FAR isx019_dev_t *priv,
                           FAR imgsensor_value_t *val)
{
  if (val == NULL)
    {
      return -EINVAL;
    }

  val->value32 = get_flip(&priv->flip_still, H_REVERSE);
  return OK;
}

static int get_vflip_still(FAR isx019_dev_t *priv,
                           FAR imgsensor_value_t *val)
{
  if (val == NULL)
    {
      return -EINVAL;
    }

  val->value32 = get_flip(&priv->flip_still, V_REVERSE);
  return OK;
}

static int get_colorfx(FAR isx019_dev_t *priv,
                       FAR imgsensor_value_t *val)
{
  if (val == NULL)
    {
      return -EINVAL;
    }

  val->value32 = priv->colorfx;
  return OK;
}

static int get_ae(FAR isx019_dev_t *priv,
                  FAR imgsensor_value_t *val)
{
  uint32_t regval;

  if (val == NULL)
    {
      return -EINVAL;
    }

  isx019_i2c_read(priv, CAT_CATAE, SHT_PRIMODE, &regval, 4);

  val->value32 = (regval == 0) ? IMGSENSOR_EXPOSURE_AUTO
                               : IMGSENSOR_EXPOSURE_MANUAL;

  return OK;
}

static int get_exptime(FAR isx019_dev_t *priv,
                       FAR imgsensor_value_t *val)
{
  uint32_t regval;

  isx019_i2c_read(priv, CAT_AESOUT, SHT_TIME, &regval, 4);

  /* Round up to the nearest 100usec for eliminating errors in reverting to
   * application value because this driver converts application value to
   * value that takes into account the clock ratio and unit difference.
   */

  val->value32 = ((regval / priv->clock_ratio) + 99) / 100;

  return OK;
}

static int get_awb(FAR isx019_dev_t *priv,
                   FAR imgsensor_value_t *val)
{
  if (val == NULL)
    {
      return -EINVAL;
    }

  val->value32 = is_awb_enable(priv);

  return OK;
}

static int get_wbmode(FAR isx019_dev_t *priv,
                      FAR imgsensor_value_t *val)
{
  if (val == NULL)
    {
      return -EINVAL;
    }

  val->value32 = priv->wb_mode;

  return OK;
}

static int get_meter(FAR isx019_dev_t *priv,
                     FAR imgsensor_value_t *val)
{
  uint8_t regval;

  if (val == NULL)
    {
      return -EINVAL;
    }

  isx019_i2c_read(priv, CAT_AUTOCTRL, AEWEIGHTMODE, &regval, 1);

  switch (regval)
    {
      case AEWEIGHT_AVERAGE:
        val->value32 = IMGSENSOR_EXPOSURE_METERING_AVERAGE;
        break;

      case AEWEIGHT_CENTER:
        val->value32 = IMGSENSOR_EXPOSURE_METERING_CENTER_WEIGHTED;
        break;

      case AEWEIGHT_SPOT:
        val->value32 = IMGSENSOR_EXPOSURE_METERING_SPOT;
        break;

      default: /* AEWEIGHT_MATRIX */
        val->value32 = IMGSENSOR_EXPOSURE_METERING_MATRIX;
        break;
    }

  return OK;
}

static uint32_t restore_spot_position(uint16_t regval,
                                      uint16_t basis,
                                      uint16_t sz,
                                      uint16_t clip_sz,
                                      uint16_t offset,
                                      uint16_t split)
{
  uint16_t ret;
  uint16_t unit;
  uint16_t border;

  /* First, convert register value to coordinate value. */

  unit = basis / split;

  ret = (regval * unit) + (unit / 2);

  /* Second, consider the ratio between basis size and frame size. */

  ret = ret * sz / basis;

  /* Third, consider offset value of clip setting. */

  if  (ret > offset)
    {
      ret = ret - offset;
    }
  else
    {
      ret = 0;
    }

  /* If the coordinate protrudes from the frame,
   * regard it as the boader of the frame.
   */

  border = (clip_sz != 0) ? (clip_sz - 1) : (sz - 1);
  if (ret > border)
    {
      ret = border;
    }

  return ret;
}

static int get_spot_position(FAR isx019_dev_t *priv,
                             FAR imgsensor_value_t *val)
{
  uint8_t regval;
  uint8_t regx;
  uint8_t regy;
  uint16_t w;
  uint16_t h;
  uint32_t x;
  uint32_t y;
  uint16_t clip_w;
  uint16_t clip_h;
  uint16_t offset_x;
  uint16_t offset_y;
  int split;

  isx019_i2c_read(priv, CAT_CATAE, SPOT_FRM_NUM, &regval, 1);

  regx = regval % ISX019_SPOT_POSITION_SPLIT_NUM_X;
  regy = regval / ISX019_SPOT_POSITION_SPLIT_NUM_X;

  get_current_framesize(priv, &w, &h);
  get_current_clip_setting(priv, &clip_w, &clip_h, &offset_x, &offset_y);
  split = ISX019_SPOT_POSITION_SPLIT_NUM_X;
  x = restore_spot_position(regx, ISX019_WIDTH,  w, clip_w, offset_x, split);
  split = ISX019_SPOT_POSITION_SPLIT_NUM_Y;
  y = restore_spot_position(regy, ISX019_HEIGHT, h, clip_h, offset_y, split);

  val->value32 = (int32_t)((x << 16) | y);
  return OK;
}

static int get_3alock(FAR isx019_dev_t *priv,
                      FAR imgsensor_value_t *val)
{
  uint8_t regval;
  uint8_t awb;
  uint8_t ae;

  if (val == NULL)
    {
      return -EINVAL;
    }

  isx019_i2c_read(priv, CAT_CATAWB, AWBMODE, &regval, 1);
  awb = (regval == AWBMODE_AUTO) ? 0 : IMGSENSOR_LOCK_WHITE_BALANCE;

  isx019_i2c_read(priv, CAT_CATAE,  AEMODE,  &regval, 1);
  ae = (regval == AEMODE_AUTO) ? 0 : IMGSENSOR_LOCK_EXPOSURE;

  val->value32 = awb | ae;

  return OK;
}

static int get_3aparameter(FAR isx019_dev_t *priv,
                           FAR imgsensor_value_t *val)
{
  if (val == NULL)
    {
      return -EINVAL;
    }

  if (val->p_u8 == NULL)
    {
      return -EINVAL;
    }

  isx019_i2c_read(priv,
    CAT_AWBSOUT, CONT_R,     &val->p_u8[OFFSET_3APARAMETER_AWB_R], 2);
  isx019_i2c_read(priv,
    CAT_AWBSOUT, CONT_B,     &val->p_u8[OFFSET_3APARAMETER_AWB_B], 2);
  isx019_i2c_read(priv,
    CAT_AESOUT,  SHT_TIME,   &val->p_u8[OFFSET_3APARAMETER_AE_SHTTIME], 4);
  isx019_i2c_read(priv,
    CAT_AECOM,   GAIN_LEVEL, &val->p_u8[OFFSET_3APARAMETER_AE_GAIN], 1);

  return OK;
}

static int get_3astatus(FAR isx019_dev_t *priv,
                        FAR imgsensor_value_t *val)
{
  uint8_t regval;

  if (val == NULL)
    {
      return -EINVAL;
    }

  isx019_i2c_read(priv, CAT_AWBSOUT, AWBSTS, &regval, 1);

  switch (regval)
    {
      case AWBSTS_STABLE:
        val->value32 = IMGSENSOR_3A_STATUS_STABLE;
        break;

      case AWBSTS_AEWAIT:
        val->value32 = IMGSENSOR_3A_STATUS_AE_OPERATING;
        break;

      default:
        val->value32 = IMGSENSOR_3A_STATUS_AE_OPERATING |
                       IMGSENSOR_3A_STATUS_AWB_OPERATING;
    }

  return OK;
}

static int32_t get_iso_from_gain(uint8_t gain)
{
  int i;

  /* g_isx019_gain and g_isx019_iso has the common index. */

  for (i = 0; i < NR_ISO; i++)
    {
      if (g_isx019_gain[i] == gain)
        {
          break;
        }
    }

  if (i >= NR_ISO)
    {
      i = NR_ISO - 1;
    }

  return g_isx019_iso[i];
}

static int get_iso(FAR isx019_dev_t *priv,
                   FAR imgsensor_value_t *val)
{
  uint8_t gain = 0;

  if (val == NULL)
    {
      return -EINVAL;
    }

  /* The current gain value register has the 0.3dB unit.
   * So, round the gain to integer, and convert to ISO.
   */

  isx019_i2c_read(priv, CAT_AECOM, GAIN_LEVEL, &gain, 1);
  gain = ((gain * 3) + 5) / 10;

  val->value32 = get_iso_from_gain(gain);

  return OK;
}

static int get_iso_auto(FAR isx019_dev_t *priv,
                        FAR imgsensor_value_t *val)
{
  uint16_t gain;

  if (val == NULL)
    {
      return -EINVAL;
    }

  isx019_i2c_read(priv, CAT_CATAE, GAIN_PRIMODE, &gain, 2);

  val->value32 = (gain == 0) ? IMGSENSOR_ISO_SENSITIVITY_AUTO
                             : IMGSENSOR_ISO_SENSITIVITY_MANUAL;
  return OK;
}

static int get_gamma(FAR isx019_dev_t *priv,
                     FAR imgsensor_value_t *val)
{
  if (val == NULL)
    {
      return -EINVAL;
    }

  val->value32 = priv->gamma;

  return OK;
}

static int get_jpg_quality(FAR isx019_dev_t *priv,
                           FAR imgsensor_value_t *val)
{
  if (val == NULL)
    {
      return -EINVAL;
    }

  val->value32 = priv->jpg_quality;
  return OK;
}

static getvalue_t get_value_func(uint32_t id)
{
  getvalue_t func = NULL;

  switch (id)
    {
      case IMGSENSOR_ID_HUE:
        func = get_hue;
        break;

      case IMGSENSOR_ID_GAMMA:
        func = get_gamma;
        break;

      case IMGSENSOR_ID_HFLIP_VIDEO:
        func = get_hflip_video;
        break;

      case IMGSENSOR_ID_VFLIP_VIDEO:
        func = get_vflip_video;
        break;

      case IMGSENSOR_ID_HFLIP_STILL:
        func = get_hflip_still;
        break;

      case IMGSENSOR_ID_VFLIP_STILL:
        func = get_vflip_still;
        break;

      case IMGSENSOR_ID_COLORFX:
        func = get_colorfx;
        break;

      case IMGSENSOR_ID_EXPOSURE_AUTO:
        func = get_ae;
        break;

      case IMGSENSOR_ID_EXPOSURE_ABSOLUTE:
        func = get_exptime;
        break;

      case IMGSENSOR_ID_AUTO_WHITE_BALANCE:
        func = get_awb;
        break;

      case IMGSENSOR_ID_AUTO_N_PRESET_WB:
        func = get_wbmode;
        break;

      case IMGSENSOR_ID_ISO_SENSITIVITY:
        func = get_iso;
        break;

      case IMGSENSOR_ID_ISO_SENSITIVITY_AUTO:
        func = get_iso_auto;
        break;

      case IMGSENSOR_ID_EXPOSURE_METERING:
        func = get_meter;
        break;

      case IMGSENSOR_ID_SPOT_POSITION:
        func = get_spot_position;
        break;

      case IMGSENSOR_ID_3A_LOCK:
        func = get_3alock;
        break;

      case IMGSENSOR_ID_3A_PARAMETER:
        func = get_3aparameter;
        break;

      case IMGSENSOR_ID_3A_STATUS:
        func = get_3astatus;
        break;

      case IMGSENSOR_ID_JPEG_QUALITY:
        func = get_jpg_quality;
        break;

      default:
        break;
    }

  return func;
}

static int isx019_get_value(FAR struct imgsensor_s *sensor,
                            uint32_t id, uint32_t size,
                            FAR imgsensor_value_t *val)
{
  FAR isx019_dev_t *priv = (FAR isx019_dev_t *)sensor;
  int ret = -EINVAL;
  isx019_reginfo_t reg;
  convert_t cvrt;
  getvalue_t get;
  union
  {
    int32_t i32;
    int16_t i16;
    int8_t  i8;
  } regval;

  DEBUGASSERT(val);

  cvrt = get_reginfo(id, false, &reg);
  if (cvrt)
    {
      memset(&regval, 0, sizeof(regval));
      ret = isx019_i2c_read(priv,
              reg.category, reg.offset, &regval.i32, reg.size);

      switch (reg.type)
        {
          case ISX019_REGTYPE_INT8:
            regval.i32 = (int32_t)regval.i8;
            break;

          case ISX019_REGTYPE_INT16:
            regval.i32 = (int32_t)regval.i16;
            break;

          default:
            break;
        }

      val->value32 = cvrt(regval.i32);
    }
  else
    {
      get = get_value_func(id);
      if (get)
        {
          ret = get(priv, val);
        }
    }

  return ret;
}

static int validate_range(int32_t val,
                          FAR imgsensor_capability_range_t *range)
{
  int ret = OK;

  if (!VALIDATE_RANGE(val, range->minimum, range->maximum, range->step))
    {
      ret = -ERANGE;
    }

  return ret;
}

static int validate_discrete(int32_t val,
                             FAR imgsensor_capability_discrete_t *disc)
{
  int ret = -EINVAL;
  int i;

  for (i = 0; i < disc->nr_values; i++)
    {
      if (val == disc->values[i])
        {
          ret = OK;
          break;
        }
    }

  return ret;
}

static int validate_elems_u8(FAR uint8_t *val, uint32_t sz,
                             FAR imgsensor_capability_elems_t *elems)
{
  int ret = OK;
  int i;

  if (sz != elems->nr_elems)
    {
      return -EINVAL;
    }

  for (i = 0; i < elems->nr_elems; i++)
    {
      if (!VALIDATE_RANGE
           (val[i], elems->minimum, elems->maximum, elems->step))
        {
          ret = -EINVAL;
          break;
        }
    }

  return ret;
}

static int validate_elems_u16(FAR uint16_t *val, uint32_t sz,
                              FAR imgsensor_capability_elems_t *elems)
{
  int ret = OK;
  int i;

  if (sz != elems->nr_elems * sizeof(uint16_t))
    {
      return -EINVAL;
    }

  for (i = 0; i < elems->nr_elems; i++)
    {
      if (!VALIDATE_RANGE
           (val[i], elems->minimum, elems->maximum, elems->step))
        {
          ret = -EINVAL;
          break;
        }
    }

  return ret;
}

static int validate_elems_u32(FAR uint32_t *val, uint32_t sz,
                              FAR imgsensor_capability_elems_t *elems)
{
  int ret = OK;
  int i;

  if (sz != elems->nr_elems * sizeof(uint32_t))
    {
      return -EINVAL;
    }

  for (i = 0; i < elems->nr_elems; i++)
    {
      if (!VALIDATE_RANGE
           (val[i], elems->minimum, elems->maximum, elems->step))
        {
          ret = -EINVAL;
          break;
        }
    }

  return ret;
}

static int validate_value(FAR isx019_dev_t *priv,
                          uint32_t id, uint32_t size,
                          imgsensor_value_t val)
{
  int ret;
  imgsensor_supported_value_t sup;

  ret = isx019_get_supported_value(&priv->sensor, id, &sup);
  if (ret != OK)
    {
      return ret;
    }

  switch (sup.type)
    {
      case IMGSENSOR_CTRL_TYPE_INTEGER_MENU:
        ret = validate_discrete(val.value32, &sup.u.discrete);
        break;

      case IMGSENSOR_CTRL_TYPE_U8:
        ret = validate_elems_u8(val.p_u8, size, &sup.u.elems);
        break;

      case IMGSENSOR_CTRL_TYPE_U16:
        ret = validate_elems_u16(val.p_u16, size, &sup.u.elems);
        break;

      case IMGSENSOR_CTRL_TYPE_U32:
        ret = validate_elems_u32(val.p_u32, size, &sup.u.elems);
        break;

      default:
        ret = validate_range(val.value32, &sup.u.range);
        break;
    }

  return ret;
}

static int isx019_set_value(FAR struct imgsensor_s *sensor,
                            uint32_t id, uint32_t size,
                            imgsensor_value_t val)
{
  FAR isx019_dev_t *priv = (FAR isx019_dev_t *)sensor;
  int ret = -EINVAL;
  isx019_reginfo_t reg;
  convert_t cvrt;
  setvalue_t set;
  int32_t val32;

  ret = validate_value(priv, id, size, val);
  if (ret != OK)
    {
      return ret;
    }

  cvrt = get_reginfo(id, true, &reg);
  if (cvrt)
    {
      val32 = cvrt(val.value32);
      ret = isx019_i2c_write(priv,
              reg.category, reg.offset, &val32, reg.size);
    }
  else
    {
      set = set_value_func(id);
      if (set)
        {
          ret = set(priv, val);
        }
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int isx019_initialize(void)
{
  FAR isx019_dev_t *priv = &g_isx019_private;
  imgsensor_register(&priv->sensor);
  return OK;
}

int isx019_uninitialize(void)
{
  return OK;
}

#ifdef CONFIG_VIDEO_ISX019_REGDEBUG
int isx019_read_register(uint8_t cat,
                         uint16_t addr,
                         FAR uint8_t *buf,
                         uint8_t size)
{
  FAR isx019_dev_t *priv = &g_isx019_private;
  int ret;

  if (cat == 0xff)
    {
      ret = fpga_i2c_read(priv, (uint8_t)addr, buf, size);
    }
  else
    {
      ret = isx019_i2c_read(priv, cat, addr, buf, size);
    }

  return ret;
}
#endif

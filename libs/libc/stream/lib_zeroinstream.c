/****************************************************************************
 * libs/libc/stream/lib_zeroinstream.c
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

#include <stdio.h>
#include <errno.h>
#include "libc.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int zeroinstream_getc(FAR struct lib_instream_s *self)
{
  self->nget++;
  return 0;
}

static ssize_t zeroinstream_gets(FAR struct lib_instream_s *self,
                                 FAR void *buffer, size_t len)
{
  self->nget += len;
  memset(buffer, 0, len);
  return len;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lib_zeroinstream
 *
 * Description:
 *   Initializes a NULL stream.  The initialized stream will return an
 *   infinitely long stream of zeroes.
 *
 * Input Parameters:
 *   stream  - User allocated, uninitialized instance of struct
 *             lib_instream_s to be initialized.
 *
 * Returned Value:
 *   None (User allocated instance initialized).
 *
 ****************************************************************************/

void lib_zeroinstream(FAR struct lib_instream_s *stream)
{
  stream->getc = zeroinstream_getc;
  stream->gets = zeroinstream_gets;
  stream->nget = 0;
}

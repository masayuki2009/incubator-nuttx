/****************************************************************************
 * boards/risc-v/k210/maix-bit/src/k210_spi.c
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

#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/spi/spi.h>
#include <arch/board/board.h>

#include "chip.h"
#include "maix-bit.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: k210_spidev_initialize
 ****************************************************************************/

void weak_function k210_spidev_initialize(void)
{
  /* TODO */
}

/****************************************************************************
 * Name:  k210_spi0select and k210_spi0status
 ****************************************************************************/

#ifdef CONFIG_K210_SPI0
void k210_spi0select(FAR struct spi_dev_s *dev,
                     uint32_t devid, bool selected)
{
  spiinfo("devid: %d CS: %s\n",
          (int)devid, selected ? "assert" : "de-assert");

  /* TODO */
}

uint8_t k210_spi0status(FAR struct spi_dev_s *dev, uint32_t devid)
{
  uint8_t status = 0;
  status |= SPI_STATUS_PRESENT;
  return status;
}
#endif

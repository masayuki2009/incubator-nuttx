/****************************************************************************
 * boards/risc-v/k210/maix-bit/src/k210_mmcsd.c
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

#include <debug.h>
#include <nuttx/config.h>
#include <nuttx/mmcsd.h>
#include <nuttx/spi/spi.h>
#include <unistd.h>

#include <arch/board/board.h>
#include "chip.h"
#include "k210_spi.h"
#include "k210_fpioa.h"
#include "maix-bit.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_DISABLE_MOUNTPOINT
#  error "SD driver requires CONFIG_DISABLE_MOUNTPOINT to be disabled"
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: k210_spi0register
 *
 * Description:
 *   Registers media change callback
 ****************************************************************************/

int k210_spi0register(struct spi_dev_s *dev, spi_mediachange_t callback,
                       void *arg)
{
  /* TODO: media change callback */

  return OK;
}

/****************************************************************************
 * Name: k210_mmcsd_initialize
 *
 * Description:
 *   Initialize SPI-based SD card and card detect thread.
 ****************************************************************************/

int k210_mmcsd_initialize(int port, int minor)
{
  struct spi_dev_s *spi;
  int rv;

  k210_configfpioa(27, FUNC_SPI0_SCK);  /* IO27 to SPI0_SCK  */
  k210_configfpioa(28, FUNC_SPI0_D0);   /* IO28 to SPI0_MOSI(D0) */
  k210_configfpioa(26, FUNC_SPI0_D1);   /* IO26 to SPI0_MISO(D1) */
  k210_configfpioa(29, FUNC_GPIO_HS7);  /* IO29 to CS(GPIOHS7) */

#if 0 /* TODO */
  k210_gpiowrite(GPIO_MMCSD_NSS, 1); /* TODO: Ensure the CS is inactive */
#endif

  mcinfo("INFO: Initializing mmcsd port %d minor %d \n",
         port, minor);

  spi = k210_spibus_initialize(port);
  if (spi == NULL)
    {
      mcerr("ERROR: Failed to initialize SPI port %d\n", port);
      return -ENODEV;
    }

  rv = mmcsd_spislotinitialize(minor, minor, spi);
  if (rv < 0)
    {
      mcerr("ERROR: Failed to bind SPI port %d to SD slot %d\n",
             port, minor);
      return rv;
    }

  spiinfo("INFO: mmcsd card has been initialized successfully\n");
  return OK;
}

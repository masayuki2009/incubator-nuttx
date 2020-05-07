/****************************************************************************
 * arch/risc-v/src/k210/k210_spi.h
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

#ifndef __ARCH_RISCV_SRC_K210_K210_SPI_H
#define __ARCH_RISCV_SRC_K210_K210_SPI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/spi/spi.h>

#include "chip.h"
#include "hardware/k210_spi.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

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
 * Public Data
 ****************************************************************************/

struct spi_dev_s;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: k210_spibus_initialize
 ****************************************************************************/

FAR struct spi_dev_s *k210_spibus_initialize(int bus);

/****************************************************************************
 * Name:  k210_spi0select and k210_spi0status
 ****************************************************************************/

#ifdef CONFIG_K210_SPI0
void k210_spi0select(FAR struct spi_dev_s *dev, uint32_t devid,
                     bool selected);
uint8_t k210_spi0status(FAR struct spi_dev_s *dev, uint32_t devid);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */

#endif /* __ARCH_RISCV_SRC_K210_K210_SPI_H */

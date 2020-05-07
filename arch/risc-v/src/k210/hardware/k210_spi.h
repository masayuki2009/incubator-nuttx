/****************************************************************************
 * arch/risc-v/src/k210/hardware/k210_spi.h
 *
 * Derives from software originally provided by Canaan Inc
 *
 *   Copyright 2018 Canaan Inc
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

#ifndef __ARCH_RISCV_SRC_K210_HARDWARE_K210_SPI_H
#define __ARCH_RISCV_SRC_K210_HARDWARE_K210_SPI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SPI_FIFO_LEN    32

#define SPI_CTL0_OFFSET  0x00  /* Control Register 0 */

#define SPI_CTL0_WMODE_SHIFT 6 /* SPI work mode */
#define SPI_CTL0_WMODE_MASK  (3 << SPI_CTL0_WMODE_SHIFT)

#define SPI_CTL0_TMODE_SHIFT 8 /* SPI transfer mode */
#define SPI_CTL0_TMODE_MASK  (3 << SPI_CTL0_TMODE_SHIFT)

#define SPI_CTL0_TMODE_TXRX  0
#define SPI_CTL0_TMODE_TX    1
#define SPI_CTL0_TMODE_RX    2

#define SPI_CTL0_DFS_SHIFT 16 /* SPI data frame size (0-31 bits) */
#define SPI_CTL0_DFS_MASK   (0x1f << SPI_CTL0_DFS_SHIFT)

#define SPI_CTL1_OFFSET  0x04  /* Control Register 1 */
#define SPI_MEN_OFFSET   0x08  /* Master Enable (SSI Enable) */

#define SPI_SEN_OFFSET   0x10  /* Slave Enable (chip select) */
#define SPI_DIV_OFFSET   0x14  /* Divider (Baud Rate Select) */

#define SPI_TFL_OFFSET   0x20  /* TX FIFO Level */
#define SPI_RFL_OFFSET   0x24  /* RX FIFO Level */
#define SPI_STS_OFFSET   0x28  /* Status */
#define SPI_DAT_OFFSET   0x60  /* Data (r/w) : 0x60-0xec */

#endif /* __ARCH_RISCV_SRC_K210_HARDWARE_K210_SPI_H */

/****************************************************************************
 * arch/risc-v/src/k210/hardware/k210_fpioa.h
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

#ifndef __ARCH_RISCV_SRC_K210_HARDWARE_K210_FPIOA_H
#define __ARCH_RISCV_SRC_K210_HARDWARE_K210_FPIOA_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define FUNC_SPI0_SCK   0x00001f11
#define FUNC_SPI0_D0    0x80b03f04
#define FUNC_SPI0_D1    0x00b03f05
#define FUNC_GPIO_HS7   0x00921f1f

#endif /* __ARCH_RISCV_SRC_K210_HARDWARE_K210_FPIOA_H */

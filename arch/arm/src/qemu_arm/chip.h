/****************************************************************************
 * arch/arm/src/a1x/chip.h
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

#ifndef __ARCH_ARM_SRC_A1X_CHIP_H
#define __ARCH_ARM_SRC_A1X_CHIP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/
#  define NUTTX_TEXT_VADDR       (CONFIG_RAM_VSTART & 0xfff00000)
#  define NUTTX_TEXT_PADDR       (CONFIG_RAM_START & 0xfff00000)
#  define NUTTX_TEXT_PEND        ((CONFIG_RAM_END + 0x000fffff) & 0xfff00000)
#  define NUTTX_TEXT_SIZE        (NUTTX_TEXT_PEND - NUTTX_TEXT_PADDR)

#  define PGTABLE_BASE_PADDR	 0x60000000
#  define PGTABLE_BASE_VADDR	 PGTABLE_BASE_PADDR

#  define CHIP_MPCORE_VBASE		 0x08000000


#define _NSECTIONS(b)        (((b)+0x000fffff) >> 20)
#define QEMU_VIRT_DDR_PSECTION     0x40000000
#define QEMU_VIRT_DDR_VSECTION     0x40000000
#define QEMU_VIRT_DDR_MMUFLAGS		MMU_MEMFLAGS
#define QEMU_VIRT_DDR_NSECTIONS	   _NSECTIONS(0x80000000)

#define QEMU_VIRT_PERIPH_PSECTION     0x0
#define QEMU_VIRT_PERIPH_VSECTION     0x0
#define QEMU_VIRT_PERIPH_MMUFLAGS	  MMU_IOFLAGS
#define QEMU_VIRT_PERIPH_NSECTIONS	  _NSECTIONS(0x0a000000 + 0x200)

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_A1X_CHIP_H */

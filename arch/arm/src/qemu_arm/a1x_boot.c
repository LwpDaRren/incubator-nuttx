/****************************************************************************
 * arch/arm/src/a1x/a1x_boot.c
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
#include <assert.h>

#ifdef CONFIG_PAGING
#  include <nuttx/page.h>
#endif

#include "arm_internal.h"
#include "sctlr.h"
#include "chip.h"
#include "mmu.h"

volatile uint32_t *g_current_regs[1];

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The vectors are, by default, positioned at the beginning of the text
 * section.  They will always have to be copied to the correct location.
 *
 * If we are using high vectors (CONFIG_ARCH_LOWVECTORS=n).  In this case,
 * the vectors will lie at virtual address 0xffff:000 and we will need
 * to a) copy the vectors to another location, and b) map the vectors
 * to that address, and
 *
 * For the case of CONFIG_ARCH_LOWVECTORS=y, defined.  Vectors will be
 * copied to SRAM A1 at address 0x0000:0000
 */

#if !defined(CONFIG_ARCH_LOWVECTORS) && defined(CONFIG_ARCH_ROMPGTABLE)
#  error High vector remap cannot be performed if we are using a ROM page table
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

extern uint32_t _vector_start; /* Beginning of vector block */
extern uint32_t _vector_end;   /* End+1 of vector block */

/****************************************************************************
 * Private Data
 ****************************************************************************/
 static const struct section_mapping_s section_mapping[] =
{
  { QEMU_VIRT_PERIPH_PSECTION,  QEMU_VIRT_PERIPH_VSECTION,
    QEMU_VIRT_PERIPH_MMUFLAGS,  QEMU_VIRT_PERIPH_NSECTIONS
  },
  
  { QEMU_VIRT_DDR_PSECTION,  QEMU_VIRT_DDR_VSECTION,
    QEMU_VIRT_DDR_MMUFLAGS,  QEMU_VIRT_DDR_NSECTIONS
  },
};

#define NMAPPINGS \
  (sizeof(section_mapping) / sizeof(struct section_mapping_s))
/****************************************************************************
 * Private Functions
 ****************************************************************************/
 
/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_irqinitialize
 *
 * Description:
 *   This function is called by up_initialize() during the bring-up of the
 *   system.  It is the responsibility of this function to but the interrupt
 *   subsystem into the working and ready state.
 *
 ****************************************************************************/

void up_irqinitialize(void)
{
  /* The following operations need to be atomic, but since this function is
   * called early in the initialization sequence, we expect to have exclusive
   * access to the GIC.
   */

  /* Initialize the Generic Interrupt Controller (GIC) for CPU0 */

  arm_gic0_initialize();  /* Initialization unique to CPU0 */
  arm_gic_initialize();   /* Initialization common to all CPUs */

#ifdef CONFIG_ARCH_LOWVECTORS
  /* If CONFIG_ARCH_LOWVECTORS is defined, then the vectors located at the
   * beginning of the .text region must appear at address at the address
   * specified in the VBAR.  There are two ways to accomplish this:
   *
   *   1. By explicitly mapping the beginning of .text region with a page
   *      table entry so that the virtual address zero maps to the beginning
   *      of the .text region.  VBAR == 0x0000:0000.
   *
   *   2. Set the Cortex-A5 VBAR register so that the vector table address
   *      is moved to a location other than 0x0000:0000.
   *
   *  The second method is used by this logic.
   */

  /* Set the VBAR register to the address of the vector table */

  DEBUGASSERT((((uintptr_t)&_vector_start) & ~VBAR_MASK) == 0);
  cp15_wrvbar((uint32_t)&_vector_start);
#endif /* CONFIG_ARCH_LOWVECTORS */

  /* currents_regs is non-NULL only while processing an interrupt */

  CURRENT_REGS = NULL;

#ifndef CONFIG_SUPPRESS_INTERRUPTS
  /* And finally, enable interrupts */

  up_irq_enable();
#endif
}

/****************************************************************************
 * Name: arm_boot
 *
 * Description:
 *   Complete boot operations started in arm_head.S
 *
 *   This logic will be executing in SDRAM.  This boot logic was started by
 *   the A10 boot logic.  At this point in time, clocking and SDRAM have
 *   already be initialized (they must be because we are executing out of
 *   SDRAM).  So all that must be done here is to:
 *
 *     1) Refine the memory mapping,
 *     2) Configure the serial console, and
 *     3) Perform board-specific initializations.
 *
 ****************************************************************************/

void arm_boot(void)
{
#if 0
#ifndef CONFIG_ARCH_ROMPGTABLE
  /* __start provided the basic MMU mappings for SRAM.  Now provide mappings
   * for all IO regions (Including the vector region).
   */

  a1x_setupmappings();

  /* Provide a special mapping for the IRAM interrupt vector positioned in
   * high memory.
   */

  a1x_vectormapping();

#endif /* CONFIG_ARCH_ROMPGTABLE */

  /* Setup up vector block.  _vector_start and _vector_end are exported from
   * arm_vector.S
   */

  a1x_copyvectorblock();

#ifdef CONFIG_ARCH_FPU
  /* Initialize the FPU */

  arm_fpuconfig();
#endif

#ifdef CONFIG_BOOT_SDRAM_DATA
  /* This setting is inappropriate for the A1x because the code is *always*
   * executing from SDRAM.  If CONFIG_BOOT_SDRAM_DATA happens to be set,
   * let's try to do the right thing and initialize the .data and .bss
   * sections.
   */

  arm_data_initialize();
#endif

  /* Perform common, low-level chip initialization (might do nothing) */

  a1x_lowsetup();

#ifdef USE_EARLYSERIALINIT
  /* Perform early serial initialization if we are going to use the serial
   * driver.
   */

  arm_earlyserialinit();
#endif

  /* Perform board-specific initialization,  This must include:
   *
   * - Initialization of board-specific memory resources (e.g., SDRAM)
   * - Configuration of board specific resources (PIOs, LEDs, etc).
   */

  a1x_boardinitialize();
#endif
  cp15_wrvbar((uint32_t)&_vector_start);

  mmu_l1_map_regions(section_mapping, NMAPPINGS);

#ifdef USE_EARLYSERIALINIT
  /* Perform early serial initialization if we are going to use the serial
   * driver.
   */
	uart_early_init();
#endif
	up_puts("hello world!\n");
}

void up_timer_initialize(void)
{
}

/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright 2017 NXP
 */

#include <arm.h>
#include <arm32_macros.S>
#include <asm.S>
#include <kernel/cache_helpers.h>

    .global psci_enable_smp_ca32
    .global psci_disable_smp_ca32


SMPEN_MASK_CA32   = 0x40  // 1 << 6

FUNC psci_enable_smp_ca32, :
    mrrc    p15, 1, r0, r1, c15
    orr     r0, r0, #SMPEN_MASK_CA32
    mcrr    p15, 1, r0, r1, c15

    isb

    bx      lr
END_FUNC psci_enable_smp_ca32

FUNC psci_disable_smp_ca32, :
    mrrc    p15, 1, r0, r1, c15
    bic     r0, r0, #SMPEN_MASK_CA32
    mcrr    p15, 1, r0, r1, c15

    isb

    bx      lr
END_FUNC psci_disable_smp_ca32

FUNC psci_disable_smp, :
	read_actlr r0
	bic	r0, r0, #ACTLR_SMP
	write_actlr r0
	isb
	bx	lr
END_FUNC psci_disable_smp

FUNC psci_enable_smp, :
	read_actlr r0
	orr	r0, r0, #ACTLR_SMP
	write_actlr r0
	isb
	bx	lr
END_FUNC psci_enable_smp

FUNC psci_armv7_cpu_off, :
	push	{r12, lr}
UNWIND(	.save	{r12, lr})

	mov     r0, #DCACHE_OP_CLEAN_INV
	bl	dcache_op_all

	/* Disable Cache */
	read_sctlr r0
	bic	r0, r0, #SCTLR_C
	write_sctlr r0
	isb
	dsb

	mov	r0, #DCACHE_OP_CLEAN_INV
	bl	dcache_op_all

	clrex

	bl	psci_disable_smp_ca32

	pop	{r12, pc}
END_FUNC psci_armv7_cpu_off

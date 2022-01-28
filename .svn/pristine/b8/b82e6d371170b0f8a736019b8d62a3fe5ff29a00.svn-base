/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (c) 2014, Linaro Limited
 */

#include <asm.S>
#include <arm.h>

/* For A600 number the A53s as 0 to 3 */
FUNC get_core_pos_mpidr , :
	/* Calculate CorePos = ((ClusterId ^ 1) * 4) + CoreId */
	and	x0, x0, #0xFF
	and	x1, x0, #MPIDR_CPU_MASK
	and	x0, x0, #MPIDR_CLUSTER_MASK
	add	x0, x1, x0, LSR #6
	ret
END_FUNC get_core_pos_mpidr
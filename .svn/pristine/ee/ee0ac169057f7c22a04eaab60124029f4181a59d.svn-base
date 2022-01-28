/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright 2019 Faraday.
 */

#ifndef PLATFORM_CONFIG_H
#define PLATFORM_CONFIG_H

#include <mm/generic_ram_layout.h>

#define STACK_ALIGNMENT		64

#define CONSOLE_UART_CLK_IN_HZ	30000000
#define CONSOLE_BAUDRATE	115200

#define CONSOLE_UART_BASE	0x54e00000

#define GIC_BASE		0x70000000
#define GICD_BASE		0x70001000
#define GICD_OFFSET 0x1000
#define GICC_OFFSET 0x2000
#if 0
#define SECURE_GPIO_BASE0	0x689d0000
#define ASIU_GPIO_INTR		190
#define GPIO_NUM_START0		0x08200000
#endif

/* NS DDR ranges */
#define BCM_DRAM0_NS_BASE      0x88000000
#define BCM_DRAM0_NS_SIZE      0xf0000000

//#define TEE_SHMEM_START		0x85000000
#if 1

#define CFG_TZDRAM_BASE	0x80000000
#define TEE_RAM_START		TZDRAM_BASE

#define CFG_TEE_LOAD_ADDR TZDRAM_BASE
/*
#ifdef CFG_TEE_LOAD_ADDR
#define TEE_LOAD_ADDR			CFG_TEE_LOAD_ADDR
#else
#define TEE_LOAD_ADDR			TEE_RAM_START
#endif
*/

#endif
#endif /*PLATFORM_CONFIG_H*/

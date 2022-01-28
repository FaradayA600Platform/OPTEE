/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright 2019 Faraday.
 */

#ifndef PLATFORM_CONFIG_H
#define PLATFORM_CONFIG_H

#include <mm/generic_ram_layout.h>

#define STACK_ALIGNMENT         64

#define CONSOLE_UART_BASE       0x20700000
#define CONSOLE_UART_CLK_IN_HZ  58823529
#define CONSOLE_BAUDRATE        115200

#define GICD_BASE               0x28601000
#define GICC_BASE               0x28602000

#define SPI_BASE                0x28300000

#define BCM_DEVICE0_BASE        CONSOLE_UART_BASE
#define BCM_DEVICE0_SIZE        0x1000

#define BCM_DEVICE1_BASE        GICD_BASE
#define BCM_DEVICE1_SIZE        0x1000

#define BCM_DEVICE2_BASE        GICC_BASE
#define BCM_DEVICE2_SIZE        0x1000

#define BCM_DEVICE3_BASE        SPI_BASE
#define BCM_DEVICE3_SIZE        0x1000

/* NS DDR ranges */
#define BCM_DRAM0_NS_BASE       0x80000000
#define BCM_DRAM0_NS_SIZE       0x00001000 //2GB

#endif /*PLATFORM_CONFIG_H*/

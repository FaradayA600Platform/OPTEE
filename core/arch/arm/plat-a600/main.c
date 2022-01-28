// SPDX-License-Identifier: BSD-2-Clause
/*
 * Copyright 2019 Broadcom.
 */

#include <console.h>
#include <drivers/gic.h>
#include <drivers/serial8250_uart.h>
#include <drivers/ftspi020_spi.h>
#include <kernel/generic_boot.h>
#include <kernel/panic.h>
#include <kernel/pm_stubs.h>
#include <mm/core_memprot.h>
#include <mm/tee_pager.h>
#include <platform_config.h>
#include <stdint.h>
#include <tee/entry_fast.h>
#include <tee/entry_std.h>

static void secure_intr_handler(void);

static const struct thread_handlers handlers = {
	.std_smc = tee_entry_std,
	.fast_smc = tee_entry_fast,
	.nintr = secure_intr_handler,
	.cpu_on = cpu_on_handler,
	.cpu_off = pm_do_nothing,
	.cpu_suspend = pm_do_nothing,
	.cpu_resume = pm_do_nothing,
	.system_off = pm_do_nothing,
	.system_reset = pm_do_nothing,
};

static struct gic_data gic_data;
struct serial8250_uart_data console_data;

#ifdef BCM_DEVICE0_BASE
register_phys_mem_pgdir(MEM_AREA_IO_SEC, BCM_DEVICE0_BASE, BCM_DEVICE0_SIZE);
#endif
#ifdef BCM_DEVICE1_BASE
register_phys_mem_pgdir(MEM_AREA_IO_SEC, BCM_DEVICE1_BASE, BCM_DEVICE1_SIZE);
#endif
#ifdef BCM_DEVICE2_BASE
register_phys_mem_pgdir(MEM_AREA_IO_SEC, BCM_DEVICE2_BASE, BCM_DEVICE2_SIZE);
#endif
#ifdef BCM_DEVICE3_BASE
register_phys_mem_pgdir(MEM_AREA_IO_SEC, BCM_DEVICE3_BASE, BCM_DEVICE3_SIZE);
#endif
#ifdef BCM_DRAM0_NS_BASE
register_dynamic_shm(BCM_DRAM0_NS_BASE, BCM_DRAM0_NS_SIZE);
#endif
#ifdef BCM_DRAM1_NS_BASE
register_dynamic_shm(BCM_DRAM1_NS_BASE, BCM_DRAM1_NS_SIZE);
#endif
#ifdef BCM_DRAM2_NS_BASE
register_dynamic_shm(BCM_DRAM2_NS_BASE, BCM_DRAM2_NS_SIZE);
#endif

const struct thread_handlers *generic_boot_get_handlers(void)
{
	return &handlers;
}

void console_init(void)
{
	serial8250_uart_init(&console_data, CONSOLE_UART_BASE,
			     CONSOLE_UART_CLK_IN_HZ, CONSOLE_BAUDRATE);
	register_serial_console(&console_data.chip);
}

static void secure_intr_handler(void)
{
	gic_it_handle(&gic_data);
}

void main_init_gic(void)
{
	vaddr_t gicd_base, gicc_base;

	gicd_base = core_mmu_get_va(GICD_BASE, MEM_AREA_IO_SEC);

	if (!gicd_base)
		panic();

	gicc_base = core_mmu_get_va(GICC_BASE, MEM_AREA_IO_SEC);

	if (!gicc_base)
		panic();

	gic_init_base_addr(&gic_data, gicc_base, gicd_base);
	itr_init(&gic_data.chip);
}

void main_secondary_init_gic(void)
{
	gic_cpu_init(&gic_data);
}

#ifdef CFG_SPI
void spi_init(void)
{
	struct ftspi020_data hw;

	hw.base = core_mmu_get_va(SPI_BASE, MEM_AREA_IO_SEC);
	hw.mode = SPI_MODE0;

	ftspi020_init(&hw);

	hw.chip.ops->configure(&hw.chip);

#ifdef CFG_SPI_TEST
	uint8_t data[1024];
	int i;

	for(i = 0; i < 256; i++)
		data[i] = i;

	ftspi020_write_reg(&hw, 0x06, NULL, 0);

	ftspi020_write(&hw, 0x0, 0x100, data);

	for(i = 0; i < 256; i++)
		data[i] = 0xFF;

	ftspi020_read(&hw, 0x0, 0x400, data);

	for(i = 0; i < 256; i++)
		EMSG("0x%x ", data[i]);
#endif
}
#endif

static TEE_Result peripherals_init(void)
{
#ifdef CFG_SPI
	spi_init();
#endif

	return TEE_SUCCESS;
}
driver_init(peripherals_init);

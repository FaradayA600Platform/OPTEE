// SPDX-License-Identifier: BSD-2-Clause
/*
 * Copyright 2019 Broadcom.
 */

#include <console.h>
#include <drivers/gic.h>
#include <drivers/serial8250_uart.h>
#include <kernel/generic_boot.h>
#include <kernel/panic.h>
#include <kernel/pm_stubs.h>
#include <mm/core_memprot.h>
#include <mm/tee_pager.h>
#include <platform_config.h>
#include <stdint.h>
#include <tee/entry_fast.h>
#include <tee/entry_std.h>
#include <io.h>                                                                                                                                                

static void secure_intr_handler(void);


#if 0
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
#else
static void main_fiq(void);
static const struct thread_handlers handlers = {
	.std_smc = tee_entry_std,
	.fast_smc = tee_entry_fast,
	.nintr = main_fiq,
	.cpu_on = pm_panic,
	.cpu_off = pm_panic,
	.cpu_suspend = pm_panic,
	.cpu_resume = pm_panic,
	.system_off = pm_panic,
	.system_reset = pm_panic,
};
#endif

static struct gic_data gic_data;
struct serial8250_uart_data console_data;
register_phys_mem_pgdir(MEM_AREA_IO_NSEC, CONSOLE_UART_BASE, SMALL_PAGE_SIZE);

register_phys_mem_pgdir(MEM_AREA_IO_SEC, GIC_BASE,SMALL_PAGE_SIZE);
register_phys_mem_pgdir(MEM_AREA_IO_SEC, GIC_BASE +GICC_OFFSET,SMALL_PAGE_SIZE);
register_phys_mem_pgdir(MEM_AREA_IO_SEC, GIC_BASE +GICD_OFFSET,SMALL_PAGE_SIZE);
register_phys_mem_pgdir(MEM_AREA_IO_SEC,
			ROUNDDOWN(GIC_BASE + GICC_OFFSET, CORE_MMU_PGDIR_SIZE),
			CORE_MMU_PGDIR_SIZE);

register_phys_mem_pgdir(MEM_AREA_IO_SEC,
			ROUNDDOWN(GIC_BASE + GICD_OFFSET, CORE_MMU_PGDIR_SIZE),
			CORE_MMU_PGDIR_SIZE);
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

vaddr_t get_gicc_base(void)
{
	struct io_pa_va base = { .pa = GIC_BASE + GICC_OFFSET };

	return io_pa_or_va_secure(&base);
}

vaddr_t get_gicd_base(void)
{
	struct io_pa_va base = { .pa = GIC_BASE + GICD_OFFSET };

	return io_pa_or_va_secure(&base);
}

static void main_fiq(void)
{
	gic_it_handle(&gic_data);
}

void main_init_gic(void)
{
	gic_init(&gic_data, get_gicc_base(), get_gicd_base());
	itr_init(&gic_data.chip);
}

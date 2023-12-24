/*
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2020 Western Digital Corporation or its affiliates.
 *
 * Authors:
 *   Anup Patel <anup.patel@wdc.com>
 */

#include <libfdt.h>
#include <platform_override.h>
#include <sbi/riscv_asm.h>
#include <sbi/sbi_hartmask.h>
#include <sbi/sbi_platform.h>
#include <sbi/sbi_string.h>
#include <sbi_utils/fdt/fdt_domain.h>
#include <sbi_utils/fdt/fdt_fixup.h>
#include <sbi_utils/fdt/fdt_helper.h>
#include <sbi_utils/irqchip/fdt_irqchip.h>
#include <sbi_utils/serial/fdt_serial.h>
#include <sbi_utils/timer/fdt_timer.h>
#include <sbi_utils/ipi/fdt_ipi.h>
#include <sbi_utils/reset/fdt_reset.h>
#include <sbi/sbi_console.h>
#include <sbi/riscv_io.h>

#define PMP_BASE_ADDR			0xffdc020000UL
#define PMP_SIZE_PER_CORE		0x4000UL
#define TCM0_START_ADDR			0xffe0180000UL
#define TCM0_END_ADDR			0xffe01c0000UL
#define TCM1_START_ADDR			0xffe01c0000UL
#define TCM1_END_ADDR			0xffe0200000UL
#define RESERVED_START_ADDR		0xffe0200000UL
#define RESERVED_END_ADDR		0xffe1000000UL
#define PMP_ENTRY_BASE_ADDR		0x100UL
#define PMP_ENTRY_START_ADDR(n)	(PMP_BASE_ADDR + PMP_ENTRY_BASE_ADDR + (n * 8))
#define PMP_ENTRY_END_ADDR(n)	(PMP_ENTRY_START_ADDR(n) + 4)
#define PMP_ENTRY_CFG_ADDR(n)	(PMP_BASE_ADDR + ((n / 4) * 4))

extern const struct platform_override sifive_fu540;
extern const struct platform_override light;

static const struct platform_override *special_platforms[] = {
	&sifive_fu540,
	&light,
};

static const struct platform_override *generic_plat = NULL;
static const struct fdt_match *generic_plat_match = NULL;

static void fw_platform_lookup_special(void *fdt, int root_offset)
{
	int pos, noff;
	const struct platform_override *plat;
	const struct fdt_match *match;

	for (pos = 0; pos < array_size(special_platforms); pos++) {
		plat = special_platforms[pos];
		if (!plat->match_table)
			continue;

		noff = fdt_find_match(fdt, -1, plat->match_table, &match);
		if (noff < 0)
			continue;

		generic_plat = plat;
		generic_plat_match = match;
		break;
	}
}

extern struct sbi_platform platform;
static u32 generic_hart_index2id[SBI_HARTMASK_MAX_BITS] = { 0 };

/*
 * The fw_platform_init() function is called very early on the boot HART
 * OpenSBI reference firmwares so that platform specific code get chance
 * to update "platform" instance before it is used.
 *
 * The arguments passed to fw_platform_init() function are boot time state
 * of A0 to A4 register. The "arg0" will be boot HART id and "arg1" will
 * be address of FDT passed by previous booting stage.
 *
 * The return value of fw_platform_init() function is the FDT location. If
 * FDT is unchanged (or FDT is modified in-place) then fw_platform_init()
 * can always return the original FDT location (i.e. 'arg1') unmodified.
 */
unsigned long fw_platform_init(unsigned long arg0, unsigned long arg1,
				unsigned long arg2, unsigned long arg3,
				unsigned long arg4)
{
	const char *model;
	void *fdt = (void *)arg1;
	u32 hartid, hart_count = 0;
	int rc, root_offset, cpus_offset, cpu_offset, len;

	root_offset = fdt_path_offset(fdt, "/");
	if (root_offset < 0)
		goto fail;

	fw_platform_lookup_special(fdt, root_offset);

	model = fdt_getprop(fdt, root_offset, "model", &len);
	if (model)
		sbi_strncpy(platform.name, model, sizeof(platform.name));

	if (generic_plat && generic_plat->features)
		platform.features = generic_plat->features(generic_plat_match);

	cpus_offset = fdt_path_offset(fdt, "/cpus");
	if (cpus_offset < 0)
		goto fail;

	fdt_for_each_subnode(cpu_offset, fdt, cpus_offset) {
		rc = fdt_parse_hart_id(fdt, cpu_offset, &hartid);
		if (rc)
			continue;

		if (SBI_HARTMASK_MAX_BITS <= hartid)
			continue;

		generic_hart_index2id[hart_count++] = hartid;
	}

	platform.hart_count = hart_count;

	/* Return original FDT pointer */
	return arg1;

fail:
	while (1)
		wfi();
}

static int generic_early_init(bool cold_boot)
{
	int rc;

	if (generic_plat && generic_plat->early_init) {
		rc = generic_plat->early_init(cold_boot, generic_plat_match);
		if (rc)
			return rc;
	}

	if (!cold_boot)
		return 0;

	return fdt_reset_init();
}

static int generic_final_init(bool cold_boot)
{
	void *fdt;
	int rc;

	if (generic_plat && generic_plat->final_init) {
		rc = generic_plat->final_init(cold_boot, generic_plat_match);
		if (rc)
			return rc;
	}

	if (!cold_boot)
		return 0;

	fdt = sbi_scratch_thishart_arg1_ptr();

	fdt_cpu_fixup(fdt);
	fdt_fixups(fdt);
	fdt_domain_fixup(fdt);

	if (generic_plat && generic_plat->fdt_fixup) {
		rc = generic_plat->fdt_fixup(fdt, generic_plat_match);
		if (rc)
			return rc;
	}

	return 0;
}

static void generic_early_exit(void)
{
	if (generic_plat && generic_plat->early_exit)
		generic_plat->early_exit(generic_plat_match);
}

static void generic_final_exit(void)
{
	if (generic_plat && generic_plat->final_exit)
		generic_plat->final_exit(generic_plat_match);
}

static int generic_domains_init(void)
{
	return fdt_domains_populate(sbi_scratch_thishart_arg1_ptr());
}

static u64 generic_tlbr_flush_limit(void)
{
	if (generic_plat && generic_plat->tlbr_flush_limit)
		return generic_plat->tlbr_flush_limit(generic_plat_match);
	return SBI_PLATFORM_TLB_RANGE_FLUSH_LIMIT_DEFAULT;
}

#include <sbi/sbi_trap.h>
#define CSR_MCOUNTERWEN  0x7c9
static void sbi_thead_pmu_init(void)
{
	unsigned long interrupts;

	interrupts = csr_read(CSR_MIDELEG) | (1 << 17);
	csr_write(CSR_MIDELEG, interrupts);

	/* CSR_MCOUNTEREN has already been set in mstatus_init() */
	csr_write(CSR_MCOUNTERWEN, 0xffffffff);
	csr_write(CSR_MHPMEVENT3, 1);
	csr_write(CSR_MHPMEVENT4, 2);
	csr_write(CSR_MHPMEVENT5, 3);
	csr_write(CSR_MHPMEVENT6, 4);
	csr_write(CSR_MHPMEVENT7, 5);
	csr_write(CSR_MHPMEVENT8, 6);
	csr_write(CSR_MHPMEVENT9, 7);
	csr_write(CSR_MHPMEVENT10, 8);
	csr_write(CSR_MHPMEVENT11, 9);
	csr_write(CSR_MHPMEVENT12, 10);
	csr_write(CSR_MHPMEVENT13, 11);
	csr_write(CSR_MHPMEVENT14, 12);
	csr_write(CSR_MHPMEVENT15, 13);
	csr_write(CSR_MHPMEVENT16, 14);
	csr_write(CSR_MHPMEVENT17, 15);
	csr_write(CSR_MHPMEVENT18, 16);
	csr_write(CSR_MHPMEVENT19, 17);
	csr_write(CSR_MHPMEVENT20, 18);
	csr_write(CSR_MHPMEVENT21, 19);
	csr_write(CSR_MHPMEVENT22, 20);
	csr_write(CSR_MHPMEVENT23, 21);
	csr_write(CSR_MHPMEVENT24, 22);
	csr_write(CSR_MHPMEVENT25, 23);
	csr_write(CSR_MHPMEVENT26, 24);
	csr_write(CSR_MHPMEVENT27, 25);
	csr_write(CSR_MHPMEVENT28, 26);
}

static void sbi_thead_pmu_map(unsigned long idx, unsigned long event_id)
{
	switch (idx) {
	case 3:
		csr_write(CSR_MHPMEVENT3, event_id);
		break;
	case 4:
		csr_write(CSR_MHPMEVENT4, event_id);
		break;
	case 5:
		csr_write(CSR_MHPMEVENT5, event_id);
		break;
	case 6:
		csr_write(CSR_MHPMEVENT6, event_id);
		break;
	case 7:
		csr_write(CSR_MHPMEVENT7, event_id);
		break;
	case 8:
		csr_write(CSR_MHPMEVENT8, event_id);
		break;
	case 9:
		csr_write(CSR_MHPMEVENT9, event_id);
		break;
	case 10:
		csr_write(CSR_MHPMEVENT10, event_id);
		break;
	case 11:
		csr_write(CSR_MHPMEVENT11, event_id);
		break;
	case 12:
		csr_write(CSR_MHPMEVENT12, event_id);
		break;
	case 13:
		csr_write(CSR_MHPMEVENT13, event_id);
		break;
	case 14:
		csr_write(CSR_MHPMEVENT14, event_id);
		break;
	case 15:
		csr_write(CSR_MHPMEVENT15, event_id);
		break;
	case 16:
		csr_write(CSR_MHPMEVENT16, event_id);
		break;
	case 17:
		csr_write(CSR_MHPMEVENT17, event_id);
		break;
	case 18:
		csr_write(CSR_MHPMEVENT18, event_id);
		break;
	case 19:
		csr_write(CSR_MHPMEVENT19, event_id);
		break;
	case 20:
		csr_write(CSR_MHPMEVENT20, event_id);
		break;
	case 21:
		csr_write(CSR_MHPMEVENT21, event_id);
		break;
	case 22:
		csr_write(CSR_MHPMEVENT22, event_id);
		break;
	case 23:
		csr_write(CSR_MHPMEVENT23, event_id);
		break;
	case 24:
		csr_write(CSR_MHPMEVENT24, event_id);
		break;
	case 25:
		csr_write(CSR_MHPMEVENT25, event_id);
		break;
	case 26:
		csr_write(CSR_MHPMEVENT26, event_id);
		break;
	case 27:
		csr_write(CSR_MHPMEVENT27, event_id);
		break;
	case 28:
		csr_write(CSR_MHPMEVENT28, event_id);
		break;
	case 29:
		csr_write(CSR_MHPMEVENT29, event_id);
		break;
	case 30:
		csr_write(CSR_MHPMEVENT30, event_id);
		break;
	case 31:
		csr_write(CSR_MHPMEVENT31, event_id);
		break;
	}
}

static void sbi_thead_pmu_set(unsigned long type, unsigned long idx, unsigned long event_id)
{
	switch (type) {
	case 2:
		sbi_thead_pmu_map(idx, event_id);
		break;
	default:
		sbi_thead_pmu_init();
		break;
	}
}

static void sbi_thead_reserved_pmp_set(void)
{
	unsigned int num, reg_val;

	for (num = 0; num < 4; num++) {
		/*	pmp entry 28 for reserved memory	*/
		writel(RESERVED_START_ADDR >> 12, (void *)(PMP_ENTRY_START_ADDR(28) + num*PMP_SIZE_PER_CORE));
		writel(RESERVED_END_ADDR >> 12, (void *)(PMP_ENTRY_END_ADDR(28) + num*PMP_SIZE_PER_CORE));

		/*	pmp entry 28 config	*/
		reg_val = readl((void *)(PMP_ENTRY_CFG_ADDR(28) + num*PMP_SIZE_PER_CORE));
		reg_val = (reg_val & 0xffffff00) | 0x040;
		writel(reg_val, (void *)((PMP_ENTRY_CFG_ADDR(28) + num*PMP_SIZE_PER_CORE)));
	}

	sync_is();
}

static void sbi_thead_tcm0_pmp_set(unsigned long auth)
{
	sbi_printf("%s: auth:%lx \n", __func__, auth);
	unsigned int num, reg_val;

	reg_val = readl((void *)PMP_ENTRY_START_ADDR(26));

	if (reg_val != TCM0_START_ADDR >> 12)
		for(num = 0; num < 4; num++) {
			/*	pmp entry 26 for dsp tcm0	*/
			writel(TCM0_START_ADDR >> 12, (void *)(PMP_ENTRY_START_ADDR(26) + num*PMP_SIZE_PER_CORE));
			writel(TCM0_END_ADDR >> 12, (void *)(PMP_ENTRY_END_ADDR(26) + num*PMP_SIZE_PER_CORE));
		}

	for(num = 0; num < 4; num++) {
		/*	pmp entry 26 config	*/
		reg_val = readl((void *)(PMP_ENTRY_CFG_ADDR(26) + num*PMP_SIZE_PER_CORE));
		reg_val = (reg_val & 0xff00ffff) | (auth << 16);
		writel(reg_val, (void *)(PMP_ENTRY_CFG_ADDR(26) + num*PMP_SIZE_PER_CORE));
	}

	sync_is();
}

static void sbi_thead_tcm1_pmp_set(unsigned long auth)
{
	sbi_printf("%s: auth:%lx \n", __func__, auth);
	unsigned int num, reg_val;

	reg_val = readl((void *)PMP_ENTRY_START_ADDR(27));
	if (reg_val != TCM1_START_ADDR >> 12)
		for (num = 0; num < 4; num++) {
			/*	pmp entry 27 for dsp tcm1	*/
			writel(TCM1_START_ADDR >> 12, (void *)(PMP_ENTRY_START_ADDR(27) + num*PMP_SIZE_PER_CORE));
			writel(TCM1_END_ADDR >> 12, (void *)(PMP_ENTRY_END_ADDR(27) + num*PMP_SIZE_PER_CORE));
		}

	for (num = 0; num < 4; num++) {
		/*	pmp entry 27 config	*/
		reg_val = readl((void *)(PMP_ENTRY_CFG_ADDR(27) + num*PMP_SIZE_PER_CORE));
		reg_val = (reg_val & 0x00ffffff) | (auth << 24);
		writel(reg_val, (void *)(PMP_ENTRY_CFG_ADDR(27) + num*PMP_SIZE_PER_CORE));
	}

	sync_is();
}

static void sbi_thead_pmp_set(unsigned long idx, unsigned long auth)
{
	unsigned int reg_val;

	if (idx !=0 && idx != 1)
		return;

	/*	read pmp entry 28	*/
	reg_val = readl((void *)PMP_ENTRY_START_ADDR(28));

	if (reg_val != RESERVED_START_ADDR >> 12)
		sbi_thead_reserved_pmp_set();

	switch (idx) {
	case 0:
		sbi_thead_tcm0_pmp_set(auth);
		break;
	case 1:
		sbi_thead_tcm1_pmp_set(auth);
		break;
	default:
		break;
	}
}

static int thead_vendor_ext_provider(long extid, long funcid,
	const struct sbi_trap_regs *regs, unsigned long *out_value,
	struct sbi_trap_info *out_trap)
{
	sbi_printf("%s: extid:%lx funcid:%lx \n", __func__,
				   extid, funcid);
	switch (extid) {
	case SBI_EXT_VENDOR_PMU:
		sbi_thead_pmu_set(regs->a0, regs->a1, regs->a2);
		break;
	case SBI_EXT_VENDOR_PMP:
		sbi_thead_pmp_set(funcid, regs->a0);
		break;
	default:
		while(1);
	}
	return 0;
}

const struct sbi_platform_operations platform_ops = {
	.vendor_ext_provider	= thead_vendor_ext_provider,
	.early_init		= generic_early_init,
	.final_init		= generic_final_init,
	.early_exit		= generic_early_exit,
	.final_exit		= generic_final_exit,
	.domains_init		= generic_domains_init,
	.console_init		= fdt_serial_init,
	.irqchip_init		= fdt_irqchip_init,
	.irqchip_exit		= fdt_irqchip_exit,
	.ipi_init		= fdt_ipi_init,
	.ipi_exit		= fdt_ipi_exit,
	.get_tlbr_flush_limit	= generic_tlbr_flush_limit,
	.timer_init		= fdt_timer_init,
	.timer_exit		= fdt_timer_exit,
};

struct sbi_platform platform = {
	.opensbi_version	= OPENSBI_VERSION,
	.platform_version	= SBI_PLATFORM_VERSION(0x0, 0x01),
	.name			= "Generic",
	.features		= SBI_PLATFORM_DEFAULT_FEATURES,
	.hart_count		= SBI_HARTMASK_MAX_BITS,
	.hart_index2id		= generic_hart_index2id,
	.hart_stack_size	= SBI_PLATFORM_DEFAULT_HART_STACK_SIZE,
	.platform_ops_addr	= (unsigned long)&platform_ops
};

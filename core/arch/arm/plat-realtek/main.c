// SPDX-License-Identifier: BSD-2-Clause
/*
 * Copyright (c) 2021, Realtek Semiconductor Corp.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <console.h>
#include <drivers/serial8250_uart.h>
#include <platform_config.h>
#include <stdint.h>
#include <string.h>
#include <drivers/gic.h>
#include <arm.h>
#include <initcall.h>
#include <keep.h>
#include <kernel/boot.h>
#include <kernel/interrupt.h>
#include <kernel/misc.h>
#include <kernel/tee_time.h>
#include <kernel/panic.h>
#include <kernel/cache_helpers.h>
#include <kernel/tee_common_otp.h>
#include <mm/core_memprot.h>
#include <mm/core_mmu.h>
#include <sm/optee_smc.h>
#include <sm/psci.h>
#include <sm/pm.h>
#include <stdint.h>
#include <string.h>
#include <trace.h>
#include <io.h>
#include <kernel/delay.h>


#define REALTEK_AEMBAD2_NUM_CORE		2


#define CA32_BASE						0x41000200
#define SYSTEM_HP						0x41000000
#define SYSTEM_LP						0x42008000
#define KM0_RAM							0x23000000
#define KM0_RAM_SIZE					0x20000
#define KM0_IPC_RAM						0x2301FD00
#define SYSTEM_HP_SRAM					0x30000000
#define MPC1_BASE_S						0x51001A00
#define PLAT_AMEBASMART_WARM_BOOT_BASE		(PLAT_AMEBASMART_TRUSTED_MAILBOX_BASE + 0x100)
#define CNT_CONTROL_BASE				0xB0002000


#define REG_HSYS_HP_PWC					0x0000
#define REG_HSYS_HP_ISO					0x0004
#define CA32_C0_RST_CTRL				0x4
#define CA32_C0_CPU_STATUS				0x8
#define CA32_NCOREPORESET(x)			((uint32_t)(((x) & 0x00000003) << 0))
#define CA32_NCORERESET(x)				((uint32_t)(((x) & 0x00000003) << 4))
#define HSYS_ISO_HP_AP_CORE(x)			((uint32_t)(((x) & 0x00000003) << 4))
#define HSYS_PSW_HP_AP_CORE(x)			((uint32_t)(((x) & 0x00000003) << 4))
#define HSYS_PSW_HP_AP_CORE_2ND(x)		((uint32_t)(((x) & 0x00000003) << 6))
#define CA32_BIT_NRESETSOCDBG			((uint32_t)0x00000001 << 8)
#define CA32_BIT_NL2RESET				((uint32_t)0x00000001 << 24)
#define CA32_BIT_NGICRESET				((uint32_t)0x00000001 << 12)
#define CA32_STANDBYWFI_CORE1			((uint32_t)0x00000001 << 9)

#define REG_LSYS_BOOT_REASON_SW			0x0264
#define REG_LSYS_SW_RST_CTRL			0x0238
#define REG_LSYS_SW_RST_TRIG			0x023C
#define SYS_RESET_KEY					0x96969696
#define SYS_RESET_TRIG 					0x69696969
#define LSYS_BIT_AP_WAKE_FROM_PG		(0x1 << 16)


#define CNTV_CTL_ENABLE					((1) << 0)
#define CNTV_CTL_IMASK					((1) << 1)
#define CNTV_CTL_ISTATUS				((1) << 2)


#define IPCAP_REG_OFFSET				0x580
#define IPC_CHAN_NUM					0x0


#define SYSTIM_CNTCR					(0x0)
#define SYSTIM_CNTSR					(0x4)
#define SYSTIM_CNTCV_RW					(0x8)
#define SYSTIM_CNTFID0					(0x20)
#define SYSTIM_CNTCV_RO					(0x0)
#define SYSTIM_CNT_EN_BIT				(0x1)


#define TRIG							(((uint32_t) 1) << 31)

#define BOOT_ROM_DERIVED_KEY			0x0404
#define PLAT_HW_UNIQUE_KEY_LENGTH		16

typedef struct {
	unsigned int IDAU_BARx;		/*!< ,	Address offset: 0x00 */
	unsigned int IDAU_LARx;		/*!< ,	Address offset: 0x04 */
} MPC_EntryTypeDef;


typedef struct {
	MPC_EntryTypeDef ENTRY[8]; /*!< ,	Address offset: 0x00 ~ 0x3C*/
	unsigned int IDAU_CTRL;		/*!< ,	Address offset: 0x40 */
	unsigned int IDAU_LOCK;		/*!< ,	Address offset: 0x44 */
} MPC_TypeDef;


typedef struct {
	uint32_t	sleep_type;
	uint32_t	sleep_time;
	uint32_t	dlps_enable;
	uint32_t	rsvd[5];
} SLEEP_ParamDef;


enum core_state_id {
	CORE_OFF = 0,
	CORE_ON,
};



static enum core_state_id core_state[REALTEK_AEMBAD2_NUM_CORE] = {0, 0};
static struct serial8250_uart_data console_data;
static struct gic_data gic_data;
static SLEEP_ParamDef pm_param;
static MPC_TypeDef mpc_backup;


register_phys_mem_pgdir(MEM_AREA_IO_SEC,
						CONSOLE_UART_BASE, SERIAL8250_UART_REG_SIZE);

register_phys_mem_pgdir(MEM_AREA_IO_SEC,
						0x41000000, 0x1000);


register_phys_mem_pgdir(MEM_AREA_IO_SEC, GICD_BASE, GIC_DIST_REG_SIZE);

register_phys_mem_pgdir(MEM_AREA_IO_SEC, GICC_BASE, GIC_DIST_REG_SIZE);

register_phys_mem_pgdir(MEM_AREA_IO_SEC,
						PLAT_AMEBASMART_TRUSTED_MAILBOX_BASE, PLAT_AMEBASMART_TRUSTED_MAILBOX_SIZE);

register_phys_mem_pgdir(MEM_AREA_RAM_NSEC, KM0_RAM, KM0_RAM_SIZE);

register_phys_mem_pgdir(MEM_AREA_IO_SEC, CNT_CONTROL_BASE, 0x1000);

register_phys_mem_pgdir(MEM_AREA_IO_SEC, MPC1_BASE_S, 0x1000);

register_phys_mem_pgdir(MEM_AREA_RAM_SEC, SYSTEM_HP_SRAM, 0x1000);


static inline void DelayNop(int count)
{
	int i;

	for (i = 0; i < count; i++) {
		asm volatile("nop");
	}
}


static inline void arm_arch_timer_enable(unsigned char enable)
{
	uint32_t cntv_ctl;

	__asm__ volatile("mrc p15, 0, %0, c14, c3, 1\n\t"
					 : "=r"(cntv_ctl) :  : "memory");

	cntv_ctl &= ~CNTV_CTL_IMASK;
	if (enable) {
		cntv_ctl |= CNTV_CTL_ENABLE;
	} else {
		cntv_ctl &= ~CNTV_CTL_ENABLE;
	}

	__asm__ volatile("mcr p15, 0, %0, c14, c3, 1\n\t"
					 : : "r"(cntv_ctl) : "memory");
}



static void rtk_cpu1_power_down(void)
{
	uint32_t val;
	int cnt = 10000;

	//DMSG("rtk_cpu1_power_down\n");

	/* wait for cpu1 enter wfi state for 10000us*/
	do {
		if ((io_read32((vaddr_t)phys_to_virt_io(CA32_BASE + CA32_C0_CPU_STATUS)) & CA32_STANDBYWFI_CORE1)) {
			break;
		}
		udelay(1);
		if(cnt-- <= 0){
			EMSG("cpu1 enter wfi state timeout!!! \n");
			break;
		}
	} while(cnt > 0);

	val =  io_read32((vaddr_t)phys_to_virt_io(SYSTEM_HP + REG_HSYS_HP_ISO));
	val |= (HSYS_ISO_HP_AP_CORE(0x2));
	io_write32((vaddr_t)phys_to_virt_io(SYSTEM_HP + REG_HSYS_HP_ISO), val);

	udelay(50);

	val = io_read32((vaddr_t)phys_to_virt_io(SYSTEM_HP + REG_HSYS_HP_PWC));
	val &= ~(HSYS_PSW_HP_AP_CORE_2ND(0x2));
	io_write32((vaddr_t)phys_to_virt_io(SYSTEM_HP + REG_HSYS_HP_PWC), val);

	udelay(50);

	val =  io_read32((vaddr_t)phys_to_virt_io(SYSTEM_HP + REG_HSYS_HP_PWC));
	val &= ~(HSYS_PSW_HP_AP_CORE(0x2));
	io_write32((vaddr_t)phys_to_virt_io(SYSTEM_HP + REG_HSYS_HP_PWC), val);

	udelay(50);
}



static void rtk_cpu1_power_on(void)
{
	uint32_t val;

	//DMSG("rtk_cpu1_power_on\n");


	val =  io_read32((vaddr_t)phys_to_virt_io(CA32_BASE + CA32_C0_RST_CTRL));
	val &= (~(CA32_NCOREPORESET(0x2) | CA32_NCORERESET(0x2)));
	io_write32((vaddr_t)phys_to_virt_io(CA32_BASE + CA32_C0_RST_CTRL), val);

	udelay(50);


	val =  io_read32((vaddr_t)phys_to_virt_io(SYSTEM_HP + REG_HSYS_HP_ISO));
	val |= (HSYS_ISO_HP_AP_CORE(0x2));
	io_write32((vaddr_t)phys_to_virt_io(SYSTEM_HP + REG_HSYS_HP_ISO), val);
	udelay(50);

	val =  io_read32((vaddr_t)phys_to_virt_io(SYSTEM_HP + REG_HSYS_HP_PWC));
	val |= (HSYS_PSW_HP_AP_CORE(0x3));
	io_write32((vaddr_t)phys_to_virt_io(SYSTEM_HP + REG_HSYS_HP_PWC), val);
	udelay(50);

	val =  io_read32((vaddr_t)phys_to_virt_io(SYSTEM_HP + REG_HSYS_HP_PWC));
	val |= (HSYS_PSW_HP_AP_CORE_2ND(0x3));
	io_write32((vaddr_t)phys_to_virt_io(SYSTEM_HP + REG_HSYS_HP_PWC), val);
	udelay(500);

	val =  io_read32((vaddr_t)phys_to_virt_io(SYSTEM_HP + REG_HSYS_HP_ISO));
	val &= ~(HSYS_ISO_HP_AP_CORE(0x3));
	io_write32((vaddr_t)phys_to_virt_io(SYSTEM_HP + REG_HSYS_HP_ISO), val);
	udelay(50);

	val =  io_read32((vaddr_t)phys_to_virt_io(CA32_BASE + CA32_C0_RST_CTRL));
	val |= (CA32_NCOREPORESET(0x2) | CA32_NCORERESET(0x2) | CA32_BIT_NRESETSOCDBG | CA32_BIT_NL2RESET | CA32_BIT_NGICRESET);
	io_write32((vaddr_t)phys_to_virt_io(CA32_BASE + CA32_C0_RST_CTRL), val);

	udelay(50);

}



static void rtk_register_online_cpu(void)
{
	size_t pos = get_core_pos();

	if (pos != 0) {
		core_state[pos] = CORE_OFF;
	}
}


void main_init_gic(void)
{
	vaddr_t gicc_base, gicd_base;


	gicc_base = (vaddr_t)phys_to_virt(GICC_BASE,
									  MEM_AREA_IO_SEC);
	gicd_base = (vaddr_t)phys_to_virt(GICD_BASE,
									  MEM_AREA_IO_SEC);

	if (!gicc_base || !gicd_base) {
		panic();
	}

#if defined(CFG_WITH_ARM_TRUSTED_FW)
	/* On ARMv8, GIC configuration is initialized in ARM-TF */
	gic_init_base_addr(&gic_data, gicc_base, gicd_base);
#else
	/* Initialize GIC */
	gic_init(&gic_data, gicc_base, gicd_base);
#endif
	itr_init(&gic_data.chip);

	rtk_register_online_cpu();
}

#if !defined(CFG_WITH_ARM_TRUSTED_FW)
void main_secondary_init_gic(void)
{
	gic_cpu_init(&gic_data);
	rtk_register_online_cpu();

	extern void psci_enable_smp_ca32(void);
	psci_enable_smp_ca32();
}
#endif

void itr_core_handler(void)
{
	gic_it_handle(&gic_data);
}

void console_init(void)
{
	serial8250_uart_init(&console_data, CONSOLE_UART_BASE,
						 CONSOLE_UART_CLK_IN_HZ, CONSOLE_BAUDRATE);
	register_serial_console(&console_data.chip);
}

static void release_secondary_early_hpen(size_t pos)
{
	struct mailbox {
		uint64_t ep;
		uint64_t hpen[];
	} *mailbox;

	if (cpu_mmu_enabled()) {
		mailbox = phys_to_virt(PLAT_AMEBASMART_TRUSTED_MAILBOX_BASE, MEM_AREA_IO_SEC);
	} else {
		mailbox = (void *)PLAT_AMEBASMART_TRUSTED_MAILBOX_BASE;
	}

	if (!mailbox) {
		panic();
	}

	mailbox->ep = TEE_LOAD_ADDR;
	dsb_ishst();
	mailbox->hpen[pos] = 1;
	dsb_ishst();
	sev();
}

static void sys_timer_enable(unsigned char en)
{
	uint32_t value = io_read32((vaddr_t)phys_to_virt(CNT_CONTROL_BASE + SYSTIM_CNTCR, MEM_AREA_IO_SEC));

	if (en) {
		value |= SYSTIM_CNT_EN_BIT;
	} else {
		value &= ~SYSTIM_CNT_EN_BIT;
	}

	io_write32((vaddr_t)phys_to_virt(CNT_CONTROL_BASE + SYSTIM_CNTCR, MEM_AREA_IO_SEC), value);
}



static int rtk_pg_enter(uint32_t arg __unused)
{
	uint32_t msg_idx;
	uint32_t val;
	void *ipc_msg_addr;

	//DMSG("=== Enter PG ===\n");

	// set warm boot entry addr and flag
	io_write32((vaddr_t)phys_to_virt(PLAT_AMEBASMART_WARM_BOOT_BASE, MEM_AREA_IO_SEC), TEE_LOAD_ADDR);
	val = io_read32((vaddr_t)phys_to_virt(SYSTEM_LP + REG_LSYS_BOOT_REASON_SW, MEM_AREA_IO_SEC));
	val |= LSYS_BIT_AP_WAKE_FROM_PG;
	io_write32((vaddr_t)phys_to_virt(SYSTEM_LP + REG_LSYS_BOOT_REASON_SW, MEM_AREA_IO_SEC), val);

	// send IPC msg
	pm_param.dlps_enable = 0;
	pm_param.sleep_time = 0;
	pm_param.sleep_type = 0;

	// IPC_Dir: 0x00000020, IPC_ChNum: 3
	msg_idx = 16 * ((0x00000020 >> 4) & 0xF) + 8 * (0x00000020 & 0xF) + IPC_CHAN_NUM;
	ipc_msg_addr =(void *)((uint32_t)phys_to_virt(KM0_IPC_RAM, MEM_AREA_RAM_NSEC) + msg_idx * 16);
	memcpy(ipc_msg_addr, (void *)&pm_param, sizeof(SLEEP_ParamDef));
	dcache_clean_range(ipc_msg_addr, sizeof(SLEEP_ParamDef));

	val = io_read32((vaddr_t)phys_to_virt(SYSTEM_HP + IPCAP_REG_OFFSET, MEM_AREA_IO_SEC));
	val |= (BIT(IPC_CHAN_NUM + 8));
	io_write32((vaddr_t)phys_to_virt(SYSTEM_HP + IPCAP_REG_OFFSET, MEM_AREA_IO_SEC), val);

	// wait for shutdown
	while (1) {
		asm volatile("wfe");
	}

	// should never reach here
	DMSG("PG ENTER ERROR!\n");

	return -1;
}



static void sm_save_mpc_regs(void)
{
	int i;
	MPC_TypeDef *MPC1 = phys_to_virt_io(MPC1_BASE_S);

	for (i = 0; i < 8; i++) {
		mpc_backup.ENTRY[i].IDAU_BARx = MPC1->ENTRY[i].IDAU_BARx;
		mpc_backup.ENTRY[i].IDAU_LARx = MPC1->ENTRY[i].IDAU_LARx;
	}

	mpc_backup.IDAU_CTRL = MPC1->IDAU_CTRL;
}


static void sm_restore_mpc_regs(void)
{
	int i;
	MPC_TypeDef *MPC1 = phys_to_virt_io(MPC1_BASE_S);

	for (i = 0; i < 8; i++) {
		MPC1->ENTRY[i].IDAU_BARx = mpc_backup.ENTRY[i].IDAU_BARx;
		MPC1->ENTRY[i].IDAU_LARx = mpc_backup.ENTRY[i].IDAU_LARx;
	}

	MPC1->IDAU_CTRL = mpc_backup.IDAU_CTRL;
	MPC1->IDAU_LOCK = 1;
}



static int rtk_cpu_suspend(uint32_t power_state __unused, uintptr_t entry,
						   struct sm_nsec_ctx *nsec)
{
	int ret;
	unsigned int temp1 = 0;
	unsigned int temp2 = 0;
	unsigned int val;

	/* save mpc regs */
	sm_save_mpc_regs();

	/* save unbanked regs */
	sm_save_unbanked_regs(&nsec->ub_regs);

	/* go to suspend */
	ret = sm_pm_cpu_suspend(0, rtk_pg_enter);

	/*
	 * Sometimes sm_pm_cpu_suspend may not really suspended,
	 * we need to check it's return value to restore reg or not
	 */
	if (ret < 0) {
		DMSG("=== Not suspended, GPC IRQ Pending ===\n");
		return PSCI_RET_INTERNAL_FAILURE;
	}

	/* restore unbanked regs */
	sm_restore_unbanked_regs(&nsec->ub_regs);

	/*restore mpc regs*/
	sm_restore_mpc_regs();

	/* enable smp bit */
	__asm volatile("MRRC p15, 1, %0, %1, c15":"=r"(temp1), "=r"(temp2));
	temp1 |= 0x40;
	__asm volatile("MCRR p15, 1, %0, %1, c15"::"r"(temp1), "r"(temp2));

	/* reset suspend flag */
	val = io_read32((vaddr_t)phys_to_virt(SYSTEM_LP + REG_LSYS_BOOT_REASON_SW, MEM_AREA_IO_SEC));
	val &= (~LSYS_BIT_AP_WAKE_FROM_PG);
	io_write32((vaddr_t)phys_to_virt(SYSTEM_LP + REG_LSYS_BOOT_REASON_SW, MEM_AREA_IO_SEC), val);

	/* set entry addr for back to Linux */
	nsec->mon_lr = (uint32_t)entry;

	/* reinit gic */
	main_init_gic();

	/* enable timer */
	sys_timer_enable(1);
	arm_arch_timer_enable(1);

	//DMSG("=== Back from PG ===\n");

	return 0;
}


int psci_cpu_suspend(uint32_t power_state,
					 uintptr_t entry, uint32_t context_id __unused,
					 struct sm_nsec_ctx *nsec)
{
	int ret = PSCI_RET_SUCCESS;
	uint32_t type;

	type = (power_state & PSCI_POWER_STATE_TYPE_MASK) >>
		   PSCI_POWER_STATE_TYPE_SHIFT;

	ret = rtk_cpu_suspend(type, entry, nsec);

	return ret;

}

int psci_affinity_info(uint32_t affinity, uint32_t lowest_affinity_level)
{
	unsigned int pos = get_core_pos_mpidr(affinity);
	//unsigned int pos1 = get_core_pos();

	if ((pos >= REALTEK_AEMBAD2_NUM_CORE) ||
		(lowest_affinity_level > PSCI_AFFINITY_LEVEL_ON)) {
		return PSCI_RET_INVALID_PARAMETERS;
	}

	//DMSG("core %zu, cur core %zu,  state %u", pos, pos1, core_state[pos]);

	if (core_state[pos] == CORE_OFF) {
		rtk_cpu1_power_down();
		return PSCI_AFFINITY_LEVEL_OFF;
	} else {
		return PSCI_AFFINITY_LEVEL_ON;
	}
}


int psci_cpu_on(uint32_t core_id, uint32_t entry, uint32_t context_id)
{
	size_t pos = get_core_pos_mpidr(core_id);

	if (!pos || pos >= REALTEK_AEMBAD2_NUM_CORE) {
		return PSCI_RET_INVALID_PARAMETERS;
	}

	//DMSG("core pos: %zu: ns_entry %#" PRIx32, pos, entry);

	if (core_state[pos] == CORE_ON) {
		DMSG("core %zu already released", pos);
		return PSCI_RET_DENIED;
	} else if (core_state[pos] == CORE_OFF) {
		/*this means core1 resume from hotplug*/
		rtk_cpu1_power_on();
	}
	core_state[pos] = CORE_ON;

	mdelay(1);

	boot_set_core_ns_entry(pos, entry, context_id);

	release_secondary_early_hpen(pos);

	return PSCI_RET_SUCCESS;
}

int psci_cpu_off(void)
{
	uint32_t core = get_core_pos();
	size_t pos = get_core_pos_mpidr(core);

	if ((core == 0) || (core >= REALTEK_AEMBAD2_NUM_CORE)) {
		return PSCI_RET_INVALID_PARAMETERS;
	}

	core_state[pos] = CORE_OFF;

	//DMSG("core pos: %zu,  %zu", core, pos);

	psci_armv7_cpu_off();
	thread_mask_exceptions(THREAD_EXCP_ALL);

	/*wait for cpu0 shutdown cpu1*/
	while (1) {
		wfi();
	}

	return PSCI_RET_INTERNAL_FAILURE;
}



void psci_system_reset(void)
{
	//DMSG("core %u", get_core_pos());

	/*system warm reset*/
	io_write32((vaddr_t)phys_to_virt_io(SYSTEM_LP + REG_LSYS_SW_RST_TRIG), SYS_RESET_KEY);
	io_write32((vaddr_t)phys_to_virt_io(SYSTEM_LP + REG_LSYS_SW_RST_CTRL), TRIG);
	io_write32((vaddr_t)phys_to_virt_io(SYSTEM_LP + REG_LSYS_SW_RST_TRIG), SYS_RESET_TRIG);

	while (1) {
		wfi();
	}
}

TEE_Result tee_otp_get_hw_unique_key(struct tee_hw_unique_key *hwkey)
{
	vaddr_t sram_base = (vaddr_t)phys_to_virt(SYSTEM_HP_SRAM, MEM_AREA_RAM_SEC);

	void *derived_key = sram_base + BOOT_ROM_DERIVED_KEY;
	memcpy(&hwkey->data[0], derived_key, sizeof(hwkey->data));

	return TEE_SUCCESS;
}


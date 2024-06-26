PLATFORM_FLAVOR ?= amebasmart_armv7a

ifeq ($(PLATFORM_FLAVOR),amebasmart_armv7a)
include core/arch/arm/cpu/cortex-a7.mk
endif

$(call force,CFG_8250_UART,y)
$(call force,CFG_GENERIC_BOOT,y)
$(call force,CFG_GIC,y)
$(call force,CFG_HWSUPP_MEM_PERM_PXN,y)
$(call force,CFG_PM_STUBS,y)
$(call force,CFG_SECURE_TIME_SOURCE_CNTPCT,y)

$(call force,CFG_ARM32_core,y)
$(call force,CFG_BOOT_SECONDARY_REQUEST,y)
$(call force,CFG_PSCI_ARM32,y)
$(call force,CFG_PM_ARM32,y)

CFG_WITH_STATS ?= y
CFG_USER_TA_TARGETS = ta_arm32
ta-targets += ta_arm32

CFG_MMAP_REGIONS ?= 20
ifeq (${CONFIG_SOC_CPU_ARMv8_2},y)
CFG_ARM_GICV3 ?= y
endif
CFG_NUM_THREADS ?= 2
CFG_WITH_STACK_CANARIES ?= y
CFG_TEE_CORE_NB_CORE = 2
CFG_INIT_CNTVOFF ?= y
# Location of trusted dram
CFG_TZDRAM_START ?= 0x70200000
CFG_TZDRAM_SIZE  ?= 0x000e0000
CFG_SHMEM_START  ?= 0x602e0000
CFG_SHMEM_SIZE   ?= 0x00020000
CFG_TEE_RAM_VA_SIZE ?= 0x000A0000

PLATFORM_FLAVOR ?= vango

include core/arch/arm/cpu/cortex-a7.mk

$(call force,CFG_8250_UART,y)
$(call force,CFG_GENERIC_BOOT,y)
$(call force,CFG_TEE_CORE_DEBUG,n)
$(call force,CFG_GIC,y)
$(call force,CFG_LEO,y)
$(call force,CFG_ARM32,y)
$(call force,CFG_WITH_LPAE,n)
$(call force,CFG_ARM_GICV3,n)
$(call force,CFG_PSCI_ARM32,y)
$(call force,CFG_CORE_CLUSTER_SHIFT,1)
$(call force,CFG_TEE_CORE_NB_CORE,1)
$(call force,CFG_BOOT_SECONDARY_REQUEST,n)
CFG_TZDRAM_START ?= 0x80e00000
CFG_TZDRAM_SIZE  ?= 0x02000000
CFG_SHMEM_START  ?= 0x80000000
CFG_SHMEM_SIZE   ?= 0x00200000
CFG_TEE_RAM_VA_SIZE := 0x400000
CFG_NS_ENTRY_ADDR ?=0x88008000

$(call force,CFG_PM_STUBS,y)
$(call force,CFG_SECURE_TIME_SOURCE_CNTPCT,y)


ifeq ($(CFG_ARM64_core),y)
$(call force,CFG_WITH_LPAE,y)
else
$(call force,CFG_ARM32_core,y)
endif

ifeq ($(DEBUG),1)
platform-cflags += -gdwarf-2
platform-aflags += -gdwarf-2
endif

PLATFORM_FLAVOR ?= a600

include core/arch/arm/cpu/cortex-armv8-0.mk

$(call force,CFG_8250_UART,y)
$(call force,CFG_GENERIC_BOOT,y)
$(call force,CFG_TEE_CORE_DEBUG,y)
$(call force,CFG_GIC,y)
$(call force,CFG_ARM_GICV3,n)
$(call force,CFG_CORE_CLUSTER_SHIFT,1)
$(call force,CFG_TEE_CORE_NB_CORE,4)
CFG_TZDRAM_START ?= 0xFFDE0000
CFG_TZDRAM_SIZE ?= 0x00200000 # 2048KB
CFG_SHMEM_START ?= 0xFF800000
CFG_SHMEM_SIZE ?=  0x00040000 # 256KB
CFG_TEE_RAM_VA_SIZE := 0x100000 # 1024KB
CFG_NS_ENTRY_ADDR ?=0x80800000

$(call force,CFG_WITH_ARM_TRUSTED_FW,y)
$(call force,CFG_PM_STUBS,y)
$(call force,CFG_SECURE_TIME_SOURCE_CNTPCT,y)

ifeq ($(CFG_ARM64_core),y)
$(call force,CFG_WITH_LPAE,y)
else
$(call force,CFG_ARM32_core,y)
endif

ifeq ($(CFG_SPI),y)
$(call force,CFG_FTSPI020,y)
$(call force,CFG_FTSPI020_V1_1_0,y)
$(call force,CFG_FTSPI020_V1_7_0,n)
endif

ifeq ($(DEBUG),1)
platform-cflags += -gdwarf-2
platform-aflags += -gdwarf-2
endif

supported-ta-targets = ta_arm64

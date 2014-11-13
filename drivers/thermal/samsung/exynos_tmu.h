/*
 * exynos_tmu.h - Samsung EXYNOS TMU (Thermal Management Unit)
 *
 *  Copyright (C) 2011 Samsung Electronics
 *  Donggeun Kim <dg77.kim@samsung.com>
 *  Amit Daniel Kachhap <amit.daniel@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#ifndef _EXYNOS_TMU_H
#define _EXYNOS_TMU_H
#include <linux/cpu_cooling.h>
#include <dt-bindings/thermal/thermal_exynos.h>

enum soc_type {
	SOC_ARCH_EXYNOS3250 = 1,
	SOC_ARCH_EXYNOS4210,
	SOC_ARCH_EXYNOS4412,
	SOC_ARCH_EXYNOS5250,
	SOC_ARCH_EXYNOS5260,
	SOC_ARCH_EXYNOS5420,
	SOC_ARCH_EXYNOS5420_TRIMINFO,
	SOC_ARCH_EXYNOS5440,
	SOC_ARCH_EXYNOS7,
};

/**
 * EXYNOS TMU supported features.
 * TMU_SUPPORT_EMULATION - This features is used to set user defined
 *			temperature to the TMU controller.
 * TMU_SUPPORT_MULTI_INST - This features denotes that the soc
 *			has many instances of TMU.
 * TMU_SUPPORT_FALLING_TRIP - This features shows that interrupt can
 *			be registered for falling trips also.
 * TMU_SUPPORT_EMUL_TIME - This features allows to set next temp emulation
 *			sample time.
 * TMU_SUPPORT_ADDRESS_MULTIPLE - This feature tells that the different TMU
 *			sensors shares some common registers.
 * TMU_SUPPORT - macro to compare the above features with the supplied.
 */
#define TMU_SUPPORT_EMULATION			BIT(0)
#define TMU_SUPPORT_MULTI_INST			BIT(1)
#define TMU_SUPPORT_FALLING_TRIP		BIT(2)
#define TMU_SUPPORT_EMUL_TIME			BIT(3)
#define TMU_SUPPORT_ADDRESS_MULTIPLE		BIT(4)

#define TMU_SUPPORTS(a, b)	(a->features & TMU_SUPPORT_ ## b)

/**
 * struct exynos_tmu_register - register descriptors to access registers.
 * The register validity may vary slightly across different exynos SOC's.
 * @tmu_intstat: Register containing the interrupt status values.
 * @tmu_intclear: Register for clearing the raised interrupt status.
 * @emul_con: TMU emulation controller register.
 */
struct exynos_tmu_registers {
	u32	tmu_intstat;
	u32	tmu_intclear;
	u32	emul_con;
};

/**
 * struct exynos_tmu_platform_data
 * @gain: gain of amplifier in the positive-TC generator block
 *	0 < gain <= 15
 * @reference_voltage: reference voltage of amplifier
 *	in the positive-TC generator block
 *	0 < reference_voltage <= 31
 * @noise_cancel_mode: noise cancellation mode
 *	000, 100, 101, 110 and 111 can be different modes
 * @type: determines the type of SOC
 * @efuse_value: platform defined fuse value
 * @min_efuse_value: minimum valid trimming data
 * @max_efuse_value: maximum valid trimming data
 * @default_temp_offset: default temperature offset in case of no trimming
 * @test_mux; information if SoC supports test MUX
 * @cal_type: calibration type for temperature
 *
 * This structure is required for configuration of exynos_tmu driver.
 */
struct exynos_tmu_platform_data {
	u8 gain;
	u8 reference_voltage;
	u8 noise_cancel_mode;

	u32 efuse_value;
	u32 min_efuse_value;
	u32 max_efuse_value;
	u8 first_point_trim;
	u8 second_point_trim;
	u8 default_temp_offset;
	u8 test_mux;

	enum soc_type type;
	u32 cal_type;
	u32 cal_mode;
};

#endif /* _EXYNOS_TMU_H */

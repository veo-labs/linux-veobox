/*
 * Copyright (C) 2012 Mentor Graphics Inc.
 * Copyright (C) 2005-2009 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 * for more details.
 */
#include <linux/export.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/clkdev.h>
#include <uapi/linux/v4l2-mediabus.h>

#include "ipu-prv.h"

struct ipu_vdi {
	void __iomem *base;
	u32 module;
	spinlock_t lock;
	int use_count;
	struct ipu_soc *ipu;
};


/* VDI Register Offsets */
#define VDI_FSIZE 0x0000
#define VDI_C     0x0004

/* VDI Register Fields */
#define VDI_C_CH_420             (0 << 1)
#define VDI_C_CH_422             (1 << 1)
#define VDI_C_MOT_SEL_MASK       (0x3 << 2)
#define VDI_C_MOT_SEL_FULL       (2 << 2)
#define VDI_C_MOT_SEL_LOW        (1 << 2)
#define VDI_C_MOT_SEL_MED        (0 << 2)
#define VDI_C_BURST_SIZE1_4      (3 << 4)
#define VDI_C_BURST_SIZE2_4      (3 << 8)
#define VDI_C_BURST_SIZE3_4      (3 << 12)
#define VDI_C_BURST_SIZE_MASK    0xF
#define VDI_C_BURST_SIZE1_OFFSET 4
#define VDI_C_BURST_SIZE2_OFFSET 8
#define VDI_C_BURST_SIZE3_OFFSET 12
#define VDI_C_VWM1_SET_1         (0 << 16)
#define VDI_C_VWM1_SET_2         (1 << 16)
#define VDI_C_VWM1_CLR_2         (1 << 19)
#define VDI_C_VWM3_SET_1         (0 << 22)
#define VDI_C_VWM3_SET_2         (1 << 22)
#define VDI_C_VWM3_CLR_2         (1 << 25)
#define VDI_C_TOP_FIELD_MAN_1    (1 << 30)
#define VDI_C_TOP_FIELD_AUTO_1   (1 << 31)

static inline u32 ipu_vdi_read(struct ipu_vdi *vdi, unsigned offset)
{
	return readl(vdi->base + offset);
}

static inline void ipu_vdi_write(struct ipu_vdi *vdi, u32 value,
				 unsigned offset)
{
	writel(value, vdi->base + offset);
}

static void __ipu_vdi_set_top_field_man(struct ipu_vdi *vdi, bool top_field_0)
{
	u32 reg;

	reg = ipu_vdi_read(vdi, VDI_C);
	if (top_field_0)
		reg &= ~VDI_C_TOP_FIELD_MAN_1;
	else
		reg |= VDI_C_TOP_FIELD_MAN_1;
	ipu_vdi_write(vdi, reg, VDI_C);
}

static void __ipu_vdi_set_motion(struct ipu_vdi *vdi,
				 enum ipu_motion_sel motion_sel)
{
	u32 reg;

	reg = ipu_vdi_read(vdi, VDI_C);

	reg &= ~VDI_C_MOT_SEL_MASK;

	switch (motion_sel) {
	case MED_MOTION:
		reg |= VDI_C_MOT_SEL_MED;
		break;
	case HIGH_MOTION:
		reg |= VDI_C_MOT_SEL_FULL;
		break;
	default:
		reg |= VDI_C_MOT_SEL_LOW;
		break;
	}

	ipu_vdi_write(vdi, reg, VDI_C);
}

void ipu_vdi_setup(struct ipu_vdi *vdi, u32 code, int xres, int yres,
		   u32 field, enum ipu_motion_sel motion_sel)
{
	unsigned long flags;
	u32 pixel_fmt, reg;

	spin_lock_irqsave(&vdi->lock, flags);

	reg = ((yres - 1) << 16) | (xres - 1);
	ipu_vdi_write(vdi, reg, VDI_FSIZE);

	/*
	 * Full motion, only vertical filter is used.
	 * Burst size is 4 accesses
	 */
	if (code == V4L2_MBUS_FMT_UYVY8_2X8 ||
	    code == V4L2_MBUS_FMT_UYVY8_1X16 ||
	    code == V4L2_MBUS_FMT_YUYV8_2X8 ||
	    code == V4L2_MBUS_FMT_YUYV8_1X16)
		pixel_fmt = VDI_C_CH_422;
	else
		pixel_fmt = VDI_C_CH_420;

	reg = ipu_vdi_read(vdi, VDI_C);
	reg |= pixel_fmt;
	reg |= VDI_C_BURST_SIZE2_4;
	reg |= VDI_C_BURST_SIZE1_4 | VDI_C_VWM1_CLR_2;
	reg |= VDI_C_BURST_SIZE3_4 | VDI_C_VWM3_CLR_2;
	ipu_vdi_write(vdi, reg, VDI_C);

	if (field == V4L2_FIELD_INTERLACED_TB)
		__ipu_vdi_set_top_field_man(vdi, false);
	else if (field == V4L2_FIELD_INTERLACED_BT)
		__ipu_vdi_set_top_field_man(vdi, true);

	__ipu_vdi_set_motion(vdi, motion_sel);

	spin_unlock_irqrestore(&vdi->lock, flags);
}
EXPORT_SYMBOL_GPL(ipu_vdi_setup);

void ipu_vdi_unsetup(struct ipu_vdi *vdi)
{
	unsigned long flags;

	spin_lock_irqsave(&vdi->lock, flags);
	ipu_vdi_write(vdi, 0, VDI_FSIZE);
	ipu_vdi_write(vdi, 0, VDI_C);
	spin_unlock_irqrestore(&vdi->lock, flags);
}
EXPORT_SYMBOL_GPL(ipu_vdi_unsetup);

void ipu_vdi_toggle_top_field_man(struct ipu_vdi *vdi)
{
	unsigned long flags;
	u32 reg;
	u32 mask_reg;

	spin_lock_irqsave(&vdi->lock, flags);

	reg = ipu_vdi_read(vdi, VDI_C);
	mask_reg = reg & VDI_C_TOP_FIELD_MAN_1;
	if (mask_reg == VDI_C_TOP_FIELD_MAN_1)
		reg &= ~VDI_C_TOP_FIELD_MAN_1;
	else
		reg |= VDI_C_TOP_FIELD_MAN_1;

	ipu_vdi_write(vdi, reg, VDI_C);

	spin_unlock_irqrestore(&vdi->lock, flags);
}
EXPORT_SYMBOL_GPL(ipu_vdi_toggle_top_field_man);

int ipu_vdi_set_src(struct ipu_vdi *vdi, bool csi)
{
	ipu_set_vdi_src_mux(vdi->ipu, csi);
	return 0;
}
EXPORT_SYMBOL_GPL(ipu_vdi_set_src);

int ipu_vdi_enable(struct ipu_vdi *vdi)
{
	unsigned long flags;

	spin_lock_irqsave(&vdi->lock, flags);

	if (!vdi->use_count)
		ipu_module_enable(vdi->ipu, vdi->module);

	vdi->use_count++;

	spin_unlock_irqrestore(&vdi->lock, flags);

	return 0;
}
EXPORT_SYMBOL_GPL(ipu_vdi_enable);

int ipu_vdi_disable(struct ipu_vdi *vdi)
{
	unsigned long flags;

	spin_lock_irqsave(&vdi->lock, flags);

	vdi->use_count--;

	if (!vdi->use_count)
		ipu_module_disable(vdi->ipu, vdi->module);

	if (vdi->use_count < 0)
		vdi->use_count = 0;

	spin_unlock_irqrestore(&vdi->lock, flags);

	return 0;
}
EXPORT_SYMBOL_GPL(ipu_vdi_disable);

struct ipu_vdi *ipu_vdi_get(struct ipu_soc *ipu)
{
	return ipu->vdi_priv;
}
EXPORT_SYMBOL_GPL(ipu_vdi_get);

void ipu_vdi_put(struct ipu_vdi *vdi)
{
}
EXPORT_SYMBOL_GPL(ipu_vdi_put);

int ipu_vdi_init(struct ipu_soc *ipu, struct device *dev,
		 unsigned long base, u32 module)
{
	struct ipu_vdi *vdi;

	vdi = devm_kzalloc(dev, sizeof(*vdi), GFP_KERNEL);
	if (!vdi)
		return -ENOMEM;

	ipu->vdi_priv = vdi;

	spin_lock_init(&vdi->lock);
	vdi->module = module;
	vdi->base = devm_ioremap(dev, base, PAGE_SIZE);
	if (!vdi->base)
		return -ENOMEM;

	dev_dbg(dev, "VDI base: 0x%08lx remapped to %p\n", base, vdi->base);
	vdi->ipu = ipu;

	return 0;
}

void ipu_vdi_exit(struct ipu_soc *ipu)
{
}

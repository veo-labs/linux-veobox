/*
 * Copyright (c) 2010 Sascha Hauer <s.hauer@pengutronix.de>
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
#ifndef __IPU_PRV_H__
#define __IPU_PRV_H__

struct ipu_soc;

#include <linux/types.h>
#include <linux/device.h>
#include <linux/clk.h>
#include <linux/platform_device.h>

#include <video/imx-ipu-v3.h>

#define IPU_MCU_T_DEFAULT	8
#define IPU_CM_IDMAC_REG_OFS	0x00008000
#define IPU_CM_IC_REG_OFS	0x00020000
#define IPU_CM_IRT_REG_OFS	0x00028000
#define IPU_CM_CSI0_REG_OFS	0x00030000
#define IPU_CM_CSI1_REG_OFS	0x00038000
#define IPU_CM_SMFC_REG_OFS	0x00050000
#define IPU_CM_DC_REG_OFS	0x00058000
#define IPU_CM_DMFC_REG_OFS	0x00060000

/* Register addresses */
/* IPU Common registers */
#define IPU_CM_REG(offset)	(offset)

#define IPU_CONF			IPU_CM_REG(0)

#define IPU_SRM_PRI1			IPU_CM_REG(0x00a0)
#define IPU_SRM_PRI2			IPU_CM_REG(0x00a4)
#define IPU_FS_PROC_FLOW1		IPU_CM_REG(0x00a8)
#define IPU_FS_PROC_FLOW2		IPU_CM_REG(0x00ac)
#define IPU_FS_PROC_FLOW3		IPU_CM_REG(0x00b0)
#define IPU_FS_DISP_FLOW1		IPU_CM_REG(0x00b4)
#define IPU_FS_DISP_FLOW2		IPU_CM_REG(0x00b8)
#define IPU_SKIP			IPU_CM_REG(0x00bc)
#define IPU_DISP_ALT_CONF		IPU_CM_REG(0x00c0)
#define IPU_DISP_GEN			IPU_CM_REG(0x00c4)
#define IPU_DISP_ALT1			IPU_CM_REG(0x00c8)
#define IPU_DISP_ALT2			IPU_CM_REG(0x00cc)
#define IPU_DISP_ALT3			IPU_CM_REG(0x00d0)
#define IPU_DISP_ALT4			IPU_CM_REG(0x00d4)
#define IPU_SNOOP			IPU_CM_REG(0x00d8)
#define IPU_MEM_RST			IPU_CM_REG(0x00dc)
#define IPU_PM				IPU_CM_REG(0x00e0)
#define IPU_GPR				IPU_CM_REG(0x00e4)
#define IPU_CHA_DB_MODE_SEL(ch)		IPU_CM_REG(0x0150 + 4 * ((ch) / 32))
#define IPU_ALT_CHA_DB_MODE_SEL(ch)	IPU_CM_REG(0x0168 + 4 * ((ch) / 32))
#define IPU_CHA_CUR_BUF(ch)		IPU_CM_REG(0x023C + 4 * ((ch) / 32))
#define IPU_ALT_CUR_BUF0		IPU_CM_REG(0x0244)
#define IPU_ALT_CUR_BUF1		IPU_CM_REG(0x0248)
#define IPU_SRM_STAT			IPU_CM_REG(0x024C)
#define IPU_PROC_TASK_STAT		IPU_CM_REG(0x0250)
#define IPU_DISP_TASK_STAT		IPU_CM_REG(0x0254)
#define IPU_CHA_BUF0_RDY(ch)		IPU_CM_REG(0x0268 + 4 * ((ch) / 32))
#define IPU_CHA_BUF1_RDY(ch)		IPU_CM_REG(0x0270 + 4 * ((ch) / 32))
#define IPU_CHA_BUF2_RDY(ch)		IPU_CM_REG(0x0288 + 4 * ((ch) / 32))
#define IPU_ALT_CHA_BUF0_RDY(ch)	IPU_CM_REG(0x0278 + 4 * ((ch) / 32))
#define IPU_ALT_CHA_BUF1_RDY(ch)	IPU_CM_REG(0x0280 + 4 * ((ch) / 32))

#define IPU_INT_CTRL(n)		IPU_CM_REG(0x003C + 4 * (n))
#define IPU_INT_STAT(n)		IPU_CM_REG(0x0200 + 4 * (n))

#define IPU_DI0_COUNTER_RELEASE			(1 << 24)
#define IPU_DI1_COUNTER_RELEASE			(1 << 25)

#define FS_PRPENC_ROT_SRC_SEL_MASK     (0xf << 0)
#define FS_PRPENC_ROT_SRC_SEL_OFFSET   0
#define FS_PRPVF_ROT_SRC_SEL_MASK      (0xf << 8)
#define FS_PRPVF_ROT_SRC_SEL_OFFSET    8
#define FS_PP_ROT_SRC_SEL_MASK         (0xf << 16)
#define FS_PP_ROT_SRC_SEL_OFFSET       16
#define FS_PP_SRC_SEL_MASK             (0xf << 12)
#define FS_PP_SRC_SEL_OFFSET           12
#define FS_VDI1_SRC_SEL_MASK           (0x3 << 20)
#define FS_VDI1_SRC_SEL_OFFSET         20
#define FS_VDI3_SRC_SEL_MASK           (0x3 << 22)
#define FS_VDI3_SRC_SEL_OFFSET         22
#define FS_PRP_SRC_SEL_MASK            (0xf << 24)
#define FS_PRP_SRC_SEL_OFFSET          24
#define FS_VF_IN_VALID                 (1 << 31)
#define FS_ENC_IN_VALID                (1 << 30)
#define FS_VDI_SRC_SEL_MASK            (0x3 << 28)

/* The same for most SRC/DEST_SEL fields */
#define FS_SEL_ARM				0

#define FS_PP_SRC_SEL_SMFC0			0x1
#define FS_PP_SRC_SEL_SMFC2			0x3
#define FS_PP_SRC_SEL_IRT_PP			0x6	/* Rotation for post-processing */
#define FS_PP_SRC_SEL_VDOA			0x8

#define FS_PRP_SRC_SEL_SMFC0			0x1
#define FS_PRP_SRC_SEL_SMFC2			0x3
#define FS_PRP_SRC_SEL_IC_DIRECT		0x5
#define FS_PRP_SRC_SEL_IRT_ENC			0x6
#define FS_PRP_SRC_SEL_IRT_VF			0x7

#define FS_PP_ROT_SRC_SEL_SMFC0			0x1
#define FS_PP_ROT_SRC_SEL_SMFC2			0x3
#define FS_PP_ROT_SRC_SEL_POST_PROCESSING	0x5

#define FS_PRPVF_ROT_SRC_SEL_SMFC0		0x1
#define FS_PRPVF_ROT_SRC_SEL_SMFC1		0x2
#define FS_PRPVF_ROT_SRC_SEL_SMFC2		0x3
#define FS_PRPVF_ROT_SRC_SEL_SMFC3		0x4
#define FS_PRPVF_ROT_SRC_SEL_IC_DIRECT		0x5	/* cb7 */
#define FS_PRPVF_ROT_SRC_SEL_VIEWFINDER		0x8

#define FS_PRPENC_ROT_SRC_SEL_SMFC0		0x1
#define FS_PRPENC_ROT_SRC_SEL_SMFC1		0x2
#define FS_PRPENC_ROT_SRC_SEL_SMFC2		0x3
#define FS_PRPENC_ROT_SRC_SEL_SMFC3		0x4
#define FS_PRPENC_ROT_SRC_SEL_IC_DIRECT		0x5	/* cb7 */
#define FS_PRPENC_ROT_SRC_SEL_ENCODING		0x7	/* ch20 */

#define FS_VDI1_SRC_SEL_IRT_VIEWFINDER	0x1	/* ch49 */
#define FS_VDI1_SRC_SEL_IRT_PLAYBACK	0x2	/* ch50 */
#define FS_VDI1_SRC_SEL_POST_PROCESSING	0x3	/* ch22 */

#define FS_VDI3_SRC_SEL_IRT_VIEWFINDER	0x1	/* ch49 */
#define FS_VDI3_SRC_SEL_IRT_PLAYBACK	0x2	/* ch50 */
#define FS_VDI3_SRC_SEL_POST_PROCESSING	0x3	/* ch22 */

#define FS_VDI_SRC_SEL_CSI_DIRECT	0x1	/* CB7 */
#define FS_VDI_SRC_SEL_VDOA		0x2

#define FS_VDOA_DEST_SEL_MASK          (0x3 << 16)
#define FS_VDOA_DEST_SEL_VDI           (0x2 << 16)
#define FS_VDOA_DEST_SEL_IC            (0x1 << 16)
#define FS_VDI_SRC_SEL_OFFSET          28

#define FS_PRPENC_DEST_SEL_MASK        (0xf << 0)
#define FS_PRPENC_DEST_SEL_OFFSET      0
#define FS_PRPVF_DEST_SEL_MASK         (0xf << 4)
#define FS_PRPVF_DEST_SEL_OFFSET       4
#define FS_PRPVF_ROT_DEST_SEL_MASK     (0xf << 8)
#define FS_PRPVF_ROT_DEST_SEL_OFFSET   8
#define FS_PP_DEST_SEL_MASK            (0xf << 12)
#define FS_PP_DEST_SEL_OFFSET          12
#define FS_PP_ROT_DEST_SEL_MASK        (0xf << 16)
#define FS_PP_ROT_DEST_SEL_OFFSET      16
#define FS_PRPENC_ROT_DEST_SEL_MASK    (0xf << 20)
#define FS_PRPENC_ROT_DEST_SEL_OFFSET  20
#define FS_PRP_DEST_SEL_MASK           (0x0f << 24)
#define FS_PRP_DEST_SEL_OFFSET         24

#define FS_PRPENC_DEST_SEL_IRT_ENCODING		0x1
#define FS_PRPENC_DEST_SEL_DC1			0x7	/* ch28 */
#define FS_PRPENC_DEST_SEL_DC2			0x8	/* ch41 */
#define FS_PRPENC_DEST_SEL_DP_SYNC0		0x9	/* ch23 */
#define FS_PRPENC_DEST_SEL_DP_SYNC1		0xa	/* ch27 */
#define FS_PRPENC_DEST_SEL_DP_ASYNC1		0xb	/* ch24 */
#define FS_PRPENC_DEST_SEL_DP_ASYNC0		0xc	/* ch29 */
#define FS_PRPENC_DEST_SEL_ALT_DC2		0xd	/* ch41 */
#define FS_PRPENC_DEST_SEL_ALT_DP_ASYNC1	0xe	/* ch24 */
#define FS_PRPENC_DEST_SEL_ALT_DP_ASYNC0	0xf	/* ch29 */

#define FS_PRPVF_DEST_SEL_IRT_VIEWFINDER	0x1
#define FS_PRPVF_DEST_SEL_DC1			0x7	/* ch28 */
#define FS_PRPVF_DEST_SEL_DC2			0x8	/* ch41 */
#define FS_PRPVF_DEST_SEL_DP_SYNC0		0x9	/* ch27 */
#define FS_PRPVF_DEST_SEL_DP_SYNC1		0xa	/* ch23 */
#define FS_PRPVF_DEST_SEL_DP_ASYNC1		0xb	/* ch24 */
#define FS_PRPVF_DEST_SEL_DP_ASYNC0		0xc	/* ch29 */
#define FS_PRPVF_DEST_SEL_ALT_DC2		0xd	/* ch41 */
#define FS_PRPVF_DEST_SEL_ALT_DP_ASYNC1		0xe	/* ch24 */
#define FS_PRPVF_DEST_SEL_ALT_DP_ASYNC0		0xf	/* ch29 */

#define FS_PRPVF_ROT_DEST_SEL_VDI_PLANE3	0x3	/* ch25 */
#define FS_PRPVF_ROT_DEST_SEL_VDI_PLANE1	0x4	/* ch26 */
#define FS_PRPVF_ROT_DEST_SEL_IC_PRP		0x5
#define FS_PRPVF_ROT_DEST_SEL_DC1		0x7	/* ch28 */
#define FS_PRPVF_ROT_DEST_SEL_DC2		0x8	/* ch41 */
#define FS_PRPVF_ROT_DEST_SEL_DP_SYNC0		0x9	/* ch23 */
#define FS_PRPVF_ROT_DEST_SEL_DP_SYNC1		0xa	/* ch27 */
#define FS_PRPVF_ROT_DEST_SEL_DP_ASYNC1		0xb	/* ch24 */
#define FS_PRPVF_ROT_DEST_SEL_DP_ASYNC0		0xc	/* ch29 */
#define FS_PRPVF_ROT_DEST_SEL_ALT_DC2		0xd	/* ch41 */
#define FS_PRPVF_ROT_DEST_SEL_ALT_DP_ASYNC1	0xe	/* ch24 */
#define FS_PRPVF_ROT_DEST_SEL_ALT_DP_ASYNC0	0xf	/* ch29 */

#define FS_PP_DEST_SEL_IRT_PLAYBACK		0x3	/* IRT playback */
#define FS_PP_DEST_SEL_VDI_PLANE3		0x4	/* ch25 */
#define FS_PP_DEST_SEL_VDI_PLANE1		0x5	/* ch26 */
#define FS_PP_DEST_SEL_DC1			0x7	/* ch28 */
#define FS_PP_DEST_SEL_DC2			0x8	/* ch41 */
#define FS_PP_DEST_SEL_DP_SYNC0			0x9	/* ch23 */
#define FS_PP_DEST_SEL_DP_SYNC1			0xa	/* ch27 */
#define FS_PP_DEST_SEL_DP_ASYNC1		0xb	/* ch24 */
#define FS_PP_DEST_SEL_DP_ASYNC0		0xc	/* ch29 */
#define FS_PP_DEST_SEL_ALT_DC2			0xd	/* ch41 */
#define FS_PP_DEST_SEL_ALT_DP_ASYNC1		0xe	/* ch24 */
#define FS_PP_DEST_SEL_ALT_DP_ASYNC0		0xf	/* ch29 */

#define FS_PP_ROT_DEST_SEL_IC_PLAYBACK		0x4	/* Post Processing */
#define FS_PP_ROT_DEST_SEL_VDI_PLANE3		0x5	/* ch25 */
#define FS_PP_ROT_DEST_SEL_VDI_PLANE1		0x6	/* ch26 */
#define FS_PP_ROT_DEST_SEL_DC1			0x7	/* ch28 */
#define FS_PP_ROT_DEST_SEL_DC2			0x8	/* ch41 */
#define FS_PP_ROT_DEST_SEL_DP_SYNC0		0x9	/* ch23 */
#define FS_PP_ROT_DEST_SEL_DP_SYNC1		0xa	/* ch27 */
#define FS_PP_ROT_DEST_SEL_DP_ASYNC1		0xb	/* ch24 */
#define FS_PP_ROT_DEST_SEL_DP_ASYNC0		0xc	/* ch29 */
#define FS_PP_ROT_DEST_SEL_ALT_DC2		0xd	/* ch41 */
#define FS_PP_ROT_DEST_SEL_ALT_DP_ASYNC1	0xe	/* ch24 */
#define FS_PP_ROT_DEST_SEL_ALT_DP_ASYNC0	0xf	/* ch29 */

#define FS_PRPENC_ROT_DEST_SEL_IC_PRP		0x5
#define FS_PRPENC_ROT_DEST_SEL_DC1		0x7	/* ch28 */
#define FS_PRPENC_ROT_DEST_SEL_DC2		0x8	/* ch41 */
#define FS_PRPENC_ROT_DEST_SEL_DP_SYNC0		0x9	/* ch23 */
#define FS_PRPENC_ROT_DEST_SEL_DP_SYNC1		0xa	/* ch27 */
#define FS_PRPENC_ROT_DEST_SEL_DP_ASYNC1	0xb	/* ch24 */
#define FS_PRPENC_ROT_DEST_SEL_DP_ASYNC0	0xc	/* ch29 */
#define FS_PRPENC_ROT_DEST_SEL_ALT_DC2		0xd	/* ch41 */
#define FS_PRPENC_ROT_DEST_SEL_ALT_DP_ASYNC1	0xe	/* ch24 */
#define FS_PRPENC_ROT_DEST_SEL_ALT_DP_ASYNC0	0xf	/* ch29 */

#define FS_PRP_DEST_SEL_IC_INPUT		0x1	/* ch12 */
#define FS_PRP_DEST_SEL_PP			0x2	/* ch11 */
#define FS_PRP_DEST_SEL_PP_ROT			0x3	/* ch47 */
#define FS_PRP_DEST_SEL_DC1			0x4	/* ch28 */
#define FS_PRP_DEST_SEL_DC2			0x5	/* ch41 */
#define FS_PRP_DEST_SEL_DP_ASYNC1		0x6	/* ch24 */
#define FS_PRP_DEST_SEL_DP_ASYNC0		0x7	/* ch29 */
#define FS_PRP_DEST_SEL_DP_SYNC1		0x8	/* ch27 */
#define FS_PRP_DEST_SEL_DP_SYNC0		0x9	/* ch23 */
#define FS_PRP_DEST_SEL_ALT_DC2			0xa	/* ch41 */
#define FS_PRP_DEST_SEL_ALT_DP_ASYNC1		0xb	/* ch24 */
#define FS_PRP_DEST_SEL_ALT_DP_ASYNC0		0xc	/* ch29 */

#define FS_SMFC0_DEST_SEL_MASK         (0xf << 0)
#define FS_SMFC0_DEST_SEL_OFFSET       0
#define FS_SMFC1_DEST_SEL_MASK         (0x7 << 4)
#define FS_SMFC1_DEST_SEL_OFFSET       4
#define FS_SMFC2_DEST_SEL_MASK         (0xf << 7)
#define FS_SMFC2_DEST_SEL_OFFSET       7
#define FS_SMFC3_DEST_SEL_MASK         (0x7 << 11)
#define FS_SMFC3_DEST_SEL_OFFSET       11

/* SMFC0 to SMFC3 */
#define FS_SMFC_DEST_SEL_IRT_ENC		0x1
#define FS_SMFC_DEST_SEL_IRT_VF			0x2
#define FS_SMFC_DEST_SEL_IRT_PP			0x3
#define FS_SMFC_DEST_SEL_IC_PP			0x4
#define FS_SMFC_DEST_SEL_IC_PRP			0x5

/* SMFC0 and SMFC2 only */
#define FS_SMFC_DEST_SEL_DC1			0x7	/* ch28 */
#define FS_SMFC_DEST_SEL_DC2			0x8	/* ch41 */
#define FS_SMFC_DEST_SEL_DP_SYNC0		0x9	/* ch23 */
#define FS_SMFC_DEST_SEL_DP_SYNC1		0xa	/* ch27 */
#define FS_SMFC_DEST_SEL_DP_ASYNC1		0xb	/* ch24 */
#define FS_SMFC_DEST_SEL_DP_ASYNC0		0xc	/* ch29 */
#define FS_SMFC_DEST_SEL_ALT_DC2		0xd	/* ch41 */
#define FS_SMFC_DEST_SEL_ALT_DP_ASYNC1		0xe	/* ch24 */
#define FS_SMFC_DEST_SEL_ALT_DP_ASYNC0		0xf	/* ch29 */

#define FS_DC1_SRC_SEL_MASK            (0xf << 20)
#define FS_DC1_SRC_SEL_OFFSET          20
#define FS_DC2_SRC_SEL_MASK            (0xf << 16)
#define FS_DC2_SRC_SEL_OFFSET          16
#define FS_DP_SYNC0_SRC_SEL_MASK       (0xf << 0)
#define FS_DP_SYNC0_SRC_SEL_OFFSET     0
#define FS_DP_SYNC1_SRC_SEL_MASK       (0xf << 4)
#define FS_DP_SYNC1_SRC_SEL_OFFSET     4
#define FS_DP_ASYNC0_SRC_SEL_MASK      (0xf << 8)
#define FS_DP_ASYNC0_SRC_SEL_OFFSET    8
#define FS_DP_ASYNC1_SRC_SEL_MASK      (0xf << 12)
#define FS_DP_ASYNC1_SRC_SEL_OFFSET    12

#define FS_AUTO_REF_PER_MASK           0
#define FS_AUTO_REF_PER_OFFSET         16

#define IPU_IDMAC_REG(offset)	(offset)

#define IDMAC_CONF			IPU_IDMAC_REG(0x0000)
#define IDMAC_CHA_EN(ch)		IPU_IDMAC_REG(0x0004 + 4 * ((ch) / 32))
#define IDMAC_SEP_ALPHA			IPU_IDMAC_REG(0x000c)
#define IDMAC_ALT_SEP_ALPHA		IPU_IDMAC_REG(0x0010)
#define IDMAC_CHA_PRI(ch)		IPU_IDMAC_REG(0x0014 + 4 * ((ch) / 32))
#define IDMAC_WM_EN(ch)			IPU_IDMAC_REG(0x001c + 4 * ((ch) / 32))
#define IDMAC_CH_LOCK_EN_1		IPU_IDMAC_REG(0x0024)
#define IDMAC_CH_LOCK_EN_2		IPU_IDMAC_REG(0x0028)
#define IDMAC_SUB_ADDR_0		IPU_IDMAC_REG(0x002c)
#define IDMAC_SUB_ADDR_1		IPU_IDMAC_REG(0x0030)
#define IDMAC_SUB_ADDR_2		IPU_IDMAC_REG(0x0034)
#define IDMAC_BAND_EN(ch)		IPU_IDMAC_REG(0x0040 + 4 * ((ch) / 32))
#define IDMAC_CHA_BUSY(ch)		IPU_IDMAC_REG(0x0100 + 4 * ((ch) / 32))

#define IPU_NUM_IRQS	(32 * 15)

enum ipu_modules {
	IPU_CONF_CSI0_EN		= (1 << 0),
	IPU_CONF_CSI1_EN		= (1 << 1),
	IPU_CONF_IC_EN			= (1 << 2),
	IPU_CONF_ROT_EN			= (1 << 3),
	IPU_CONF_ISP_EN			= (1 << 4),
	IPU_CONF_DP_EN			= (1 << 5),
	IPU_CONF_DI0_EN			= (1 << 6),
	IPU_CONF_DI1_EN			= (1 << 7),
	IPU_CONF_SMFC_EN		= (1 << 8),
	IPU_CONF_DC_EN			= (1 << 9),
	IPU_CONF_DMFC_EN		= (1 << 10),

	IPU_CONF_VDI_EN			= (1 << 12),

	IPU_CONF_IDMAC_DIS		= (1 << 22),

	IPU_CONF_IC_DMFC_SEL		= (1 << 25),
	IPU_CONF_IC_DMFC_SYNC		= (1 << 26),
	IPU_CONF_VDI_DMFC_SYNC		= (1 << 27),

	IPU_CONF_CSI0_DATA_SOURCE	= (1 << 28),
	IPU_CONF_CSI1_DATA_SOURCE	= (1 << 29),
	IPU_CONF_IC_INPUT		= (1 << 30),
	IPU_CONF_CSI_SEL		= (1 << 31),
};

struct ipuv3_channel {
	unsigned int num;

	bool enabled;
	bool busy;

	struct ipu_soc *ipu;
};

struct ipu_cpmem;
struct ipu_csi;
struct ipu_dc_priv;
struct ipu_dmfc_priv;
struct ipu_di;
struct ipu_ic_priv;
struct ipu_smfc_priv;

struct ipu_devtype;
struct v4l2_subdev;

struct ipu_soc {
	struct device		*dev;
	const struct ipu_devtype	*devtype;
	enum ipuv3_type		ipu_type;
	spinlock_t		lock;
	struct mutex		channel_lock;

	void __iomem		*cm_reg;
	void __iomem		*idmac_reg;

	int			id;
	int			usecount;

	struct clk		*clk;

	struct ipuv3_channel	channel[64];

	int			irq_sync;
	int			irq_err;
	struct irq_domain	*domain;

	struct ipu_cpmem	*cpmem_priv;
	struct ipu_dc_priv	*dc_priv;
	struct ipu_dp_priv	*dp_priv;
	struct ipu_dmfc_priv	*dmfc_priv;
	struct ipu_di		*di_priv[2];
	struct ipu_csi		*csi_priv[2];
	struct ipu_ic_priv	*ic_priv;
	struct ipu_smfc_priv	*smfc_priv;

	struct v4l2_subdev	*subdevs[IPU_NUM_ENTITIES];
};

void ipu_cm_update_bits(struct ipu_soc *ipu, unsigned int reg,
			unsigned int mask, unsigned int val);

static inline u32 ipu_idmac_read(struct ipu_soc *ipu, unsigned offset)
{
	return readl(ipu->idmac_reg + offset);
}

static inline void ipu_idmac_write(struct ipu_soc *ipu, u32 value,
				   unsigned offset)
{
	writel(value, ipu->idmac_reg + offset);
}

void ipu_srm_dp_sync_update(struct ipu_soc *ipu);

int ipu_module_enable(struct ipu_soc *ipu, u32 mask);
int ipu_module_disable(struct ipu_soc *ipu, u32 mask);

bool ipu_idmac_channel_busy(struct ipu_soc *ipu, unsigned int chno);
int ipu_wait_interrupt(struct ipu_soc *ipu, int irq, int ms);

int ipu_csi_init(struct ipu_soc *ipu, struct device *dev, int id,
		 unsigned long base, u32 module, struct clk *clk_ipu);
void ipu_csi_exit(struct ipu_soc *ipu, int id);
int ipu_csi_set_dest(struct ipu_csi *csi, enum ipu_csi_dest csi_dest);

int ipu_ic_init(struct ipu_soc *ipu, struct device *dev,
		unsigned long base, unsigned long tpmem_base);
void ipu_ic_exit(struct ipu_soc *ipu);
void ipu_ic_csi_mem_wr_en(struct ipu_ic_priv *priv, bool mem_wr_en);

int ipu_di_init(struct ipu_soc *ipu, struct device *dev, int id,
		unsigned long base, u32 module, struct clk *ipu_clk);
void ipu_di_exit(struct ipu_soc *ipu, int id);

int ipu_dmfc_init(struct ipu_soc *ipu, struct device *dev, unsigned long base,
		struct clk *ipu_clk);
void ipu_dmfc_exit(struct ipu_soc *ipu);

int ipu_dp_init(struct ipu_soc *ipu, struct device *dev, unsigned long base);
void ipu_dp_exit(struct ipu_soc *ipu);

int ipu_dc_init(struct ipu_soc *ipu, struct device *dev, unsigned long base,
		unsigned long template_base);
void ipu_dc_exit(struct ipu_soc *ipu);

int ipu_cpmem_init(struct ipu_soc *ipu, struct device *dev, unsigned long base);
void ipu_cpmem_exit(struct ipu_soc *ipu);

int ipu_smfc_init(struct ipu_soc *ipu, struct device *dev, unsigned long base);
void ipu_smfc_exit(struct ipu_soc *ipu);
int ipu_smfc_set_csi(struct ipu_soc *ipu, int chno, int csi_id);

int ipu_media_init(struct ipu_soc *ipu);
void ipu_media_exit(struct ipu_soc *ipu);

#endif				/* __IPU_PRV_H__ */

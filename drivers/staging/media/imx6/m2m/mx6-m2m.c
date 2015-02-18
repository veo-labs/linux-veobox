/*
 * This is a mem2mem driver for the Freescale i.MX6 SOC. It carries out
 * color-space conversion, downsizing, resizing, and rotation transformations
 * on input buffers using the IPU Image Converter's Post-Processing task.
 *
 * Based on mem2mem_testdev.c by Pawel Osciak.
 *
 * Copyright (c) 2012-2013 Mentor Graphics Inc.
 * Steve Longerbeam <steve_longerbeam@mentor.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the
 * License, or (at your option) any later version
 */
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/fs.h>
#include <linux/log2.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/timer.h>
#include <media/imx6.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-dma-contig.h>
#include <video/imx-ipu-v3.h>

MODULE_DESCRIPTION("i.MX6 Post-Processing mem2mem device");
MODULE_AUTHOR("Steve Longerbeam <steve_longerbeam@mentor.com>");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.1");

static int instrument;
module_param(instrument, int, 0);
MODULE_PARM_DESC(instrument, "1 = enable conversion time measurement");

#define MIN_W		128
#define MIN_H		128
#define MAX_W		4096
#define MAX_H		4096
#define DEFAULT_W	1280
#define DEFAULT_H	720
#define S_ALIGN		1 /* multiple of  2 pixels */

/*
 * The IC Resizer has a restriction that the output frame from the
 * resizer must be 1024 or less in both width (pixels) and height
 * (lines).
 *
 * This driver attempts to split up a conversion when the desired
 * capture (output) frame resolution exceeds the IC resizer limit
 * of 1024 in either dimension.
 *
 * If either dimension of the output frame exceeds the limit, the
 * dimension is split into 1, 2, or 4 equal stripes, for a maximum
 * of 4*4 or 16 segments. A conversion is then carried out for each
 * segment (but taking care to pass the full frame stride length to
 * the DMA channel's parameter memory!). IDMA double-buffering is used
 * to convert each segment back-to-back when possible (see note below
 * when double_buffering boolean is set).
 *
 * Note that the input frame must be split up into the same number
 * of segments as the output frame.
 */
#define MAX_SEG_W    4
#define MAX_SEG_H    4
#define MAX_SEGMENTS (MAX_SEG_W * MAX_SEG_H)


/* Flags that indicate a format can be used for capture/output */
#define MEM2MEM_CAPTURE	(1 << 0)
#define MEM2MEM_OUTPUT	(1 << 1)

#define MEM2MEM_NAME		"mx6-m2m"

/* Per queue */
#define MEM2MEM_DEF_NUM_BUFS	VIDEO_MAX_FRAME
/* In bytes, per queue */
#define MEM2MEM_VID_MEM_LIMIT	SZ_256M

#define dprintk(dev, fmt, arg...) \
	v4l2_dbg(1, 1, &dev->v4l2_dev, "%s: " fmt, __func__, ## arg)

struct m2mx6_pixfmt {
	char	*name;
	u32	fourcc;
	int     depth;		/* total bpp */
	int     y_depth;	/* depth of Y plane for planar formats */
	int     ybpp;		/* bpp of Y plane for planar formats */
	int     uv_width_dec;	/* decimation in width for U/V planes */
	int     uv_height_dec;	/* decimation in height for U/V planes */
	bool    uv_packed;	/* partial planar (U and V in same plane) */
	u32	types;		/* Types the format can be used for */
};

struct m2mx6_seg_off {
	/* start Y or packed offset of this segment */
	u32     offset;
	/* offset from start to segment in U plane, for planar formats */
	u32     u_off;
	/* offset from start to segment in V plane, for planar formats */
	u32     v_off;
};

/* Per-queue, driver-specific private data */
struct m2mx6_q_data {
	unsigned int	width;
	unsigned int	height;
	unsigned int    bytesperline;
	unsigned int    stride;
	unsigned int    rot_stride;
	unsigned int    sizeimage;
	struct m2mx6_pixfmt *fmt;

	dma_addr_t      phys_start;
	struct m2mx6_seg_off seg_off[MAX_SEGMENTS];
	/* width of each segment */
	unsigned int    seg_width;
	/* height of each segment */
	unsigned int    seg_height;
	unsigned int	sequence;
};

struct m2mx6_dev {
	struct v4l2_device	v4l2_dev;
	struct video_device	*vfd;

	struct mutex		dev_mutex;
	spinlock_t		irqlock;

	struct ipu_soc          *ipu;

	struct v4l2_m2m_dev	*m2m_dev;
	struct vb2_alloc_ctx    *alloc_ctx;
};

struct m2mx6_ctx {
	struct v4l2_fh		fh;
	struct m2mx6_dev	*dev;

	struct v4l2_ctrl_handler hdl;

	/* the IPU resources this context will need */
	struct ipuv3_channel    *ipu_mem_pp_ch;
	struct ipuv3_channel    *ipu_pp_mem_ch;
	struct ipuv3_channel    *ipu_mem_rot_pp_ch;
	struct ipuv3_channel    *ipu_rot_pp_mem_ch;
	struct ipu_ic           *ic;

	/* the IPU end-of-frame irqs */
	int                     pp_mem_irq;
	int                     rot_pp_mem_irq;
	/* current buffer number for double buffering */
	int                     cur_buf_num;

	/* intermediate buffer for rotation */
	void                    *rot_intermediate_buf[2];
	dma_addr_t              rot_intermediate_phys[2];
	unsigned long           rot_intermediate_buf_size;

	/* The rotation controls */
	int                     rotation; /* degrees */
	bool                    hflip;
	bool                    vflip;

	/* derived from rotation, hflip, vflip controls */
	enum ipu_rotate_mode    rot_mode;

	/* Abort requested by m2m */
	int			aborting;

	struct v4l2_m2m_ctx	*m2m_ctx;

	/* Source and destination queue data */
	struct m2mx6_q_data     q_data[2];

	/* can we use double-buffering for this operation? */
	bool              double_buffering;
	/* # of rows (horizontal stripes) if dest height is > 1024 */
	unsigned int      num_rows;
	/* # of columns (vertical stripes) if dest width is > 1024 */
	unsigned int      num_cols;
	/* num_rows * num_cols */
	unsigned int      num_segs;
	/* Next segment to process */
	unsigned int      next_seg;

	/* for instrumenting */
	struct timeval    start;
};

enum {
	V4L2_M2M_SRC = 0,
	V4L2_M2M_DST = 1,
};


static struct m2mx6_pixfmt m2mx6_formats[] = {
	{
		.name	= "RGB565",
		.fourcc	= V4L2_PIX_FMT_RGB565,
		.depth  = 16,
		.ybpp	= 2,
		.types	= MEM2MEM_CAPTURE | MEM2MEM_OUTPUT,
	}, {
		.name	= "RGB24",
		.fourcc	= V4L2_PIX_FMT_RGB24,
		.depth  = 24,
		.ybpp	= 3,
		.types	= MEM2MEM_CAPTURE | MEM2MEM_OUTPUT,
	}, {
		.name	= "BGR24",
		.fourcc	= V4L2_PIX_FMT_BGR24,
		.depth  = 24,
		.ybpp	= 3,
		.types	= MEM2MEM_CAPTURE | MEM2MEM_OUTPUT,
	}, {
		.name	= "RGB32",
		.fourcc	= V4L2_PIX_FMT_RGB32,
		.depth  = 32,
		.ybpp	= 4,
		.types	= MEM2MEM_CAPTURE | MEM2MEM_OUTPUT,
	}, {
		.name	= "BGR32",
		.fourcc	= V4L2_PIX_FMT_BGR32,
		.depth  = 32,
		.ybpp	= 4,
		.types	= MEM2MEM_CAPTURE | MEM2MEM_OUTPUT,
	}, {
		.name	= "4:2:2 packed, YUYV",
		.fourcc	= V4L2_PIX_FMT_YUYV,
		.depth  = 16,
		.ybpp	= 2,
		.uv_width_dec = 2,
		.uv_height_dec = 1,
		.types	= MEM2MEM_CAPTURE | MEM2MEM_OUTPUT,
	}, {
		.name	= "4:2:2 packed, UYVY",
		.fourcc	= V4L2_PIX_FMT_UYVY,
		.depth  = 16,
		.ybpp	= 2,
		.uv_width_dec = 2,
		.uv_height_dec = 1,
		.types	= MEM2MEM_CAPTURE | MEM2MEM_OUTPUT,
	}, {
		.name	= "4:2:0 planar, YUV",
		.fourcc	= V4L2_PIX_FMT_YUV420,
		.depth  = 12,
		.ybpp	= 1,
		.uv_width_dec = 2,
		.uv_height_dec = 2,
		.types	= MEM2MEM_CAPTURE | MEM2MEM_OUTPUT,
	}, {
		.name	= "4:2:0 planar, YVU",
		.fourcc	= V4L2_PIX_FMT_YVU420,
		.depth  = 12,
		.ybpp	= 1,
		.uv_width_dec = 2,
		.uv_height_dec = 2,
		.types	= MEM2MEM_CAPTURE | MEM2MEM_OUTPUT,
	}, {
		.name   = "4:2:0 partial planar, NV12",
		.fourcc = V4L2_PIX_FMT_NV12,
		.depth  = 12,
		.ybpp	= 1,
		.y_depth = 8,
		.uv_width_dec = 2,
		.uv_height_dec = 2,
		.uv_packed = true,
		.types	= MEM2MEM_CAPTURE | MEM2MEM_OUTPUT,
	}, {
		.name   = "4:2:0 partial planar, NV21",
		.fourcc = V4L2_PIX_FMT_NV21,
		.depth  = 12,
		.ybpp	= 1,
		.y_depth = 8,
		.uv_width_dec = 2,
		.uv_height_dec = 2,
		.uv_packed = true,
		.types	= MEM2MEM_CAPTURE | MEM2MEM_OUTPUT,
	}, {
		.name   = "4:2:2 planar, YUV",
		.fourcc = V4L2_PIX_FMT_YUV422P,
		.depth  = 16,
		.ybpp	= 1,
		.y_depth = 8,
		.uv_width_dec = 2,
		.uv_height_dec = 1,
		.types	= MEM2MEM_CAPTURE | MEM2MEM_OUTPUT,
	}, {
		.name   = "4:2:2 partial planar, NV16",
		.fourcc = V4L2_PIX_FMT_NV16,
		.depth  = 16,
		.ybpp	= 1,
		.y_depth = 8,
		.uv_width_dec = 2,
		.uv_height_dec = 1,
		.uv_packed = true,
		.types	= MEM2MEM_CAPTURE | MEM2MEM_OUTPUT,
	}, {
		.name   = "4:2:2 partial planar, NV61",
		.fourcc = V4L2_PIX_FMT_NV61,
		.depth  = 16,
		.ybpp	= 1,
		.y_depth = 8,
		.uv_width_dec = 2,
		.uv_height_dec = 1,
		.uv_packed = true,
		.types	= MEM2MEM_CAPTURE | MEM2MEM_OUTPUT,
	},
};

#define NUM_FORMATS ARRAY_SIZE(m2mx6_formats)

static struct m2mx6_pixfmt *m2mx6_get_format(struct v4l2_format *f)
{
	struct m2mx6_pixfmt *ret = NULL;
	int i;

	for (i = 0; i < NUM_FORMATS; i++) {
		if (m2mx6_formats[i].fourcc == f->fmt.pix.pixelformat) {
			ret = &m2mx6_formats[i];
			break;
		}
	}

	return ret;
}

static struct m2mx6_pixfmt *m2mx6_get_format_from_fourcc(u32 fourcc)
{
	struct m2mx6_pixfmt *ret = NULL;
	int i;

	for (i = 0; i < NUM_FORMATS; i++) {
		if (m2mx6_formats[i].fourcc == fourcc) {
			ret = &m2mx6_formats[i];
			break;
		}
	}

	return ret;
}

static struct m2mx6_q_data *get_q_data(struct m2mx6_ctx *ctx,
				       enum v4l2_buf_type type)
{
	switch (type) {
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
		return &ctx->q_data[V4L2_M2M_SRC];
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		return &ctx->q_data[V4L2_M2M_DST];
	default:
		BUG();
	}
	return NULL;
}

static inline int m2mx6_num_stripes(int dim)
{
	if (dim <= 1024)
		return 1;
	else if (dim <= 2048)
		return 2;
	else
		return 4;
}

static void m2mx6_calc_seg_offsets_planar(struct m2mx6_ctx *ctx,
					  enum v4l2_buf_type type)
{
	struct m2mx6_q_data *q_data = get_q_data(ctx, type);
	struct m2mx6_pixfmt *fmt = q_data->fmt;
	u32 W, H, w, h, y_depth, y_stride, uv_stride;
	u32 uv_row_off, uv_col_off, uv_off, u_off, v_off;
	u32 y_row_off, y_col_off, y_off;
	u32 y_size, uv_size;
	int row, col, seg = 0;

	/* setup some convenience vars */
	W = q_data->width;
	H = q_data->height;
	w = q_data->seg_width;
	h = q_data->seg_height;

	y_depth = fmt->y_depth;
	y_stride = q_data->stride;
	uv_stride = y_stride / fmt->uv_width_dec;
	if (fmt->uv_packed)
		uv_stride *= 2;

	y_size = H * y_stride;
	uv_size = y_size / (fmt->uv_width_dec * fmt->uv_height_dec);

	for (row = 0; row < ctx->num_rows; row++) {
		y_row_off = row * h * y_stride;
		uv_row_off = (row * h * uv_stride) / fmt->uv_height_dec;

		for (col = 0; col < ctx->num_cols; col++) {
			y_col_off = (col * w * y_depth) >> 3;
			uv_col_off = y_col_off / fmt->uv_width_dec;
			if (fmt->uv_packed)
				uv_col_off *= 2;

			y_off = y_row_off + y_col_off;
			uv_off = uv_row_off + uv_col_off;

			u_off = y_size - y_off + uv_off;
			v_off = (fmt->uv_packed) ? 0 : u_off + uv_size;

			q_data->seg_off[seg].offset = y_off;
			q_data->seg_off[seg].u_off = u_off;
			q_data->seg_off[seg++].v_off = v_off;

			dprintk(ctx->dev,
				"%s@[%d,%d]: y_off %08x, u_off %08x, v_off %08x\n",
				type == V4L2_BUF_TYPE_VIDEO_OUTPUT ?
				"Output" : "Capture", row, col,
				y_off, u_off, v_off);
		}
	}
}

static void m2mx6_calc_seg_offsets_packed(struct m2mx6_ctx *ctx,
					  enum v4l2_buf_type type)
{
	struct m2mx6_q_data *q_data = get_q_data(ctx, type);
	struct m2mx6_pixfmt *fmt = q_data->fmt;
	u32 W, H, w, h, depth, stride;
	u32 row_off, col_off;
	int row, col, seg = 0;

	/* setup some convenience vars */
	W = q_data->width;
	H = q_data->height;
	w = q_data->seg_width;
	h = q_data->seg_height;
	stride = q_data->stride;
	depth = fmt->depth;

	for (row = 0; row < ctx->num_rows; row++) {
		row_off = row * h * stride;

		for (col = 0; col < ctx->num_cols; col++) {
			col_off = (col * w * depth) >> 3;

			q_data->seg_off[seg].offset = row_off + col_off;
			q_data->seg_off[seg].u_off = 0;
			q_data->seg_off[seg++].v_off = 0;

			dprintk(ctx->dev, "%s@[%d,%d]: phys %08x\n",
				type == V4L2_BUF_TYPE_VIDEO_OUTPUT ?
				"Output" : "Capture", row, col,
				row_off + col_off);
		}
	}
}

static void m2mx6_calc_seg_offsets(struct m2mx6_ctx *ctx,
				   enum v4l2_buf_type type)
{
	struct m2mx6_q_data *q_data = get_q_data(ctx, type);

	memset(q_data->seg_off, 0, sizeof(q_data->seg_off));

	if (q_data->fmt->y_depth)
		m2mx6_calc_seg_offsets_planar(ctx, type);
	else
		m2mx6_calc_seg_offsets_packed(ctx, type);
}

static void set_default_params(struct m2mx6_ctx *ctx)
{
	/* Prepare DST format */
	ctx->q_data[V4L2_M2M_DST].fmt = m2mx6_get_format_from_fourcc(V4L2_PIX_FMT_NV12);
	ctx->q_data[V4L2_M2M_DST].width = DEFAULT_W;
	ctx->q_data[V4L2_M2M_DST].height = DEFAULT_H;
	ctx->q_data[V4L2_M2M_DST].bytesperline = (ctx->q_data[V4L2_M2M_DST].width * ctx->q_data[V4L2_M2M_DST].fmt->depth) >> 3;
	ctx->q_data[V4L2_M2M_DST].sizeimage = ctx->q_data[V4L2_M2M_DST].height * ctx->q_data[V4L2_M2M_DST].bytesperline;
	ctx->num_rows = m2mx6_num_stripes(ctx->q_data[V4L2_M2M_DST].height);
	ctx->num_cols = m2mx6_num_stripes(ctx->q_data[V4L2_M2M_DST].width);
	ctx->num_segs = ctx->num_cols * ctx->num_rows;
	if (ctx->q_data[V4L2_M2M_DST].fmt->y_depth) {
		ctx->q_data[V4L2_M2M_DST].stride  = (ctx->q_data[V4L2_M2M_DST].fmt->y_depth * ctx->q_data[V4L2_M2M_DST].width) >> 3;
		ctx->q_data[V4L2_M2M_DST].rot_stride =
			(ctx->q_data[V4L2_M2M_DST].fmt->y_depth * ctx->q_data[V4L2_M2M_DST].height) >> 3;
	} else {
		ctx->q_data[V4L2_M2M_DST].stride  = ctx->q_data[V4L2_M2M_DST].bytesperline;
		ctx->q_data[V4L2_M2M_DST].rot_stride =
			(ctx->q_data[V4L2_M2M_DST].fmt->depth * ctx->q_data[V4L2_M2M_DST].height) >> 3;
	}

	ctx->q_data[V4L2_M2M_DST].seg_height = ctx->q_data[V4L2_M2M_DST].height / ctx->num_rows;
	ctx->q_data[V4L2_M2M_DST].seg_width = ctx->q_data[V4L2_M2M_DST].width / ctx->num_cols;

	m2mx6_calc_seg_offsets(ctx, V4L2_BUF_TYPE_VIDEO_CAPTURE);

	v4l2_info(&ctx->dev->v4l2_dev,
		  "Default capture format: %dx%d (%d %dx%d segments), %c%c%c%c\n",
		  ctx->q_data[V4L2_M2M_DST].width, ctx->q_data[V4L2_M2M_DST].height,
		  ctx->num_segs, ctx->q_data[V4L2_M2M_DST].seg_width, ctx->q_data[V4L2_M2M_DST].seg_height,
		  ctx->q_data[V4L2_M2M_DST].fmt->fourcc & 0xff,
		  (ctx->q_data[V4L2_M2M_DST].fmt->fourcc >> 8) & 0xff,
		  (ctx->q_data[V4L2_M2M_DST].fmt->fourcc >> 16) & 0xff,
		  (ctx->q_data[V4L2_M2M_DST].fmt->fourcc >> 24) & 0xff);


	/* Now, go for SRC format */
	ctx->q_data[V4L2_M2M_SRC].fmt = m2mx6_get_format_from_fourcc(V4L2_PIX_FMT_YUYV);
	ctx->q_data[V4L2_M2M_SRC].width = DEFAULT_W;
	ctx->q_data[V4L2_M2M_SRC].height = DEFAULT_H;
	ctx->q_data[V4L2_M2M_SRC].bytesperline = (ctx->q_data[V4L2_M2M_SRC].width * ctx->q_data[V4L2_M2M_SRC].fmt->depth) >> 3;
	ctx->q_data[V4L2_M2M_SRC].sizeimage = ctx->q_data[V4L2_M2M_SRC].height * ctx->q_data[V4L2_M2M_SRC].bytesperline;

	if (ctx->q_data[V4L2_M2M_SRC].fmt->y_depth) {
		ctx->q_data[V4L2_M2M_SRC].stride  = (ctx->q_data[V4L2_M2M_SRC].fmt->y_depth * ctx->q_data[V4L2_M2M_SRC].width) >> 3;
		ctx->q_data[V4L2_M2M_SRC].rot_stride =
			(ctx->q_data[V4L2_M2M_SRC].fmt->y_depth * ctx->q_data[V4L2_M2M_SRC].height) >> 3;
	} else {
		ctx->q_data[V4L2_M2M_SRC].stride  = ctx->q_data[V4L2_M2M_SRC].bytesperline;
		ctx->q_data[V4L2_M2M_SRC].rot_stride =
			(ctx->q_data[V4L2_M2M_SRC].fmt->depth * ctx->q_data[V4L2_M2M_SRC].height) >> 3;
	}

	ctx->q_data[V4L2_M2M_SRC].seg_height = ctx->q_data[V4L2_M2M_SRC].height / ctx->num_rows;
	ctx->q_data[V4L2_M2M_SRC].seg_width = ctx->q_data[V4L2_M2M_SRC].width / ctx->num_cols;

	m2mx6_calc_seg_offsets(ctx, V4L2_BUF_TYPE_VIDEO_OUTPUT);

	v4l2_info(&ctx->dev->v4l2_dev,
		  "Default output format: %dx%d (%d %dx%d segments), %c%c%c%c\n",
		  ctx->q_data[V4L2_M2M_SRC].width, ctx->q_data[V4L2_M2M_SRC].height,
		  ctx->num_segs, ctx->q_data[V4L2_M2M_SRC].seg_width, ctx->q_data[V4L2_M2M_SRC].seg_height,
		  ctx->q_data[V4L2_M2M_SRC].fmt->fourcc & 0xff,
		  (ctx->q_data[V4L2_M2M_SRC].fmt->fourcc >> 8) & 0xff,
		  (ctx->q_data[V4L2_M2M_SRC].fmt->fourcc >> 16) & 0xff,
		  (ctx->q_data[V4L2_M2M_SRC].fmt->fourcc >> 24) & 0xff);

}


/*
 * mem2mem callbacks
 */

static void m2mx6_job_abort(void *priv)
{
	struct m2mx6_ctx *ctx = priv;

	/* Will cancel the transaction in the next interrupt handler */
	ctx->aborting = 1;
}

/* hold spinlock when calling */
static void m2mx6_norotate_stop(struct m2mx6_ctx *ctx)
{
	/* disable IC tasks and the channels */
	ipu_ic_task_disable(ctx->ic);
	ipu_idmac_disable_channel(ctx->ipu_mem_pp_ch);
	ipu_idmac_disable_channel(ctx->ipu_pp_mem_ch);

	ipu_ic_disable(ctx->ic);
}

/* hold spinlock when calling */
static void m2mx6_rotate_stop(struct m2mx6_ctx *ctx)
{
	/* disable IC tasks and the channels */
	ipu_ic_task_disable(ctx->ic);

	ipu_idmac_disable_channel(ctx->ipu_mem_pp_ch);
	ipu_idmac_disable_channel(ctx->ipu_pp_mem_ch);
	ipu_idmac_disable_channel(ctx->ipu_mem_rot_pp_ch);
	ipu_idmac_disable_channel(ctx->ipu_rot_pp_mem_ch);

	ipu_idmac_unlink(ctx->ipu_pp_mem_ch, ctx->ipu_mem_rot_pp_ch);

	ipu_ic_disable(ctx->ic);
}

/* hold spinlock when calling */
static void init_idmac_channel(struct m2mx6_ctx *ctx,
			       struct ipuv3_channel *channel,
			       struct m2mx6_q_data *q_data,
			       enum ipu_rotate_mode rot_mode,
			       bool rot_swap_width_height)
{
	unsigned int burst_size;
	u32 width, height, stride;
	dma_addr_t addr0, addr1 = 0;
	struct ipu_image image;

	if (rot_swap_width_height) {
		width = q_data->seg_height;
		height = q_data->seg_width;
		stride = q_data->rot_stride;
		addr0 = ctx->rot_intermediate_phys[0];
		if (ctx->double_buffering)
			addr1 = ctx->rot_intermediate_phys[1];
	} else {
		width = q_data->seg_width;
		height = q_data->seg_height;
		stride = q_data->stride;
		addr0 = q_data->phys_start + q_data->seg_off[0].offset;
		if (ctx->double_buffering)
			addr1 = q_data->phys_start + q_data->seg_off[1].offset;
	}

	ipu_cpmem_zero(channel);

	memset(&image, 0, sizeof(image));
	image.pix.width = image.rect.width = width;
	image.pix.height = image.rect.height = height;
	image.pix.bytesperline = stride;
	image.pix.pixelformat =  q_data->fmt->fourcc;
	image.phys0 = addr0;
	image.phys1 = addr1;
	ipu_cpmem_set_image(channel, &image);

	ipu_cpmem_set_uv_offset(channel, q_data->seg_off[0].u_off,
				q_data->seg_off[0].v_off);

	if (rot_mode)
		ipu_cpmem_set_rotation(channel, rot_mode);

	if (channel == ctx->ipu_mem_rot_pp_ch ||
	    channel == ctx->ipu_rot_pp_mem_ch) {
		burst_size = 8;
		ipu_cpmem_set_block_mode(channel);
	} else
		burst_size = (width % 16) ? 8 : 16;

	ipu_cpmem_set_burstsize(channel, burst_size);

	ipu_ic_task_idma_init(ctx->ic, channel, width, height,
			      burst_size, rot_mode);

	ipu_cpmem_set_axi_id(channel, 1);
	ipu_idmac_lock_enable(channel, 8);

	ipu_idmac_set_double_buffer(channel, ctx->double_buffering);
}

/* hold spinlock when calling */
static void m2mx6_norotate_start(struct m2mx6_ctx *ctx)
{
	struct m2mx6_dev *dev = ctx->dev;
	struct m2mx6_q_data *s_q_data, *d_q_data;
	enum ipu_color_space src_cs, dest_cs;
	int ret;

	s_q_data = get_q_data(ctx, V4L2_BUF_TYPE_VIDEO_OUTPUT);
	d_q_data = get_q_data(ctx, V4L2_BUF_TYPE_VIDEO_CAPTURE);

	src_cs = ipu_pixelformat_to_colorspace(s_q_data->fmt->fourcc);
	dest_cs = ipu_pixelformat_to_colorspace(d_q_data->fmt->fourcc);

	/* setup the IC resizer and CSC */
	ret = ipu_ic_task_init(ctx->ic,
			       s_q_data->seg_width,
			       s_q_data->seg_height,
			       d_q_data->seg_width,
			       d_q_data->seg_height,
			       src_cs, dest_cs);
	if (ret) {
		v4l2_err(&dev->v4l2_dev, "ipu_ic_task_init failed, %d\n", ret);
		return;
	}

	/* init the source MEM-->IC PP IDMAC channel */
	init_idmac_channel(ctx, ctx->ipu_mem_pp_ch, s_q_data,
			   IPU_ROTATE_NONE, false);

	/* init the destination IC PP-->MEM IDMAC channel */
	init_idmac_channel(ctx, ctx->ipu_pp_mem_ch, d_q_data,
			   ctx->rot_mode, false);

	/* enable the IC */
	ipu_ic_enable(ctx->ic);

	/* set buffers ready */
	ipu_idmac_select_buffer(ctx->ipu_mem_pp_ch, 0);
	ipu_idmac_select_buffer(ctx->ipu_pp_mem_ch, 0);
	if (ctx->double_buffering) {
		ipu_idmac_select_buffer(ctx->ipu_mem_pp_ch, 1);
		ipu_idmac_select_buffer(ctx->ipu_pp_mem_ch, 1);
	}

	/* enable the channels! */
	ipu_idmac_enable_channel(ctx->ipu_mem_pp_ch);
	ipu_idmac_enable_channel(ctx->ipu_pp_mem_ch);

	ipu_ic_task_enable(ctx->ic);

	ipu_cpmem_dump(ctx->ipu_mem_pp_ch);
	ipu_cpmem_dump(ctx->ipu_pp_mem_ch);
	ipu_ic_dump(ctx->ic);
	ipu_dump(dev->ipu);
}

/* hold spinlock when calling */
static void m2mx6_rotate_start(struct m2mx6_ctx *ctx)
{
	struct m2mx6_dev *dev = ctx->dev;
	struct m2mx6_q_data *s_q_data, *d_q_data;
	enum ipu_color_space src_cs, dest_cs;
	int ret;

	s_q_data = get_q_data(ctx, V4L2_BUF_TYPE_VIDEO_OUTPUT);
	d_q_data = get_q_data(ctx, V4L2_BUF_TYPE_VIDEO_CAPTURE);

	src_cs = ipu_pixelformat_to_colorspace(s_q_data->fmt->fourcc);
	dest_cs = ipu_pixelformat_to_colorspace(d_q_data->fmt->fourcc);

	/* setup the IC resizer and CSC (swap output width/height) */
	ret = ipu_ic_task_init(ctx->ic,
			       s_q_data->seg_width,
			       s_q_data->seg_height,
			       d_q_data->seg_height,
			       d_q_data->seg_width,
			       src_cs, dest_cs);
	if (ret) {
		v4l2_err(&dev->v4l2_dev, "ipu_ic_task_init failed, %d\n", ret);
		return;
	}

	/* init the source MEM-->IC PP IDMAC channel */
	init_idmac_channel(ctx, ctx->ipu_mem_pp_ch, s_q_data,
			   IPU_ROTATE_NONE, false);

	/* init the IC PP-->MEM IDMAC channel */
	init_idmac_channel(ctx, ctx->ipu_pp_mem_ch, d_q_data,
			   IPU_ROTATE_NONE, true);

	/* init the MEM-->IC PP ROT IDMAC channel */
	init_idmac_channel(ctx, ctx->ipu_mem_rot_pp_ch, d_q_data,
			   ctx->rot_mode, true);

	/* init the destination IC PP ROT-->MEM IDMAC channel */
	init_idmac_channel(ctx, ctx->ipu_rot_pp_mem_ch, d_q_data,
			   IPU_ROTATE_NONE, false);

	/* now link IC PP-->MEM to MEM-->IC PP ROT */
	ipu_idmac_link(ctx->ipu_pp_mem_ch, ctx->ipu_mem_rot_pp_ch);

	/* enable the IC */
	ipu_ic_enable(ctx->ic);

	/* set buffers ready */
	ipu_idmac_select_buffer(ctx->ipu_mem_pp_ch, 0);
	ipu_idmac_select_buffer(ctx->ipu_pp_mem_ch, 0);
	ipu_idmac_select_buffer(ctx->ipu_rot_pp_mem_ch, 0);
	if (ctx->double_buffering) {
		ipu_idmac_select_buffer(ctx->ipu_mem_pp_ch, 1);
		ipu_idmac_select_buffer(ctx->ipu_pp_mem_ch, 1);
		ipu_idmac_select_buffer(ctx->ipu_rot_pp_mem_ch, 1);
	}

	/* enable the channels! */
	ipu_idmac_enable_channel(ctx->ipu_mem_pp_ch);
	ipu_idmac_enable_channel(ctx->ipu_pp_mem_ch);
	ipu_idmac_enable_channel(ctx->ipu_mem_rot_pp_ch);
	ipu_idmac_enable_channel(ctx->ipu_rot_pp_mem_ch);

	ipu_ic_task_enable(ctx->ic);

	ipu_cpmem_dump(ctx->ipu_mem_pp_ch);
	ipu_cpmem_dump(ctx->ipu_pp_mem_ch);
	ipu_cpmem_dump(ctx->ipu_mem_rot_pp_ch);
	ipu_cpmem_dump(ctx->ipu_rot_pp_mem_ch);
	ipu_ic_dump(ctx->ic);
	ipu_dump(dev->ipu);
}

static void m2mx6_device_run(void *priv)
{
	struct m2mx6_ctx *ctx = priv;
	struct m2mx6_dev *dev = ctx->dev;
	struct m2mx6_q_data *s_q_data, *d_q_data;
	struct vb2_buffer *src_buf, *dst_buf;
	unsigned long flags;

	src_buf = v4l2_m2m_next_src_buf(ctx->fh.m2m_ctx);
	dst_buf = v4l2_m2m_next_dst_buf(ctx->fh.m2m_ctx);
	s_q_data = get_q_data(ctx, V4L2_BUF_TYPE_VIDEO_OUTPUT);
	d_q_data = get_q_data(ctx, V4L2_BUF_TYPE_VIDEO_CAPTURE);

	s_q_data->phys_start = vb2_dma_contig_plane_dma_addr(src_buf, 0);
	d_q_data->phys_start = vb2_dma_contig_plane_dma_addr(dst_buf, 0);
	if (!s_q_data->phys_start || !d_q_data->phys_start) {
		v4l2_err(&dev->v4l2_dev,
			 "Acquiring kernel pointers to buffers failed\n");
		return;
	}

	dst_buf->v4l2_buf.sequence = d_q_data->sequence++;
	src_buf->v4l2_buf.sequence = s_q_data->sequence++;

	memcpy(&dst_buf->v4l2_buf.timestamp,
		&src_buf->v4l2_buf.timestamp,
		sizeof(struct timeval));
	if (src_buf->v4l2_buf.flags & V4L2_BUF_FLAG_TIMECODE)
		memcpy(&dst_buf->v4l2_buf.timecode, &src_buf->v4l2_buf.timecode,
			sizeof(struct v4l2_timecode));
	dst_buf->v4l2_buf.field = src_buf->v4l2_buf.field;
	dst_buf->v4l2_buf.flags = src_buf->v4l2_buf.flags &
		(V4L2_BUF_FLAG_TIMECODE |
		V4L2_BUF_FLAG_KEYFRAME |
		V4L2_BUF_FLAG_PFRAME |
		V4L2_BUF_FLAG_BFRAME |
		V4L2_BUF_FLAG_TSTAMP_SRC_MASK);

	spin_lock_irqsave(&dev->irqlock, flags);

	if (instrument)
		do_gettimeofday(&ctx->start);

	/*
	 * Can we use double-buffering for this operation? If there is
	 * only one segment (the whole image can be converted in a single
	 * operation) there's no point in using double-buffering. Also,
	 * the IPU's IDMAC channels allow only a single U and V plane
	 * offset shared between both buffers, but these offsets change
	 * for every segment, and therefore would have to be updated for
	 * each buffer which is not possible. So double-buffering is
	 * impossible when either the source or destination images are
	 * a planar format (YUV420, YUV422P, etc.).
	 */
	ctx->double_buffering = (ctx->num_segs > 1 &&
				 !s_q_data->fmt->y_depth &&
				 !d_q_data->fmt->y_depth);

	ctx->cur_buf_num = 0;
	ctx->next_seg = 1;

	if (ctx->rot_mode >= IPU_ROTATE_90_RIGHT)
		m2mx6_rotate_start(ctx);
	else
		m2mx6_norotate_start(ctx);

	spin_unlock_irqrestore(&dev->irqlock, flags);
}

/* hold spinlock when calling */
static bool m2mx6_doirq(struct m2mx6_ctx *ctx)
{
	struct m2mx6_q_data *s_q_data, *d_q_data;
	struct vb2_buffer *src_vb, *dst_vb;
	struct ipuv3_channel *outch;
	struct m2mx6_seg_off *src_off, *dst_off;

	outch = (ctx->rot_mode >= IPU_ROTATE_90_RIGHT) ?
		ctx->ipu_rot_pp_mem_ch : ctx->ipu_pp_mem_ch;

	s_q_data = get_q_data(ctx, V4L2_BUF_TYPE_VIDEO_OUTPUT);
	d_q_data = get_q_data(ctx, V4L2_BUF_TYPE_VIDEO_CAPTURE);

	/*
	 * It is difficult to stop the channel DMA before the channels
	 * enter the paused state. Without double-buffering the channels
	 * are always in a paused state when the EOF irq occurs, so it
	 * is safe to stop the channels now. For double-buffering we
	 * just ignore the abort until the operation completes, when it
	 * is safe to shut down.
	 */
	if (ctx->aborting && !ctx->double_buffering) {
		if (ctx->rot_mode >= IPU_ROTATE_90_RIGHT)
			m2mx6_rotate_stop(ctx);
		else
			m2mx6_norotate_stop(ctx);
		return true;
	}

	if (ctx->next_seg == ctx->num_segs) {
		/*
		 * the conversion is complete
		 */
		if (ctx->rot_mode >= IPU_ROTATE_90_RIGHT)
			m2mx6_rotate_stop(ctx);
		else
			m2mx6_norotate_stop(ctx);

		if (!ctx->aborting) {
			src_vb = v4l2_m2m_src_buf_remove(ctx->fh.m2m_ctx);
			dst_vb = v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx);

			v4l2_m2m_buf_done(src_vb, VB2_BUF_STATE_DONE);
			v4l2_m2m_buf_done(dst_vb, VB2_BUF_STATE_DONE);

			if (instrument) {
				struct timeval stop;
				do_gettimeofday(&stop);
				stop.tv_sec -= ctx->start.tv_sec;
				stop.tv_usec -= ctx->start.tv_usec;
				if (stop.tv_usec < 0) {
					stop.tv_usec += 1000 * 1000;
					stop.tv_sec -= 1;
				}
				v4l2_info(&ctx->dev->v4l2_dev,
					  "buf%d completed in %lu usec\n",
					  dst_vb->v4l2_buf.index,
					  stop.tv_sec * 1000 * 1000 +
					  stop.tv_usec);
			}
		}

		return true;
	}

	/*
	 * not done, place the next segment buffers.
	 */
	if (!ctx->double_buffering) {

		src_off = &s_q_data->seg_off[ctx->next_seg];
		dst_off = &d_q_data->seg_off[ctx->next_seg];

		ipu_cpmem_set_buffer(ctx->ipu_mem_pp_ch, 0,
				     s_q_data->phys_start + src_off->offset);
		ipu_cpmem_set_buffer(outch, 0,
				     d_q_data->phys_start + dst_off->offset);
		ipu_cpmem_set_uv_offset(ctx->ipu_mem_pp_ch,
					src_off->u_off, src_off->v_off);
		ipu_cpmem_set_uv_offset(outch,
					dst_off->u_off, dst_off->v_off);

		ipu_idmac_select_buffer(ctx->ipu_mem_pp_ch, 0);
		ipu_idmac_select_buffer(outch, 0);

	} else if (ctx->next_seg < ctx->num_segs - 1) {

		src_off = &s_q_data->seg_off[ctx->next_seg + 1];
		dst_off = &d_q_data->seg_off[ctx->next_seg + 1];

		ipu_cpmem_set_buffer(ctx->ipu_mem_pp_ch, ctx->cur_buf_num,
				     s_q_data->phys_start + src_off->offset);
		ipu_cpmem_set_buffer(outch, ctx->cur_buf_num,
				     d_q_data->phys_start + dst_off->offset);

		ipu_idmac_select_buffer(ctx->ipu_mem_pp_ch, ctx->cur_buf_num);
		ipu_idmac_select_buffer(outch, ctx->cur_buf_num);

		ctx->cur_buf_num ^= 1;
	}

	ctx->next_seg++;
	return false;
}

static irqreturn_t m2mx6_norotate_irq(int irq, void *data)
{
	struct m2mx6_dev *dev = (struct m2mx6_dev *)data;
	struct m2mx6_ctx *curr_ctx;
	unsigned long flags;
	bool done;

	spin_lock_irqsave(&dev->irqlock, flags);
	curr_ctx = v4l2_m2m_get_curr_priv(dev->m2m_dev);
	if (curr_ctx == NULL) {
		v4l2_err(&dev->v4l2_dev,
			 "Instance released before the end of transaction\n");
		spin_unlock_irqrestore(&dev->irqlock, flags);
		return IRQ_HANDLED;
	}

	if (curr_ctx->rot_mode >= IPU_ROTATE_90_RIGHT) {
		/* this is a rotation operation, just ignore */
		spin_unlock_irqrestore(&dev->irqlock, flags);
		return IRQ_HANDLED;
	}

	done = m2mx6_doirq(curr_ctx);

	spin_unlock_irqrestore(&dev->irqlock, flags);

	if (done)
		v4l2_m2m_job_finish(dev->m2m_dev, curr_ctx->fh.m2m_ctx);

	return IRQ_HANDLED;
}

static irqreturn_t m2mx6_rotate_irq(int irq, void *data)
{
	struct m2mx6_dev *dev = (struct m2mx6_dev *)data;
	struct m2mx6_ctx *curr_ctx;
	unsigned long flags;
	bool done;

	spin_lock_irqsave(&dev->irqlock, flags);

	curr_ctx = v4l2_m2m_get_curr_priv(dev->m2m_dev);
	if (curr_ctx == NULL) {
		v4l2_err(&dev->v4l2_dev,
			 "Instance released before the end of transaction\n");
		spin_unlock_irqrestore(&dev->irqlock, flags);
		return IRQ_HANDLED;
	}

	if (curr_ctx->rot_mode < IPU_ROTATE_90_RIGHT) {
		/* this was NOT a rotation operation, shouldn't happen */
		v4l2_err(&dev->v4l2_dev, "Unexpected rotation interrupt\n");
		spin_unlock_irqrestore(&dev->irqlock, flags);
		return IRQ_HANDLED;
	}

	done = m2mx6_doirq(curr_ctx);

	spin_unlock_irqrestore(&dev->irqlock, flags);

	if (done)
		v4l2_m2m_job_finish(dev->m2m_dev, curr_ctx->fh.m2m_ctx);

	return IRQ_HANDLED;
}

static void m2mx6_release_ipu_resources(struct m2mx6_ctx *ctx)
{
	struct m2mx6_dev *dev = ctx->dev;

	if (ctx->pp_mem_irq >= 0)
		devm_free_irq(dev->v4l2_dev.dev, ctx->pp_mem_irq, dev);
	if (ctx->rot_pp_mem_irq >= 0)
		devm_free_irq(dev->v4l2_dev.dev, ctx->rot_pp_mem_irq, dev);

	if (!IS_ERR_OR_NULL(ctx->ipu_mem_pp_ch))
		ipu_idmac_put(ctx->ipu_mem_pp_ch);
	if (!IS_ERR_OR_NULL(ctx->ipu_pp_mem_ch))
		ipu_idmac_put(ctx->ipu_pp_mem_ch);
	if (!IS_ERR_OR_NULL(ctx->ipu_mem_rot_pp_ch))
		ipu_idmac_put(ctx->ipu_mem_rot_pp_ch);
	if (!IS_ERR_OR_NULL(ctx->ipu_rot_pp_mem_ch))
		ipu_idmac_put(ctx->ipu_rot_pp_mem_ch);

	if (!IS_ERR_OR_NULL(ctx->ic))
		ipu_ic_put(ctx->ic);

	ctx->ipu_mem_pp_ch = ctx->ipu_pp_mem_ch = ctx->ipu_mem_rot_pp_ch =
		ctx->ipu_rot_pp_mem_ch = NULL;
	ctx->ic = NULL;
	ctx->pp_mem_irq = ctx->rot_pp_mem_irq = -1;
}

static int m2mx6_get_ipu_resources(struct m2mx6_ctx *ctx)
{
	struct m2mx6_dev *dev = ctx->dev;
	int ret;

	ctx->ic = ipu_ic_get(dev->ipu, IC_TASK_POST_PROCESSOR);
	if (IS_ERR(ctx->ic)) {
		v4l2_err(&dev->v4l2_dev, "could not get IC PP\n");
		ret = PTR_ERR(ctx->ic);
		goto err;
	}

	/* get our IDMAC channels */
	ctx->ipu_mem_pp_ch = ipu_idmac_get(dev->ipu,
					   IPUV3_CHANNEL_MEM_IC_PP);
	ctx->ipu_pp_mem_ch = ipu_idmac_get(dev->ipu,
					   IPUV3_CHANNEL_IC_PP_MEM);
	ctx->ipu_mem_rot_pp_ch = ipu_idmac_get(dev->ipu,
					       IPUV3_CHANNEL_MEM_ROT_PP);
	ctx->ipu_rot_pp_mem_ch = ipu_idmac_get(dev->ipu,
					       IPUV3_CHANNEL_ROT_PP_MEM);
	if (IS_ERR(ctx->ipu_mem_pp_ch) ||
	    IS_ERR(ctx->ipu_pp_mem_ch) ||
	    IS_ERR(ctx->ipu_mem_rot_pp_ch) ||
	    IS_ERR(ctx->ipu_rot_pp_mem_ch)) {
		v4l2_err(&dev->v4l2_dev, "could not acquire IDMAC channels\n");
		ret = -EBUSY;
		goto err;
	}

	/* acquire the EOF interrupts */
	ctx->pp_mem_irq = ipu_idmac_channel_irq(dev->ipu,
						ctx->ipu_pp_mem_ch,
						IPU_IRQ_EOF);
	ctx->rot_pp_mem_irq = ipu_idmac_channel_irq(dev->ipu,
						    ctx->ipu_rot_pp_mem_ch,
						    IPU_IRQ_EOF);

	ret = devm_request_irq(dev->v4l2_dev.dev, ctx->pp_mem_irq,
			       m2mx6_norotate_irq, 0, MEM2MEM_NAME, dev);
	if (ret < 0) {
		v4l2_err(&dev->v4l2_dev, "could not acquire irq %d\n",
			 ctx->pp_mem_irq);
		goto err;
	}

	ret = devm_request_irq(dev->v4l2_dev.dev, ctx->rot_pp_mem_irq,
			       m2mx6_rotate_irq, 0, MEM2MEM_NAME, dev);
	if (ret < 0) {
		v4l2_err(&dev->v4l2_dev, "could not acquire irq %d\n",
			 ctx->rot_pp_mem_irq);
		goto err;
	}

	return 0;
err:
	m2mx6_release_ipu_resources(ctx);
	return ret;
}

/*
 * video ioctls
 */
static int vidioc_querycap(struct file *file, void *priv,
			   struct v4l2_capability *cap)
{
	strlcpy(cap->driver, MEM2MEM_NAME, sizeof(cap->driver));
	strlcpy(cap->card, MEM2MEM_NAME, sizeof(cap->card));
	strlcpy(cap->bus_info, "platform:mx6-m2m", sizeof(cap->bus_info));
	cap->device_caps = V4L2_CAP_STREAMING | V4L2_CAP_VIDEO_M2M;
	cap->capabilities = cap->device_caps | V4L2_CAP_DEVICE_CAPS;

	return 0;
}

static int enum_fmt(struct v4l2_fmtdesc *f, u32 type)
{
	struct m2mx6_pixfmt *fmt;
	int i, num = 0;

	for (i = 0; i < NUM_FORMATS; ++i) {
		if (m2mx6_formats[i].types & type) {
			/* index-th format of type type found ? */
			if (num == f->index)
				break;
			/* Correct type but haven't reached our index yet,
			 * just increment per-type index */
			++num;
		}
	}

	if (i < NUM_FORMATS) {
		/* Format found */
		fmt = &m2mx6_formats[i];
		strncpy(f->description, fmt->name, sizeof(f->description) - 1);
		f->pixelformat = fmt->fourcc;
		return 0;
	}

	/* Format not found */
	return -EINVAL;
}

static void free_rot_intermediate_buffer(struct m2mx6_ctx *ctx)
{
	struct m2mx6_dev *dev = ctx->dev;

	if (ctx->rot_intermediate_buf[0]) {
		dma_free_coherent(dev->v4l2_dev.dev,
				  2 * ctx->rot_intermediate_buf_size,
				  ctx->rot_intermediate_buf[0],
				  ctx->rot_intermediate_phys[0]);
		ctx->rot_intermediate_buf[0] =
			ctx->rot_intermediate_buf[1] = NULL;
	}

}

static int alloc_rot_intermediate_buffer(struct m2mx6_ctx *ctx)
{
	struct m2mx6_dev *dev = ctx->dev;
	struct m2mx6_q_data *d_q_data;
	unsigned long newlen;

	d_q_data = get_q_data(ctx, V4L2_BUF_TYPE_VIDEO_CAPTURE);
	newlen = PAGE_ALIGN(d_q_data->sizeimage);

	if (ctx->rot_intermediate_buf[0]) {
		if (ctx->rot_intermediate_buf_size == newlen)
			goto out;
		free_rot_intermediate_buffer(ctx);
	}

	ctx->rot_intermediate_buf_size = newlen;
	ctx->rot_intermediate_buf[0] =
		(void *)dma_alloc_coherent(dev->v4l2_dev.dev,
					   2 * ctx->rot_intermediate_buf_size,
					   &ctx->rot_intermediate_phys[0],
					   GFP_DMA | GFP_KERNEL);
	if (!ctx->rot_intermediate_buf[0]) {
		v4l2_err(&dev->v4l2_dev,
			 "failed to alloc rotation intermediate buffer\n");
		return -ENOMEM;
	}

	ctx->rot_intermediate_buf[1] = ctx->rot_intermediate_buf[0] +
		ctx->rot_intermediate_buf_size;
	ctx->rot_intermediate_phys[1] = ctx->rot_intermediate_phys[0] +
		ctx->rot_intermediate_buf_size;
out:
	return 0;
}

static int vidioc_enum_fmt_vid_cap(struct file *file, void *priv,
				   struct v4l2_fmtdesc *f)
{
	return enum_fmt(f, MEM2MEM_CAPTURE);
}

static int vidioc_enum_fmt_vid_out(struct file *file, void *priv,
				   struct v4l2_fmtdesc *f)
{
	return enum_fmt(f, MEM2MEM_OUTPUT);
}

static int m2mx6_g_fmt(struct m2mx6_ctx *ctx, struct v4l2_format *f)
{
	struct vb2_queue *vq;
	struct m2mx6_q_data *q_data;

	vq = v4l2_m2m_get_vq(ctx->fh.m2m_ctx, f->type);
	if (!vq)
		return -EINVAL;

	q_data = get_q_data(ctx, f->type);

	f->fmt.pix.width	= q_data->width;
	f->fmt.pix.height	= q_data->height;
	f->fmt.pix.field	= V4L2_FIELD_NONE;
	f->fmt.pix.pixelformat	= q_data->fmt->fourcc;
	f->fmt.pix.bytesperline	= (q_data->width * q_data->fmt->depth) >> 3;
	f->fmt.pix.sizeimage	= q_data->bytesperline * q_data->height;
	f->fmt.pix.colorspace	= V4L2_COLORSPACE_REC709;

	return 0;
}

static int vidioc_g_fmt_vid_out(struct file *file, void *priv,
				struct v4l2_format *f)
{
	return m2mx6_g_fmt(priv, f);
}

static int vidioc_g_fmt_vid_cap(struct file *file, void *priv,
				struct v4l2_format *f)
{
	return m2mx6_g_fmt(priv, f);
}

static int m2mx6_try_fmt(struct m2mx6_ctx *ctx, struct v4l2_format *f,
			 struct m2mx6_pixfmt *fmt)
{
	unsigned int num_rows, num_cols;
	enum v4l2_field field;
	u32 w_align, h_align;

	field = f->fmt.pix.field;

	if (field == V4L2_FIELD_ANY)
		field = V4L2_FIELD_NONE;
	else if (V4L2_FIELD_NONE != field)
		return -EINVAL;

	if (f->type == V4L2_BUF_TYPE_VIDEO_OUTPUT) {
		if (!ctx->num_rows || !ctx->num_cols) {
			v4l2_err(&ctx->dev->v4l2_dev,
				 "call capture S_FMT first to determine segmentation\n");
			return -EAGAIN;
		}

		num_rows = ctx->num_rows;
		num_cols = ctx->num_cols;
	} else {
		num_rows = m2mx6_num_stripes(f->fmt.pix.height);
		num_cols = m2mx6_num_stripes(f->fmt.pix.width);
	}

	/* V4L2 specification suggests the driver corrects the format struct
	 * if any of the dimensions is unsupported */

	f->fmt.pix.field = field;

	/*
	 * We have to adjust the width such that the segment physaddrs and
	 * U and V plane offsets are multiples of 8 bytes as required by
	 * the IPU DMA Controller. For the planar formats, this corresponds
	 * to a pixel alignment of 16 times num_cols (but use a more formal
	 * equation since the variables are available). For all the packed
	 * formats, 8 times num_cols is good enough.
	 *
	 * For height alignment, we just have to ensure that the segment
	 * heights are even whole numbers, so h_align = 2 * num_rows.
	 */
	if (fmt->y_depth)
		w_align = (64 * fmt->uv_width_dec * num_cols) / fmt->y_depth;
	else
		w_align = 8 * num_cols;

	w_align = ilog2(w_align);
	h_align = ilog2(2 * num_rows);

	v4l_bound_align_image(&f->fmt.pix.width, MIN_W, MAX_W, w_align,
			      &f->fmt.pix.height, MIN_H, MAX_H, h_align,
			      S_ALIGN);

	f->fmt.pix.bytesperline = f->fmt.pix.width * fmt->ybpp;
	f->fmt.pix.sizeimage = (f->fmt.pix.height * f->fmt.pix.width * fmt->depth) >> 3;

	return 0;
}

static int vidioc_try_fmt_vid_cap(struct file *file, void *priv,
				  struct v4l2_format *f)
{
	struct m2mx6_pixfmt *fmt;
	struct m2mx6_ctx *ctx = priv;

	fmt = m2mx6_get_format(f);
	if (!fmt) {
		f->fmt.pix.pixelformat = m2mx6_formats[0].fourcc;
		fmt = m2mx6_get_format(f);
	}
	if (!(fmt->types & MEM2MEM_CAPTURE)) {
		v4l2_err(&ctx->dev->v4l2_dev,
			 "Fourcc format (0x%08x) invalid.\n",
			 f->fmt.pix.pixelformat);
		return -EINVAL;
	}
	f->fmt.pix.colorspace = V4L2_COLORSPACE_REC709;

	return m2mx6_try_fmt(ctx, f, fmt);
}

static int vidioc_try_fmt_vid_out(struct file *file, void *priv,
				  struct v4l2_format *f)
{
	struct m2mx6_pixfmt *fmt;
	struct m2mx6_ctx *ctx = priv;

	fmt = m2mx6_get_format(f);
	if (!fmt) {
		f->fmt.pix.pixelformat = m2mx6_formats[0].fourcc;
		fmt = m2mx6_get_format(f);
	}

	if (!(fmt->types & MEM2MEM_OUTPUT)) {
		v4l2_err(&ctx->dev->v4l2_dev,
			 "Fourcc format (0x%08x) invalid.\n",
			 f->fmt.pix.pixelformat);
		return -EINVAL;
	}
	if (!f->fmt.pix.colorspace)
		f->fmt.pix.colorspace = V4L2_COLORSPACE_REC709;

	return m2mx6_try_fmt(ctx, f, fmt);
}

static int m2mx6_s_fmt(struct m2mx6_ctx *ctx, struct v4l2_format *f)
{
	struct m2mx6_dev *dev = ctx->dev;
	struct m2mx6_q_data *q_data;
	struct vb2_queue *vq;

	vq = v4l2_m2m_get_vq(ctx->fh.m2m_ctx, f->type);
	if (!vq)
		return -EINVAL;

	q_data = get_q_data(ctx, f->type);
	if (!q_data)
		return -EINVAL;

	if (vb2_is_busy(vq)) {
		v4l2_err(&dev->v4l2_dev, "%s queue busy\n", __func__);
		return -EBUSY;
	}

	if (f->type == V4L2_BUF_TYPE_VIDEO_CAPTURE) {
		/* set segmentation */
		ctx->num_rows = m2mx6_num_stripes(f->fmt.pix.height);
		ctx->num_cols = m2mx6_num_stripes(f->fmt.pix.width);
		ctx->num_segs = ctx->num_cols * ctx->num_rows;
	}

	q_data->fmt		= m2mx6_get_format(f);
	q_data->width		= f->fmt.pix.width;
	q_data->height		= f->fmt.pix.height;
	q_data->bytesperline    = f->fmt.pix.bytesperline;
	if (q_data->fmt->y_depth) {
		q_data->stride  = (q_data->fmt->y_depth * q_data->width) >> 3;
		q_data->rot_stride =
			(q_data->fmt->y_depth * q_data->height) >> 3;
	} else {
		q_data->stride  = q_data->bytesperline;
		q_data->rot_stride =
			(q_data->fmt->depth * q_data->height) >> 3;
	}

	q_data->sizeimage = q_data->bytesperline * q_data->height;

	q_data->seg_height = q_data->height / ctx->num_rows;
	q_data->seg_width = q_data->width / ctx->num_cols;

	m2mx6_calc_seg_offsets(ctx, f->type);

	v4l2_info(&dev->v4l2_dev,
		  "%s format: %dx%d (%d %dx%d segments), %c%c%c%c\n",
		  f->type == V4L2_BUF_TYPE_VIDEO_CAPTURE ? "Capture" : "Output",
		  q_data->width, q_data->height,
		  ctx->num_segs, q_data->seg_width, q_data->seg_height,
		  q_data->fmt->fourcc & 0xff,
		  (q_data->fmt->fourcc >> 8) & 0xff,
		  (q_data->fmt->fourcc >> 16) & 0xff,
		  (q_data->fmt->fourcc >> 24) & 0xff);

	return 0;
}

static int vidioc_s_fmt_vid_cap(struct file *file, void *priv,
				struct v4l2_format *f)
{
	struct m2mx6_ctx *ctx = priv;
	int ret;

	ret = vidioc_try_fmt_vid_cap(file, ctx, f);
	if (ret)
		return ret;

	ret = m2mx6_s_fmt(ctx, f);
	if (ret)
		return ret;

	return 0;
}

static int vidioc_s_fmt_vid_out(struct file *file, void *priv,
				struct v4l2_format *f)
{
	int ret;

	ret = vidioc_try_fmt_vid_out(file, priv, f);
	if (ret)
		return ret;

	return m2mx6_s_fmt(priv, f);
}

static int vidioc_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct m2mx6_ctx *ctx =
		container_of(ctrl->handler, struct m2mx6_ctx, hdl);
	struct m2mx6_dev *dev = ctx->dev;
	enum ipu_rotate_mode rot_mode;
	bool hflip, vflip;
	int rotation;
	int ret = 0;

	rotation = ctx->rotation;
	hflip = ctx->hflip;
	vflip = ctx->vflip;

	switch (ctrl->id) {
	case V4L2_CID_HFLIP:
		hflip = (ctrl->val == 1);
		break;
	case V4L2_CID_VFLIP:
		vflip = (ctrl->val == 1);
		break;
	case V4L2_CID_ROTATE:
		rotation = ctrl->val;
		break;
	default:
		v4l2_err(&dev->v4l2_dev, "Invalid control\n");
		return -EINVAL;
	}

	ret = ipu_degrees_to_rot_mode(&rot_mode, rotation, hflip, vflip);
	if (ret)
		return ret;

	ctx->rotation = rotation;
	ctx->hflip = hflip;
	ctx->vflip = vflip;
	ctx->rot_mode = rot_mode;

	return 0;
}

static int vidioc_qbuf(struct file *file, void *priv,
			struct v4l2_buffer *buf)
{
	struct m2mx6_ctx *ctx = container_of(priv, struct m2mx6_ctx, fh);
	return v4l2_m2m_qbuf(file, ctx->fh.m2m_ctx, buf);
}


static const struct v4l2_ioctl_ops m2mx6_ioctl_ops = {
	.vidioc_querycap	= vidioc_querycap,

	.vidioc_enum_fmt_vid_cap = vidioc_enum_fmt_vid_cap,
	.vidioc_g_fmt_vid_cap	= vidioc_g_fmt_vid_cap,
	.vidioc_try_fmt_vid_cap	= vidioc_try_fmt_vid_cap,
	.vidioc_s_fmt_vid_cap	= vidioc_s_fmt_vid_cap,

	.vidioc_enum_fmt_vid_out = vidioc_enum_fmt_vid_out,
	.vidioc_g_fmt_vid_out	= vidioc_g_fmt_vid_out,
	.vidioc_try_fmt_vid_out	= vidioc_try_fmt_vid_out,
	.vidioc_s_fmt_vid_out	= vidioc_s_fmt_vid_out,

/*	.vidioc_create_bufs	= v4l2_m2m_ioctl_create_bufs,*/
	.vidioc_reqbufs		= v4l2_m2m_ioctl_reqbufs,
	.vidioc_querybuf	= v4l2_m2m_ioctl_querybuf,
	.vidioc_expbuf		= v4l2_m2m_ioctl_expbuf,
	.vidioc_qbuf		= vidioc_qbuf,/*v4l2_m2m_ioctl_qbuf,*/
	.vidioc_dqbuf		= v4l2_m2m_ioctl_dqbuf,

	.vidioc_streamon	= v4l2_m2m_ioctl_streamon,
	.vidioc_streamoff	= v4l2_m2m_ioctl_streamoff,
	.vidioc_subscribe_event = v4l2_ctrl_subscribe_event,
	.vidioc_unsubscribe_event = v4l2_event_unsubscribe,
};

static const struct v4l2_ctrl_ops m2mx6_ctrl_ops = {
	.s_ctrl = vidioc_s_ctrl,
};

/*
 * Queue operations
 */

static int m2mx6_queue_setup(struct vb2_queue *vq,
				const struct v4l2_format *fmt,
				unsigned int *nbuffers, unsigned int *nplanes,
				unsigned int sizes[], void *alloc_ctxs[])
{
	struct m2mx6_ctx *ctx = vb2_get_drv_priv(vq);
	struct m2mx6_q_data *q_data;
	unsigned int size, count = *nbuffers;

	q_data = get_q_data(ctx, vq->type);

	size = q_data->width * q_data->height * q_data->fmt->depth >> 3;

	while (size * count > MEM2MEM_VID_MEM_LIMIT)
		count--;

	*nplanes = 1;
	*nbuffers = count;
	sizes[0] = size;

	alloc_ctxs[0] = ctx->dev->alloc_ctx;

	dprintk(ctx->dev, "get %d buffer(s) of size %d each.\n", count, size);

	return 0;
}

static int m2mx6_buf_prepare(struct vb2_buffer *vb)
{
	struct m2mx6_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct m2mx6_dev *dev = ctx->dev;
	struct m2mx6_q_data *q_data;

	q_data = get_q_data(ctx, vb->vb2_queue->type);

	if (V4L2_TYPE_IS_OUTPUT(vb->vb2_queue->type)) {
		if (vb->v4l2_buf.field == V4L2_FIELD_ANY)
			vb->v4l2_buf.field = V4L2_FIELD_NONE;
		if (vb->v4l2_buf.field != V4L2_FIELD_NONE) {
			dprintk(ctx->dev, "%s field isn't supported\n",
					__func__);
			return -EINVAL;
		}
	}

	if (vb2_plane_size(vb, 0) < q_data->sizeimage) {
		v4l2_err(&dev->v4l2_dev,
			 "%s: data will not fit into plane (%lu < %lu)\n",
			 __func__, vb2_plane_size(vb, 0),
			 (long)q_data->sizeimage);
		return -EINVAL;
	}

	vb2_set_plane_payload(vb, 0, q_data->sizeimage);

	return 0;
}

static void m2mx6_buf_queue(struct vb2_buffer *vb)
{
	struct m2mx6_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	v4l2_m2m_buf_queue(ctx->fh.m2m_ctx, vb);
}

static int m2mx6_start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct m2mx6_ctx *ctx = vb2_get_drv_priv(q);
	struct m2mx6_q_data *q_data = get_q_data(ctx, q->type);
	int ret;

	if (!ctx->ipu_mem_pp_ch) {
		ret = m2mx6_get_ipu_resources(ctx);
		if (ret)
			return ret;
	}

	if (ctx->rot_mode >= IPU_ROTATE_90_RIGHT &&
	    !ctx->rot_intermediate_buf[0]) {
		ret = alloc_rot_intermediate_buffer(ctx);
		if (ret)
			goto relres;
	}

	q_data->sequence = 0;

	return 0;
relres:
	m2mx6_release_ipu_resources(ctx);
	return ret;
}

static void m2mx6_stop_streaming(struct vb2_queue *q)
{
	struct m2mx6_ctx *ctx = vb2_get_drv_priv(q);
	struct vb2_buffer *vb;
	unsigned long flags;

	m2mx6_release_ipu_resources(ctx);
	free_rot_intermediate_buffer(ctx);
	for (;;) {
		if (V4L2_TYPE_IS_OUTPUT(q->type))
			vb = v4l2_m2m_src_buf_remove(ctx->fh.m2m_ctx);
		else
			vb = v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx);
		if (vb == NULL)
			return;
		spin_lock_irqsave(&ctx->dev->irqlock, flags);
		v4l2_m2m_buf_done(vb, VB2_BUF_STATE_ERROR);
		spin_unlock_irqrestore(&ctx->dev->irqlock, flags);
	}
}

static struct vb2_ops m2mx6_qops = {
	.queue_setup	 = m2mx6_queue_setup,
	.buf_prepare	 = m2mx6_buf_prepare,
	.buf_queue	 = m2mx6_buf_queue,
	.wait_prepare	 = vb2_ops_wait_prepare,
	.wait_finish	 = vb2_ops_wait_finish,
	.start_streaming = m2mx6_start_streaming,
	.stop_streaming  = m2mx6_stop_streaming,
};

static int queue_init(void *priv, struct vb2_queue *src_vq,
		      struct vb2_queue *dst_vq)
{
	struct m2mx6_ctx *ctx = priv;
	int ret;

	src_vq->type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
	src_vq->io_modes = VB2_MMAP | VB2_DMABUF;
	src_vq->drv_priv = ctx;
	src_vq->buf_struct_size = sizeof(struct v4l2_m2m_buffer);
	src_vq->ops = &m2mx6_qops;
	src_vq->mem_ops = &vb2_dma_contig_memops;
	src_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	src_vq->lock = &ctx->dev->dev_mutex;
	src_vq->io_flags = VB2_FILEIO_ALLOW_ZERO_BYTESUSED;

	ret = vb2_queue_init(src_vq);
	if (ret)
		return ret;

	dst_vq->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	dst_vq->io_modes = VB2_MMAP | VB2_DMABUF;
	dst_vq->drv_priv = ctx;
	dst_vq->buf_struct_size = sizeof(struct v4l2_m2m_buffer);
	dst_vq->ops = &m2mx6_qops;
	dst_vq->mem_ops = &vb2_dma_contig_memops;
	dst_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	dst_vq->lock = &ctx->dev->dev_mutex;
	dst_vq->io_flags = VB2_FILEIO_ALLOW_ZERO_BYTESUSED;

	return vb2_queue_init(dst_vq);
}

/*
 * File operations
 */
static int m2mx6_open(struct file *file)
{
	struct m2mx6_dev *dev = video_drvdata(file);
	struct m2mx6_ctx *ctx;
	struct v4l2_ctrl_handler *hdl;
	int ret = 0;

	if (mutex_lock_interruptible(&dev->dev_mutex))
		return -ERESTARTSYS;

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx) {
		ret = -ENOMEM;
		goto unlock;
	}

	v4l2_fh_init(&ctx->fh, video_devdata(file));
	file->private_data = &ctx->fh;
	ctx->dev = dev;
	hdl = &ctx->hdl;
	v4l2_ctrl_handler_init(hdl, 3);
	v4l2_ctrl_new_std(hdl, &m2mx6_ctrl_ops, V4L2_CID_HFLIP, 0, 1, 1, 0);
	v4l2_ctrl_new_std(hdl, &m2mx6_ctrl_ops, V4L2_CID_VFLIP, 0, 1, 1, 0);
	v4l2_ctrl_new_std(hdl, &m2mx6_ctrl_ops, V4L2_CID_ROTATE, 0, 270, 90, 0);
	if (hdl->error) {
		ret = hdl->error;
		v4l2_ctrl_handler_free(hdl);
		goto unlock;
	}
	ctx->fh.ctrl_handler = hdl;
	dprintk(dev, "Setup handler: %p\n", hdl);
	v4l2_ctrl_handler_setup(hdl);

	ctx->pp_mem_irq = ctx->rot_pp_mem_irq = -1;

	set_default_params(ctx);
	ctx->fh.m2m_ctx = v4l2_m2m_ctx_init(dev->m2m_dev, ctx, &queue_init);

	if (IS_ERR(ctx->fh.m2m_ctx)) {
		ret = PTR_ERR(ctx->fh.m2m_ctx);
		v4l2_ctrl_handler_free(hdl);
		kfree(ctx);
		goto unlock;
	}

	v4l2_fh_add(&ctx->fh);
	dprintk(dev, "Created instance: %p, m2m_ctx: %p\n", ctx, ctx->fh.m2m_ctx);

unlock:
	mutex_unlock(&dev->dev_mutex);
	return ret;

}

static int m2mx6_release(struct file *file)
{
	struct m2mx6_dev *dev = video_drvdata(file);
	struct m2mx6_ctx *ctx = file->private_data;

	dprintk(dev, "Releasing instance %p\n", ctx);

	v4l2_fh_del(&ctx->fh);
	v4l2_fh_exit(&ctx->fh);
	v4l2_ctrl_handler_free(&ctx->hdl);

	mutex_lock(&dev->dev_mutex);

	/*
	 * in case appplication did not call streamoff,
	 * release IPU resources here
	 */
	m2mx6_release_ipu_resources(ctx);

	free_rot_intermediate_buffer(ctx);

	v4l2_m2m_ctx_release(ctx->fh.m2m_ctx);
	mutex_unlock(&dev->dev_mutex);
	kfree(ctx);

	return 0;
}

static const struct v4l2_file_operations m2mx6_fops = {
	.owner		= THIS_MODULE,
	.open		= m2mx6_open,
	.release	= m2mx6_release,
	.poll		= v4l2_m2m_fop_poll,
	.unlocked_ioctl	= video_ioctl2,
	.mmap		= v4l2_m2m_fop_mmap,
};

static struct video_device m2mx6_videodev = {
	.name		= MEM2MEM_NAME,
	.fops		= &m2mx6_fops,
	.ioctl_ops	= &m2mx6_ioctl_ops,
	.minor		= -1,
	.release	= video_device_release,
	.vfl_dir	= VFL_DIR_M2M,
};

static struct v4l2_m2m_ops m2m_ops = {
	.device_run	= m2mx6_device_run,
	.job_abort	= m2mx6_job_abort,
};

static int of_dev_node_match(struct device *dev, void *data)
{
	return dev->of_node == data;
}

static struct ipu_soc *m2mx6_find_ipu(struct m2mx6_dev *dev,
				      struct device_node *node)
{
	struct device_node *ipu_node;
	struct device *ipu_dev;

	ipu_node = of_parse_phandle(node, "ipu", 0);
	if (!ipu_node) {
		v4l2_err(&dev->v4l2_dev, "missing ipu phandle!\n");
		return NULL;
	}

	ipu_dev = bus_find_device(&platform_bus_type, NULL,
				  ipu_node, of_dev_node_match);
	of_node_put(ipu_node);

	if (!ipu_dev) {
		v4l2_err(&dev->v4l2_dev, "failed to find ipu device!\n");
		return NULL;
	}

	return dev_get_drvdata(ipu_dev);
}

static int m2mx6_probe(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	struct m2mx6_dev *dev;
	struct video_device *vfd;
	int ret;

	dev = devm_kzalloc(&pdev->dev, sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	spin_lock_init(&dev->irqlock);

	ret = v4l2_device_register(&pdev->dev, &dev->v4l2_dev);
	if (ret)
		return ret;

	pdev->dev.coherent_dma_mask = DMA_BIT_MASK(32);

	/* get our IPU */
	dev->ipu = m2mx6_find_ipu(dev, node);
	if (IS_ERR_OR_NULL(dev->ipu)) {
		v4l2_err(&dev->v4l2_dev, "could not get ipu\n");
		ret = -ENODEV;
		goto unreg_dev;
	}

	mutex_init(&dev->dev_mutex);

	vfd = video_device_alloc();
	if (!vfd) {
		v4l2_err(&dev->v4l2_dev, "Failed to allocate video device\n");
		ret = -ENOMEM;
		goto unreg_dev;
	}

	*vfd = m2mx6_videodev;
	vfd->v4l2_dev = &dev->v4l2_dev;
	vfd->lock = &dev->dev_mutex;

	dev->m2m_dev = v4l2_m2m_init(&m2m_ops);
	if (IS_ERR(dev->m2m_dev)) {
		v4l2_err(&dev->v4l2_dev, "Failed to init mem2mem device\n");
		ret = PTR_ERR(dev->m2m_dev);
		video_device_release(vfd);
		goto unreg_dev;
	}

	ret = video_register_device(vfd, VFL_TYPE_GRABBER, 0);
	if (ret) {
		v4l2_err(&dev->v4l2_dev, "Failed to register video device\n");
		video_device_release(vfd);
		goto rel_m2m;
	}

	video_set_drvdata(vfd, dev);
	snprintf(vfd->name, sizeof(vfd->name), "%s", m2mx6_videodev.name);
	dev->vfd = vfd;
	v4l2_info(&dev->v4l2_dev,
		  "Device registered as /dev/video%d, on ipu%d\n",
		  vfd->num, ipu_get_num(dev->ipu));

	platform_set_drvdata(pdev, dev);

	dev->alloc_ctx = vb2_dma_contig_init_ctx(&pdev->dev);
	if (IS_ERR(dev->alloc_ctx)) {
		v4l2_err(&dev->v4l2_dev, "Failed to alloc vb2 context\n");
		ret = PTR_ERR(dev->alloc_ctx);
		goto unreg_vdev;
	}

	return 0;

unreg_vdev:
	video_unregister_device(dev->vfd);
rel_m2m:
	v4l2_m2m_release(dev->m2m_dev);
unreg_dev:
	v4l2_device_unregister(&dev->v4l2_dev);
	return ret;
}

static int m2mx6_remove(struct platform_device *pdev)
{
	struct m2mx6_dev *dev =
		(struct m2mx6_dev *)platform_get_drvdata(pdev);

	v4l2_info(&dev->v4l2_dev, "Removing " MEM2MEM_NAME "\n");
	v4l2_m2m_release(dev->m2m_dev);
	vb2_dma_contig_cleanup_ctx(dev->alloc_ctx);
	video_unregister_device(dev->vfd);

	v4l2_device_unregister(&dev->v4l2_dev);

	return 0;
}

static struct of_device_id m2mx6_dt_ids[] = {
	{ .compatible = "fsl,imx6-v4l2-mem2mem" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, m2mx6_dt_ids);

static struct platform_driver m2mx6_pdrv = {
	.probe		= m2mx6_probe,
	.remove		= m2mx6_remove,
	.driver		= {
		.name	= MEM2MEM_NAME,
		.owner	= THIS_MODULE,
		.of_match_table	= m2mx6_dt_ids,
	},
};

module_platform_driver(m2mx6_pdrv);

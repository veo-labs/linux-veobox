/*
 * V4L2 Driver for i.MXL/i.MXL camera (CSI) host
 *
 * Copyright (C) 2008, Paulius Zaleckas <paulius.zaleckas@teltonika.lt>
 * Copyright (C) 2009, Darius Augulis <augulis.darius@gmail.com>
 *
 * Based on PXA SoC camera driver
 * Copyright (C) 2006, Sascha Hauer, Pengutronix
 * Copyright (C) 2008, Guennadi Liakhovetski <kernel@pengutronix.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/moduleparam.h>
#include <linux/interrupt.h>
#include <linux/videodev2.h>
#include <linux/version.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/time.h>
#include <linux/clk.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/mm.h>

#include <video/imx-ipu-v3.h>
#include "imx-ipu.h"

#include <media/imx.h>
#include <linux/of_graph.h>
#include <media/videobuf2-dma-contig.h>
#include <media/v4l2-common.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-dev.h>
#include <media/v4l2-of.h>

#define DRIVER_NAME "imx-ipuv3-camera"

struct ipucsi_format {
	const char *name;
	u32 mbus_code;
	u32 fourcc;
	u32 bytes_per_pixel; /* memory */
	int bytes_per_sample; /* mbus */
	unsigned rgb:1;
	unsigned yuv:1;
	unsigned raw:1;
};

int v4l2_media_subdev_s_stream(struct media_entity *entity, int enable);

static struct ipucsi_format ipucsi_formats[] = {
	{
		.name = "Monochrome 8 bit",
		.fourcc = V4L2_PIX_FMT_GREY,
		.mbus_code = V4L2_MBUS_FMT_Y8_1X8,
		.bytes_per_pixel = 1,
		.bytes_per_sample = 1,
		.raw = 1,
	}, {
		.name = "Monochrome 10 bit",
		.fourcc = V4L2_PIX_FMT_Y10,
		.mbus_code = V4L2_MBUS_FMT_Y10_1X10,
		.bytes_per_pixel = 2,
		.bytes_per_sample = 2,
		.raw = 1,
	}, {
		.name = "Monochrome 12 bit",
		.fourcc = V4L2_PIX_FMT_Y16,
		.mbus_code = V4L2_MBUS_FMT_Y12_1X12,
		.bytes_per_pixel = 2,
		.bytes_per_sample = 2,
		.raw = 1,
	}, {
		.name = "UYUV 2x8 bit",
		.fourcc = V4L2_PIX_FMT_UYVY,
		.mbus_code = V4L2_MBUS_FMT_UYVY8_2X8,
		.bytes_per_pixel = 2,
		.bytes_per_sample = 1,
		.yuv = 1,
	}, {
		.name = "YUYV 2x8 bit",
		.fourcc = V4L2_PIX_FMT_YUYV,
		.mbus_code = V4L2_MBUS_FMT_YUYV8_2X8,
		.bytes_per_pixel = 2,
		.bytes_per_sample = 1,
		.yuv = 1,
	}, {
		.name = "UYUV 1x16 bit",
		.fourcc = V4L2_PIX_FMT_UYVY,
		.mbus_code = V4L2_MBUS_FMT_UYVY8_1X16,
		.bytes_per_pixel = 2,
		.bytes_per_sample = 2,
		.raw = 1,
	}, {
		.name = "YUYV 1x16 bit",
		.fourcc = V4L2_PIX_FMT_YUYV,
		.mbus_code = V4L2_MBUS_FMT_YUYV8_1X16,
		.bytes_per_pixel = 2,
		.bytes_per_sample = 2,
		.raw = 1,
	},
};

static struct ipucsi_format ipucsi_format_testpattern = {
	.name = "RGB888 32bit",
	.fourcc = V4L2_PIX_FMT_RGB32,
	.mbus_code = V4L2_MBUS_FMT_FIXED,
	.bytes_per_pixel = 4,
	.bytes_per_sample = 4,
	.rgb = 1,
};

/* buffer for one video frame */
struct ipucsi_buffer {
	struct vb2_buffer		vb;
	struct list_head		queue;
};

struct ipucsi {
	struct device			*dev;
	struct v4l2_device		*v4l2_dev;
	struct video_device		vdev;
	/* The currently active buffer, set by NFACK and cleared by EOF interrupt */
	struct ipucsi_buffer		*active;
	struct list_head		capture;
	int				ilo;
	u32				id;

	struct vb2_queue		vb2_vidq;

	spinlock_t			lock;
	struct mutex			mutex;
	struct vb2_alloc_ctx		*alloc_ctx;
	enum v4l2_field			field;
	int				sequence;
	/* These two should be combined: */
	struct media_entity		*csi_entity;
	struct ipu_csi			*csi;
	struct ipu_smfc			*smfc;
	struct ipu_ic			*ic;
	struct ipuv3_channel		*ipuch;
	struct ipu_soc			*ipu;
	struct v4l2_of_endpoint		endpoint;

	struct v4l2_format		format;
	struct ipucsi_format		ipucsifmt;
	struct v4l2_ctrl_handler	ctrls;
	struct v4l2_ctrl_handler	ctrls_vdev;
	struct v4l2_ctrl		*ctrl_test_pattern;
	struct media_pad		media_pad;
	struct media_pipeline		pipe;

	struct v4l2_subdev		subdev;
	struct media_pad		subdev_pad[5];
	struct v4l2_mbus_framefmt	format_mbus[2];
	struct ipu_media_link		*link;
	struct v4l2_fh			fh;
};

static struct ipucsi_buffer *to_ipucsi_vb(struct vb2_buffer *vb)
{
	return container_of(vb, struct ipucsi_buffer, vb);
}

static int ipu_csi_get_mbus_config(struct ipucsi *ipucsi,
				   struct v4l2_mbus_config *config)
{
	struct v4l2_subdev *sd;
	struct media_pad *pad;
	int ret;

	/*
	 * Retrieve media bus configuration from the entity connected directly
	 * to the CSI subdev sink pad.
	 */
	pad = media_entity_remote_pad(&ipucsi->subdev_pad[0]);
	sd = media_entity_to_v4l2_subdev(pad->entity);
	ret = v4l2_subdev_call(sd, video, g_mbus_config, config);
	if (ret == -ENOIOCTLCMD) {
		/* Fall back to static mbus configuration from device tree */
		config->type = ipucsi->endpoint.bus_type;
		config->flags = ipucsi->endpoint.bus.parallel.flags;
		ret = 0;
	}

	return ret;
}

static int local_ipu_csi_init_interface(struct ipucsi *ipucsi,
			   uint16_t width, uint16_t height)
{
	struct v4l2_mbus_config mbus_config;
	int ret;

	ret = ipu_csi_get_mbus_config(ipucsi, &mbus_config);
	if (ret)
		return ret;

#if 0
	if (ipucsi->ctrl_test_pattern->val) {
		BUG_ON(ipucsi->ctrl_test_pattern->val > ARRAY_SIZE(csi_test_ctrl_patterns));
		test_ctrl = csi_test_ctrl_patterns[ipucsi->ctrl_test_pattern->val - 1];
		test_ctrl |= CSI_TST_CTRL_TEST_GEN_MODE_EN;
		cfg.ext_vsync = 1;
		cfg.data_width = 8;
		cfg.clk_mode = IPU_CSI_CLK_MODE_NONGATED_CLK;
		cfg.pixclk_pol = 1;
	}
#endif

	ret = ipu_csi_init_interface(ipucsi->csi, &mbus_config,
				     &ipucsi->format_mbus[0]);
	if (ret)
		return ret;

	ipu_csi_set_test_generator(ipucsi->csi, ipucsi->ctrl_test_pattern->val,
		(ipucsi->ctrl_test_pattern->val == 1) ? 0xff : 0x00,
		(ipucsi->ctrl_test_pattern->val == 2) ? 0xff : 0x00,
		(ipucsi->ctrl_test_pattern->val == 3) ? 0xff : 0x00, 18432000);

	return 0;
}

static inline void ipucsi_set_inactive_buffer(struct ipucsi *ipucsi,
					      struct vb2_buffer *vb)
{
	int bufptr = !ipu_idmac_get_current_buffer(ipucsi->ipuch);
	dma_addr_t eba = vb2_dma_contig_plane_dma_addr(vb, 0);

	if (ipucsi->ilo < 0)
		eba -= ipucsi->ilo;

	ipu_cpmem_set_buffer(ipucsi->ipuch, bufptr, eba);

	ipu_idmac_select_buffer(ipucsi->ipuch, bufptr);
}

static irqreturn_t ipucsi_new_frame_handler(int irq, void *context)
{
	struct ipucsi *ipucsi = context;
	struct ipucsi_buffer *buf;
	struct vb2_buffer *vb;
	unsigned long flags;

	/* The IDMAC just started to write pixel data into the current buffer */

	spin_lock_irqsave(&ipucsi->lock, flags);

	/*
	 * If there is a previously active frame, mark it as done to hand it off
	 * to userspace. Or, if there are no further frames queued, hold on to it.
	 */
	if (ipucsi->active) {
		vb = &ipucsi->active->vb;
		buf = to_ipucsi_vb(vb);

		if (vb2_is_streaming(vb->vb2_queue) && list_is_singular(&ipucsi->capture)) {
			pr_debug("%s: reusing 0x%08x\n", __func__,
				vb2_dma_contig_plane_dma_addr(vb, 0));
			/* DEBUG: check if buf == EBA(active) */
		} else {
			/* Otherwise, mark buffer as finished */
			list_del_init(&buf->queue);

			vb2_buffer_done(vb, VB2_BUF_STATE_DONE);
		}
	}

	if (list_empty(&ipucsi->capture))
		goto out;

	ipucsi->active = list_first_entry(&ipucsi->capture,
					   struct ipucsi_buffer, queue);
	vb = &ipucsi->active->vb;
	do_gettimeofday(&vb->v4l2_buf.timestamp);
	vb->v4l2_buf.field = ipucsi->format.fmt.pix.field;
	vb->v4l2_buf.sequence = ipucsi->sequence++;

	/*
	 * Point the inactive buffer address to the next queued buffer,
	 * if available. Otherwise, prepare to reuse the currently active
	 * buffer, unless ipucsi_videobuf_queue gets called in time.
	 */
	if (!list_is_singular(&ipucsi->capture)) {
		buf = list_entry(ipucsi->capture.next->next,
				 struct ipucsi_buffer, queue);
		vb = &buf->vb;
	}
	ipucsi_set_inactive_buffer(ipucsi, vb);
out:
	spin_unlock_irqrestore(&ipucsi->lock, flags);

	return IRQ_HANDLED;
}

static struct ipucsi_format *ipucsi_current_format(struct ipucsi *ipucsi)
{
	if (ipucsi->ctrl_test_pattern->val)
		return &ipucsi_format_testpattern;
	else
		return &ipucsi->ipucsifmt;
}


/*
 * Depending on which IPU media entity links are enabled, the final output from
 * each CSI could be routed to any of these channels.
 *
 * TODO: Find all output entities connected to this CSI by walking the entity
 * graph. For now a fixed channel list will do.
 */
static const int ipucsi_channels[] = {
	/* SMFC output channels */
	IPUV3_CHANNEL_CSI0,
	IPUV3_CHANNEL_CSI1,
	IPUV3_CHANNEL_CSI2,
	IPUV3_CHANNEL_CSI3,
	/* IC direct and preprocessor task output channels */
	IPUV3_CHANNEL_VDI_MEM_IC_VF,
	IPUV3_CHANNEL_IC_PRP_VF_MEM,
	IPUV3_CHANNEL_IC_PRP_ENC_MEM,
	/* IRT preprocessor task output channels */
	IPUV3_CHANNEL_ROT_ENC_MEM,
	IPUV3_CHANNEL_ROT_VF_MEM,
};

static int ipucsi_get_resources(struct ipucsi *ipucsi)
{
	struct ipu_soc *ipu = ipucsi->ipu;
	struct media_entity_graph graph;
	struct media_entity *entity;
	int channel;
	int ret;
	int i;

	entity = ipu_get_entity(ipu, ipucsi->id ? IPU_CSI1: IPU_CSI0);
	if (!entity)
		return -ENODEV;
	ipucsi->csi_entity = entity;

	/* Walk the graph starting from the CSI entity */
	media_entity_graph_walk_start(&graph, entity);
	for (entity = media_entity_graph_walk_next(&graph); entity != NULL;
	     entity = media_entity_graph_walk_next(&graph)) {
		if (entity->num_pads < 2)
			continue;

		channel = ipu_media_pad_to_channel(ipu, &entity->pads[1]);
		if (channel < 0)
			continue;

		for (i = 0; i < ARRAY_SIZE(ipucsi_channels); i++) {
			if (channel == ipucsi_channels[i])
				break;
		}

		if (i < ARRAY_SIZE(ipucsi_channels))
			break;
	}

	if (!entity) {
		dev_err(ipucsi->dev, "No output channel connected to CSI%d\n",
			ipucsi->id);
		return -EINVAL;
	}

	channel = ipu_media_pad_to_channel(ipu, &entity->pads[1]);
	dev_info(ipucsi->dev, "Selected output entity '%s':1, channel %d\n",
		 entity->name, channel);

	ipucsi->ipuch = ipu_idmac_get(ipu, channel);
	if (IS_ERR(ipucsi->ipuch))
		return PTR_ERR(ipucsi->ipuch);

	switch (channel) {
	case IPUV3_CHANNEL_CSI0 ... IPUV3_CHANNEL_CSI3:
		ipucsi->smfc = ipu_smfc_get(ipu, channel - IPUV3_CHANNEL_CSI0);
		if (!ipucsi->smfc) {
			ret = -EBUSY;
			goto err;
		}
		break;
	case IPUV3_CHANNEL_VDI_MEM_IC_VF:
	case IPUV3_CHANNEL_IC_PRP_VF_MEM:
	case IPUV3_CHANNEL_ROT_VF_MEM:
		ipucsi->ic = ipu_ic_get(ipu, IC_TASK_VIEWFINDER);
		if (IS_ERR(ipucsi->ic)) {
			ret = PTR_ERR(ipucsi->ic);
			goto err;
		}
		break;
	case IPUV3_CHANNEL_IC_PRP_ENC_MEM:
	case IPUV3_CHANNEL_ROT_ENC_MEM:
		ipucsi->ic = ipu_ic_get(ipu, IC_TASK_ENCODER);
		if (IS_ERR(ipucsi->ic)) {
			ret = PTR_ERR(ipucsi->ic);
			goto err;
		}
		break;
	}

	return 0;
err:
	dev_err(ipucsi->dev, "Failed to get %s: %d\n", entity->name, ret);
	ipu_idmac_put(ipucsi->ipuch);
	return ret;
}

static void ipucsi_put_resources(struct ipucsi *ipucsi)
{
	if (ipucsi->ipuch) {
		ipu_idmac_put(ipucsi->ipuch);
		ipucsi->ipuch = NULL;
	}

	if (ipucsi->smfc) {
		ipu_smfc_put(ipucsi->smfc);
		ipucsi->smfc = NULL;
	}

	if (ipucsi->ic) {
		ipu_ic_put(ipucsi->ic);
		ipucsi->ic = NULL;
	}

	ipucsi->csi_entity = NULL;
}

/*
 *  Videobuf operations
 */
static int ipucsi_videobuf_setup(struct vb2_queue *vq, const struct v4l2_format *fmt,
		unsigned int *count, unsigned int *num_planes,
		unsigned int sizes[], void *alloc_ctxs[])
{
	struct ipucsi *ipucsi = vq->drv_priv;
	int bytes_per_line;
	struct ipucsi_format *ipucsifmt = ipucsi_current_format(ipucsi);

	if (!fmt)
		fmt = &ipucsi->format;

	bytes_per_line = fmt->fmt.pix.width * ipucsifmt->bytes_per_pixel;

	dev_dbg(ipucsi->dev, "bytes: %d x: %d y: %d",
			bytes_per_line, fmt->fmt.pix.width, fmt->fmt.pix.height);

	*num_planes = 1;

	ipucsi->sequence = 0;
	sizes[0] = fmt->fmt.pix.sizeimage;
	alloc_ctxs[0] = ipucsi->alloc_ctx;

	if (!*count)
		*count = 32;

	return 0;
}

static int ipucsi_videobuf_prepare(struct vb2_buffer *vb)
{
	struct ipucsi *ipucsi = vb->vb2_queue->drv_priv;
	struct ipucsi_buffer *buf;
	struct v4l2_pix_format *pix = &ipucsi->format.fmt.pix;

	buf = to_ipucsi_vb(vb);

	if (vb2_plane_size(vb, 0) < pix->sizeimage)
		return -ENOBUFS;

	vb2_set_plane_payload(vb, 0, pix->sizeimage);

	return 0;
}

static void ipucsi_videobuf_queue(struct vb2_buffer *vb)
{
	struct vb2_queue *vq = vb->vb2_queue;
	struct ipucsi *ipucsi = vq->drv_priv;
	struct ipucsi_buffer *buf = to_ipucsi_vb(vb);
	unsigned long flags;

	spin_lock_irqsave(&ipucsi->lock, flags);

	/*
	 * If there is no next buffer queued, point the inactive buffer
	 * address to the incoming buffer
	 */
	if (vb2_is_streaming(vb->vb2_queue) && list_is_singular(&ipucsi->capture))
		ipucsi_set_inactive_buffer(ipucsi, vb);

	list_add_tail(&buf->queue, &ipucsi->capture);

	spin_unlock_irqrestore(&ipucsi->lock, flags);
}

static void ipucsi_videobuf_release(struct vb2_buffer *vb)
{
	struct vb2_queue *vq = vb->vb2_queue;
	struct ipucsi *ipucsi = vq->drv_priv;
	struct ipucsi_buffer *buf = to_ipucsi_vb(vb);
	unsigned long flags;

	spin_lock_irqsave(&ipucsi->lock, flags);

	if (ipucsi->active == buf)
		ipucsi->active = NULL;

	if (!list_empty(&buf->queue))
		list_del_init(&buf->queue);

	spin_unlock_irqrestore(&ipucsi->lock, flags);

	ipucsi_put_resources(ipucsi);
}

static int ipucsi_videobuf_init(struct vb2_buffer *vb)
{
	struct ipucsi_buffer *buf = to_ipucsi_vb(vb);

	/* This is for locking debugging only */
	INIT_LIST_HEAD(&buf->queue);

	return 0;
}

#define pixfmtstr(x) (x) & 0xff, ((x) >> 8) & 0xff, ((x) >> 16) & 0xff, \
	((x) >> 24) & 0xff

static int ipucsi_videobuf_start_streaming(struct vb2_queue *vq, unsigned int count)
{
	struct ipucsi *ipucsi = vq->drv_priv;
	struct ipucsi_format *ipucsifmt = ipucsi_current_format(ipucsi);
	u32 width = ipucsi->format.fmt.pix.width;
	u32 height = ipucsi->format.fmt.pix.height;
	struct device *dev = ipucsi->dev;
	int burstsize;
	struct vb2_buffer *vb;
	struct ipucsi_buffer *buf;
	int nfack_irq;
	int ret;
	const char *irq_name[2] = { "CSI0", "CSI1" };
	struct v4l2_rect window;

	ret = ipucsi_get_resources(ipucsi);
	if (ret < 0) {
		dev_err(ipucsi->dev, "Failed to get resources: %d\n", ret);
		goto err_dequeue;
	}

	ipu_cpmem_zero(ipucsi->ipuch);

	nfack_irq = ipu_idmac_channel_irq(ipucsi->ipu, ipucsi->ipuch,
			IPU_IRQ_NFACK);
	ret = request_threaded_irq(nfack_irq, NULL, ipucsi_new_frame_handler,
			IRQF_ONESHOT, irq_name[ipucsi->id], ipucsi);
	if (ret) {
		dev_err(dev, "Failed to request NFACK interrupt: %d\n", nfack_irq);
		return ret;
	}

	dev_dbg(dev, "width: %d height: %d, %c%c%c%c (%c%c%c%c)\n",
		width, height, pixfmtstr(ipucsi->format.fmt.pix.pixelformat),
		pixfmtstr(ipucsifmt->raw));

	/*
	 * TODO:
	 * replace set_resolution, set_stride, set_fmt_*, set_yuv_*, set_buffer
	 * with ipu_cpmem_set_image
	 */

	ipu_cpmem_set_resolution(ipucsi->ipuch, width, height);

	if (ipucsifmt->raw && ipucsi->smfc) {
		/*
		 * raw formats. We can only pass them through to memory
		 */
		ipu_cpmem_set_stride(ipucsi->ipuch, width * ipucsifmt->bytes_per_pixel);
		ipu_cpmem_set_format_passthrough(ipucsi->ipuch, ipucsifmt->bytes_per_sample * 8);
		/*
		 * Set SMFC BURST_SIZE to NBP[6:4] according to table
		 * 38-317. SMFC Burst Size
		 */
		burstsize = 4 / ipucsifmt->bytes_per_sample;
	} else {
		/*
		 * formats we understand, we can write it in any format not requiring
		 * colorspace conversion.
		 */
		u32 fourcc = ipucsi->format.fmt.pix.pixelformat;
		switch (fourcc) {
		case V4L2_PIX_FMT_RGB32:
			ipu_cpmem_set_stride(ipucsi->ipuch, width * 4);
			ipu_cpmem_set_fmt(ipucsi->ipuch, fourcc);
			break;
		case V4L2_PIX_FMT_UYVY:
		case V4L2_PIX_FMT_YUYV:
			ipu_cpmem_set_stride(ipucsi->ipuch, width * 2);
			ipu_cpmem_set_yuv_interleaved(ipucsi->ipuch, fourcc);
			break;
		case V4L2_PIX_FMT_YUV420:
		case V4L2_PIX_FMT_YVU420:
		case V4L2_PIX_FMT_NV12:
			/* HACK: Allow setting YUV 4:2:0 format on grayscale sources */
			if (ipucsifmt->fourcc != V4L2_PIX_FMT_GREY) {
				ipu_cpmem_set_stride(ipucsi->ipuch, width);
				ipu_cpmem_set_fmt(ipucsi->ipuch, fourcc);
				ipu_cpmem_set_yuv_planar(ipucsi->ipuch, fourcc, width, height);
				burstsize = 16;
				break;
			}
			/* else: fallthrough */
		default:
			ret = -EINVAL;
			goto free_irq;
		}
	}

	if (ipucsi->ilo)
		ipu_cpmem_interlaced_scan(ipucsi->ipuch, ipucsi->ilo);

	/*
	 * Some random value. The reference manual tells us that the burstsize
	 * is a function of the IDMACs PFS, BPP and NPB settings. Unfortunately
	 * it doesn't tell us which function this is.
	 */
	/* FIXME */
//	burstsize = (width % 16) ? 8 : 16;

	if (ipucsi->smfc) {
#if 0
		/*
		 * Set the channel for the direct CSI-->memory via SMFC
		 * use-case to very high priority, by enabling the watermark
		 * signal in the SMFC, enabling WM in the channel, and setting
		 * the channel priority to high.
		 *
		 * Refer to the iMx6 rev. D TRM Table 36-8: Calculated priority
		 * value.
		 *
		 * The WM's are set very low by intention here to ensure that
		 * the SMFC FIFOs do not overflow.
		 */
		ipu_smfc_set_wmc(ipucsi->smfc, false, 0x01);
		ipu_smfc_set_wmc(ipucsi->smfc, true, 0x02);
		ipu_idmac_enable_watermark(ipucsi->ipuch, true);
		ipu_cpmem_set_axi_id(ipucsi->ipuch, 0);
		ipu_idmac_lock_enable(ipucsi->ipuch, 8);
#endif
		ipu_smfc_set_burstsize(ipucsi->smfc, burstsize - 1);
		ipu_smfc_map_channel(ipucsi->smfc, ipucsi->id, 0);

		ipu_cpmem_set_high_priority(ipucsi->ipuch);
	}
	if (ipucsi->ic) {
		/* TODO: s_fmt */
		ipu_ic_task_idma_init(ipucsi->ic, ipucsi->ipuch, width, height,
				      burstsize, /*rot_mode*/0);
		ipu_cpmem_set_axi_id(ipucsi->ipuch, 1);
	}

	window.left = 0;
	/* FIXME */
	if (width == 720 && height == 480)
		window.top = 3;
	else
		window.top = 0;
	window.width = width;
	window.height = height;

	ipu_csi_set_window(ipucsi->csi, &window);

	/* Set the internal media pipeline to streaming state */
	ret = media_entity_pipeline_start(ipucsi->csi_entity, &ipucsi->pipe);
	if (ret)
		goto free_irq;

	/* Set the external media pipeline to streaming state */
	ret = media_entity_pipeline_start(&ipucsi->subdev.entity, &ipucsi->pipe);
	if (ret)
		goto stop_pipe;

	if (ipucsi->ic) {
		ipu_ic_enable(ipucsi->ic);
		/* setup RSC and CSC */
		ipu_ic_task_init(ipucsi->ic, width, height, width, height,
				 IPUV3_COLORSPACE_RGB, IPUV3_COLORSPACE_RGB, 0);
	}

	/* FIXME */
	ret = local_ipu_csi_init_interface(ipucsi, width, height);
	if (ret)
		goto free_irq;

	ipu_idmac_set_double_buffer(ipucsi->ipuch, 1);

	if (list_empty(&ipucsi->capture)) {
		dev_err(dev, "No capture buffers\n");
		ret = -ENOMEM;
		goto free_irq;
	}

	ipucsi->active = NULL;

	/* Point the inactive buffer address to the first buffer */
	buf = list_first_entry(&ipucsi->capture, struct ipucsi_buffer, queue);
	vb = &buf->vb;
	ipucsi_set_inactive_buffer(ipucsi, vb);

	ipu_idmac_enable_channel(ipucsi->ipuch);

	/* FIXME */
	if (ipucsi->ic)
		ipu_ic_task_enable(ipucsi->ic);
	if (ipucsi->smfc)
		ipu_smfc_enable(ipucsi->smfc);

	ipu_csi_enable(ipucsi->csi);

	if (ipucsi->ic) {
		ipu_dump(ipucsi->ipu);
		ipu_ic_dump(ipucsi->ic);
		ipu_csi_dump(ipucsi->csi);
	}

	ret = v4l2_media_subdev_s_stream(&ipucsi->subdev.entity, 1);
	if (ret)
		goto free_irq;

	return 0;

stop_pipe:
	media_entity_pipeline_stop(ipucsi->csi_entity);
free_irq:
	free_irq(nfack_irq, ipucsi);
err_dequeue:
	while (!list_empty(&vq->queued_list)) {
		struct vb2_buffer *buf;

		buf = list_first_entry(&vq->queued_list, struct vb2_buffer,
				       queued_entry);
		list_del(&buf->queued_entry);
		vb2_buffer_done(buf, VB2_BUF_STATE_ERROR);
	}
	return ret;
}

static void ipucsi_videobuf_stop_streaming(struct vb2_queue *vq)
{
	struct ipucsi *ipucsi = vq->drv_priv;
	unsigned long flags;
	int nfack_irq = ipu_idmac_channel_irq(ipucsi->ipu, ipucsi->ipuch,
				IPU_IRQ_NFACK);

	free_irq(nfack_irq, ipucsi);
	ipu_csi_disable(ipucsi->csi);
	ipu_idmac_disable_channel(ipucsi->ipuch);
	if (ipucsi->smfc) {
		ipu_smfc_disable(ipucsi->smfc);
	}
	if (ipucsi->ic) {
		ipu_ic_task_disable(ipucsi->ic);
		ipu_ic_disable(ipucsi->ic);
	}

	spin_lock_irqsave(&ipucsi->lock, flags);
	while (!list_empty(&ipucsi->capture)) {
		struct ipucsi_buffer *buf = list_entry(ipucsi->capture.next,
						 struct ipucsi_buffer, queue);
		list_del_init(ipucsi->capture.next);
		vb2_buffer_done(&buf->vb, VB2_BUF_STATE_ERROR);
	}
	spin_unlock_irqrestore(&ipucsi->lock, flags);

	media_entity_pipeline_stop(&ipucsi->subdev.entity);
	media_entity_pipeline_stop(ipucsi->csi_entity);

	v4l2_media_subdev_s_stream(&ipucsi->subdev.entity, 0);

	ipucsi_put_resources(ipucsi);
}

static void ipucsi_lock(struct vb2_queue *vq)
{
	struct ipucsi *ipucsi = vq->drv_priv;
	mutex_lock(&ipucsi->mutex);
}

static void ipucsi_unlock(struct vb2_queue *vq)
{
	struct ipucsi *ipucsi = vq->drv_priv;
	mutex_unlock(&ipucsi->mutex);
}

static struct vb2_ops ipucsi_videobuf_ops = {
	.queue_setup		= ipucsi_videobuf_setup,
	.buf_prepare		= ipucsi_videobuf_prepare,
	.buf_queue		= ipucsi_videobuf_queue,
	.buf_cleanup		= ipucsi_videobuf_release,
	.buf_init		= ipucsi_videobuf_init,
	.start_streaming	= ipucsi_videobuf_start_streaming,
	.stop_streaming		= ipucsi_videobuf_stop_streaming,
	.wait_prepare		= ipucsi_unlock,
	.wait_finish		= ipucsi_lock,
};

static int ipucsi_querycap(struct file *file, void *priv,
					struct v4l2_capability *cap)
{
	strlcpy(cap->driver, "imx-ipuv3-csi", sizeof(cap->driver));
	/* cap->name is set by the friendly caller:-> */
	strlcpy(cap->card, "imx-ipuv3-camera", sizeof(cap->card));
	strlcpy(cap->bus_info, "platform:imx-ipuv3-csi", sizeof(cap->bus_info));
	cap->version = 0;
	cap->capabilities = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;

	return 0;
}

static int ipucsi_try_fmt(struct file *file, void *fh,
		struct v4l2_format *f)
{
	struct ipucsi *ipucsi = video_drvdata(file);
	struct ipucsi_format *ipucsifmt = ipucsi_current_format(ipucsi);
	enum v4l2_field in = ipucsi->format_mbus[1].field;
	enum v4l2_field out = f->fmt.pix.field;
	struct ipu_fmt *fmt = NULL;
	int bytes_per_pixel;

	if (ipucsifmt->rgb)
		fmt = ipu_find_fmt_rgb(f->fmt.pix.pixelformat);
	if (ipucsifmt->yuv)
		fmt = ipu_find_fmt_yuv(f->fmt.pix.pixelformat);

	if (ipucsifmt->raw) {
		f->fmt.pix.pixelformat = ipucsifmt->fourcc;
		bytes_per_pixel = ipucsifmt->bytes_per_pixel;
	} else {
		if (!fmt)
			return -EINVAL;
		bytes_per_pixel = fmt->bytes_per_pixel;
	}

	v4l_bound_align_image(&f->fmt.pix.width, 128,
			      ipucsi->format_mbus[1].width, 3,
			      &f->fmt.pix.height, 128,
			      ipucsi->format_mbus[1].height, 1, 0);

	f->fmt.pix.bytesperline = f->fmt.pix.width * bytes_per_pixel;
	f->fmt.pix.sizeimage = f->fmt.pix.bytesperline * f->fmt.pix.height;
	if (fmt->fourcc == V4L2_PIX_FMT_YUV420 ||
	    fmt->fourcc == V4L2_PIX_FMT_YVU420 ||
	    fmt->fourcc == V4L2_PIX_FMT_NV12)
		f->fmt.pix.sizeimage = f->fmt.pix.sizeimage * 3 / 2;
	else if (fmt->fourcc == V4L2_PIX_FMT_YUV422P)
		f->fmt.pix.sizeimage *= 2;

	if ((in == V4L2_FIELD_SEQ_TB && out == V4L2_FIELD_INTERLACED_TB) ||
	    (in == V4L2_FIELD_INTERLACED_TB && out == V4L2_FIELD_SEQ_TB) ||
	    (in == V4L2_FIELD_SEQ_BT && out == V4L2_FIELD_INTERLACED_BT) ||
	    (in == V4L2_FIELD_INTERLACED_BT && out == V4L2_FIELD_SEQ_BT)) {
		/*
		 * IDMAC scan order can be used for translation between
		 * interlaced and sequential field formats.
		 */
	} else if (out == V4L2_FIELD_NONE || out == V4L2_FIELD_INTERLACED) {
		/*
		 * If userspace requests progressive or interlaced frames,
		 * interlace sequential fields as closest approximation.
		 */
		if (in == V4L2_FIELD_SEQ_TB)
			out = V4L2_FIELD_INTERLACED_TB;
		else if (in == V4L2_FIELD_SEQ_BT)
			out = V4L2_FIELD_INTERLACED_BT;
		else
			out = in;
	} else {
		/* Translation impossible or userspace doesn't care */
		out = in;
	}
	f->fmt.pix.field = out;

	return 0;
}

static int ipucsi_s_fmt(struct file *file, void *fh,
		struct v4l2_format *f)
{
	struct ipucsi *ipucsi = video_drvdata(file);
	enum v4l2_field in, out;
	int ret;

	ret = ipucsi_try_fmt(file, fh, f);
	if (ret)
		return ret;

	ipucsi->format = *f;

	/*
	 * Set IDMAC scan order interlace offset (ILO) for translation between
	 * interlaced and sequential field formats.
	 */
	in = ipucsi->format_mbus[1].field;
	out = f->fmt.pix.field;
	if ((in == V4L2_FIELD_SEQ_TB && out == V4L2_FIELD_INTERLACED_TB) ||
	    (in == V4L2_FIELD_INTERLACED_TB && out == V4L2_FIELD_SEQ_TB))
		ipucsi->ilo = f->fmt.pix.bytesperline;
	else if ((in == V4L2_FIELD_SEQ_BT && out == V4L2_FIELD_INTERLACED_BT) ||
		 (in == V4L2_FIELD_INTERLACED_BT && out == V4L2_FIELD_SEQ_BT))
		ipucsi->ilo = -f->fmt.pix.bytesperline;
	else
		ipucsi->ilo = 0;

	return 0;
}

static int ipucsi_g_fmt(struct file *file, void *fh,
		struct v4l2_format *f)
{
	struct ipucsi *ipucsi = video_drvdata(file);

	if (f->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	*f = ipucsi->format;

	return 0;
}

static int ipucsi_enum_fmt(struct file *file, void *priv,
				    struct v4l2_fmtdesc *f)
{
	struct ipucsi *ipucsi = video_drvdata(file);

	if (ipucsi->ctrl_test_pattern->val) {
		if (f->index)
			return -EINVAL;
		strlcpy(f->description, ipucsi_format_testpattern.name,
				sizeof(f->description));
		f->pixelformat = ipucsi_format_testpattern.fourcc;

		return 0;
	}

	if (ipucsi->ipucsifmt.rgb)
		return ipu_enum_fmt_rgb(file, priv, f);
	if (ipucsi->ipucsifmt.yuv)
		return ipu_enum_fmt_yuv(file, priv, f);

	if (f->index)
		return -EINVAL;

	if (!ipucsi->ipucsifmt.name)
		return -EINVAL;

	strlcpy(f->description, ipucsi->ipucsifmt.name, sizeof(f->description));
	f->pixelformat = ipucsi->ipucsifmt.fourcc;

	return 0;
}

static struct v4l2_mbus_framefmt *
__ipucsi_get_pad_format(struct ipucsi *ipucsi, struct v4l2_subdev_fh *fh,
			unsigned int pad, u32 which)
{
	switch (which) {
	case V4L2_SUBDEV_FORMAT_TRY:
		return v4l2_subdev_get_try_format(fh, pad);
	case V4L2_SUBDEV_FORMAT_ACTIVE:
		return &ipucsi->format_mbus[pad ? 1 : 0];
	default:
		return NULL;
	}
}

static int ipucsi_subdev_get_format(struct v4l2_subdev *subdev,
		struct v4l2_subdev_fh *fh,
		struct v4l2_subdev_format *sdformat)
{
	struct ipucsi *ipucsi = container_of(subdev, struct ipucsi, subdev);

	sdformat->format = *__ipucsi_get_pad_format(ipucsi, fh, sdformat->pad,
						sdformat->which);
	return 0;
}

static struct ipucsi_format *ipucsi_find_subdev_format(const u32 *fourcc,
		const u32 *mbus_code)
{
	struct ipucsi_format *ipucsifmt;
	int i;

	for (i = 0; i < ARRAY_SIZE(ipucsi_formats); i++) {
		ipucsifmt = &ipucsi_formats[i];
		if (fourcc && *fourcc == ipucsifmt->fourcc)
			return ipucsifmt;
		if (mbus_code && *mbus_code == ipucsifmt->mbus_code)
			return ipucsifmt;
	}

	return NULL;
}

static int ipucsi_subdev_set_format(struct v4l2_subdev *subdev,
		struct v4l2_subdev_fh *fh,
		struct v4l2_subdev_format *sdformat)
{
	struct ipucsi *ipucsi = container_of(subdev, struct ipucsi, subdev);
	struct v4l2_mbus_framefmt *mbusformat;
	struct ipucsi_format *ipucsiformat;
	unsigned int width, height;

	ipucsiformat = ipucsi_find_subdev_format(NULL, &sdformat->format.code);
	if (!ipucsiformat)
		return -EINVAL;

	width = clamp_t(unsigned int, sdformat->format.width, 16, 8192);
	height = clamp_t(unsigned int, sdformat->format.height, 16, 4096);

	mbusformat = __ipucsi_get_pad_format(ipucsi, fh, sdformat->pad,
					    sdformat->which);
	mbusformat->width = width;
	mbusformat->height = height;
	mbusformat->code = ipucsiformat->mbus_code;
	mbusformat->field = sdformat->format.field;

	if (mbusformat->field == V4L2_FIELD_SEQ_TB &&
	    mbusformat->width == 720 && mbusformat->height == 480 &&
	    ipucsi->endpoint.bus_type == V4L2_MBUS_BT656) {
		/* We capture NTSC bottom field first */
		mbusformat->field = V4L2_FIELD_SEQ_BT;
	} else if (mbusformat->field == V4L2_FIELD_ANY) {
		mbusformat->field = ipucsi->format_mbus[!sdformat->pad].field;
	}

	sdformat->format = *mbusformat;

	ipucsi->ipucsifmt = *ipucsiformat;

	return 0;
}

static struct v4l2_subdev_pad_ops ipucsi_subdev_pad_ops = {
	.get_fmt = ipucsi_subdev_get_format,
	.set_fmt = ipucsi_subdev_set_format,
};

static const struct v4l2_subdev_ops ipucsi_subdev_ops = {
	.pad    = &ipucsi_subdev_pad_ops,
};

struct media_entity_operations ipucsi_entity_ops = {
	.link_setup = ipu_csi_link_setup,
	.link_validate = v4l2_subdev_link_validate,
};

int v4l2_media_subdev_s_stream(struct media_entity *entity, int enable)
{
	struct media_entity_graph graph;
	struct media_entity *first;
	struct v4l2_subdev *sd;
	int ret = 0;

	first = entity;

	media_entity_graph_walk_start(&graph, entity);
	if (!enable) {
		first = NULL;
		goto disable;
	}

	while (!ret && (entity = media_entity_graph_walk_next(&graph))) {
		if (media_entity_type(entity) == MEDIA_ENT_T_V4L2_SUBDEV) {
			sd = media_entity_to_v4l2_subdev(entity);
			ret = v4l2_subdev_call(sd, video, s_stream, 1);
			if (ret == -ENOIOCTLCMD)
				ret = 0;
		}
	}

	if (!ret)
		return 0;

	media_entity_graph_walk_start(&graph, first);

disable:
	while ((entity = media_entity_graph_walk_next(&graph)) && first != entity) {
		if (media_entity_type(entity) == MEDIA_ENT_T_V4L2_SUBDEV) {
			sd = media_entity_to_v4l2_subdev(entity);
			v4l2_subdev_call(sd, video, s_stream, 0);
		}
	}

	return ret;
}

int v4l2_media_subdev_s_power(struct ipucsi *ipucsi, int enable)
{
	struct media_entity *entity = &ipucsi->subdev.entity;
	struct media_entity_graph graph;
	struct media_entity *first;
	struct v4l2_subdev *sd;
	int ret = 0;

	first = entity;

	media_entity_graph_walk_start(&graph, entity);
	if (!enable) {
		first = NULL;
		goto disable;
	}

	v4l2_ctrl_handler_init(&ipucsi->ctrls_vdev, 1);

	while (!ret && (entity = media_entity_graph_walk_next(&graph))) {
		if (media_entity_type(entity) == MEDIA_ENT_T_V4L2_SUBDEV) {
			sd = media_entity_to_v4l2_subdev(entity);
			ret = v4l2_subdev_call(sd, core, s_power, 1);
			if (ret == -ENOIOCTLCMD)
				ret = 0;

			ret = v4l2_ctrl_add_handler(&ipucsi->ctrls_vdev,
						    sd->ctrl_handler, NULL);
			if (ret)
				return ret;
		}
	}

	if (!ret)
		return 0;

	media_entity_graph_walk_start(&graph, first);

disable:
	while ((entity = media_entity_graph_walk_next(&graph)) && first != entity) {
		if (media_entity_type(entity) == MEDIA_ENT_T_V4L2_SUBDEV) {
			sd = media_entity_to_v4l2_subdev(entity);
			v4l2_subdev_call(sd, core, s_power, 0);
		}
	}

	return ret;
}

static void ipucsi_create_links(struct ipucsi *ipucsi)
{
	int i, ret;

	if (ipucsi->vdev.entity.num_links)
		return;

	/* Create links to capture device for each IDMAC channel */
	for (i = 0; i < ARRAY_SIZE(ipucsi_channels); i++) {
		struct media_pad *pad;

		pad = ipu_channel_to_media_pad(ipucsi->ipu, ipucsi_channels[i]);
		if (!pad) {
			v4l2_err(ipucsi->v4l2_dev,
				 "no media pad found for IDMAC channel %d\n",
				 ipucsi_channels[i]);
			continue;
		}

		ret = media_entity_create_link(pad->entity, pad->index,
					       &ipucsi->vdev.entity, 0, 0);
		if (ret < 0) {
			v4l2_err(ipucsi->v4l2_dev,
				 "failed to create link for '%s':%d: %d\n",
				 pad->entity->name, pad->index, ret);
		}
	}
}

static int ipucsi_open(struct file *file)
{
	struct ipucsi *ipucsi = video_drvdata(file);
	int ret;

	/*
	 * FIXME - this should be done somewhere else, once, after CSI, SMFC,
	 * and IC entities are registered:
	 */
	ipucsi_create_links(ipucsi);

	mutex_lock(&ipucsi->mutex);
	ret = v4l2_fh_open(file);
	if (ret)
		goto out;

	if (v4l2_fh_is_singular_file(file))
		ret = v4l2_media_subdev_s_power(ipucsi, 1);

out:
	mutex_unlock(&ipucsi->mutex);
	return ret;
}

static int ipucsi_release(struct file *file)
{
	struct ipucsi *ipucsi = video_drvdata(file);

	mutex_lock(&ipucsi->mutex);
	if (v4l2_fh_is_singular_file(file)) {
		v4l2_media_subdev_s_power(ipucsi, 0);
		mutex_unlock(&ipucsi->mutex);

		v4l2_ctrl_handler_free(&ipucsi->ctrls_vdev);

		vb2_fop_release(file);
	} else {
		mutex_unlock(&ipucsi->mutex);

		v4l2_fh_release(file);
	}

	return 0;
}

static u64 camera_mask = DMA_BIT_MASK(32);

static const struct v4l2_file_operations ipucsi_capture_fops = {
	.owner		= THIS_MODULE,
	.open		= ipucsi_open,
	.release	= ipucsi_release,
	.unlocked_ioctl	= video_ioctl2,
	.mmap		= vb2_fop_mmap,
	.poll		= vb2_fop_poll,
};

static int ipucsi_enum_framesizes(struct file *file, void *fh,
				  struct v4l2_frmsizeenum *fsize)
{
	struct ipucsi *ipucsi = video_drvdata(file);
	struct ipucsi_format *ipucsifmt = ipucsi_current_format(ipucsi);
	struct v4l2_mbus_config mbus_config;
	struct ipu_fmt *fmt = NULL;
	int ret;

	ret = ipu_csi_get_mbus_config(ipucsi, &mbus_config);
	if (ret)
		return ret;

	if (((fsize->index != 0) &&
	     (mbus_config.type != V4L2_MBUS_BT656)) ||
	    (fsize->index > 1))
		return -EINVAL;

	if (ipucsifmt->rgb)
		fmt = ipu_find_fmt_rgb(fsize->pixel_format);
	if (ipucsifmt->yuv)
		fmt = ipu_find_fmt_yuv(fsize->pixel_format);
	if (!fmt && !ipucsifmt->raw)
		return -EINVAL;

	if (mbus_config.type == V4L2_MBUS_BT656) {
		fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
		fsize->discrete.width = 720;
		fsize->discrete.height = fsize->index ? 576 : 480;
	} else {
		fsize->type = V4L2_FRMSIZE_TYPE_CONTINUOUS;
		fsize->stepwise.min_width = 1;
		fsize->stepwise.min_height = 1;
		fsize->stepwise.max_width = ipucsi->format_mbus[1].width;
		fsize->stepwise.max_height = ipucsi->format_mbus[1].height;
		fsize->stepwise.step_width = 1;
		fsize->stepwise.step_height = 1;
	}

	return 0;
}

static const struct v4l2_ioctl_ops ipucsi_capture_ioctl_ops = {
	.vidioc_querycap		= ipucsi_querycap,

	.vidioc_enum_fmt_vid_cap	= ipucsi_enum_fmt,
	.vidioc_try_fmt_vid_cap		= ipucsi_try_fmt,
	.vidioc_s_fmt_vid_cap		= ipucsi_s_fmt,
	.vidioc_g_fmt_vid_cap		= ipucsi_g_fmt,

	.vidioc_create_bufs		= vb2_ioctl_create_bufs,
	.vidioc_reqbufs			= vb2_ioctl_reqbufs,
	.vidioc_querybuf		= vb2_ioctl_querybuf,

	.vidioc_qbuf			= vb2_ioctl_qbuf,
	.vidioc_dqbuf			= vb2_ioctl_dqbuf,
	.vidioc_expbuf			= vb2_ioctl_expbuf,

	.vidioc_streamon		= vb2_ioctl_streamon,
	.vidioc_streamoff		= vb2_ioctl_streamoff,

	.vidioc_enum_framesizes		= ipucsi_enum_framesizes,
};

static int ipucsi_subdev_s_ctrl(struct v4l2_ctrl *ctrl)
{
	switch (ctrl->id) {
	case V4L2_CID_TEST_PATTERN:
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static const struct v4l2_ctrl_ops ipucsi_subdev_ctrl_ops = {
	.s_ctrl = ipucsi_subdev_s_ctrl,
};

static const char * const ipucsi_test_pattern_menu[] = {
	"Disabled",
	"Checkerboard-red",
	"Checkerboard-green",
	"Checkerboard-blue",
};

static int ipucsi_create_controls(struct ipucsi *ipucsi)
{
	struct v4l2_ctrl_handler *handler = &ipucsi->ctrls;

	v4l2_ctrl_handler_init(handler, 1);

	ipucsi->ctrl_test_pattern = v4l2_ctrl_new_std_menu_items(handler,
			&ipucsi_subdev_ctrl_ops, V4L2_CID_TEST_PATTERN,
			ARRAY_SIZE(ipucsi_test_pattern_menu) - 1, 0, 0,
			ipucsi_test_pattern_menu);

	return ipucsi->ctrl_test_pattern ? 0 : -ENOMEM;
}

static int ipucsi_subdev_init(struct ipucsi *ipucsi, struct device_node *node)
{
	struct device_node *endpoint;
	int ret;

	v4l2_subdev_init(&ipucsi->subdev, &ipucsi_subdev_ops);

	ipucsi->subdev.ctrl_handler = &ipucsi->ctrls;

	snprintf(ipucsi->subdev.name, sizeof(ipucsi->subdev.name), "%s",
			node->full_name);

	endpoint = of_get_next_child(node, NULL);
	if (endpoint)
		v4l2_of_parse_endpoint(endpoint, &ipucsi->endpoint);

	ipucsi->subdev.entity.ops = &ipucsi_entity_ops;

	ipucsi->subdev.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;

	ipucsi->subdev_pad[0].flags = MEDIA_PAD_FL_SINK;
	ipucsi->subdev_pad[1].flags = MEDIA_PAD_FL_SOURCE;
	ipucsi->subdev_pad[2].flags = MEDIA_PAD_FL_SOURCE;
	ipucsi->subdev_pad[3].flags = MEDIA_PAD_FL_SOURCE;
	ipucsi->subdev_pad[4].flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_init(&ipucsi->subdev.entity, 5, ipucsi->subdev_pad, 0);
	if (ret < 0)
		return ret;

	ret = ipu_register_subdev(ipucsi->ipu, ipucsi->id ? IPU_CSI1 : IPU_CSI0,
				  &ipucsi->subdev);
	if (ret < 0)
		return ret;

	return 0;
}

static int ipucsi_video_device_init(struct platform_device *pdev,
		struct ipucsi *ipucsi)
{
	struct video_device *vdev = &ipucsi->vdev;
	int ret;

	snprintf(vdev->name, sizeof(vdev->name), DRIVER_NAME ".%d", pdev->id);
	vdev->release	= video_device_release_empty;
	vdev->fops	= &ipucsi_capture_fops;
	vdev->ioctl_ops	= &ipucsi_capture_ioctl_ops;
	vdev->v4l2_dev	= ipucsi->v4l2_dev;
	vdev->minor	= -1;
	vdev->release	= video_device_release_empty;
	vdev->lock	= &ipucsi->mutex;
	vdev->ctrl_handler = &ipucsi->ctrls_vdev;
	vdev->queue	= &ipucsi->vb2_vidq;

	video_set_drvdata(vdev, ipucsi);

	ipucsi->media_pad.flags = MEDIA_PAD_FL_SINK;
	ret = media_entity_init(&vdev->entity, 1, &ipucsi->media_pad, 0);
	if (ret < 0)
		video_device_release(vdev);

	return ret;
}

static int ipucsi_vb2_init(struct ipucsi *ipucsi)
{
	struct vb2_queue *q;
	int ret;

	ipucsi->alloc_ctx = vb2_dma_contig_init_ctx(ipucsi->dev);
	if (IS_ERR(ipucsi->alloc_ctx))
		return PTR_ERR(ipucsi->alloc_ctx);

	q = &ipucsi->vb2_vidq;
	q->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	q->io_modes = VB2_MMAP | VB2_USERPTR | VB2_DMABUF;
	q->drv_priv = ipucsi;
	q->ops = &ipucsi_videobuf_ops;
	q->mem_ops = &vb2_dma_contig_memops;
	q->buf_struct_size = sizeof(struct ipucsi_buffer);
	q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;

	ret = vb2_queue_init(q);
	if (ret)
		return ret;

	return 0;
}

static int ipucsi_async_init(struct ipucsi *ipucsi, struct device_node *node)
{
	struct device_node *rp;

	rp = of_get_next_child(node, NULL);
	if (!rp)
		return 0;

	ipucsi->link = ipu_media_entity_create_link(&ipucsi->subdev, 0, rp,
			MEDIA_LNK_FL_IMMUTABLE | MEDIA_LNK_FL_ENABLED);

	if (IS_ERR(ipucsi->link)) {
		ipucsi->link = NULL;
		return PTR_ERR(ipucsi->link);
	}
	return 0;
}

static struct device_node *ipucsi_get_port(struct device_node *node, int id)
{
	struct device_node *port;
	int reg;

	for_each_child_of_node(node, port) {
		if (!of_property_read_u32(port, "reg", &reg) && reg == id)
			return of_node_get(port);
	}

	return NULL;
}

static int ipucsi_probe(struct platform_device *pdev)
{
	struct ipu_client_platformdata *pdata = pdev->dev.platform_data;
	struct ipu_soc *ipu = dev_get_drvdata(pdev->dev.parent);
	struct ipucsi *ipucsi;
	int ret;
	struct device_node *node;

	pdev->dev.dma_mask = &camera_mask;
	pdev->dev.coherent_dma_mask = DMA_BIT_MASK(32);

	if (!pdata)
		return -EINVAL;

	ipucsi = devm_kzalloc(&pdev->dev, sizeof(*ipucsi), GFP_KERNEL);
	if (!ipucsi)
		return -ENOMEM;

	INIT_LIST_HEAD(&ipucsi->capture);
	spin_lock_init(&ipucsi->lock);
	mutex_init(&ipucsi->mutex);

	ipucsi->ipu = ipu;
	ipucsi->id = pdata->csi;
	ipucsi->dev = &pdev->dev;
	ipucsi->v4l2_dev = ipu_media_get_v4l2_dev();
	if (!ipucsi->v4l2_dev)
		return -EPROBE_DEFER;

	node = ipucsi_get_port(pdev->dev.parent->of_node, pdata->csi);
	if (!node) {
		dev_err(&pdev->dev, "cannot find node port@%d\n", pdata->csi);
		return -ENODEV;
	}

	ipucsi->csi = ipu_csi_get(ipu, ipucsi->id);
	if (!ipucsi->csi)
		return -EBUSY;

	ret = ipucsi_video_device_init(pdev, ipucsi);
	if (ret)
		goto failed;

	ret = ipucsi_create_controls(ipucsi);
	if (ret)
		goto failed;

	ret = ipucsi_vb2_init(ipucsi);
	if (ret)
		goto failed;

	ret = ipucsi_subdev_init(ipucsi, node);
	if (ret)
		goto failed;

	ret = ipucsi_async_init(ipucsi, node);
	if (ret)
		goto failed;

	of_node_put(node);

	platform_set_drvdata(pdev, ipucsi);

	ret = video_register_device(&ipucsi->vdev, VFL_TYPE_GRABBER, -1);
	if (ret)
		goto failed;

	dev_info(&pdev->dev, "loaded\n");

	return 0;

failed:
	v4l2_ctrl_handler_free(&ipucsi->ctrls);
	if (ipucsi->link)
		ipu_media_entity_remove_link(ipucsi->link);
	if (ipucsi->vdev.entity.links)
		media_entity_cleanup(&ipucsi->vdev.entity);
	if (ipucsi->alloc_ctx)
		vb2_dma_contig_cleanup_ctx(ipucsi->alloc_ctx);
	if (ipucsi->ipuch)
		ipu_idmac_put(ipucsi->ipuch);

	return ret;
}

static int ipucsi_remove(struct platform_device *pdev)
{
	struct ipucsi *ipucsi = platform_get_drvdata(pdev);

	video_unregister_device(&ipucsi->vdev);
	ipu_media_entity_remove_link(ipucsi->link);
	media_entity_cleanup(&ipucsi->vdev.entity);
	vb2_dma_contig_cleanup_ctx(ipucsi->alloc_ctx);
	ipucsi_put_resources(ipucsi);
	v4l2_ctrl_handler_free(&ipucsi->ctrls);

	return 0;
}

static struct platform_driver ipucsi_driver = {
	.driver = {
		.name = DRIVER_NAME,
	},
	.probe = ipucsi_probe,
	.remove = ipucsi_remove,
};

static int __init ipucsi_init(void)
{
	return platform_driver_register(&ipucsi_driver);
}

static void __exit ipucsi_exit(void)
{
	return platform_driver_unregister(&ipucsi_driver);
}

subsys_initcall(ipucsi_init);
module_exit(ipucsi_exit);

MODULE_DESCRIPTION("i.MX51/53 IPUv3 Camera interface driver");
MODULE_AUTHOR("Sascha Hauer <s.hauer@pengutronix.de>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" DRIVER_NAME);

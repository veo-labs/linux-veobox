/*
 * V4L2 Capture Encoder Subdev for Freescale i.MX6 SOC
 *
 * Copyright (c) 2012-2014 Mentor Graphics Inc.
 * Copyright 2004-2012 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/timer.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/platform_device.h>
#include <linux/platform_data/camera-mx6.h>
#include <linux/pinctrl/consumer.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/videobuf2-dma-contig.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-of.h>
#include <media/v4l2-ctrls.h>
#include <video/imx-ipu-v3.h>
#include <media/imx6.h>
#include "mx6-camif.h"

struct encoder_priv {
	struct mx6cam_dev    *dev;
	struct v4l2_subdev    sd;

	struct ipuv3_channel *enc_ch;
	struct ipuv3_channel *enc_rot_in_ch;
	struct ipuv3_channel *enc_rot_out_ch;
	struct ipu_ic *ic_enc;
	struct ipu_smfc *smfc;
	struct ipu_csi *csi;

	struct v4l2_mbus_framefmt inf; /* input sensor format */
	struct v4l2_pix_format outf;   /* output user format */
	enum ipu_color_space in_cs;    /* input colorspace */
	enum ipu_color_space out_cs;   /* output colorspace */

	/* active (undergoing DMA) buffers, one for each IPU buffer */
	struct mx6cam_buffer *active_frame[2];

	struct mx6cam_dma_buf rot_buf[2];
	struct mx6cam_dma_buf underrun_buf;
	int buf_num;

	struct timer_list eof_timeout_timer;
	int eof_irq;
	int nfb4eof_irq;

	bool last_eof;  /* waiting for last EOF at encoder off */
	struct completion last_eof_comp;
};

/*
 * Update the CSI whole sensor and active windows, and initialize
 * the CSI interface and muxes.
 */
static void encoder_setup_csi(struct encoder_priv *priv)
{
	struct mx6cam_dev *dev = priv->dev;
	int csi_id = dev->ep->ep.base.port;
	int ipu_id = ipu_get_num(dev->ipu);
	bool is_csi2 = dev->ep->ep.bus_type == V4L2_MBUS_CSI2;

	ipu_csi_set_window(priv->csi, &dev->crop);
	ipu_csi_init_interface(priv->csi, &dev->mbus_cfg, &dev->sensor_fmt);
	if (is_csi2)
		ipu_csi_set_mipi_datatype(priv->csi, dev->ep->ep.base.id,
					  &dev->sensor_fmt);

	/* setup the video iomux */
	dev->pdata->set_video_mux(ipu_id, csi_id, is_csi2, dev->ep->ep.base.id);
	/* select either parallel or MIPI-CSI2 as input to our CSI */
	ipu_csi_set_src(priv->csi, dev->ep->ep.base.id, is_csi2);

	/* set CSI destination to IC or direct to mem via SMFC */
	ipu_csi_set_dest(priv->csi, dev->using_ic ?
			 IPU_CSI_DEST_IC : IPU_CSI_DEST_IDMAC);
	if (dev->using_ic) {
		/* set IC to receive from CSI */
		ipu_ic_set_src(priv->ic_enc, csi_id, false);
	}
}

static void encoder_put_ipu_resources(struct encoder_priv *priv)
{
	if (!IS_ERR_OR_NULL(priv->ic_enc))
		ipu_ic_put(priv->ic_enc);
	priv->ic_enc = NULL;

	if (!IS_ERR_OR_NULL(priv->enc_ch))
		ipu_idmac_put(priv->enc_ch);
	priv->enc_ch = NULL;

	if (!IS_ERR_OR_NULL(priv->enc_rot_in_ch))
		ipu_idmac_put(priv->enc_rot_in_ch);
	priv->enc_rot_in_ch = NULL;

	if (!IS_ERR_OR_NULL(priv->enc_rot_out_ch))
		ipu_idmac_put(priv->enc_rot_out_ch);
	priv->enc_rot_out_ch = NULL;

	if (!IS_ERR_OR_NULL(priv->smfc))
		ipu_smfc_put(priv->smfc);
	priv->smfc = NULL;

	if (!IS_ERR_OR_NULL(priv->csi))
		ipu_csi_put(priv->csi);
	priv->csi = NULL;
}

static int encoder_get_ipu_resources(struct encoder_priv *priv)
{
	struct mx6cam_dev *dev = priv->dev;
	int csi_id, csi_ch_num, err;

	csi_id = dev->ep->ep.base.port;
	priv->csi = ipu_csi_get(dev->ipu, csi_id);
	if (IS_ERR(priv->csi)) {
		v4l2_err(&priv->sd, "failed to get CSI %d\n", csi_id);
		return PTR_ERR(priv->csi);
	}

	if (dev->using_ic) {
		v4l2_err(&priv->sd, "Using IC\n");
		priv->ic_enc = ipu_ic_get(dev->ipu, IC_TASK_ENCODER);
		if (IS_ERR(priv->ic_enc)) {
			v4l2_err(&priv->sd, "failed to get IC ENC\n");
			err = PTR_ERR(priv->ic_enc);
			goto out;
		}

		priv->enc_ch = ipu_idmac_get(dev->ipu,
					     IPUV3_CHANNEL_IC_PRP_ENC_MEM);
		if (IS_ERR(priv->enc_ch)) {
			v4l2_err(&priv->sd, "could not get IDMAC channel %u\n",
				 IPUV3_CHANNEL_IC_PRP_ENC_MEM);
			err = PTR_ERR(priv->enc_ch);
			goto out;
		}

		priv->enc_rot_in_ch = ipu_idmac_get(dev->ipu,
						    IPUV3_CHANNEL_MEM_ROT_ENC);
		if (IS_ERR(priv->enc_rot_in_ch)) {
			v4l2_err(&priv->sd, "could not get IDMAC channel %u\n",
				 IPUV3_CHANNEL_MEM_ROT_ENC);
			err = PTR_ERR(priv->enc_rot_in_ch);
			goto out;
		}

		priv->enc_rot_out_ch = ipu_idmac_get(dev->ipu,
						     IPUV3_CHANNEL_ROT_ENC_MEM);
		if (IS_ERR(priv->enc_rot_out_ch)) {
			v4l2_err(&priv->sd, "could not get IDMAC channel %u\n",
				 IPUV3_CHANNEL_ROT_ENC_MEM);
			err = PTR_ERR(priv->enc_rot_out_ch);
			goto out;
		}
	} else {
		v4l2_err(&priv->sd, "Direct CSI -> SMFC -> MEM\n");
		/*
		 * Choose the direct CSI-->SMFC-->MEM channel corresponding
		 * to the IPU and CSI IDs.
		 */
		csi_ch_num = IPUV3_CHANNEL_CSI0 +
			(ipu_get_num(dev->ipu) << 1) + csi_id;

		priv->smfc = ipu_smfc_get(dev->ipu, csi_ch_num);
		if (IS_ERR(priv->smfc)) {
			v4l2_err(&priv->sd, "failed to get SMFC\n");
			err = PTR_ERR(priv->smfc);
			goto out;
		}

		priv->enc_ch = ipu_idmac_get(dev->ipu, csi_ch_num);
		if (IS_ERR(priv->enc_ch)) {
			v4l2_err(&priv->sd, "could not get IDMAC channel %u\n",
				 csi_ch_num);
			err = PTR_ERR(priv->enc_ch);
			goto out;
		}
	}

	return 0;
out:
	encoder_put_ipu_resources(priv);
	return err;
}

static irqreturn_t encoder_eof_interrupt(int irq, void *dev_id)
{
	struct encoder_priv *priv = dev_id;
	struct mx6cam_dev *dev = priv->dev;
	struct mx6cam_ctx *ctx = dev->io_ctx;
	struct mx6cam_buffer *frame;
	struct ipuv3_channel *channel;
	enum vb2_buffer_state state;
	struct timeval cur_time;
	unsigned long flags;
	dma_addr_t phys;

	spin_lock_irqsave(&dev->irqlock, flags);
	v4l2_err(&priv->sd, "Encoder EOF interrupt\n");

	/* timestamp and return the completed frame */
	frame = priv->active_frame[priv->buf_num];
	if (frame) {
		v4l2_get_timestamp(&cur_time);
		frame->vb.v4l2_buf.timestamp = cur_time;
		frame->vb.v4l2_buf.field = V4L2_FIELD_NONE;
		frame->vb.v4l2_buf.sequence = dev->sequence++;
		state = dev->signal_locked ?
			VB2_BUF_STATE_DONE : VB2_BUF_STATE_ERROR;
		v4l2_err(&priv->sd, "[EOF int]frame seq: %d, state %s\n", dev->signal_locked ? "DONE" : "ERROR");
		vb2_buffer_done(&frame->vb, state);
	}

	if (priv->last_eof) {
		complete(&priv->last_eof_comp);
		priv->active_frame[priv->buf_num] = NULL;
		priv->last_eof = false;
		goto unlock;
	}

	/* bump the EOF timeout timer */
	mod_timer(&priv->eof_timeout_timer,
		  jiffies + msecs_to_jiffies(MX6CAM_EOF_TIMEOUT));

	if (!list_empty(&dev->buf_list)) {
		frame = list_entry(dev->buf_list.next,
				   struct mx6cam_buffer, list);
		phys = vb2_dma_contig_plane_dma_addr(&frame->vb, 0);
		list_del(&frame->list);
		priv->active_frame[priv->buf_num] = frame;
	} else {
		phys = priv->underrun_buf.phys;
		priv->active_frame[priv->buf_num] = NULL;
	}

	channel = (dev->rot_mode >= IPU_ROTATE_90_RIGHT) ?
		priv->enc_rot_out_ch : priv->enc_ch;

	if (ipu_idmac_buffer_is_ready(channel, priv->buf_num))
		ipu_idmac_clear_buffer(channel, priv->buf_num);

	ipu_cpmem_set_buffer(channel, priv->buf_num, phys);
	ipu_idmac_select_buffer(channel, priv->buf_num);

	priv->buf_num ^= 1;

unlock:
	spin_unlock_irqrestore(&dev->irqlock, flags);
	return IRQ_HANDLED;
}

static irqreturn_t encoder_nfb4eof_interrupt(int irq, void *dev_id)
{
	struct encoder_priv *priv = dev_id;
	struct mx6cam_dev *dev = priv->dev;

	v4l2_err(&priv->sd, "NFB4EOF\n");

	/*
	 * It has been discovered that with rotation, encoder disable
	 * creates a single NFB4EOF event which is 100% repeatable. So
	 * scheduling a restart here causes an endless NFB4EOF-->restart
	 * cycle. The error itself seems innocuous, capture is not adversely
	 * affected.
	 *
	 * So don't schedule a restart on NFB4EOF error. If the source
	 * of the NFB4EOF event on disable is ever found, it can
	 * be re-enabled, but is probably not necessary. Detecting the
	 * interrupt (and clearing the irq status in the IPU) seems to
	 * be enough.
	 */
	if (!dev->using_ic)
		v4l2_subdev_notify(&priv->sd, MX6CAM_NFB4EOF_NOTIFY, NULL);

	return IRQ_HANDLED;
}

/*
 * EOF timeout timer function.
 */
static void encoder_eof_timeout(unsigned long data)
{
	struct encoder_priv *priv = (struct encoder_priv *)data;

	v4l2_err(&priv->sd, "encoder EOF timeout\n");

	v4l2_subdev_notify(&priv->sd, MX6CAM_EOF_TIMEOUT_NOTIFY, NULL);
}

static void encoder_free_dma_buf(struct encoder_priv *priv,
				 struct mx6cam_dma_buf *buf)
{
	struct mx6cam_dev *dev = priv->dev;

	if (buf->virt)
		dma_free_coherent(dev->dev, buf->len, buf->virt, buf->phys);

	buf->virt = NULL;
	buf->phys = 0;
}

static int encoder_alloc_dma_buf(struct encoder_priv *priv,
				 struct mx6cam_dma_buf *buf,
				 int size)
{
	struct mx6cam_dev *dev = priv->dev;

	encoder_free_dma_buf(priv, buf);

	buf->len = PAGE_ALIGN(size);
	buf->virt = dma_alloc_coherent(dev->dev, buf->len, &buf->phys,
				       GFP_DMA | GFP_KERNEL);
	if (!buf->virt) {
		v4l2_err(&priv->sd, "failed to alloc dma buffer\n");
		return -ENOMEM;
	}

	return 0;
}

static void encoder_setup_channel(struct encoder_priv *priv,
				  struct ipuv3_channel *channel,
				  enum ipu_rotate_mode rot_mode,
				  dma_addr_t addr0, dma_addr_t addr1,
				  bool rot_swap_width_height)
{
	struct mx6cam_dev *dev = priv->dev;
	u32 width, height, stride;
	unsigned int burst_size;
	struct ipu_image image;

	if (dev->using_ic && rot_swap_width_height) {
		width = priv->outf.height;
		height = priv->outf.width;
	} else {
		width = priv->outf.width;
		height = priv->outf.height;
	}

	stride = dev->user_pixfmt->y_depth ?
		(width * dev->user_pixfmt->y_depth) >> 3 :
		(width * dev->user_pixfmt->depth) >> 3;

	ipu_cpmem_zero(channel);

	memset(&image, 0, sizeof(image));
	image.pix.width = image.rect.width = width;
	image.pix.height = image.rect.height = height;
	image.pix.bytesperline = stride;
	image.pix.pixelformat = priv->outf.pixelformat;
	image.phys0 = addr0;
	image.phys1 = addr1;
	ipu_cpmem_set_image(channel, &image);

	if (channel == priv->enc_rot_in_ch ||
	    channel == priv->enc_rot_out_ch) {
		burst_size = 8;
		ipu_cpmem_set_block_mode(channel);
	} else
		burst_size = (width & 0xf) ? 8 : 16;

	ipu_cpmem_set_burstsize(channel, burst_size);

	if (!dev->using_ic) {
		int csi_id = dev->ep->ep.base.port;
		bool passthrough;

		/*
		 * If the sensor uses 16-bit parallel CSI bus, we must handle
		 * the data internally in the IPU as 16-bit generic, aka
		 * passthrough mode.
		 */
		passthrough = (dev->ep->ep.bus_type != V4L2_MBUS_CSI2 &&
			       dev->ep->ep.bus.parallel.bus_width >= 16);

		if (passthrough)
			ipu_cpmem_set_format_passthrough(channel, 16);

		if (dev->ep->ep.bus_type == V4L2_MBUS_CSI2)
			ipu_smfc_map_channel(priv->smfc, csi_id,
					     dev->ep->ep.base.id);
		else
			ipu_smfc_map_channel(priv->smfc, csi_id, 0);

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
		ipu_smfc_set_watermark(priv->smfc, 0x02, 0x01);
		ipu_cpmem_set_high_priority(channel);
		ipu_idmac_enable_watermark(channel, true);
		ipu_cpmem_set_axi_id(channel, 0);
		ipu_idmac_lock_enable(channel, 8);

		burst_size = passthrough ?
			(burst_size >> 3) - 1 : (burst_size >> 2) - 1;

		ipu_smfc_set_burstsize(priv->smfc, burst_size);
	}

	if (rot_mode)
		ipu_cpmem_set_rotation(channel, rot_mode);

	if (ipu_csi_is_interlaced(priv->csi) && channel == priv->enc_ch)
		ipu_cpmem_interlaced_scan(channel, stride);

	if (dev->using_ic) {
		ipu_ic_task_idma_init(priv->ic_enc, channel, width, height,
				      burst_size, rot_mode);
		ipu_cpmem_set_axi_id(channel, 1);
	}

	ipu_idmac_set_double_buffer(channel, true);
}

static int encoder_setup_rotation(struct encoder_priv *priv,
				  dma_addr_t phys0, dma_addr_t phys1)
{
	struct mx6cam_dev *dev = priv->dev;
	int err;

	err = encoder_alloc_dma_buf(priv, &priv->underrun_buf,
				    priv->outf.sizeimage);
	if (err) {
		v4l2_err(&priv->sd, "failed to alloc underrun_buf, %d\n", err);
		return err;
	}

	err = encoder_alloc_dma_buf(priv, &priv->rot_buf[0],
				    priv->outf.sizeimage);
	if (err) {
		v4l2_err(&priv->sd, "failed to alloc rot_buf[0], %d\n", err);
		goto free_underrun;
	}
	err = encoder_alloc_dma_buf(priv, &priv->rot_buf[1],
				    priv->outf.sizeimage);
	if (err) {
		v4l2_err(&priv->sd, "failed to alloc rot_buf[1], %d\n", err);
		goto free_rot0;
	}

	err = ipu_ic_task_init(priv->ic_enc,
			       priv->inf.width, priv->inf.height,
			       priv->outf.height, priv->outf.width,
			       priv->in_cs, priv->out_cs);
	if (err) {
		v4l2_err(&priv->sd, "ipu_ic_task_init failed, %d\n", err);
		goto free_rot1;
	}

	/* init the IC ENC-->MEM IDMAC channel */
	encoder_setup_channel(priv, priv->enc_ch,
			      IPU_ROTATE_NONE,
			      priv->rot_buf[0].phys,
			      priv->rot_buf[1].phys,
			      true);

	/* init the MEM-->IC ENC ROT IDMAC channel */
	encoder_setup_channel(priv, priv->enc_rot_in_ch,
			      dev->rot_mode,
			      priv->rot_buf[0].phys,
			      priv->rot_buf[1].phys,
			      true);

	/* init the destination IC ENC ROT-->MEM IDMAC channel */
	encoder_setup_channel(priv, priv->enc_rot_out_ch,
			      IPU_ROTATE_NONE,
			      phys0, phys1,
			      false);

	/* now link IC ENC-->MEM to MEM-->IC ENC ROT */
	ipu_idmac_link(priv->enc_ch, priv->enc_rot_in_ch);

	/* enable the IC */
	ipu_ic_enable(priv->ic_enc);

	/* set buffers ready */
	ipu_idmac_select_buffer(priv->enc_ch, 0);
	ipu_idmac_select_buffer(priv->enc_ch, 1);
	ipu_idmac_select_buffer(priv->enc_rot_out_ch, 0);
	ipu_idmac_select_buffer(priv->enc_rot_out_ch, 1);

	/* enable the channels */
	ipu_idmac_enable_channel(priv->enc_ch);
	ipu_idmac_enable_channel(priv->enc_rot_in_ch);
	ipu_idmac_enable_channel(priv->enc_rot_out_ch);

	/* and finally enable the IC PRPENC task */
	ipu_ic_task_enable(priv->ic_enc);

	return 0;

free_rot1:
	encoder_free_dma_buf(priv, &priv->rot_buf[1]);
free_rot0:
	encoder_free_dma_buf(priv, &priv->rot_buf[0]);
free_underrun:
	encoder_free_dma_buf(priv, &priv->underrun_buf);
	return err;
}

static int encoder_setup_norotation(struct encoder_priv *priv,
				    dma_addr_t phys0, dma_addr_t phys1)
{
	struct mx6cam_dev *dev = priv->dev;
	int err;

	err = encoder_alloc_dma_buf(priv, &priv->underrun_buf,
				    priv->outf.sizeimage);
	if (err) {
		v4l2_err(&priv->sd, "failed to alloc underrun_buf, %d\n", err);
		return err;
	}

	if (dev->using_ic) {
		err = ipu_ic_task_init(priv->ic_enc,
				       priv->inf.width, priv->inf.height,
				       priv->outf.width, priv->outf.height,
				       priv->in_cs, priv->out_cs);
		if (err) {
			v4l2_err(&priv->sd, "ipu_ic_task_init failed, %d\n",
				 err);
			goto free_underrun;
		}
	}

	/* init the IC PRP-->MEM IDMAC channel */
	encoder_setup_channel(priv, priv->enc_ch, dev->rot_mode,
			      phys0, phys1, false);

	if (dev->using_ic)
		ipu_ic_enable(priv->ic_enc);
	else
		ipu_smfc_enable(priv->smfc);

	/* set buffers ready */
	ipu_idmac_select_buffer(priv->enc_ch, 0);
	ipu_idmac_select_buffer(priv->enc_ch, 1);

	/* enable the channels */
	ipu_idmac_enable_channel(priv->enc_ch);

	if (dev->using_ic) {
		/* enable the IC ENCODE task */
		ipu_ic_task_enable(priv->ic_enc);
	}

	return 0;

free_underrun:
	encoder_free_dma_buf(priv, &priv->underrun_buf);
	return err;
}

static int encoder_start(struct encoder_priv *priv)
{
	struct mx6cam_dev *dev = priv->dev;
	struct mx6cam_ctx *ctx = dev->io_ctx;
	struct mx6cam_buffer *frame, *tmp;
	dma_addr_t phys[2] = {0};
	int i = 0, err;

	err = encoder_get_ipu_resources(priv);
	if (err)
		return err;

	list_for_each_entry_safe(frame, tmp, &dev->buf_list, list) {
		phys[i] = vb2_dma_contig_plane_dma_addr(&frame->vb, 0);
		list_del(&frame->list);
		priv->active_frame[i++] = frame;
		if (i >= 2)
			break;
	}

	/* if preview is enabled it has already setup the CSI */
	if (!dev->preview_on) {
		v4l2_err(&priv->sd, "Preview is off\n");
		encoder_setup_csi(priv);
	}

	priv->inf = dev->sensor_fmt;
	priv->inf.width = dev->crop.width;
	priv->inf.height = dev->crop.height;
	priv->in_cs = ipu_mbus_code_to_colorspace(priv->inf.code);

	priv->outf = dev->format;
	priv->out_cs = ipu_pixelformat_to_colorspace(priv->outf.pixelformat);

	priv->buf_num = 0;

	if (dev->rot_mode >= IPU_ROTATE_90_RIGHT)
		err = encoder_setup_rotation(priv, phys[0], phys[1]);
	else
		err = encoder_setup_norotation(priv, phys[0], phys[1]);
	if (err)
		goto out_put_ipu;

	priv->nfb4eof_irq = ipu_idmac_channel_irq(dev->ipu,
						 priv->enc_ch,
						 IPU_IRQ_NFB4EOF);
	err = devm_request_irq(dev->dev, priv->nfb4eof_irq,
			       encoder_nfb4eof_interrupt, 0,
			       "mx6cam-enc-nfb4eof", priv);
	if (err) {
		v4l2_err(&priv->sd,
			 "Error registering encode NFB4EOF irq: %d\n", err);
		goto out_put_ipu;
	}

	if (dev->rot_mode >= IPU_ROTATE_90_RIGHT)
		priv->eof_irq = ipu_idmac_channel_irq(
			dev->ipu, priv->enc_rot_out_ch, IPU_IRQ_EOF);
	else
		priv->eof_irq = ipu_idmac_channel_irq(
			dev->ipu, priv->enc_ch, IPU_IRQ_EOF);

	err = devm_request_irq(dev->dev, priv->eof_irq,
			       encoder_eof_interrupt, 0,
			       "mx6cam-enc-eof", priv);
	if (err) {
		v4l2_err(&priv->sd,
			 "Error registering encode eof irq: %d\n", err);
		goto out_free_nfb4eof_irq;
	}

	v4l2_err(&priv->sd, "Enable CSI\n");
	err = ipu_csi_enable(priv->csi);
	if (err) {
		v4l2_err(&priv->sd, "CSI enable error: %d\n", err);
		goto out_free_eof_irq;
	}
	ipu_csi_dump(priv->csi);

	/* start the EOF timeout timer */
	mod_timer(&priv->eof_timeout_timer,
		  jiffies + msecs_to_jiffies(MX6CAM_EOF_TIMEOUT));

	return 0;

out_free_eof_irq:
	devm_free_irq(dev->dev, priv->eof_irq, priv);
out_free_nfb4eof_irq:
	devm_free_irq(dev->dev, priv->nfb4eof_irq, priv);
out_put_ipu:
	encoder_put_ipu_resources(priv);
	return err;
}

static int encoder_stop(struct encoder_priv *priv)
{
	struct mx6cam_dev *dev = priv->dev;
	struct mx6cam_buffer *frame;
	struct timeval cur_time;
	int i, ret;

	v4l2_err(&priv->sd, "Entering encoder stop\n");
	/* stop the EOF timeout timer */
	del_timer_sync(&priv->eof_timeout_timer);

	/*
	 * Mark next EOF interrupt as the last before encoder off,
	 * and then wait for interrupt handler to mark completion.
	 */
	init_completion(&priv->last_eof_comp);
	priv->last_eof = true;
	ret = wait_for_completion_timeout(&priv->last_eof_comp,
					  msecs_to_jiffies(MX6CAM_EOF_TIMEOUT));
	if (ret == 0)
		v4l2_warn(&priv->sd, "wait last encode EOF timeout\n");

	ipu_csi_disable(priv->csi);

	devm_free_irq(dev->dev, priv->eof_irq, priv);
	devm_free_irq(dev->dev, priv->nfb4eof_irq, priv);

	/* disable IC tasks and the channels */
	if (dev->using_ic)
		ipu_ic_task_disable(priv->ic_enc);

	ipu_idmac_disable_channel(priv->enc_ch);
	if (dev->rot_mode >= IPU_ROTATE_90_RIGHT) {
		ipu_idmac_disable_channel(priv->enc_rot_in_ch);
		ipu_idmac_disable_channel(priv->enc_rot_out_ch);
	}

	if (dev->rot_mode >= IPU_ROTATE_90_RIGHT)
		ipu_idmac_unlink(priv->enc_ch, priv->enc_rot_in_ch);

	if (dev->using_ic)
		ipu_ic_disable(priv->ic_enc);
	else
		ipu_smfc_disable(priv->smfc);

	encoder_free_dma_buf(priv, &priv->rot_buf[0]);
	encoder_free_dma_buf(priv, &priv->rot_buf[1]);
	encoder_free_dma_buf(priv, &priv->underrun_buf);

	encoder_put_ipu_resources(priv);

	/* return any remaining active frames with error */
	for (i = 0; i < 2; i++) {
		frame = priv->active_frame[i];
		if (frame && frame->vb.state == VB2_BUF_STATE_ACTIVE) {
			do_gettimeofday(&cur_time);
			frame->vb.v4l2_buf.timestamp = cur_time;
			frame->vb.v4l2_buf.field = V4L2_FIELD_NONE;
			frame->vb.v4l2_buf.sequence = dev->sequence++;
			v4l2_err(&priv->sd, "frame seq: %d\n");
			vb2_buffer_done(&frame->vb, VB2_BUF_STATE_ERROR);
			priv->active_frame[i] = NULL;
		}
	}

	return 0;
}

static int encoder_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct encoder_priv *priv = v4l2_get_subdevdata(sd);

	return enable ? encoder_start(priv) : encoder_stop(priv);
}

static struct v4l2_subdev_video_ops encoder_video_ops = {
	.s_stream = encoder_s_stream,
};

static struct v4l2_subdev_ops encoder_subdev_ops = {
	.video = &encoder_video_ops,
};

struct v4l2_subdev *mx6cam_encoder_init(struct mx6cam_dev *dev)
{
	struct encoder_priv *priv;

	priv = devm_kzalloc(dev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return ERR_PTR(-ENOMEM);

	init_timer(&priv->eof_timeout_timer);
	priv->eof_timeout_timer.data = (unsigned long)priv;
	priv->eof_timeout_timer.function = encoder_eof_timeout;

	v4l2_subdev_init(&priv->sd, &encoder_subdev_ops);
	strlcpy(priv->sd.name, "mx6-camera-encoder", sizeof(priv->sd.name));
	v4l2_set_subdevdata(&priv->sd, priv);

	priv->dev = dev;
	return &priv->sd;
}

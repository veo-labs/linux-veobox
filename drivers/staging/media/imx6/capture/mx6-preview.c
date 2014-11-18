/*
 * V4L2 Preview Subdev for Freescale i.MX6 SOC
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

struct preview_framebuffer {
	struct dma_buf            *dbuf;
	struct dma_buf_attachment *attach;
	struct sg_table           *sgt;
	dma_addr_t                phys;
};

struct preview_priv {
	struct mx6cam_dev    *dev;
	struct v4l2_subdev    sd;
	struct v4l2_ctrl_handler ctrl_hdlr;

	struct preview_framebuffer fbuf;

	struct ipuv3_channel *preview_ch;
	struct ipuv3_channel *preview_rot_in_ch;
	struct ipuv3_channel *preview_rot_out_ch;
	struct ipu_ic *ic_vf;
	struct ipu_csi *csi;

	struct v4l2_mbus_framefmt inf; /* input sensor format */
	struct v4l2_pix_format outf;   /* output user format */
	enum ipu_color_space in_cs;    /* input colorspace */
	enum ipu_color_space out_cs;   /* output colorspace */

	/* v4l2 controls */
	int                   rotation; /* degrees */
	bool                  hflip;
	bool                  vflip;
	/* derived from rotation, hflip, vflip controls */
	enum ipu_rotate_mode  rot_mode;

	struct timer_list eof_timeout_timer;
	int eof_irq;
	int nfb4eof_irq;

	struct mx6cam_dma_buf rot_buf[2];
	int buf_num;

	bool preview_active;
	bool last_eof;  /* waiting for last EOF at preview off */
	struct completion last_eof_comp;
};

/*
 * Update the CSI whole sensor and active windows, and initialize
 * the CSI interface and muxes.
 */
static void preview_setup_csi(struct preview_priv *priv)
{
	struct mx6cam_dev *dev = priv->dev;
	int csi_id = dev->ep->ep.base.port;
	int ipu_id = ipu_get_num(dev->ipu);
	bool is_csi2 = dev->ep->ep.bus_type == V4L2_MBUS_CSI2;

	ipu_csi_set_window(priv->csi, &dev->crop);
	ipu_csi_init_interface(priv->csi, &dev->mbus_cfg, &dev->subdev_fmt);
	if (is_csi2)
		ipu_csi_set_mipi_datatype(priv->csi, dev->ep->ep.base.id,
					  &dev->subdev_fmt);

	/* setup the video iomux */
	dev->pdata->set_video_mux(ipu_id, csi_id, is_csi2, dev->ep->ep.base.id);
	/* select either parallel or MIPI-CSI2 as input to our CSI */
	ipu_csi_set_src(priv->csi, dev->ep->ep.base.id, is_csi2);
	/* set CSI destination to IC */
	ipu_csi_set_dest(priv->csi, IPU_CSI_DEST_IC);
	/* set IC to receive from CSI */
	ipu_ic_set_src(priv->ic_vf, csi_id, false);
}

static void preview_put_ipu_resources(struct preview_priv *priv)
{
	if (!IS_ERR_OR_NULL(priv->ic_vf))
		ipu_ic_put(priv->ic_vf);
	priv->ic_vf = NULL;

	if (!IS_ERR_OR_NULL(priv->preview_ch))
		ipu_idmac_put(priv->preview_ch);
	priv->preview_ch = NULL;

	if (!IS_ERR_OR_NULL(priv->preview_rot_in_ch))
		ipu_idmac_put(priv->preview_rot_in_ch);
	priv->preview_rot_in_ch = NULL;

	if (!IS_ERR_OR_NULL(priv->preview_rot_out_ch))
		ipu_idmac_put(priv->preview_rot_out_ch);
	priv->preview_rot_out_ch = NULL;

	if (!IS_ERR_OR_NULL(priv->csi))
		ipu_csi_put(priv->csi);
	priv->csi = NULL;
}

static int preview_get_ipu_resources(struct preview_priv *priv)
{
	struct mx6cam_dev *dev = priv->dev;
	int csi_id, err;

	csi_id = dev->ep->ep.base.port;
	priv->csi = ipu_csi_get(dev->ipu, csi_id);
	if (IS_ERR(priv->csi)) {
		v4l2_err(&priv->sd, "failed to get CSI %d\n", csi_id);
		return PTR_ERR(priv->csi);
	}

	priv->ic_vf = ipu_ic_get(dev->ipu, IC_TASK_VIEWFINDER);
	if (IS_ERR(priv->ic_vf)) {
		v4l2_err(&priv->sd, "failed to get IC VF\n");
		err = PTR_ERR(priv->ic_vf);
		goto out;
	}

	priv->preview_ch = ipu_idmac_get(dev->ipu,
					 IPUV3_CHANNEL_IC_PRP_VF_MEM);
	if (IS_ERR(priv->preview_ch)) {
		v4l2_err(&priv->sd, "could not get IDMAC channel %u\n",
			 IPUV3_CHANNEL_IC_PRP_VF_MEM);
		err = PTR_ERR(priv->preview_ch);
		goto out;
	}

	priv->preview_rot_in_ch = ipu_idmac_get(dev->ipu,
						IPUV3_CHANNEL_MEM_ROT_VF);
	if (IS_ERR(priv->preview_rot_in_ch)) {
		v4l2_err(&priv->sd, "could not get IDMAC channel %u\n",
			 IPUV3_CHANNEL_MEM_ROT_ENC);
		err = PTR_ERR(priv->preview_rot_in_ch);
		goto out;
	}

	priv->preview_rot_out_ch = ipu_idmac_get(dev->ipu,
						 IPUV3_CHANNEL_ROT_VF_MEM);
	if (IS_ERR(priv->preview_rot_out_ch)) {
		v4l2_err(&priv->sd, "could not get IDMAC channel %u\n",
			 IPUV3_CHANNEL_ROT_ENC_MEM);
		err = PTR_ERR(priv->preview_rot_out_ch);
		goto out;
	}

	return 0;
out:
	preview_put_ipu_resources(priv);
	return err;
}

static irqreturn_t preview_eof_interrupt(int irq, void *dev_id)
{
	struct preview_priv *priv = dev_id;

	if (priv->last_eof) {
		complete(&priv->last_eof_comp);
		priv->last_eof = false;
		return IRQ_HANDLED;
	}

	/* bump the EOF timeout timer */
	mod_timer(&priv->eof_timeout_timer,
		  jiffies + msecs_to_jiffies(MX6CAM_EOF_TIMEOUT));

	if (priv->rot_mode >= IPU_ROTATE_90_RIGHT)
		ipu_idmac_select_buffer(priv->preview_rot_out_ch,
					priv->buf_num);
	else
		ipu_idmac_select_buffer(priv->preview_ch, priv->buf_num);

	priv->buf_num ^= 1;

	return IRQ_HANDLED;
}

static irqreturn_t preview_nfb4eof_interrupt(int irq, void *dev_id)
{
	struct preview_priv *priv = dev_id;

	v4l2_err(&priv->sd, "preview NFB4EOF\n");

	/*
	 * It has been discovered that with rotation, preview disable
	 * creates a single NFB4EOF event which is 100% repeatable. So
	 * scheduling a restart here causes an endless NFB4EOF-->restart
	 * cycle. The error itself seems innocuous, capture is not adversely
	 * affected.
	 *
	 * So don't schedule a restart on NFB4EOF error. If the source
	 * of the NFB4EOF event on preview disable is ever found, it can
	 * be re-enabled, but is probably not necessary. Detecting the
	 * interrupt (and clearing the irq status in the IPU) seems to
	 * be enough.
	 */
#if 0
	v4l2_subdev_notify(&priv->sd, MX6CAM_NFB4EOF_NOTIFY, NULL);
#endif

	return IRQ_HANDLED;
}

/*
 * EOF timeout timer function.
 */
static void preview_eof_timeout(unsigned long data)
{
	struct preview_priv *priv = (struct preview_priv *)data;

	v4l2_err(&priv->sd, "preview EOF timeout\n");

	v4l2_subdev_notify(&priv->sd, MX6CAM_EOF_TIMEOUT_NOTIFY, NULL);
}

static void preview_free_dma_buf(struct preview_priv *priv,
				 struct mx6cam_dma_buf *buf)
{
	struct mx6cam_dev *dev = priv->dev;

	if (buf->virt)
		dma_free_coherent(dev->dev, buf->len, buf->virt, buf->phys);

	buf->virt = NULL;
	buf->phys = 0;
}

static int preview_alloc_dma_buf(struct preview_priv *priv,
				 struct mx6cam_dma_buf *buf,
				 int size)
{
	struct mx6cam_dev *dev = priv->dev;

	preview_free_dma_buf(priv, buf);

	buf->len = PAGE_ALIGN(size);
	buf->virt = dma_alloc_coherent(dev->dev, buf->len, &buf->phys,
				       GFP_DMA | GFP_KERNEL);
	if (!buf->virt) {
		v4l2_err(&priv->sd, "failed to alloc dma buffer\n");
		return -ENOMEM;
	}

	return 0;
}

static void preview_setup_channel(struct preview_priv *priv,
				  struct ipuv3_channel *channel,
				  enum ipu_rotate_mode rot_mode,
				  dma_addr_t addr0, dma_addr_t addr1,
				  bool rot_swap_width_height)
{
	struct mx6cam_dev *dev = priv->dev;
	unsigned int burst_size;
	u32 width, height, stride;
	struct ipu_image image;

	if (rot_swap_width_height) {
		width = priv->outf.height;
		height = priv->outf.width;
	} else {
		width = priv->outf.width;
		height = priv->outf.height;
	}
	stride = dev->fbuf_pixfmt->y_depth ?
		(width * dev->fbuf_pixfmt->y_depth) >> 3 :
		(width * dev->fbuf_pixfmt->depth) >> 3;

	ipu_cpmem_zero(channel);

	memset(&image, 0, sizeof(image));
	image.pix.width = image.rect.width = width;
	image.pix.height = image.rect.height = height;
	image.pix.bytesperline = stride;
	image.pix.pixelformat = priv->outf.pixelformat;
	image.phys0 = addr0;
	image.phys1 = addr1;
	ipu_cpmem_set_image(channel, &image);

	if (rot_mode)
		ipu_cpmem_set_rotation(channel, rot_mode);

	if (channel == priv->preview_rot_in_ch ||
	    channel == priv->preview_rot_out_ch) {
		burst_size = 8;
		ipu_cpmem_set_block_mode(channel);
	} else
		burst_size = (width % 16) ? 8 : 16;

	ipu_cpmem_set_burstsize(channel, burst_size);

	if (ipu_csi_is_interlaced(priv->csi) && channel == priv->preview_ch)
		ipu_cpmem_interlaced_scan(channel, stride);

	ipu_ic_task_idma_init(priv->ic_vf, channel, width, height,
			      burst_size, rot_mode);

	ipu_cpmem_set_axi_id(channel, 1);

	ipu_idmac_set_double_buffer(channel, true);
}

static int preview_setup_rotation(struct preview_priv *priv,
				  dma_addr_t phys0, dma_addr_t phys1)
{
	int err;

	err = preview_alloc_dma_buf(priv, &priv->rot_buf[0],
				    priv->outf.sizeimage);
	if (err) {
		v4l2_err(&priv->sd, "failed to alloc rot_buf[0], %d\n", err);
		return err;
	}
	err = preview_alloc_dma_buf(priv, &priv->rot_buf[1],
				    priv->outf.sizeimage);
	if (err) {
		v4l2_err(&priv->sd, "failed to alloc rot_buf[1], %d\n", err);
		goto free_rot0;
	}

	err = ipu_ic_task_init(priv->ic_vf,
			       priv->inf.width, priv->inf.height,
			       priv->outf.height, priv->outf.width,
			       priv->in_cs, priv->out_cs);
	if (err) {
		v4l2_err(&priv->sd, "ipu_ic_task_init failed, %d\n", err);
		goto free_rot1;
	}

	/* init the IC PREVIEW-->MEM IDMAC channel */
	preview_setup_channel(priv, priv->preview_ch,
			      IPU_ROTATE_NONE,
			      priv->rot_buf[0].phys,
			      priv->rot_buf[1].phys,
			      true);

	/* init the MEM-->IC PREVIEW ROT IDMAC channel */
	preview_setup_channel(priv, priv->preview_rot_in_ch,
			      priv->rot_mode,
			      priv->rot_buf[0].phys,
			      priv->rot_buf[1].phys,
			      true);

	/* init the destination IC PREVIEW ROT-->MEM IDMAC channel */
	preview_setup_channel(priv, priv->preview_rot_out_ch,
			      IPU_ROTATE_NONE,
			      phys0, phys1,
			      false);

	/* now link IC PREVIEW-->MEM to MEM-->IC PREVIEW ROT */
	ipu_idmac_link(priv->preview_ch, priv->preview_rot_in_ch);

	/* enable the IC */
	ipu_ic_enable(priv->ic_vf);

	/* set buffers ready */
	ipu_idmac_select_buffer(priv->preview_ch, 0);
	ipu_idmac_select_buffer(priv->preview_ch, 1);
	ipu_idmac_select_buffer(priv->preview_rot_out_ch, 0);
	ipu_idmac_select_buffer(priv->preview_rot_out_ch, 1);

	/* enable the channels */
	ipu_idmac_enable_channel(priv->preview_ch);
	ipu_idmac_enable_channel(priv->preview_rot_in_ch);
	ipu_idmac_enable_channel(priv->preview_rot_out_ch);

	/* and finally enable the IC PREVIEW task */
	ipu_ic_task_enable(priv->ic_vf);

	return 0;

free_rot1:
	preview_free_dma_buf(priv, &priv->rot_buf[1]);
free_rot0:
	preview_free_dma_buf(priv, &priv->rot_buf[0]);
	return err;
}

static int preview_setup_norotation(struct preview_priv *priv,
				    dma_addr_t phys0, dma_addr_t phys1)
{
	int err;

	err = ipu_ic_task_init(priv->ic_vf,
			       priv->inf.width, priv->inf.height,
			       priv->outf.width, priv->outf.height,
			       priv->in_cs, priv->out_cs);
	if (err) {
		v4l2_err(&priv->sd, "ipu_ic_task_init failed, %d\n", err);
		return err;
	}

	/* init the IC PREVIEW-->MEM IDMAC channel */
	preview_setup_channel(priv, priv->preview_ch, priv->rot_mode,
			      phys0, phys1, false);

	ipu_ic_enable(priv->ic_vf);

	/* set buffers ready */
	ipu_idmac_select_buffer(priv->preview_ch, 0);
	ipu_idmac_select_buffer(priv->preview_ch, 1);

	/* enable the channels */
	ipu_idmac_enable_channel(priv->preview_ch);

	/* and finally enable the IC PREVIEW task */
	ipu_ic_task_enable(priv->ic_vf);

	return 0;
}

static void unmap_fbuf(struct preview_priv *priv)
{
	if (!priv->fbuf.phys)
		return;

	dma_buf_unmap_attachment(priv->fbuf.attach, priv->fbuf.sgt,
				 DMA_TO_DEVICE);
	dma_buf_detach(priv->fbuf.dbuf, priv->fbuf.attach);
	dma_buf_put(priv->fbuf.dbuf);
	priv->fbuf.phys = 0;
}

static int map_fbuf(struct preview_priv *priv)
{
	struct mx6cam_dev *dev = priv->dev;

	unmap_fbuf(priv);

	priv->fbuf.dbuf = dma_buf_get((int)dev->fbuf.base);
	if (IS_ERR_OR_NULL(priv->fbuf.dbuf)) {
		v4l2_err(&priv->sd, "invalid dmabuf fd\n");
		return -EINVAL;
	}

	priv->fbuf.attach = dma_buf_attach(priv->fbuf.dbuf, dev->dev);
	if (IS_ERR(priv->fbuf.attach)) {
		v4l2_err(&priv->sd, "failed to attach dmabuf\n");
		dma_buf_put(priv->fbuf.dbuf);
		return PTR_ERR(priv->fbuf.attach);
	}

	priv->fbuf.sgt = dma_buf_map_attachment(priv->fbuf.attach,
						DMA_TO_DEVICE);
	if (IS_ERR(priv->fbuf.sgt)) {
		v4l2_err(&priv->sd, "failed to map dmabuf\n");
		dma_buf_detach(priv->fbuf.dbuf, priv->fbuf.attach);
		dma_buf_put(priv->fbuf.dbuf);
		return PTR_ERR(priv->fbuf.sgt);
	}

	priv->fbuf.phys = sg_dma_address(priv->fbuf.sgt->sgl);

	return 0;
}

static int preview_start(struct preview_priv *priv)
{
	struct mx6cam_dev *dev = priv->dev;
	dma_addr_t fb_buf;
	int err = 0;

	if (priv->preview_active) {
		v4l2_warn(&priv->sd, "preview already started\n");
		return 0;
	}

	err = map_fbuf(priv);
	if (err)
		return err;

	err = preview_get_ipu_resources(priv);
	if (err)
		goto out_unmap_fbuf;

	/* if encoder is enabled it has already setup the CSI */
	if (!dev->encoder_on)
		preview_setup_csi(priv);

	priv->inf = dev->subdev_fmt;
	priv->inf.width = dev->crop.width;
	priv->inf.height = dev->crop.height;
	priv->in_cs = ipu_mbus_code_to_colorspace(priv->inf.code);

	priv->outf.width = dev->win.w.width;
	priv->outf.height = dev->win.w.height;
	priv->outf.pixelformat = dev->fbuf.fmt.pixelformat;
	priv->outf.sizeimage =
		(priv->outf.width * priv->outf.height *
		 dev->fbuf_pixfmt->depth) >> 3;
	priv->out_cs = ipu_pixelformat_to_colorspace(priv->outf.pixelformat);

	fb_buf = priv->fbuf.phys;

	priv->buf_num = 0;

	if (priv->rot_mode >= IPU_ROTATE_90_RIGHT)
		err = preview_setup_rotation(priv, fb_buf, fb_buf);
	else
		err = preview_setup_norotation(priv, fb_buf, fb_buf);
	if (err)
		goto out_put_ipu;

	priv->nfb4eof_irq = ipu_idmac_channel_irq(dev->ipu,
						  priv->preview_ch,
						  IPU_IRQ_NFB4EOF);
	err = devm_request_irq(dev->dev, priv->nfb4eof_irq,
			       preview_nfb4eof_interrupt, 0,
			       "mx6cam-preview-nfb4eof", priv);
	if (err) {
		v4l2_err(&priv->sd,
			 "Error registering preview NFB4EOF irq: %d\n", err);
		goto out_put_ipu;
	}

	if (priv->rot_mode >= IPU_ROTATE_90_RIGHT)
		priv->eof_irq = ipu_idmac_channel_irq(dev->ipu,
						      priv->preview_rot_out_ch,
						      IPU_IRQ_EOF);
	else
		priv->eof_irq = ipu_idmac_channel_irq(dev->ipu,
						      priv->preview_ch,
						      IPU_IRQ_EOF);

	err = devm_request_irq(dev->dev, priv->eof_irq,
			       preview_eof_interrupt, 0,
			       "mx6cam-preview-eof", priv);
	if (err) {
		v4l2_err(&priv->sd,
			 "Error registering preview eof irq: %d\n", err);
		goto out_free_nfb4eof_irq;
	}

	err = ipu_csi_enable(priv->csi);
	if (err) {
		v4l2_err(&priv->sd, "CSI enable error: %d\n", err);
		goto out_free_eof_irq;
	}

	priv->preview_active = true;

	/* start the VF EOF timeout timer */
	mod_timer(&priv->eof_timeout_timer,
		  jiffies + msecs_to_jiffies(MX6CAM_EOF_TIMEOUT));

	return 0;

out_free_eof_irq:
	devm_free_irq(dev->dev, priv->eof_irq, priv);
out_free_nfb4eof_irq:
	devm_free_irq(dev->dev, priv->nfb4eof_irq, priv);
out_put_ipu:
	preview_put_ipu_resources(priv);
out_unmap_fbuf:
	unmap_fbuf(priv);
	return err;
}

static int preview_stop(struct preview_priv *priv)
{
	struct mx6cam_dev *dev = priv->dev;
	int ret;

	if (!priv->preview_active)
		return 0;

	/* stop the VF EOF timeout timer */
	del_timer_sync(&priv->eof_timeout_timer);

	/*
	 * Mark next EOF interrupt as the last before preview off,
	 * and then wait for interrupt handler to mark completion.
	 */
	init_completion(&priv->last_eof_comp);
	priv->last_eof = true;
	ret = wait_for_completion_timeout(&priv->last_eof_comp,
					  msecs_to_jiffies(MX6CAM_EOF_TIMEOUT));
	if (ret == 0)
		v4l2_warn(&priv->sd, "wait last preview EOF timeout\n");

	ipu_csi_disable(priv->csi);

	devm_free_irq(dev->dev, priv->eof_irq, priv);
	devm_free_irq(dev->dev, priv->nfb4eof_irq, priv);

	/* disable IC tasks and the channels */
	ipu_ic_task_disable(priv->ic_vf);

	ipu_idmac_disable_channel(priv->preview_ch);
	if (priv->rot_mode >= IPU_ROTATE_90_RIGHT) {
		ipu_idmac_disable_channel(priv->preview_rot_in_ch);
		ipu_idmac_disable_channel(priv->preview_rot_out_ch);
	}

	if (priv->rot_mode >= IPU_ROTATE_90_RIGHT)
		ipu_idmac_unlink(priv->preview_ch, priv->preview_rot_in_ch);

	ipu_ic_disable(priv->ic_vf);

	preview_free_dma_buf(priv, &priv->rot_buf[0]);
	preview_free_dma_buf(priv, &priv->rot_buf[1]);

	preview_put_ipu_resources(priv);

	unmap_fbuf(priv);

	priv->preview_active = false;
	return 0;
}

static int preview_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct preview_priv *priv = v4l2_get_subdevdata(sd);

	return enable ? preview_start(priv) : preview_stop(priv);
}

static void preview_unregistered(struct v4l2_subdev *sd)
{
	struct preview_priv *priv = v4l2_get_subdevdata(sd);

	v4l2_ctrl_handler_free(&priv->ctrl_hdlr);
}

/* Controls */

static int preview_set_rotation(struct preview_priv *priv,
				int rotation, bool hflip, bool vflip)
{
	struct mx6cam_dev *dev = priv->dev;
	enum ipu_rotate_mode rot_mode;
	int ret;

	ret = ipu_degrees_to_rot_mode(&rot_mode, rotation,
				      hflip, vflip);
	if (ret)
		return ret;

	priv->rotation = rotation;
	priv->hflip = hflip;
	priv->vflip = vflip;

	if (rot_mode != priv->rot_mode) {
		if (dev->preview_on)
			preview_stop(priv);
		priv->rot_mode = rot_mode;
		if (dev->preview_on)
			preview_start(priv);
	}

	return 0;
}

static int preview_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct preview_priv *priv = container_of(ctrl->handler,
						 struct preview_priv,
						 ctrl_hdlr);
	struct mx6cam_dev *dev = priv->dev;
	bool hflip, vflip;
	int rotation;

	rotation = priv->rotation;
	hflip = priv->hflip;
	vflip = priv->vflip;

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

	return preview_set_rotation(priv, rotation, hflip, vflip);
}

static const struct v4l2_ctrl_ops preview_ctrl_ops = {
	.s_ctrl = preview_s_ctrl,
};

static int preview_setup_controls(struct preview_priv *priv)
{
	struct v4l2_ctrl_handler *hdlr = &priv->ctrl_hdlr;
	int ret;

	v4l2_ctrl_handler_init(hdlr, 3);

	v4l2_ctrl_new_std(hdlr, &preview_ctrl_ops, V4L2_CID_HFLIP,
			  0, 1, 1, 0);
	v4l2_ctrl_new_std(hdlr, &preview_ctrl_ops, V4L2_CID_VFLIP,
			  0, 1, 1, 0);
	v4l2_ctrl_new_std(hdlr, &preview_ctrl_ops, V4L2_CID_ROTATE,
			  0, 270, 90, 0);

	priv->sd.ctrl_handler = hdlr;

	if (hdlr->error) {
		ret = hdlr->error;
		v4l2_ctrl_handler_free(hdlr);
		return ret;
	}

	v4l2_ctrl_handler_setup(hdlr);

	return 0;
}

static struct v4l2_subdev_core_ops preview_core_ops = {
	.g_ext_ctrls = v4l2_subdev_g_ext_ctrls,
	.try_ext_ctrls = v4l2_subdev_try_ext_ctrls,
	.s_ext_ctrls = v4l2_subdev_s_ext_ctrls,
	.g_ctrl = v4l2_subdev_g_ctrl,
	.s_ctrl = v4l2_subdev_s_ctrl,
	.queryctrl = v4l2_subdev_queryctrl,
	.querymenu = v4l2_subdev_querymenu,
};

static struct v4l2_subdev_video_ops preview_video_ops = {
	.s_stream = preview_s_stream,
};

static struct v4l2_subdev_ops preview_subdev_ops = {
	.core = &preview_core_ops,
	.video = &preview_video_ops,
};

static struct v4l2_subdev_internal_ops preview_internal_ops = {
	.unregistered = preview_unregistered,
};

struct v4l2_subdev *mx6cam_preview_init(struct mx6cam_dev *dev)
{
	struct preview_priv *priv;
	int ret;

	priv = devm_kzalloc(dev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return ERR_PTR(-ENOMEM);

	init_timer(&priv->eof_timeout_timer);
	priv->eof_timeout_timer.data = (unsigned long)priv;
	priv->eof_timeout_timer.function = preview_eof_timeout;

	v4l2_subdev_init(&priv->sd, &preview_subdev_ops);
	strlcpy(priv->sd.name, "mx6-camera-preview", sizeof(priv->sd.name));
	v4l2_set_subdevdata(&priv->sd, priv);
	priv->sd.internal_ops = &preview_internal_ops;
	priv->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;

	ret = preview_setup_controls(priv);
	if (ret) {
		kfree(priv);
		return ERR_PTR(ret);
	}

	priv->dev = dev;
	return &priv->sd;
}

/*
 * Video Camera Capture driver for Freescale i.MX6 SOC
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
#include <linux/spinlock.h>
#include <linux/platform_device.h>
#include <linux/platform_data/camera-mx6.h>
#include <linux/pinctrl/consumer.h>
#include <linux/of_platform.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/videobuf2-dma-contig.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-of.h>
#include <media/v4l2-ctrls.h>
#include <video/imx-ipu-v3.h>
#include <media/imx6.h>
#include "mx6-camif.h"

/*
 * Min/Max supported width and heights.
 */
#define MIN_W       176
#define MIN_H       144
#define MAX_W      4096
#define MAX_H      4096
#define MAX_W_IC   1024
#define MAX_H_IC   1024
#define MAX_W_VDIC  968
#define MAX_H_VDIC 2048

#define H_ALIGN    1 /* multiple of 2 */
#define S_ALIGN    1 /* multiple of 2 */

#define DEVICE_NAME "mx6-camera"

/* In bytes, per queue */
#define VID_MEM_LIMIT	SZ_64M

static struct vb2_ops mx6cam_qops;

/*
 * The Gstreamer v4l2src plugin appears to have a bug, it doesn't handle
 * frame sizes of type V4L2_FRMSIZE_TYPE_STEPWISE correctly. Set this
 * module param to get around this bug. We can remove once v4l2src handles
 * stepwise frame sizes correctly.
 */
static int v4l2src_compat = 1;
module_param(v4l2src_compat, int, 0644);
MODULE_PARM_DESC(v4l2src_compat,
		 "Gstreamer v4l2src plugin compatibility (default: 1)");

static inline struct mx6cam_dev *sd2dev(struct v4l2_subdev *sd)
{
	return container_of(sd->v4l2_dev, struct mx6cam_dev, v4l2_dev);
}

static inline struct mx6cam_ctx *file2ctx(struct file *file)
{
	return container_of(file->private_data, struct mx6cam_ctx, fh);
}

/* Supported user and sensor pixel formats */
static struct mx6cam_pixfmt mx6cam_pixformats[] = {
	{
		.name	= "RGB565",
		.fourcc	= V4L2_PIX_FMT_RGB565,
		.codes  = {V4L2_MBUS_FMT_RGB565_2X8_LE},
		.depth  = 16,
	}, {
		.name	= "RGB24",
		.fourcc	= V4L2_PIX_FMT_RGB24,
		.codes  = {V4L2_MBUS_FMT_RGB888_1X24,
			   V4L2_MBUS_FMT_RGB888_2X12_LE},
		.depth  = 24,
	}, {
		.name	= "BGR24",
		.fourcc	= V4L2_PIX_FMT_BGR24,
		.depth  = 24,
	}, {
		.name	= "RGB32",
		.fourcc	= V4L2_PIX_FMT_RGB32,
		.codes = {V4L2_MBUS_FMT_ARGB8888_1X32},
		.depth  = 32,
	}, {
		.name	= "BGR32",
		.fourcc	= V4L2_PIX_FMT_BGR32,
		.depth  = 32,
	}, {
		.name	= "4:2:2 packed, YUYV",
		.fourcc	= V4L2_PIX_FMT_YUYV,
		.codes = {V4L2_MBUS_FMT_YUYV8_2X8, V4L2_MBUS_FMT_YUYV8_1X16},
		.depth  = 16,
	}, {
		.name	= "4:2:2 packed, UYVY",
		.fourcc	= V4L2_PIX_FMT_UYVY,
		.codes = {V4L2_MBUS_FMT_UYVY8_2X8, V4L2_MBUS_FMT_UYVY8_1X16},
		.depth  = 16,
	}, {
		.name	= "4:2:0 planar, YUV",
		.fourcc	= V4L2_PIX_FMT_YUV420,
		.depth  = 12,
		.y_depth = 8,
	}, {
		.name   = "4:2:0 planar, YVU",
		.fourcc = V4L2_PIX_FMT_YVU420,
		.depth  = 12,
		.y_depth = 8,
	}, {
		.name   = "4:2:2 planar, YUV",
		.fourcc = V4L2_PIX_FMT_YUV422P,
		.depth  = 16,
		.y_depth = 8,
	}, {
		.name   = "4:2:0 planar, Y/CbCr",
		.fourcc = V4L2_PIX_FMT_NV12,
		.depth  = 12,
		.y_depth = 8,
	}, {
		.name   = "4:2:2 planar, Y/CbCr",
		.fourcc = V4L2_PIX_FMT_NV16,
		.depth  = 16,
		.y_depth = 8,
	},
};
#define NUM_FORMATS ARRAY_SIZE(mx6cam_pixformats)

static struct mx6cam_pixfmt *mx6cam_get_format(u32 fourcc, u32 code)
{
	struct mx6cam_pixfmt *fmt, *ret = NULL;
	int i, j;

	for (i = 0; i < NUM_FORMATS; i++) {
		fmt = &mx6cam_pixformats[i];

		if (fourcc && fmt->fourcc == fourcc) {
			ret = fmt;
			goto out;
		}

		for (j = 0; fmt->codes[j]; j++) {
			if (fmt->codes[j] == code) {
				ret = fmt;
				goto out;
			}
		}
	}
out:
	return ret;
}

/* Support functions */

/* find the endpoint that is handling this input index */
static struct mx6cam_endpoint *find_ep_by_input_index(struct mx6cam_dev *dev,
						      int input_idx)
{
	struct mx6cam_endpoint *ep;
	int i;

	for (i = 0; i < dev->num_eps; i++) {
		ep = &dev->eplist[i];
		if (!ep->sd)
			continue;

		if (input_idx >= ep->sensor_input.first &&
		    input_idx <= ep->sensor_input.last)
			break;
	}

	return (i < dev->num_eps) ? ep : NULL;
}

/*
 * Query sensor and update signal lock status. Returns true if lock
 * status has changed.
 */
static bool update_signal_lock_status(struct mx6cam_dev *dev)
{
	bool locked, changed;
	u32 status;
	int ret;

	ret = v4l2_subdev_call(dev->ep->sd, video, g_input_status, &status);
	if (ret)
		return false;

	locked = ((status & V4L2_IN_ST_NO_SYNC) == 0);
	changed = (dev->signal_locked != locked);
	dev->signal_locked = locked;

	return changed;
}

/*
 * Return true if the VDIC deinterlacer is needed. We need the VDIC
 * if the sensor is transmitting fields, and userland is requesting
 * motion compensation (rather than simple weaving).
 */
static bool need_vdic(struct mx6cam_dev *dev,
		      struct v4l2_mbus_framefmt *sf)
{
	return dev->motion != MOTION_NONE && V4L2_FIELD_HAS_BOTH(sf->field);
}

/*
 * Return true if sensor format currently meets the VDIC
 * restrictions:
 *     o the full-frame resolution to the VDIC must be at or below 968x2048.
 *     o the pixel format to the VDIC must be YUV422
 */
static bool can_use_vdic(struct mx6cam_dev *dev,
			 struct v4l2_mbus_framefmt *sf)
{
	return sf->width <= MAX_W_VDIC &&
		sf->height <= MAX_H_VDIC &&
		(sf->code == V4L2_MBUS_FMT_UYVY8_2X8 ||
		 sf->code == V4L2_MBUS_FMT_UYVY8_1X16 ||
		 sf->code == V4L2_MBUS_FMT_YUYV8_2X8 ||
		 sf->code == V4L2_MBUS_FMT_YUYV8_1X16);
}

/*
 * Return true if the current capture parameters require the use of
 * the Image Converter. We need the IC for scaling, colorspace conversion,
 * preview, and rotation.
 */
static bool need_ic(struct mx6cam_dev *dev,
		    struct v4l2_mbus_framefmt *sf,
		    struct v4l2_format *uf,
		    struct v4l2_rect *crop)
{
	struct v4l2_pix_format *user_fmt = &uf->fmt.pix;
	enum ipu_color_space sensor_cs, user_cs;
	bool ret;

	sensor_cs = ipu_mbus_code_to_colorspace(sf->code);
	user_cs = ipu_pixelformat_to_colorspace(user_fmt->pixelformat);

	ret = (user_fmt->width != crop->width ||
	       user_fmt->height != crop->height ||
	       user_cs != sensor_cs ||
	       dev->preview_on ||
	       dev->rot_mode != IPU_ROTATE_NONE);

	return ret;
}

/*
 * Return true if user and sensor formats currently meet the IC
 * restrictions:
 *     o the parallel CSI bus cannot be 16-bit wide.
 *     o the endpoint id must be 0 (for MIPI CSI2, the endpoint id is the
 *       virtual channel number, and only VC0 can pass through the IC).
 *     o the resizer output size must be at or below 1024x1024.
 */
static bool can_use_ic(struct mx6cam_dev *dev,
		       struct v4l2_mbus_framefmt *sf,
		       struct v4l2_format *uf)
{
	struct mx6cam_endpoint *ep = dev->ep;

	return (ep->ep.bus_type == V4L2_MBUS_CSI2 ||
		ep->ep.bus.parallel.bus_width < 16) &&
		ep->ep.base.id == 0 &&
		uf->fmt.pix.width <= MAX_W_IC &&
		uf->fmt.pix.height <= MAX_H_IC;
}

/*
 * Adjusts passed width and height to meet IC resizer limits.
 */
static void adjust_to_resizer_limits(struct mx6cam_dev *dev,
				     struct v4l2_format *uf,
				     struct v4l2_rect *crop)
{
	u32 *width, *height;

	if (uf->type == V4L2_BUF_TYPE_VIDEO_CAPTURE) {
		width = &uf->fmt.pix.width;
		height = &uf->fmt.pix.height;
	} else {
		width = &uf->fmt.win.w.width;
		height = &uf->fmt.win.w.height;
	}

	/* output of resizer can't be above 1024x1024 */
	*width = min_t(__u32, *width, MAX_W_IC);
	*height = min_t(__u32, *height, MAX_H_IC);

	/* resizer cannot downsize more than 8:1 */
	if (dev->rot_mode >= IPU_ROTATE_90_RIGHT) {
		*height = max_t(__u32, *height, crop->width / 8);
		*width = max_t(__u32, *width, crop->height / 8);
	} else {
		*width = max_t(__u32, *width, crop->width / 8);
		*height = max_t(__u32, *height, crop->height / 8);
	}
}

static void adjust_user_fmt(struct mx6cam_dev *dev,
			    struct v4l2_mbus_framefmt *sf,
			    struct v4l2_format *uf,
			    struct v4l2_rect *crop)
{
	struct mx6cam_pixfmt *fmt;

	/*
	 * Make sure resolution is within IC resizer limits
	 * if we need the Image Converter.
	 */
	if (need_ic(dev, sf, uf, crop))
		adjust_to_resizer_limits(dev, uf, crop);

	/*
	 * Force the resolution to match crop window if
	 * we can't use the Image Converter.
	 */
	if (!can_use_ic(dev, sf, uf)) {
		uf->fmt.pix.width = crop->width;
		uf->fmt.pix.height = crop->height;
	}

	fmt = mx6cam_get_format(uf->fmt.pix.pixelformat, 0);

	uf->fmt.pix.bytesperline = (uf->fmt.pix.width * fmt->depth) >> 3;
	uf->fmt.pix.sizeimage = uf->fmt.pix.height * uf->fmt.pix.bytesperline;
}

/*
 * Calculate what the default active crop window should be. Ask
 * the sensor via g_crop. This crop window will be stored to dev->crop.
 */
static void calc_default_crop(struct mx6cam_dev *dev,
			      struct v4l2_rect *rect,
			      struct v4l2_mbus_framefmt *sf)
{
	struct v4l2_crop crop;
	int ret;

	crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	ret = v4l2_subdev_call(dev->ep->sd, video, g_crop, &crop);
	if (ret) {
		/* sensor doesn't support .g_crop(), assume sensor frame */
		rect->top = rect->left = 0;
		rect->width = sf->width;
		rect->height = sf->height;
	} else
		*rect = crop.c;

	/* adjust crop window to h/w alignment restrictions */
	rect->width &= ~0x7;
	rect->left &= ~0x3;
}

static int update_sensor_std(struct mx6cam_dev *dev)
{
	struct mx6cam_sensor_input *epinput;
	struct mx6cam_endpoint *ep;
	int sensor_input;

	ep = find_ep_by_input_index(dev, dev->current_input);
	if (!ep)
		return -EINVAL;

	epinput = &ep->sensor_input;
	sensor_input = dev->current_input - epinput->first;

	if (epinput->caps[sensor_input] == V4L2_IN_CAP_DV_TIMINGS) {
		v4l2_err(&dev->v4l2_dev, "Update STD on input %s : DV Timings only\n", epinput->name[sensor_input]);
		return -ENODATA;
	}
	v4l2_err(&dev->v4l2_dev, "Update STD on input %s\n", epinput->name[sensor_input]);

	return v4l2_subdev_call(dev->ep->sd, video, querystd,
				&dev->current_std);
}

static int update_sensor_fmt(struct mx6cam_dev *dev)
{
	int ret;

	ret = v4l2_subdev_call(dev->ep->sd, video, g_mbus_fmt,
			       &dev->sensor_fmt);
	if (ret)
		return ret;

	ret = v4l2_subdev_call(dev->ep->sd, video, g_mbus_config,
			       &dev->mbus_cfg);
	if (ret)
		return ret;

	dev->sensor_pixfmt = mx6cam_get_format(0, dev->sensor_fmt.code);

	/* update sensor crop bounds */
	dev->crop_bounds.top = dev->crop_bounds.left = 0;
	dev->crop_bounds.width = dev->sensor_fmt.width;
	dev->crop_bounds.height = dev->sensor_fmt.height;
	dev->crop_defrect = dev->crop_bounds;

	return 0;
}

/*
 * Turn current sensor power on/off according to power_count.
 */
static int sensor_set_power(struct mx6cam_dev *dev, int on)
{
	struct mx6cam_endpoint *ep = dev->ep;
	struct v4l2_subdev *sd = ep->sd;
	int ret;

	if (on && ep->power_count++ > 0)
		return 0;
	else if (!on && (ep->power_count == 0 || --ep->power_count > 0))
		return 0;

	ret = v4l2_subdev_call(sd, core, s_power, on);
	return ret != -ENOIOCTLCMD ? ret : 0;
}

/*
 * Turn current sensor streaming on/off according to stream_count.
 */
static int sensor_set_stream(struct mx6cam_dev *dev, int on)
{
	struct mx6cam_endpoint *ep = dev->ep;
	struct v4l2_subdev *sd = ep->sd;
	int ret;

	if (on && ep->stream_count++ > 0)
		return 0;
	else if (!on && (ep->stream_count == 0 || --ep->stream_count > 0))
		return 0;

	ret = v4l2_subdev_call(sd, video, s_stream, on);
	return ret != -ENOIOCTLCMD ? ret : 0;
}

/*
 * Start the encoder for buffer streaming. There must be at least two
 * frames in the vb2 queue.
 */
static int start_encoder(struct mx6cam_dev *dev)
{
	struct v4l2_subdev *streaming_sd;
	int ret;

	if (dev->encoder_on)
		return 0;

	/* sensor stream on */
	ret = sensor_set_stream(dev, 1);
	if (ret) {
		v4l2_err(&dev->v4l2_dev, "sensor stream on failed\n");
		return ret;
	}

	/* encoder/vdic stream on */
	streaming_sd = dev->using_vdic ? dev->vdic_sd : dev->encoder_sd;

	ret = v4l2_subdev_call(streaming_sd, video, s_stream, 1);
	if (ret) {
		v4l2_err(&dev->v4l2_dev, "encoder stream on failed\n");
		return ret;
	}

	dev->encoder_on = true;
	return 0;
}

/*
 * Stop the encoder.
 */
static int stop_encoder(struct mx6cam_dev *dev)
{
	struct v4l2_subdev *streaming_sd;
	int ret;

	if (!dev->encoder_on)
		return 0;

	streaming_sd = dev->using_vdic ? dev->vdic_sd : dev->encoder_sd;

	/* encoder/vdic off */
	ret = v4l2_subdev_call(streaming_sd, video, s_stream, 0);
	if (ret)
		v4l2_err(&dev->v4l2_dev, "encoder stream off failed\n");

	/* sensor stream off */
	ret = sensor_set_stream(dev, 0);
	if (ret)
		v4l2_err(&dev->v4l2_dev, "sensor stream off failed\n");

	dev->encoder_on = false;
	return ret;
}

/*
 * Start preview.
 */
static int start_preview(struct mx6cam_dev *dev)
{
	int ret;

	if (atomic_read(&dev->status_change)) {
		update_signal_lock_status(dev);
		update_sensor_std(dev);
		update_sensor_fmt(dev);
		/* reset active crop window */
		calc_default_crop(dev, &dev->crop, &dev->sensor_fmt);
		atomic_set(&dev->status_change, 0);
		v4l2_info(&dev->v4l2_dev, "at preview on: %s, %s\n",
			  v4l2_norm_to_name(dev->current_std),
			  dev->signal_locked ? "signal locked" : "no signal");
	}

	/* sensor stream on */
	ret = sensor_set_stream(dev, 1);
	if (ret) {
		v4l2_err(&dev->v4l2_dev, "sensor stream on failed\n");
		return ret;
	}

	/* preview stream on */
	ret = v4l2_subdev_call(dev->preview_sd, video, s_stream, 1);
	if (ret)
		v4l2_err(&dev->v4l2_dev, "preview stream on failed\n");

	return ret;
}

/*
 * Stop preview.
 */
static int stop_preview(struct mx6cam_dev *dev)
{
	int ret;

	/* preview stream off */
	ret = v4l2_subdev_call(dev->preview_sd, video, s_stream, 0);
	if (ret) {
		v4l2_err(&dev->v4l2_dev, "preview stream off failed\n");
		return ret;
	}

	/* sensor stream off */
	ret = sensor_set_stream(dev, 0);
	if (ret)
		v4l2_err(&dev->v4l2_dev, "sensor stream off failed\n");

	return ret;
}

/*
 * Start/Stop streaming.
 */
static int set_stream(struct mx6cam_ctx *ctx, bool on)
{
	struct mx6cam_dev *dev = ctx->dev;
	int ret = 0;

	if (on) {
		if (atomic_read(&dev->status_change)) {
			update_signal_lock_status(dev);
			update_sensor_std(dev);
			update_sensor_fmt(dev);
			/* reset active crop window */
			calc_default_crop(dev, &dev->crop, &dev->sensor_fmt);
			atomic_set(&dev->status_change, 0);
			v4l2_info(&dev->v4l2_dev, "at stream on: %s, %s\n",
				  v4l2_norm_to_name(dev->current_std),
				  dev->signal_locked ?
				  "signal locked" : "no signal");
		}

		dev->using_ic =
			(need_ic(dev, &dev->sensor_fmt, &dev->user_fmt,
				 &dev->crop) &&
			 can_use_ic(dev, &dev->sensor_fmt, &dev->user_fmt));

		dev->using_vdic = need_vdic(dev, &dev->sensor_fmt) &&
			can_use_vdic(dev, &dev->sensor_fmt);

		if (dev->preview_on)
			stop_preview(dev);

		/*
		 * If there are two or more frames in the queue, we can start
		 * the encoder now. Otherwise the encoding will start once
		 * two frames have been queued.
		 */
		if (!list_empty(&ctx->ready_q) &&
		    !list_is_singular(&ctx->ready_q))
			ret = start_encoder(dev);

		if (dev->preview_on)
			start_preview(dev);
	} else {
		ret = stop_encoder(dev);
	}

	return ret;
}

/*
 * Restart work handler. This is called in three cases during active
 * streaming and/or preview:
 *
 * o NFB4EOF errors
 * o A decoder's signal lock status or autodetected video standard changes
 * o End-of-Frame timeouts
 */
static void restart_work_handler(struct work_struct *w)
{
	struct mx6cam_ctx *ctx = container_of(w, struct mx6cam_ctx,
					      restart_work);
	struct mx6cam_dev *dev = ctx->dev;

	mutex_lock(&dev->mutex);

	if (!vb2_is_streaming(&dev->buffer_queue)) {
		/* just restart preview if on */
		if (dev->preview_on) {
			v4l2_warn(&dev->v4l2_dev, "restarting preview\n");
			stop_preview(dev);
			start_preview(dev);
		}
		goto out_unlock;
	}

	v4l2_warn(&dev->v4l2_dev, "restarting\n");

	set_stream(ctx, false);
	set_stream(ctx, true);

out_unlock:
	mutex_unlock(&dev->mutex);
}

/*
 * Stop work handler. Not currently needed but keep around.
 */
static void stop_work_handler(struct work_struct *w)
{
	struct mx6cam_ctx *ctx = container_of(w, struct mx6cam_ctx,
					      stop_work);
	struct mx6cam_dev *dev = ctx->dev;

	mutex_lock(&dev->mutex);

	if (dev->preview_on) {
		v4l2_err(&dev->v4l2_dev, "stopping preview\n");
		stop_preview(dev);
		dev->preview_on = false;
	}

	if (vb2_is_streaming(&dev->buffer_queue)) {
		v4l2_err(&dev->v4l2_dev, "stopping\n");
		vb2_streamoff(&dev->buffer_queue, V4L2_BUF_TYPE_VIDEO_CAPTURE);
	}

	mutex_unlock(&dev->mutex);
}

/*
 * Restart timer function. Schedules a restart.
 */
static void mx6cam_restart_timeout(unsigned long data)
{
	struct mx6cam_ctx *ctx = (struct mx6cam_ctx *)data;

	schedule_work(&ctx->restart_work);
}

/* Controls */
static int mx6cam_set_rotation(struct mx6cam_dev *dev,
			       int rotation, bool hflip, bool vflip)
{
	enum ipu_rotate_mode rot_mode;
	int ret;

	ret = ipu_degrees_to_rot_mode(&rot_mode, rotation,
				      hflip, vflip);
	if (ret)
		return ret;

	if (rot_mode != dev->rot_mode) {
		/* can't change rotation mid-streaming */
		if (vb2_is_streaming(&dev->buffer_queue)) {
			v4l2_err(&dev->v4l2_dev,
				 "%s: not allowed while streaming\n",
				 __func__);
			return -EBUSY;
		}

		if (rot_mode != IPU_ROTATE_NONE &&
		    !can_use_ic(dev, &dev->sensor_fmt, &dev->user_fmt)) {
			v4l2_err(&dev->v4l2_dev,
				"%s: current format does not allow rotation\n",
				 __func__);
			return -EINVAL;
		}
	}

	dev->rot_mode = rot_mode;
	dev->rotation = rotation;
	dev->hflip = hflip;
	dev->vflip = vflip;

	return 0;
}

static int mx6cam_set_motion(struct mx6cam_dev *dev,
			     enum ipu_motion_sel motion)
{
	if (motion != dev->motion) {
		/* can't change motion setting mid-streaming */
		if (vb2_is_streaming(&dev->buffer_queue)) {
			v4l2_err(&dev->v4l2_dev,
				 "%s: not allowed while streaming\n",
				 __func__);
			return -EBUSY;
		}

		if (motion != MOTION_NONE && dev->preview_on) {
			v4l2_err(&dev->v4l2_dev,
				 "Preview is on, cannot enable deinterlace\n");
			return -EBUSY;
		}

		if (motion != MOTION_NONE &&
		    !can_use_vdic(dev, &dev->sensor_fmt)) {
			v4l2_err(&dev->v4l2_dev,
				 "sensor format does not allow deinterlace\n");
			return -EINVAL;
		}
	}

	dev->motion = motion;
	return 0;
}

static int mx6cam_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct mx6cam_dev *dev = container_of(ctrl->handler,
					      struct mx6cam_dev, ctrl_hdlr);
	enum ipu_motion_sel motion;
	bool hflip, vflip;
	int rotation;

	rotation = dev->rotation;
	hflip = dev->hflip;
	vflip = dev->vflip;

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
	case V4L2_CID_IMX6_MOTION:
		motion = ctrl->val;
		return mx6cam_set_motion(dev, motion);
	default:
		v4l2_err(&dev->v4l2_dev, "Invalid control\n");
		return -ERANGE;
	}

	return mx6cam_set_rotation(dev, rotation, hflip, vflip);
}

static const struct v4l2_ctrl_ops mx6cam_ctrl_ops = {
	.s_ctrl = mx6cam_s_ctrl,
};

static const struct v4l2_ctrl_config motion_cfg = {
	.ops = &mx6cam_ctrl_ops,
	.id = V4L2_CID_IMX6_MOTION,
	.name = "Motion Compensation",
	.type = V4L2_CTRL_TYPE_INTEGER,
	.def = MOTION_NONE,
	.min = MOTION_NONE,
	.max = HIGH_MOTION,
	.step = 1,
};

static int mx6cam_init_controls(struct mx6cam_dev *dev)
{
	struct v4l2_ctrl_handler *hdlr = &dev->ctrl_hdlr;
	int ret;

	v4l2_ctrl_handler_init(hdlr, 4);

	v4l2_ctrl_new_std(hdlr, &mx6cam_ctrl_ops, V4L2_CID_HFLIP,
			  0, 1, 1, 0);
	v4l2_ctrl_new_std(hdlr, &mx6cam_ctrl_ops, V4L2_CID_VFLIP,
			  0, 1, 1, 0);
	v4l2_ctrl_new_std(hdlr, &mx6cam_ctrl_ops, V4L2_CID_ROTATE,
			  0, 270, 90, 0);
	v4l2_ctrl_new_custom(hdlr, &motion_cfg, NULL);

	if (hdlr->error) {
		ret = hdlr->error;
		v4l2_ctrl_handler_free(hdlr);
		return ret;
	}

	dev->v4l2_dev.ctrl_handler = hdlr;
	dev->vfd->ctrl_handler = hdlr;

	v4l2_ctrl_handler_setup(hdlr);

	return 0;
}


/*
 * Video ioctls follow
 */

static int vidioc_querycap(struct file *file, void *priv,
			   struct v4l2_capability *cap)
{
	strlcpy(cap->driver, DEVICE_NAME, sizeof(cap->driver));
	strlcpy(cap->card, DEVICE_NAME, sizeof(cap->card));
	strlcpy(cap->bus_info, "platform:mx6-camera", sizeof(cap->bus_info));
	cap->device_caps = V4L2_CAP_STREAMING | V4L2_CAP_VIDEO_CAPTURE |
				V4L2_CAP_VIDEO_OVERLAY;
	cap->capabilities = cap->device_caps | V4L2_CAP_DEVICE_CAPS;

	return 0;
}

static int vidioc_enum_fmt_vid_cap(struct file *file, void *priv,
				   struct v4l2_fmtdesc *f)
{
	struct mx6cam_pixfmt *fmt;

	if (f->index >= NUM_FORMATS)
		return -EINVAL;

	fmt = &mx6cam_pixformats[f->index];
	strncpy(f->description, fmt->name, sizeof(f->description) - 1);
	f->pixelformat = fmt->fourcc;
	return 0;
}

static int vidioc_enum_fmt_vid_overlay(struct file *file, void *priv,
				       struct v4l2_fmtdesc *f)
{
	return vidioc_enum_fmt_vid_cap(file, priv, f);
}

static int vidioc_g_fmt_vid_cap(struct file *file, void *priv,
				struct v4l2_format *f)
{
	struct mx6cam_ctx *ctx = file2ctx(file);
	struct mx6cam_dev *dev = ctx->dev;
	struct v4l2_pix_format *pix = &f->fmt.pix;
	struct v4l2_subdev_format sd_fmt;
	struct mx6cam_pixfmt *fmt;
	struct v4l2_rect crop;

	/* TODO: This should be dynamic */
	sd_fmt.pad = 1;
	sd_fmt.which = V4L2_SUBDEV_FORMAT_ACTIVE;
	v4l2_subdev_call(dev->ep->sd, pad, get_fmt, NULL, &sd_fmt);
	v4l2_fill_pix_format(pix, &sd_fmt.format);

	fmt = mx6cam_get_format(0, sd_fmt.format.code);
	if (!fmt) {
		v4l2_err(&dev->v4l2_dev,
			 "Fourcc format (0x%08x) invalid.\n",
			 f->fmt.pix.pixelformat);
		return -EINVAL;
	}
	pix->pixelformat = fmt->fourcc;

	pix->bytesperline = (pix->width * fmt->depth) >> 3;
	pix->sizeimage = pix->height * pix->bytesperline;

	return 0;
}

static int vidioc_g_fmt_vid_overlay(struct file *file, void *priv,
				    struct v4l2_format *f)
{
	struct mx6cam_ctx *ctx = file2ctx(file);
	struct mx6cam_dev *dev = ctx->dev;

	f->fmt.win = dev->win;
	return 0;
}

static int vidioc_try_fmt_vid_cap(struct file *file, void *priv,
				  struct v4l2_format *f)
{
	struct mx6cam_ctx *ctx = file2ctx(file);
	struct mx6cam_dev *dev = ctx->dev;
	struct v4l2_mbus_framefmt mbus_fmt;
	struct mx6cam_pixfmt *fmt;
	struct v4l2_pix_format *pix = &f->fmt.pix;
	struct v4l2_subdev_format sd_fmt;
	unsigned int width_align;
	struct v4l2_rect crop;
	int ret;

	fmt = mx6cam_get_format(f->fmt.pix.pixelformat, 0);
	if (!fmt) {
		v4l2_err(&dev->v4l2_dev,
			 "Fourcc format (0x%08x) invalid.\n",
			 f->fmt.pix.pixelformat);
		return -EINVAL;
	}

	/*
	 * We have to adjust the width such that the physaddrs and U and
	 * U and V plane offsets are multiples of 8 bytes as required by
	 * the IPU DMA Controller. For the planar formats, this corresponds
	 * to a pixel alignment of 16. For all the packed formats, 8 is
	 * good enough.
	 */
	width_align = ipu_pixelformat_is_planar(fmt->fourcc) ? 4 : 3;

	v4l_bound_align_image(&f->fmt.pix.width, MIN_W, MAX_W,
			      width_align, &f->fmt.pix.height,
			      MIN_H, MAX_H, H_ALIGN, S_ALIGN);

#if 0
	v4l2_fill_mbus_format(&mbus_fmt, &f->fmt.pix, 0);

	ret = v4l2_subdev_call(dev->ep->sd, video, try_mbus_fmt, &mbus_fmt);
	if (ret)
		return ret;

	fmt = mx6cam_get_format(0, mbus_fmt.code);
	if (!fmt) {
		v4l2_err(&dev->v4l2_dev,
			 "Sensor mbus format (0x%08x) invalid.\n",
			 mbus_fmt.code);
		return -EINVAL;
	}

#else
	sd_fmt.pad = 1; /* TODO: Modify this later */
	sd_fmt.which = V4L2_SUBDEV_FORMAT_ACTIVE;
	v4l2_subdev_call(dev->ep->sd, pad, get_fmt, NULL, &sd_fmt);
	v4l2_fill_pix_format(pix, &sd_fmt.format);
#endif
	/*
	 * calculate what the optimal crop window will be for this
	 * sensor format and make any user format adjustments.
	 */
	calc_default_crop(dev, &crop, &sd_fmt.format);
	adjust_user_fmt(dev, &sd_fmt.format, f, &crop);
	pix->field = V4L2_FIELD_NONE;
	pix->priv = 0;

	return 0;
}

static int vidioc_try_fmt_vid_overlay(struct file *file, void *priv,
				      struct v4l2_format *f)
{
	struct mx6cam_ctx *ctx = file2ctx(file);
	struct mx6cam_dev *dev = ctx->dev;
	struct v4l2_window *win = &f->fmt.win;
	unsigned int width_align;

	width_align = ipu_pixelformat_is_planar(dev->fbuf.fmt.pixelformat) ?
		4 : 3;

	v4l_bound_align_image(&win->w.width, MIN_W, MAX_W_IC,
			      width_align, &win->w.height,
			      MIN_H, MAX_H_IC, H_ALIGN, S_ALIGN);

	adjust_to_resizer_limits(dev, f, &dev->crop);

	return 0;
}

static int vidioc_s_fmt_vid_cap(struct file *file, void *priv,
				struct v4l2_format *f)
{
	struct mx6cam_ctx *ctx = file2ctx(file);
	struct mx6cam_dev *dev = ctx->dev;
	struct v4l2_mbus_framefmt mbus_fmt;
	struct v4l2_pix_format *pix = &f->fmt.pix;
	struct v4l2_subdev_format sd_fmt;
	int ret;

	if (vb2_is_busy(&dev->buffer_queue)) {
		v4l2_err(&dev->v4l2_dev, "%s queue busy\n", __func__);
		return -EBUSY;
	}

	ret = vidioc_try_fmt_vid_cap(file, priv, f);
	if (ret)
		return ret;
#if 0
	v4l2_fill_mbus_format(&mbus_fmt, &f->fmt.pix, 0);
	ret = v4l2_subdev_call(dev->ep->sd, video, s_mbus_fmt, &mbus_fmt);
	if (ret) {
		v4l2_err(&dev->v4l2_dev, "%s s_mbus_fmt failed\n", __func__);
		return ret;
	}
	ret = update_sensor_fmt(dev);
	if (ret)
		return ret;

#else
	sd_fmt.pad = 1; /* TODO: Modify this later */
	sd_fmt.which = V4L2_SUBDEV_FORMAT_ACTIVE;
	sd_fmt.format.code = V4L2_MBUS_FMT_YUYV8_1X16;

	ret = v4l2_subdev_call(dev->ep->sd, pad, set_fmt, NULL, &sd_fmt);
	if (ret) {
		v4l2_err(&dev->v4l2_dev, "%s set_fmt failed\n", __func__);
		return ret;
	}
#endif
	/* reset active crop window */
	calc_default_crop(dev, &dev->crop, &dev->sensor_fmt);

	dev->user_fmt = *f;
	dev->user_pixfmt = mx6cam_get_format(f->fmt.pix.pixelformat, 0);

	return 0;
}

static int vidioc_enum_framesizes(struct file *file, void *priv,
				  struct v4l2_frmsizeenum *fsize)
{
	struct mx6cam_ctx *ctx = file2ctx(file);
	struct mx6cam_dev *dev = ctx->dev;
	struct mx6cam_pixfmt *fmt;
	struct v4l2_format uf;

	fmt = mx6cam_get_format(fsize->pixel_format, 0);
	if (!fmt)
		return -EINVAL;

	if (v4l2src_compat)
		return v4l2_subdev_call(dev->ep->sd, video,
					enum_framesizes, fsize);

	if (fsize->index)
		return -EINVAL;

	fsize->type = V4L2_FRMSIZE_TYPE_STEPWISE;
	fsize->stepwise.min_width = MIN_W;
	fsize->stepwise.step_width =
		ipu_pixelformat_is_planar(fmt->fourcc) ? 16 : 8;
	fsize->stepwise.min_height = MIN_H;
	fsize->stepwise.step_height = 1 << H_ALIGN;

	uf = dev->user_fmt;
	uf.fmt.pix.pixelformat = fmt->fourcc;

	if (need_ic(dev, &dev->sensor_fmt, &uf, &dev->crop)) {
		fsize->stepwise.max_width = MAX_W_IC;
		fsize->stepwise.max_height = MAX_H_IC;
	} else {
		fsize->stepwise.max_width = MAX_W;
		fsize->stepwise.max_height = MAX_H;
	}

	return 0;
}

static int vidioc_enum_frameintervals(struct file *file, void *priv,
				      struct v4l2_frmivalenum *fival)
{
	struct mx6cam_ctx *ctx = file2ctx(file);
	struct mx6cam_dev *dev = ctx->dev;
	struct mx6cam_pixfmt *fmt;

	fmt = mx6cam_get_format(fival->pixel_format, 0);
	if (!fmt)
		return -EINVAL;

	return v4l2_subdev_call(dev->ep->sd, video, enum_frameintervals, fival);
}

static int vidioc_s_fmt_vid_overlay(struct file *file, void *priv,
				    struct v4l2_format *f)
{
	struct mx6cam_ctx *ctx = file2ctx(file);
	struct mx6cam_dev *dev = ctx->dev;
	struct v4l2_window *win = &f->fmt.win;
	int ret;

	ret = vidioc_try_fmt_vid_overlay(file, priv, f);
	if (ret)
		return ret;

	if (dev->preview_on)
		stop_preview(dev);

	dev->win = *win;

	if (dev->preview_on)
		start_preview(dev);

	return 0;
}

static int vidioc_querystd(struct file *file, void *priv, v4l2_std_id *std)
{
	struct mx6cam_ctx *ctx = file2ctx(file);
	struct mx6cam_dev *dev = ctx->dev;
	struct mx6cam_sensor_input *epinput;
	struct mx6cam_endpoint *ep;
	int sensor_input;

	ep = find_ep_by_input_index(dev, dev->current_input);
	if (!ep)
		return -EINVAL;

	epinput = &ep->sensor_input;
	sensor_input = dev->current_input - epinput->first;

	if (epinput->caps[sensor_input] == V4L2_IN_CAP_DV_TIMINGS) {
		v4l2_err(&dev->v4l2_dev, "Query STD on input %s : DV Timings only\n", epinput->name[sensor_input]);
		return -ENODATA;
	}

	v4l2_err(&dev->v4l2_dev, "Query STD on input %s\n", epinput->name[sensor_input]);
	return update_sensor_std(dev);
}

static int vidioc_g_std(struct file *file, void *priv, v4l2_std_id *std)
{
	struct mx6cam_ctx *ctx = file2ctx(file);
	struct mx6cam_dev *dev = ctx->dev;
	struct mx6cam_sensor_input *epinput;
	struct mx6cam_endpoint *ep;
	int ret;
	int sensor_input;

	ep = find_ep_by_input_index(dev, dev->current_input);
	if (!ep)
		return -EINVAL;

	epinput = &ep->sensor_input;
	sensor_input = dev->current_input - epinput->first;

	if (epinput->caps[sensor_input] == V4L2_IN_CAP_DV_TIMINGS) {
		v4l2_err(&dev->v4l2_dev, "Get STD on input %s : DV Timings only\n", epinput->name[sensor_input]);
		return -ENODATA;
	}

	v4l2_err(&dev->v4l2_dev, "Get STD on input %s\n", epinput->name[sensor_input]);
	ret = v4l2_subdev_call(dev->ep->sd, video, g_std, std);
	if (ret < 0)
		return ret;

	*std = dev->current_std;
	return 0;
}

static int vidioc_s_std(struct file *file, void *priv, v4l2_std_id std)
{
	struct mx6cam_ctx *ctx = file2ctx(file);
	struct mx6cam_dev *dev = ctx->dev;
	struct mx6cam_sensor_input *epinput;
	struct mx6cam_endpoint *ep;
	int ret;
	int sensor_input;

	ep = find_ep_by_input_index(dev, dev->current_input);
	if (!ep)
		return -EINVAL;

	epinput = &ep->sensor_input;
	sensor_input = dev->current_input - epinput->first;

	if (epinput->caps[sensor_input] == V4L2_IN_CAP_DV_TIMINGS) {
		v4l2_err(&dev->v4l2_dev, "Set STD on input %s : DV Timings only\n", epinput->name[sensor_input]);
		return -ENODATA;
	}
	v4l2_err(&dev->v4l2_dev, "Set STD on input %s\n", epinput->name[sensor_input]);

	if (vb2_is_busy(&dev->buffer_queue))
		return -EBUSY;

	ret = v4l2_subdev_call(dev->ep->sd, video, s_std, std);
	if (ret < 0)
		return ret;

	dev->current_std = std;
	return 0;
}

static int vidioc_enum_input(struct file *file, void *priv,
			     struct v4l2_input *input)
{
	struct mx6cam_ctx *ctx = file2ctx(file);
	struct mx6cam_dev *dev = ctx->dev;
	struct mx6cam_sensor_input *epinput;
	struct mx6cam_endpoint *ep;
	int sensor_input;

	/* find the endpoint that is handling this input */
	ep = find_ep_by_input_index(dev, input->index);
	if (!ep)
		return -EINVAL;

	epinput = &ep->sensor_input;
	sensor_input = input->index - epinput->first;

	input->type = V4L2_INPUT_TYPE_CAMERA;
	/* TODO: Modify it to make it dynamic */
	input->capabilities = V4L2_IN_CAP_DV_TIMINGS;
	epinput->caps[sensor_input] = input->capabilities;
	strncpy(input->name, epinput->name[sensor_input], sizeof(input->name));

	if (input->index == dev->current_input) {
		v4l2_subdev_call(ep->sd, video, g_input_status, &input->status);
/*		update_sensor_std(dev);
		input->std = dev->current_std;*/
	} else {
		input->status = V4L2_IN_ST_NO_SIGNAL;
		input->std = V4L2_STD_UNKNOWN;
	}

	return 0;
}

static int vidioc_g_input(struct file *file, void *priv, unsigned int *index)
{
	struct mx6cam_ctx *ctx = file2ctx(file);
	struct mx6cam_dev *dev = ctx->dev;

	*index = dev->current_input;
	return 0;
}

static int vidioc_s_input(struct file *file, void *priv, unsigned int index)
{
	struct mx6cam_ctx *ctx = file2ctx(file);
	struct mx6cam_dev *dev = ctx->dev;
	struct mx6cam_sensor_input *epinput;
	struct mx6cam_endpoint *ep;
	int ret, sensor_input;

	if (index == dev->current_input)
		return 0;

	/* find the endpoint that is handling this input */
	ep = find_ep_by_input_index(dev, index);
	if (!ep)
		return -EINVAL;

	if (dev->ep != ep) {
		/*
		 * don't allow switching sensors if there are queued buffers,
		 * preview is on, or there are other users of the current
		 * sensor besides us.
		 */
		if (vb2_is_busy(&dev->buffer_queue) || dev->preview_on ||
		    dev->ep->power_count > 1)
			return -EBUSY;

		v4l2_info(&dev->v4l2_dev, "switching to sensor %s\n",
			  ep->sd->name);

		/* power down current sensor before enabling new one */
		ret = sensor_set_power(dev, 0);
		if (ret)
			v4l2_warn(&dev->v4l2_dev, "sensor power off failed\n");

		/* set new endpoint */
		dev->ep = ep;

		/* power-on the new sensor */
		ret = sensor_set_power(dev, 1);
		if (ret)
			v4l2_warn(&dev->v4l2_dev, "sensor power on failed\n");

		/* power-on the csi2 receiver */
		if (dev->ep->ep.bus_type == V4L2_MBUS_CSI2 && dev->csi2_sd) {
			ret = v4l2_subdev_call(dev->csi2_sd, core, s_power,
					       true);
			if (ret)
				v4l2_err(&dev->v4l2_dev,
					 "csi2 power on failed\n");
		}
	}

	/* finally select the sensor's input */
	epinput = &ep->sensor_input;
	sensor_input = index - epinput->first;
	dev->vfd->tvnorms = 0;
/*
 * TODO: Make it dynamic
 *	if (epinput->caps[sensor_input] != V4L2_IN_CAP_DV_TIMINGS)
 *		dev->vfd->tvnorms = V4L2_STD_ALL;
 */
	ret = v4l2_subdev_call(dev->ep->sd, video, s_routing,
			       epinput->value[sensor_input], 0, 0);

	dev->current_input = index;

	/*
	 * sometimes on switching video input on video decoder devices
	 * no lock status change event is generated, but vertical sync
	 * is messed up nevertheless. So schedule a restart to correct it.
	 */
	if (ctx->io_allowed)
		mod_timer(&ctx->restart_timer,
			  jiffies + msecs_to_jiffies(MX6CAM_RESTART_DELAY));

	return 0;
}

static int vidioc_g_parm(struct file *file, void *fh,
			 struct v4l2_streamparm *a)
{
	struct mx6cam_ctx *ctx = file2ctx(file);
	struct mx6cam_dev *dev = ctx->dev;

	if (a->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	return v4l2_subdev_call(dev->ep->sd, video, g_parm, a);
}

static int vidioc_s_parm(struct file *file, void *fh,
			 struct v4l2_streamparm *a)
{
	struct mx6cam_ctx *ctx = file2ctx(file);
	struct mx6cam_dev *dev = ctx->dev;

	if (a->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	return v4l2_subdev_call(dev->ep->sd, video, s_parm, a);
}

static int vidioc_cropcap(struct file *file, void *priv,
			  struct v4l2_cropcap *cropcap)
{
	struct mx6cam_ctx *ctx = file2ctx(file);
	struct mx6cam_dev *dev = ctx->dev;

	if (cropcap->type != V4L2_BUF_TYPE_VIDEO_CAPTURE &&
	    cropcap->type != V4L2_BUF_TYPE_VIDEO_OVERLAY)
		return -EINVAL;

	cropcap->bounds = dev->crop_bounds;
	cropcap->defrect = dev->crop_defrect;
	cropcap->pixelaspect.numerator = 1;
	cropcap->pixelaspect.denominator = 1;
	return 0;
}

static int vidioc_g_crop(struct file *file, void *priv,
			 struct v4l2_crop *crop)
{
	struct mx6cam_ctx *ctx = file2ctx(file);
	struct mx6cam_dev *dev = ctx->dev;

	if (crop->type != V4L2_BUF_TYPE_VIDEO_CAPTURE &&
	    crop->type != V4L2_BUF_TYPE_VIDEO_OVERLAY)
		return -EINVAL;

	crop->c = dev->crop;
	return 0;
}

static int vidioc_s_crop(struct file *file, void *priv,
			 const struct v4l2_crop *crop)
{
	struct mx6cam_ctx *ctx = file2ctx(file);
	struct mx6cam_dev *dev = ctx->dev;
	struct v4l2_rect *bounds = &dev->crop_bounds;

	if (crop->type != V4L2_BUF_TYPE_VIDEO_CAPTURE &&
	    crop->type != V4L2_BUF_TYPE_VIDEO_OVERLAY)
		return -EINVAL;

	if (vb2_is_busy(&dev->buffer_queue))
		return -EBUSY;

	/* make sure crop window is within bounds */
	if (crop->c.top < 0 || crop->c.left < 0 ||
	    crop->c.left + crop->c.width > bounds->width ||
	    crop->c.top + crop->c.height > bounds->height)
		return -EINVAL;

	/*
	 * FIXME: the IPU currently does not setup the CCIR code
	 * registers properly to handle arbitrary vertical crop
	 * windows. So return error if the sensor bus is BT.656
	 * and user is asking to change vertical cropping.
	 */
	if (dev->ep->ep.bus_type == V4L2_MBUS_BT656 &&
	    (crop->c.top != dev->crop.top ||
	     crop->c.height != dev->crop.height)) {
		v4l2_err(&dev->v4l2_dev,
			 "vertical crop is not supported for this sensor!\n");
		return -EINVAL;
	}

	dev->crop = crop->c;

	/* adjust crop window to h/w alignment restrictions */
	dev->crop.width &= ~0x7;
	dev->crop.left &= ~0x3;

	/*
	 * Crop window has changed, we need to adjust the user
	 * width/height to meet new IC resizer restrictions or to
	 * match the new crop window if the IC can't be used.
	 */
	adjust_user_fmt(dev, &dev->sensor_fmt, &dev->user_fmt,
			&dev->crop);

	return 0;
}

static int vidioc_s_fbuf(struct file *file, void *priv,
			 const struct v4l2_framebuffer *fbuf)
{
	struct mx6cam_ctx *ctx = file2ctx(file);
	struct mx6cam_dev *dev = ctx->dev;
	struct mx6cam_pixfmt *fmt;

	if (fbuf->flags != V4L2_FBUF_FLAG_OVERLAY || !fbuf->base)
		return -EINVAL;

	fmt = mx6cam_get_format(fbuf->fmt.pixelformat, 0);
	if (!fmt) {
		v4l2_err(&dev->v4l2_dev,
			 "Fourcc format (0x%08x) invalid.\n",
			 fbuf->fmt.pixelformat);
		return -EINVAL;
	}

	if (dev->preview_on)
		stop_preview(dev);

	dev->fbuf = *fbuf;
	dev->fbuf_pixfmt = fmt;

	if (dev->preview_on)
		start_preview(dev);

	return 0;
}

static int vidioc_g_fbuf(struct file *file, void *priv,
			 struct v4l2_framebuffer *fbuf)
{
	struct mx6cam_ctx *ctx = file2ctx(file);
	struct mx6cam_dev *dev = ctx->dev;

	*fbuf = dev->fbuf;

	return 0;
}

static int vidioc_reqbufs(struct file *file, void *priv,
			  struct v4l2_requestbuffers *reqbufs)
{
	struct mx6cam_ctx *ctx = file2ctx(file);
	struct mx6cam_dev *dev = ctx->dev;
	struct vb2_queue *vq = &dev->buffer_queue;
	int ret;

	if (vb2_is_busy(vq))
		return -EBUSY;

	ctx->alloc_ctx = vb2_dma_contig_init_ctx(dev->dev);
	if (IS_ERR(ctx->alloc_ctx)) {
		v4l2_err(&dev->v4l2_dev, "failed to alloc vb2 context\n");
		return PTR_ERR(ctx->alloc_ctx);
	}

	INIT_LIST_HEAD(&ctx->ready_q);
	INIT_WORK(&ctx->restart_work, restart_work_handler);
	INIT_WORK(&ctx->stop_work, stop_work_handler);
	init_timer(&ctx->restart_timer);
	ctx->restart_timer.data = (unsigned long)ctx;
	ctx->restart_timer.function = mx6cam_restart_timeout;

	vq->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	vq->io_modes = VB2_MMAP | VB2_USERPTR | VB2_DMABUF;
	vq->drv_priv = ctx;
	vq->buf_struct_size = sizeof(struct mx6cam_buffer);
	vq->ops = &mx6cam_qops;
	vq->mem_ops = &vb2_dma_contig_memops;
	vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	ret = vb2_queue_init(vq);
	if (ret) {
		v4l2_err(&dev->v4l2_dev, "vb2_queue_init failed\n");
		goto alloc_ctx_free;
	}

	ctx->io_allowed = true;
	dev->io_ctx = ctx;

	return vb2_reqbufs(vq, reqbufs);

alloc_ctx_free:
	vb2_dma_contig_cleanup_ctx(ctx->alloc_ctx);
	return ret;
}

static int vidioc_querybuf(struct file *file, void *priv,
			   struct v4l2_buffer *buf)
{
	struct mx6cam_ctx *ctx = file2ctx(file);
	struct vb2_queue *vq = &ctx->dev->buffer_queue;

	return vb2_querybuf(vq, buf);
}

static int vidioc_qbuf(struct file *file, void *priv, struct v4l2_buffer *buf)
{
	struct mx6cam_ctx *ctx = file2ctx(file);
	struct vb2_queue *vq = &ctx->dev->buffer_queue;

	if (!ctx->io_allowed)
		return -EBUSY;

	return vb2_qbuf(vq, buf);
}

static int vidioc_dqbuf(struct file *file, void *priv, struct v4l2_buffer *buf)
{
	struct mx6cam_ctx *ctx = file2ctx(file);
	struct vb2_queue *vq = &ctx->dev->buffer_queue;

	if (!ctx->io_allowed)
		return -EBUSY;

	return vb2_dqbuf(vq, buf, file->f_flags & O_NONBLOCK);
}

static int vidioc_expbuf(struct file *file, void *priv,
			 struct v4l2_exportbuffer *eb)
{
	struct mx6cam_ctx *ctx = file2ctx(file);
	struct vb2_queue *vq = &ctx->dev->buffer_queue;

	if (!ctx->io_allowed)
		return -EBUSY;

	return vb2_expbuf(vq, eb);
}

static int vidioc_streamon(struct file *file, void *priv,
			   enum v4l2_buf_type type)
{
	struct mx6cam_ctx *ctx = file2ctx(file);
	struct vb2_queue *vq = &ctx->dev->buffer_queue;

	if (!ctx->io_allowed)
		return -EBUSY;

	return vb2_streamon(vq, type);
}

static int vidioc_streamoff(struct file *file, void *priv,
			    enum v4l2_buf_type type)
{
	struct mx6cam_ctx *ctx = file2ctx(file);
	struct vb2_queue *vq = &ctx->dev->buffer_queue;

	if (!ctx->io_allowed)
		return -EBUSY;

	return vb2_streamoff(vq, type);
}

static int vidioc_overlay(struct file *file, void *priv,
			  unsigned int enable)
{
	struct mx6cam_ctx *ctx = file2ctx(file);
	struct mx6cam_dev *dev = ctx->dev;
	int ret = 0;

	if (!ctx->io_allowed)
		return -EBUSY;

	if (enable && !dev->preview_on) {
		if (vb2_is_streaming(&dev->buffer_queue) && !dev->using_ic) {
			v4l2_err(&dev->v4l2_dev,
				 "%s: not allowed while streaming w/o IC\n",
				 __func__);
			return -EBUSY;
		}

		if (dev->motion != MOTION_NONE) {
			v4l2_err(&dev->v4l2_dev,
				 "%s: not allowed with deinterlacing\n",
				 __func__);
			return -EBUSY;
		}

		if (!can_use_ic(dev, &dev->sensor_fmt, &dev->user_fmt)) {
			v4l2_err(&dev->v4l2_dev,
				 "%s: current format does not allow preview\n",
				 __func__);
			return -EINVAL;
		}

		ret = start_preview(dev);
		if (!ret)
			dev->preview_on = true;
	} else if (!enable && dev->preview_on) {
		ret = stop_preview(dev);
		dev->preview_on = false;
	}

	return ret;
}

static int vidioc_log_status(struct file *file, void *f)
{
	struct video_device *vdev = video_devdata(file);

	v4l2_ctrl_log_status(file, f);
	v4l2_device_call_all(vdev->v4l2_dev, 0, core, log_status);
	return 0;
}

static int vidioc_s_edid(struct file *file, void *f, struct v4l2_edid *edid)
{
	struct mx6cam_ctx *ctx = file2ctx(file);
	struct mx6cam_dev *dev = ctx->dev;

	return v4l2_subdev_call(dev->ep->sd, pad, set_edid, edid);
}

static int vidioc_g_edid(struct file *file, void *f, struct v4l2_edid *edid)
{
	struct mx6cam_ctx *ctx = file2ctx(file);
	struct mx6cam_dev *dev = ctx->dev;

	return v4l2_subdev_call(dev->ep->sd, pad, get_edid, edid);
}

static int vidioc_g_dv_timings(struct file *file, void *priv_fh,
				    struct v4l2_dv_timings *timings)
{
	struct mx6cam_ctx *ctx = file2ctx(file);
	struct mx6cam_dev *dev = ctx->dev;
	struct mx6cam_sensor_input *epinput;
	struct mx6cam_endpoint *ep;
	int sensor_input;

	ep = find_ep_by_input_index(dev, dev->current_input);
	if (!ep)
		return -EINVAL;

	epinput = &ep->sensor_input;
	sensor_input = dev->current_input - epinput->first;
	if (epinput->caps[sensor_input] != V4L2_IN_CAP_DV_TIMINGS) {
		return -ENODATA;
	}

	return v4l2_subdev_call(dev->ep->sd,
			video, g_dv_timings, timings);
}

static int vidioc_s_dv_timings(struct file *file, void *priv_fh,
				    struct v4l2_dv_timings *timings)
{
	struct mx6cam_ctx *ctx = file2ctx(file);
	struct mx6cam_dev *dev = ctx->dev;
	struct mx6cam_sensor_input *epinput;
	struct mx6cam_endpoint *ep;
	int sensor_input;

	ep = find_ep_by_input_index(dev, dev->current_input);
	if (!ep)
		return -EINVAL;

	epinput = &ep->sensor_input;
	sensor_input = dev->current_input - epinput->first;
	if (epinput->caps[sensor_input] != V4L2_IN_CAP_DV_TIMINGS) {
		return -ENODATA;
	}

	return v4l2_subdev_call(dev->ep->sd,
			video, s_dv_timings, timings);
}

static int vidioc_query_dv_timings(struct file *file, void *priv_fh,
				    struct v4l2_dv_timings *timings)
{
	struct mx6cam_ctx *ctx = file2ctx(file);
	struct mx6cam_dev *dev = ctx->dev;
	struct mx6cam_sensor_input *epinput;
	struct mx6cam_endpoint *ep;
	int sensor_input;

	ep = find_ep_by_input_index(dev, dev->current_input);
	if (!ep)
		return -EINVAL;

	epinput = &ep->sensor_input;
	sensor_input = dev->current_input - epinput->first;
	if (epinput->caps[sensor_input] != V4L2_IN_CAP_DV_TIMINGS) {
		return -ENODATA;
	}

	return v4l2_subdev_call(dev->ep->sd,
			video, query_dv_timings, timings);
}
static int vidioc_enum_dv_timings(struct file *file, void *priv_fh,
				    struct v4l2_enum_dv_timings *timings)
{
	struct mx6cam_ctx *ctx = file2ctx(file);
	struct mx6cam_dev *dev = ctx->dev;
	struct mx6cam_sensor_input *epinput;
	struct mx6cam_endpoint *ep;
	int sensor_input;

	ep = find_ep_by_input_index(dev, dev->current_input);
	if (!ep)
		return -EINVAL;

	epinput = &ep->sensor_input;
	sensor_input = dev->current_input - epinput->first;
	if (epinput->caps[sensor_input] != V4L2_IN_CAP_DV_TIMINGS) {
		return -ENODATA;
	}

	timings->pad = 0;
	v4l2_err(&dev->v4l2_dev, "Enum DV Timings on input %s\n", epinput->name[sensor_input]);
	return v4l2_subdev_call(dev->ep->sd,
			pad, enum_dv_timings, timings);
}

static int vidioc_dv_timings_cap(struct file *file, void *priv_fh,
				    struct v4l2_dv_timings_cap *cap)
{
	struct mx6cam_ctx *ctx = file2ctx(file);
	struct mx6cam_dev *dev = ctx->dev;

	cap->pad = 0;
	return v4l2_subdev_call(dev->ep->sd,
			pad, dv_timings_cap, cap);
}

static const struct v4l2_ioctl_ops mx6cam_ioctl_ops = {
	.vidioc_querycap	= vidioc_querycap,

	.vidioc_enum_fmt_vid_cap        = vidioc_enum_fmt_vid_cap,
	.vidioc_g_fmt_vid_cap           = vidioc_g_fmt_vid_cap,
	.vidioc_try_fmt_vid_cap         = vidioc_try_fmt_vid_cap,
	.vidioc_s_fmt_vid_cap           = vidioc_s_fmt_vid_cap,

	.vidioc_enum_framesizes         = vidioc_enum_framesizes,
	.vidioc_enum_frameintervals     = vidioc_enum_frameintervals,

	.vidioc_enum_fmt_vid_overlay    = vidioc_enum_fmt_vid_overlay,
	.vidioc_g_fmt_vid_overlay	= vidioc_g_fmt_vid_overlay,
	.vidioc_try_fmt_vid_overlay	= vidioc_try_fmt_vid_overlay,
	.vidioc_s_fmt_vid_overlay	= vidioc_s_fmt_vid_overlay,

	.vidioc_querystd		= vidioc_querystd,
	.vidioc_g_std           = vidioc_g_std,
	.vidioc_s_std           = vidioc_s_std,

	.vidioc_enum_input      = vidioc_enum_input,
	.vidioc_g_input         = vidioc_g_input,
	.vidioc_s_input         = vidioc_s_input,

	.vidioc_g_parm          = vidioc_g_parm,
	.vidioc_s_parm          = vidioc_s_parm,

	.vidioc_g_fbuf          = vidioc_g_fbuf,
	.vidioc_s_fbuf          = vidioc_s_fbuf,

	.vidioc_cropcap         = vidioc_cropcap,
	.vidioc_g_crop          = vidioc_g_crop,
	.vidioc_s_crop          = vidioc_s_crop,

	.vidioc_reqbufs		= vidioc_reqbufs,
	.vidioc_querybuf	= vidioc_querybuf,
	.vidioc_qbuf		= vidioc_qbuf,
	.vidioc_dqbuf		= vidioc_dqbuf,
	.vidioc_expbuf		= vidioc_expbuf,

	.vidioc_streamon	= vidioc_streamon,
	.vidioc_streamoff	= vidioc_streamoff,
	.vidioc_overlay         = vidioc_overlay,
	.vidioc_log_status      = vidioc_log_status,
	.vidioc_s_edid          = vidioc_s_edid,
	.vidioc_g_edid          = vidioc_g_edid,
	.vidioc_s_dv_timings	= vidioc_s_dv_timings,
	.vidioc_g_dv_timings	= vidioc_g_dv_timings,
	.vidioc_query_dv_timings = vidioc_query_dv_timings,
	.vidioc_enum_dv_timings	= vidioc_enum_dv_timings,
	.vidioc_dv_timings_cap	= vidioc_dv_timings_cap,
};


/*
 * Queue operations
 */

static int mx6cam_queue_setup(struct vb2_queue *vq,
			      const struct v4l2_format *fmt,
			      unsigned int *nbuffers, unsigned int *nplanes,
			      unsigned int sizes[], void *alloc_ctxs[])
{
	struct mx6cam_ctx *ctx = vb2_get_drv_priv(vq);
	struct mx6cam_dev *dev = ctx->dev;
	unsigned int count = *nbuffers;
	u32 sizeimage = dev->user_fmt.fmt.pix.sizeimage;

	if (vq->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	while (sizeimage * count > VID_MEM_LIMIT)
		count--;

	*nplanes = 1;
	*nbuffers = count;
	sizes[0] = sizeimage;

	alloc_ctxs[0] = ctx->alloc_ctx;

	dprintk(dev, "get %d buffer(s) of size %d each.\n", count, sizeimage);

	return 0;
}

static int mx6cam_buf_init(struct vb2_buffer *vb)
{
	struct mx6cam_buffer *buf = to_mx6cam_vb(vb);

	INIT_LIST_HEAD(&buf->list);
	return 0;
}

static int mx6cam_buf_prepare(struct vb2_buffer *vb)
{
	struct mx6cam_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct mx6cam_dev *dev = ctx->dev;

	if (vb2_plane_size(vb, 0) < dev->user_fmt.fmt.pix.sizeimage) {
		v4l2_err(&dev->v4l2_dev,
			 "data will not fit into plane (%lu < %lu)\n",
			 vb2_plane_size(vb, 0),
			 (long)dev->user_fmt.fmt.pix.sizeimage);
		return -EINVAL;
	}

	vb2_set_plane_payload(vb, 0, dev->user_fmt.fmt.pix.sizeimage);

	return 0;
}

static void mx6cam_buf_queue(struct vb2_buffer *vb)
{
	struct mx6cam_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct mx6cam_dev *dev = ctx->dev;
	struct mx6cam_buffer *buf = to_mx6cam_vb(vb);
	unsigned long flags;
	bool kickstart;

	spin_lock_irqsave(&dev->irqlock, flags);

	list_add_tail(&buf->list, &ctx->ready_q);

	/* kickstart DMA chain if we have two frames in active q */
	kickstart = (vb2_is_streaming(vb->vb2_queue) &&
		     !(list_empty(&ctx->ready_q) ||
		       list_is_singular(&ctx->ready_q)));

	spin_unlock_irqrestore(&dev->irqlock, flags);

	if (kickstart)
		start_encoder(dev);
}

static void mx6cam_lock(struct vb2_queue *vq)
{
	struct mx6cam_ctx *ctx = vb2_get_drv_priv(vq);
	struct mx6cam_dev *dev = ctx->dev;

	mutex_lock(&dev->mutex);
}

static void mx6cam_unlock(struct vb2_queue *vq)
{
	struct mx6cam_ctx *ctx = vb2_get_drv_priv(vq);
	struct mx6cam_dev *dev = ctx->dev;

	mutex_unlock(&dev->mutex);
}

static int mx6cam_start_streaming(struct vb2_queue *vq, unsigned int count)
{
	struct mx6cam_ctx *ctx = vb2_get_drv_priv(vq);

	if (vb2_is_streaming(vq))
		return 0;

	return set_stream(ctx, true);
}

static void mx6cam_stop_streaming(struct vb2_queue *vq)
{
	struct mx6cam_ctx *ctx = vb2_get_drv_priv(vq);
	struct mx6cam_dev *dev = ctx->dev;
	struct mx6cam_buffer *frame;
	unsigned long flags;

	if (!vb2_is_streaming(vq))
		return;

	set_stream(ctx, false);

	spin_lock_irqsave(&dev->irqlock, flags);

	/* release all active buffers */
	while (!list_empty(&ctx->ready_q)) {
		frame = list_entry(ctx->ready_q.next,
				   struct mx6cam_buffer, list);
		list_del(&frame->list);
		vb2_buffer_done(&frame->vb, VB2_BUF_STATE_ERROR);
	}

	spin_unlock_irqrestore(&dev->irqlock, flags);
}

static struct vb2_ops mx6cam_qops = {
	.queue_setup	 = mx6cam_queue_setup,
	.buf_init        = mx6cam_buf_init,
	.buf_prepare	 = mx6cam_buf_prepare,
	.buf_queue	 = mx6cam_buf_queue,
	.wait_prepare	 = mx6cam_unlock,
	.wait_finish	 = mx6cam_lock,
	.start_streaming = mx6cam_start_streaming,
	.stop_streaming  = mx6cam_stop_streaming,
};

/*
 * File operations
 */
static int mx6cam_open(struct file *file)
{
	struct mx6cam_dev *dev = video_drvdata(file);
	struct mx6cam_ctx *ctx;
	int ret;

	if (mutex_lock_interruptible(&dev->mutex))
		return -ERESTARTSYS;

	if (!dev->ep || !dev->ep->sd) {
		v4l2_err(&dev->v4l2_dev, "no subdevice registered\n");
		ret = -ENODEV;
		goto unlock;
	}

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx) {
		ret = -ENOMEM;
		goto unlock;
	}

	v4l2_fh_init(&ctx->fh, video_devdata(file));
	file->private_data = &ctx->fh;
	ctx->dev = dev;
	v4l2_fh_add(&ctx->fh);
	ctx->io_allowed = false;

	ret = sensor_set_power(dev, 1);
	if (ret) {
		v4l2_err(&dev->v4l2_dev, "sensor power on failed\n");
		goto ctx_free;
	}

	if (dev->ep->ep.bus_type == V4L2_MBUS_CSI2 && dev->csi2_sd) {
		ret = v4l2_subdev_call(dev->csi2_sd, core, s_power, true);
		if (ret) {
			v4l2_err(&dev->v4l2_dev, "csi2 power on failed\n");
			goto sensor_off;
		}
	}

	/* update the sensor's current format */
	update_sensor_fmt(dev);
	/* and init crop window if needed */
	if (!dev->crop.width || !dev->crop.height)
		calc_default_crop(dev, &dev->crop, &dev->sensor_fmt);

	mutex_unlock(&dev->mutex);
	return 0;

sensor_off:
	sensor_set_power(dev, 0);
ctx_free:
	v4l2_fh_del(&ctx->fh);
	v4l2_fh_exit(&ctx->fh);
	kfree(ctx);
unlock:
	mutex_unlock(&dev->mutex);
	return ret;
}

static int mx6cam_release(struct file *file)
{
	struct mx6cam_ctx *ctx = file2ctx(file);
	struct mx6cam_dev *dev = ctx->dev;
	int ret = 0;

	v4l2_fh_del(&ctx->fh);
	v4l2_fh_exit(&ctx->fh);

	mutex_lock(&dev->mutex);

	if (ctx->io_allowed) {
		BUG_ON(dev->io_ctx != ctx);

		vb2_queue_release(&dev->buffer_queue);
		vb2_dma_contig_cleanup_ctx(ctx->alloc_ctx);

		if (dev->preview_on) {
			stop_preview(dev);
			dev->preview_on = false;
		}

		dev->io_ctx = NULL;
	}

	if (dev->ep == NULL || dev->ep->sd == NULL) {
		v4l2_warn(&dev->v4l2_dev, "lost the slave?\n");
		goto unlock;
	}

	ret = sensor_set_power(dev, 0);
	if (ret)
		v4l2_err(&dev->v4l2_dev, "sensor power off failed\n");

unlock:
	mutex_unlock(&dev->mutex);
	kfree(ctx);
	return ret;
}

static unsigned int mx6cam_poll(struct file *file,
				 struct poll_table_struct *wait)
{
	struct mx6cam_ctx *ctx = file2ctx(file);
	struct mx6cam_dev *dev = ctx->dev;
	struct vb2_queue *vq = &dev->buffer_queue;
	int ret;

	if (mutex_lock_interruptible(&dev->mutex))
		return -ERESTARTSYS;

	ret = vb2_poll(vq, file, wait);

	mutex_unlock(&dev->mutex);
	return ret;
}

static int mx6cam_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct mx6cam_ctx *ctx = file2ctx(file);
	struct mx6cam_dev *dev = ctx->dev;
	struct vb2_queue *vq = &dev->buffer_queue;
	int ret;

	if (mutex_lock_interruptible(&dev->mutex))
		return -ERESTARTSYS;

	ret = vb2_mmap(vq, vma);

	mutex_unlock(&dev->mutex);
	return ret;
}

static const struct v4l2_file_operations mx6cam_fops = {
	.owner		= THIS_MODULE,
	.open		= mx6cam_open,
	.release	= mx6cam_release,
	.poll		= mx6cam_poll,
	.unlocked_ioctl	= video_ioctl2,
	.mmap		= mx6cam_mmap,
};

static struct video_device mx6cam_videodev = {
	.name		= DEVICE_NAME,
	.fops		= &mx6cam_fops,
	.ioctl_ops	= &mx6cam_ioctl_ops,
	.minor		= -1,
	.release	= video_device_release,
	.vfl_dir	= VFL_DIR_RX,
};

/*
 * Handle notifications from the subdevs.
 */
static void mx6cam_subdev_notification(struct v4l2_subdev *sd,
				       unsigned int notification,
				       void *arg)
{
	struct mx6cam_dev *dev;
	struct mx6cam_ctx *ctx;

	if (sd == NULL)
		return;

	dev = sd2dev(sd);
	ctx = dev->io_ctx;

	switch (notification) {
	case MX6CAM_NFB4EOF_NOTIFY:
		if (ctx)
			mod_timer(&ctx->restart_timer, jiffies +
				  msecs_to_jiffies(MX6CAM_RESTART_DELAY));
		break;
	case DECODER_STATUS_CHANGE_NOTIFY:
		atomic_set(&dev->status_change, 1);
		if (ctx) {
			v4l2_warn(&dev->v4l2_dev, "decoder status change\n");
			mod_timer(&ctx->restart_timer, jiffies +
				  msecs_to_jiffies(MX6CAM_RESTART_DELAY));
		}
		break;
	case MX6CAM_EOF_TIMEOUT_NOTIFY:
		if (ctx) {
			/* cancel a running restart timer since we are
			   restarting now anyway */
			del_timer_sync(&ctx->restart_timer);
			/* and restart now */
			schedule_work(&ctx->restart_work);
		}
		break;
	}
}

static int mx6cam_add_sensor(struct mx6cam_dev *dev,
			     struct device_node *remote,
			     struct mx6cam_endpoint *ep)
{
	struct platform_device *sensor_pdev = NULL;
	struct i2c_client *i2c_client;
	struct device *sensor_dev;
	int ret = 0;

	i2c_client = of_find_i2c_device_by_node(remote);
	if (!i2c_client)
		sensor_pdev = of_find_device_by_node(remote);

	if (!i2c_client && !sensor_pdev)
		return -EPROBE_DEFER;

	sensor_dev = i2c_client ? &i2c_client->dev : &sensor_pdev->dev;

	device_lock(sensor_dev);

	if (!sensor_dev->driver ||
	    !try_module_get(sensor_dev->driver->owner)) {
		ret = -EPROBE_DEFER;
		v4l2_info(&dev->v4l2_dev, "No driver found for %s\n",
			  remote->full_name);
		goto unlock;
	}

	ep->sd = i2c_client ? i2c_get_clientdata(i2c_client) :
		dev_get_drvdata(sensor_dev);

	ret = v4l2_device_register_subdev(&dev->v4l2_dev, ep->sd);
	if (ret < 0) {
		v4l2_err(&dev->v4l2_dev, "failed to register subdev %s\n",
			 ep->sd->name);
		goto mod_put;
	}

	v4l2_info(&dev->v4l2_dev, "Registered sensor subdev %s on CSI%d\n",
		  ep->sd->name, ep->ep.base.port);
	ret = 0;

mod_put:
	module_put(sensor_dev->driver->owner);
unlock:
	device_unlock(sensor_dev);
	put_device(sensor_dev);
	return ret;
}

static int mx6cam_add_csi2_receiver(struct mx6cam_dev *dev)
{
	struct platform_device *pdev;
	struct device_node *node;
	int ret = -EPROBE_DEFER;

	node = of_find_compatible_node(NULL, NULL, "fsl,imx6-mipi-csi2");
	if (!node)
		return 0;

	pdev = of_find_device_by_node(node);
	of_node_put(node);
	if (!pdev)
		return 0;

	/* Lock to ensure dev->driver won't change. */
	device_lock(&pdev->dev);

	if (!pdev->dev.driver || !try_module_get(pdev->dev.driver->owner))
		goto dev_unlock;

	dev->csi2_sd = dev_get_drvdata(&pdev->dev);

	ret = v4l2_device_register_subdev(&dev->v4l2_dev, dev->csi2_sd);
	if (ret) {
		v4l2_err(&dev->v4l2_dev, "failed to register mipi_csi2!\n");
		goto mod_put;
	}

	v4l2_info(&dev->v4l2_dev, "Registered subdev %s\n",
		  dev->csi2_sd->name);
mod_put:
	module_put(pdev->dev.driver->owner);
dev_unlock:
	device_unlock(&pdev->dev);
	if (ret == -EPROBE_DEFER)
		v4l2_info(&dev->v4l2_dev,
			  "deferring mipi_csi2 registration\n");
	else if (ret < 0)
		v4l2_err(&dev->v4l2_dev,
			 "mipi_csi2 registration failed (%d)\n", ret);
	return ret;
}

/* parse inputs property from v4l2_of_endpoint node */
static int mx6cam_parse_inputs(struct mx6cam_dev *dev,
			       struct device_node *node,
			       int next_input,
			       struct mx6cam_endpoint *ep)
{
	struct mx6cam_sensor_input *epinput = &ep->sensor_input;
	int ret, i;

	for (i = 0; i < MX6CAM_MAX_INPUTS; i++) {
		const char *input_name;
		u32 val;

		ret = of_property_read_u32_index(node, "inputs", i, &val);
		if (ret)
			break;

		epinput->value[i] = val;

		ret = of_property_read_string_index(node, "input-names", i,
						    &input_name);
		if (!ret)
			strncpy(epinput->name[i], input_name,
				sizeof(epinput->name[i]));
		else
			snprintf(epinput->name[i], sizeof(epinput->name[i]),
				 "%s-%d", ep->sd->name, i);

		val = 0;
		ret = of_property_read_u32_index(node, "input-caps", i, &val);
		epinput->caps[i] = val;
	}

	epinput->num = i;

	/* if no inputs provided just assume a single input */
	if (epinput->num == 0) {
		epinput->num = 1;
		epinput->caps[0] = 0;
		strncpy(epinput->name[0], ep->sd->name,
			sizeof(epinput->name[0]));
	}

	epinput->first = next_input;
	epinput->last = next_input + epinput->num - 1;
	return epinput->last + 1;
}

static int mx6cam_parse_endpoints(struct mx6cam_dev *dev,
				  struct device_node *node)
{
	struct device_node *remote, *epnode = NULL;
	struct v4l2_of_endpoint ep;
	int ret, next_input = 0;

	while (dev->num_eps < MX6CAM_MAX_ENDPOINTS) {
		epnode = of_graph_get_next_endpoint(node, epnode);
		if (!epnode)
			break;

		v4l2_of_parse_endpoint(epnode, &ep);

		if (ep.base.port > 1) {
			v4l2_err(&dev->v4l2_dev, "invalid port %d\n",
				 ep.base.port);
			of_node_put(epnode);
			return -EINVAL;
		}

		remote = of_graph_get_remote_port_parent(epnode);
		if (!remote) {
			v4l2_err(&dev->v4l2_dev,
				 "failed to find remote port parent\n");
			of_node_put(epnode);
			return -EINVAL;
		}

		dev->eplist[dev->num_eps].ep = ep;
		ret = mx6cam_add_sensor(dev, remote,
					&dev->eplist[dev->num_eps]);
		if (ret)
			goto out;

		next_input = mx6cam_parse_inputs(dev, epnode, next_input,
						 &dev->eplist[dev->num_eps]);

		dev->num_eps++;

		of_node_put(remote);
		of_node_put(epnode);
	}

	if (!dev->num_eps) {
		v4l2_err(&dev->v4l2_dev, "no endpoints defined!\n");
		return -EINVAL;
	}

	dev->ep = &dev->eplist[0];
	return 0;
out:
	of_node_put(remote);
	of_node_put(epnode);
	return ret;
}

static void mx6cam_unregister_subdevs(struct mx6cam_dev *dev)
{
	struct i2c_adapter *adapter;
	struct i2c_client *client;
	struct mx6cam_endpoint *ep;
	int i;

	if (!IS_ERR_OR_NULL(dev->encoder_sd))
		v4l2_device_unregister_subdev(dev->encoder_sd);

	if (!IS_ERR_OR_NULL(dev->preview_sd))
		v4l2_device_unregister_subdev(dev->preview_sd);

	if (!IS_ERR_OR_NULL(dev->vdic_sd))
		v4l2_device_unregister_subdev(dev->vdic_sd);

	if (!IS_ERR_OR_NULL(dev->csi2_sd))
		v4l2_device_unregister_subdev(dev->csi2_sd);

	for (i = 0; i < dev->num_eps; i++) {
		ep = &dev->eplist[i];
		if (!ep->sd)
			continue;
		client = v4l2_get_subdevdata(ep->sd);
		if (!client)
			continue;

		v4l2_device_unregister_subdev(ep->sd);

		if (!client->dev.of_node) {
			adapter = client->adapter;
			i2c_unregister_device(client);
			if (adapter)
				i2c_put_adapter(adapter);
		}
	}
}


static int of_dev_node_match(struct device *dev, void *data)
{
	return dev->of_node == data;
}

static struct ipu_soc *mx6cam_find_ipu(struct mx6cam_dev *dev,
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

static int mx6cam_probe(struct platform_device *pdev)
{
	struct mx6_camera_pdata *pdata = pdev->dev.platform_data;
	struct device_node *node = pdev->dev.of_node;
	struct mx6cam_dev *dev;
	struct video_device *vfd;
	struct pinctrl *pinctrl;
	int ret;

	dev = devm_kzalloc(&pdev->dev, sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	dev->dev = &pdev->dev;
	mutex_init(&dev->mutex);
	spin_lock_init(&dev->irqlock);

	ret = v4l2_device_register(&pdev->dev, &dev->v4l2_dev);
	if (ret)
		return ret;

	pdev->dev.coherent_dma_mask = DMA_BIT_MASK(32);

	if (!pdata) {
		v4l2_err(&dev->v4l2_dev, "no platform data!\n");
		ret = -EINVAL;
		goto unreg_dev;
	}
	dev->pdata = pdata;

	/* get our IPU */
	dev->ipu = mx6cam_find_ipu(dev, node);
	if (IS_ERR_OR_NULL(dev->ipu)) {
		v4l2_err(&dev->v4l2_dev, "could not get ipu\n");
		ret = -ENODEV;
		goto unreg_dev;
	}

	vfd = video_device_alloc();
	if (!vfd) {
		v4l2_err(&dev->v4l2_dev, "Failed to allocate video device\n");
		ret = -ENOMEM;
		goto unreg_dev;
	}

	*vfd = mx6cam_videodev;
	vfd->lock = &dev->mutex;
	vfd->v4l2_dev = &dev->v4l2_dev;
	dev->v4l2_dev.notify = mx6cam_subdev_notification;

	ret = video_register_device(vfd, VFL_TYPE_GRABBER, 0);
	if (ret) {
		v4l2_err(&dev->v4l2_dev, "Failed to register video device\n");
		goto unreg_vdev;
	}

	video_set_drvdata(vfd, dev);
	snprintf(vfd->name, sizeof(vfd->name), "%s", mx6cam_videodev.name);
	dev->vfd = vfd;

	platform_set_drvdata(pdev, dev);

	/* Get any pins needed */
	pinctrl = devm_pinctrl_get_select_default(&pdev->dev);

	/* setup some defaults */
	dev->user_fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	dev->user_fmt.fmt.pix.width = 640;
	dev->user_fmt.fmt.pix.height = 480;
	dev->user_fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUV420;
	dev->user_fmt.fmt.pix.bytesperline = (640 * 12) >> 3;
	dev->user_fmt.fmt.pix.sizeimage =
		(480 * dev->user_fmt.fmt.pix.bytesperline);
	dev->user_pixfmt =
		mx6cam_get_format(dev->user_fmt.fmt.pix.pixelformat, 0);
	dev->current_std = V4L2_STD_ALL;

	/* init our controls */
	ret = mx6cam_init_controls(dev);
	if (ret) {
		v4l2_err(&dev->v4l2_dev, "init controls failed\n");
		goto unreg_vdev;
	}

	/* find and register mipi csi2 receiver subdev */
	ret = mx6cam_add_csi2_receiver(dev);
	if (ret)
		goto free_ctrls;

	/* parse and register all sensor endpoints */
	ret = mx6cam_parse_endpoints(dev, node);
	if (ret)
		goto unreg_subdevs;

	dev->encoder_sd = mx6cam_encoder_init(dev);
	if (IS_ERR(dev->encoder_sd)) {
		ret = PTR_ERR(dev->encoder_sd);
		goto unreg_subdevs;
	}

	dev->preview_sd = mx6cam_preview_init(dev);
	if (IS_ERR(dev->preview_sd)) {
		ret = PTR_ERR(dev->preview_sd);
		goto unreg_subdevs;
	}

	dev->vdic_sd = mx6cam_vdic_init(dev);
	if (IS_ERR(dev->vdic_sd)) {
		ret = PTR_ERR(dev->vdic_sd);
		goto unreg_subdevs;
	}

	ret = v4l2_device_register_subdev(&dev->v4l2_dev, dev->encoder_sd);
	if (ret < 0) {
		v4l2_err(&dev->v4l2_dev,
			 "failed to register encoder subdev\n");
		goto unreg_subdevs;
	}
	v4l2_info(&dev->v4l2_dev, "Registered subdev %s\n",
		  dev->encoder_sd->name);

	ret = v4l2_device_register_subdev(&dev->v4l2_dev, dev->preview_sd);
	if (ret < 0) {
		v4l2_err(&dev->v4l2_dev,
			 "failed to register preview subdev\n");
		goto unreg_subdevs;
	}
	v4l2_info(&dev->v4l2_dev, "Registered subdev %s\n",
		  dev->preview_sd->name);

	ret = v4l2_device_register_subdev(&dev->v4l2_dev, dev->vdic_sd);
	if (ret < 0) {
		v4l2_err(&dev->v4l2_dev,
			 "failed to register vdic subdev\n");
		goto unreg_subdevs;
	}
	v4l2_info(&dev->v4l2_dev, "Registered subdev %s\n",
		  dev->vdic_sd->name);

	ret = v4l2_device_register_subdev_nodes(&dev->v4l2_dev);
	if (ret)
		goto unreg_subdevs;

	v4l2_info(&dev->v4l2_dev,
		  "Device registered as /dev/video%d, on ipu%d\n",
		  vfd->num, ipu_get_num(dev->ipu));

	return 0;

unreg_subdevs:
	mx6cam_unregister_subdevs(dev);
free_ctrls:
	v4l2_ctrl_handler_free(&dev->ctrl_hdlr);
unreg_vdev:
	video_unregister_device(dev->vfd);
unreg_dev:
	v4l2_device_unregister(&dev->v4l2_dev);
	return ret;
}

static int mx6cam_remove(struct platform_device *pdev)
{
	struct mx6cam_dev *dev =
		(struct mx6cam_dev *)platform_get_drvdata(pdev);

	v4l2_info(&dev->v4l2_dev, "Removing " DEVICE_NAME "\n");
	v4l2_ctrl_handler_free(&dev->ctrl_hdlr);
	video_unregister_device(dev->vfd);
	mx6cam_unregister_subdevs(dev);
	v4l2_device_unregister(&dev->v4l2_dev);

	return 0;
}

static struct of_device_id mx6cam_dt_ids[] = {
	{ .compatible = "fsl,imx6-v4l2-capture" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, mx6cam_dt_ids);

static struct platform_driver mx6cam_pdrv = {
	.probe		= mx6cam_probe,
	.remove		= mx6cam_remove,
	.driver		= {
		.name	= DEVICE_NAME,
		.owner	= THIS_MODULE,
		.of_match_table	= mx6cam_dt_ids,
	},
};

module_platform_driver(mx6cam_pdrv);

MODULE_DESCRIPTION("i.MX6 v4l2 capture driver");
MODULE_AUTHOR("Mentor Graphics Inc.");
MODULE_LICENSE("GPL");
